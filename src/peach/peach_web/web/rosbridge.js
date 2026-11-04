"use strict";

// Peach projections and command policy on top of the installed ros2_web_link SDK.
(function (root) {
  const explicitMode = new URLSearchParams(root.location.search).get("transport");
  const changes = new EventTarget(), frames = new Map(), goals = new Map();
  const topics = new Set();
  const paramTypes = {bool: 1, int: 2, double: 3, string: 4};
  const qos = {history: "keep_last", depth: 1, reliability: "reliable", durability: "volatile"};
  let config, client, ready, stopped = false;
  const fail = (message, kind = "network") => Object.assign(new Error(message), {kind});
  const reply = (status, body) => ({response: {ok: status >= 200 && status < 300, status}, body});
  const mode = () => ["http", "rosbridge"].includes(explicitMode) ? explicitMode : config?.default;

  async function configuration() {
    const response = await fetch("/api/transport", {cache: "no-store", signal: AbortSignal.timeout(6000)});
    if (!response.ok) throw fail(`通信配置 HTTP ${response.status}`);
    const metadata = await response.json();
    if (!["http", "rosbridge"].includes(metadata.default)) throw fail("通信配置缺少有效的默认传输");
    config = metadata;
  }

  async function initialize() {
    await configuration();
    if (mode() === "http") return;
    if (config.default === "http") throw fail("部署已停用 ROSbridge，请选择 HTTP");
    if (!root.Ros2WebLink) {
      await new Promise((resolve, reject) => {
        const script = document.createElement("script");
        script.src = "/vendor/ros2-web-link.js";
        const failed = () => {
          clearTimeout(timer);
          script.remove();
          reject(fail("ros2_web_link 客户端加载失败"));
        };
        const timer = setTimeout(failed, 6000);
        script.onload = () => { clearTimeout(timer); resolve(); };
        script.onerror = failed;
        document.head.append(script);
      });
    }
    if (stopped) throw fail("页面已关闭", "abort");
    client = new root.Ros2WebLink.RosClient({url: config.url});
    client.onState(() => {
      if (!client.connected) frames.clear();
      changes.dispatchEvent(new Event("change"));
    });
    // Reads can wait for discovery. A command always requires an open connection.
    client.connect().catch(() => {});
  }

  async function waitForTransport(signal) {
    signal?.throwIfAborted();
    ready ??= initialize().catch((error) => { ready = null; throw error; });
    if (!signal) return ready;
    // One short-lived read may expire while other callers still need the shared SDK.
    let onAbort;
    const aborted = new Promise((_, reject) => {
      onAbort = () => reject(fail("请求已取消", "abort"));
      signal.addEventListener("abort", onAbort, {once: true});
    });
    try { await Promise.race([ready, aborted]); }
    finally { signal.removeEventListener("abort", onAbort); }
  }

  function waitFor(read, {signal, timeoutMs = 6000}) {
    return new Promise((resolve, reject) => {
      const finish = (error, value) => {
        changes.removeEventListener("change", check);
        signal?.removeEventListener("abort", abort);
        clearTimeout(timer);
        if (error) reject(error); else resolve(value);
      };
      const abort = () => finish(fail("请求已取消", "abort"));
      const check = () => {
        if (stopped || signal?.aborted) return abort();
        if (client.state === "reconnecting" || client.state === "disconnected") {
          return finish(fail("ROSbridge 连接中断"));
        }
        const value = read();
        if (value) finish(null, value);
      };
      const timer = setTimeout(() => finish(fail("等待 ROS 响应超时", "timeout")), timeoutMs);
      changes.addEventListener("change", check);
      signal?.addEventListener("abort", abort, {once: true});
      check();
    });
  }

  async function snapshot(key, options) {
    if (!topics.has(key)) {
      client.subscribe(config.topics[key], "std_msgs/msg/String", (message, {receivedAt}) => {
        try {
          frames.set(key, {value: JSON.parse(message.data), at: performance.now(), receivedAt});
        } catch (_) { frames.delete(key); }
        changes.dispatchEvent(new Event("change"));
      }, {qos});
      topics.add(key);
    }
    const frame = await waitFor(() => {
      const sample = frames.get(key);
      return client.connected && sample && performance.now() - sample.at <= (key === "force" ? 1500 : 5000) && sample;
    }, options);
    const result = reply(200, frame.value);
    result.response.receivedAt = frame.receivedAt;
    return result;
  }

  async function gates(options) {
    const {values: [enabled, motion]} = await client.parameters("/peach_observability")
      .get(["debug.enabled", "debug.motion_enabled"], options);
    return {debug_enabled: enabled?.value === true, motion_enabled: motion?.value === true};
  }

  async function parameters(options) {
    const groups = await Promise.all(config.groups.map(async (group) => {
      const values = new Map(), api = client.parameters(`/${group.node}`);
      let running = false, error = "";
      try {
        const listed = await api.list([], 0, options);
        running = true;
        // Jazzy rclpy can return an empty array for the whole get if any name is undeclared.
        const declared = group.params.filter((p) => listed.names.includes(p.name));
        if (declared.length) {
          const result = await api.get(declared.map((p) => p.name), options);
          declared.forEach((p, i) => {
            const item = result.values[i];
            if (item && item.type === paramTypes[p.type]) values.set(p.name, item.value);
          });
        }
        const missing = group.params.filter((p) => !values.has(p.name)).map((p) => p.name);
        if (missing.length) error = `未声明或类型不符: ${missing.join(", ")}`;
      } catch (reason) {
        if (options.signal?.aborted || !client.connected) throw reason;
        running = false;
        error = reason.message;
      }
      return {...group, running, error, params: group.params.map((p) => ({
        ...p, value: values.get(p.name) ?? null, present: values.has(p.name), node_running: running,
      }))};
    }));
    return {groups, ...await gates(options)};
  }

  function parameterWrite(spec, value) {
    const type = paramTypes[spec.type];
    try { root.Ros2WebLink.encodeParameter(type, value); }
    catch (_) { throw fail(`需要有效的 ${spec.type} 值`, "validation"); }
    if (spec.min !== undefined && value < spec.min) throw fail(`低于下限 ${spec.min}`, "validation");
    if (spec.max !== undefined && value > spec.max) throw fail(`高于上限 ${spec.max}`, "validation");
    if (spec.choices && !spec.choices.includes(value)) throw fail("值不在允许选项中", "validation");
    return {name: spec.name, type, value};
  }

  async function setParameters(payload, options) {
    if (!Array.isArray(payload.changes) || !payload.changes.length) return reply(400, {accepted: false, message: "changes 必须是非空数组"});
    let gate = await gates(options);
    if (!gate.debug_enabled) return reply(503, {accepted: false, message: "debug.enabled=false，参数写入已关闭"});
    const results = [];
    let stopReason = "";
    for (const change of payload.changes) {
      const row = {node: change?.node || "", name: change?.name || "", accepted: false, message: ""};
      if (stopReason || !gate.debug_enabled) {
        results.push({...row, outcome: "not_sent", message: stopReason || "debug.enabled=false，后续参数未发送"});
        continue;
      }
      try {
        const spec = config.groups.find((g) => g.node === row.node)?.params.find((p) => p.name === row.name);
        if (!spec) throw fail("不在白名单", "validation");
        if (spec.motion && !gate.motion_enabled) throw fail("运动类参数需先开启 debug.motion_enabled", "validation");
        const write = parameterWrite(spec, change.value);
        const response = await client.parameters(`/${row.node}`).set([write], options);
        if (response.results?.length !== 1 || typeof response.results[0]?.successful !== "boolean") {
          throw fail("节点未返回单项写入结果");
        }
        row.accepted = response.results[0].successful;
        row.message = row.accepted ? `已设置${spec.static ? "（静态参数，可能需重启生效）" : ""}` : (response.results?.[0]?.reason || "节点拒绝");
        if (row.node === "peach_observability" && row.accepted) gate = await gates(options);
      } catch (error) {
        if (error.kind === "validation") row.message = error.message;
        else {
          stopReason = row.accepted ? "调试开关读回失败，后续参数未发送" : "前一项写入结果未知，后续参数未发送";
          if (!row.accepted) row.outcome = "unknown";
          row.message = row.accepted ? `已设置，调试开关读回失败：${error.message}` : `写入结果未知：${error.message}`;
        }
      }
      results.push(row);
    }
    const count = results.filter((r) => r.accepted).length;
    const body = {accepted: !stopReason && count === results.length, accepted_count: count, results, motion_enabled: gate.motion_enabled};
    if (stopReason) return reply(504, {...body, message: "请重新读回参数，核对本次写入"});
    return reply(body.accepted ? 200 : count ? 207 : 400, body);
  }

  const enums = {
    run_harvest_action: ["intent", {PICK_ALL: 0, PICK_SELECTED: 1, SURVEY_ONLY: 2}, "PICK_ALL"],
    execute_action: ["mode", {PREVIEW: 0, OBSERVE_ONLY: 1, FULL: 2, PREGRASP_ONLY: 3}, "PREVIEW"],
    control_service: ["command", {PAUSE: 0, RESUME: 1, ENTER_MAINTENANCE: 2, EXIT_MAINTENANCE: 3, CANCEL_NOW: 4, SKIP_TARGET: 5, ACKNOWLEDGE_RECOVERY: 6}, ""],
    manage_nodes_service: ["command", {STARTUP: 0, PAUSE: 1, RESUME: 2, RESET: 3, SHUTDOWN: 4}, ""],
  };
  const actions = {run_harvest_action: "RunHarvest", survey_action: "SurveyScene", execute_action: "ExecuteTarget", build_action: "BuildTargetModel"};
  const services = {control_service: "peach_interfaces/srv/ControlTask", begin_scene_service: "peach_interfaces/srv/BeginScene", check_reachability_service: "peach_interfaces/srv/CheckReachability", arm_service: "std_srvs/srv/SetBool", manage_nodes_service: "nav2_msgs/srv/ManageLifecycleNodes", set_enables_service: "peach_interfaces/srv/SetEnables"};
  const triggers = new Set(["photo_pose_service", "preview_approach_service", "preview_full_service", "ack_recovery_service", "skill_cancel_service", "recon_save_session_service", "recon_reset_service", "recon_finalize_service", "recon_query_service"]);
  const fields = {run_harvest_action: ["request_id", "scene_key"], survey_action: ["request_id", "scene_key"], execute_action: ["request_id", "run_id", "cycle_id", "target_id", "model_revision", "tool_profile_id"], build_action: ["request_id", "target_id"], control_service: ["request_id", "reason"], begin_scene_service: ["request_id", "scene_key"]};
  function goalFor(key, payload) {
    const goal = {};
    for (const name of fields[key] || []) goal[name] = String(payload[name] || "");
    if (enums[key]) {
      const [field, values, fallback] = enums[key];
      const value = String(payload[field] || fallback).toUpperCase();
      if (!(value in values)) throw fail(`未知 ${field}: ${value}`, "validation");
      goal[field] = values[value];
    }
    if (key === "run_harvest_action") Object.assign(goal, {selection_mode: payload.target_ids?.length ? 1 : 0, target_ids: (payload.target_ids || []).map(String)});
    if (key === "execute_action") goal.skip_observation = Boolean(payload.skip_observation);
    if (key === "build_action") goal.scene_epoch = Number(payload.scene_epoch || 0);
    if (key === "control_service") goal.expected_state_seq = Number(payload.expected_state_seq || 0);
    if (key === "arm_service") goal.data = enableFlag(payload, "data");
    if (key === "set_enables_service") {
      for (const name of ["execution", "grasp", "tool"]) goal[name] = enableFlag(payload, name);
      goal.reason = String(payload.reason || "单步调试");
      if (goal.grasp && !goal.execution || goal.tool && !goal.grasp) throw fail("使能依赖：tool 需要 grasp，grasp 需要 execution", "validation");
    }
    if (key === "check_reachability_service") {
      goal.timeout_s = Number(payload.timeout_s || 0);
      goal.require_sleeve = Boolean(payload.require_sleeve);
      goal.tcp_poses = (payload.tcp_poses || []).map((item) => ({header: {frame_id: item.frame_id || "base_link", stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0, y: 0, z: 0, ...item.position}, orientation: {x: 0, y: 0, z: 0, w: 1, ...item.orientation}}}));
      goal.suggested_travel_m = (payload.tcp_poses || []).map((item) => Number(item.suggested_travel_m || 0));
    }
    return goal;
  }
  function enableFlag(payload, name) {
    if (!(name in payload)) return false;
    if (typeof payload[name] !== "boolean") throw fail(`${name} 必须是布尔值`, "validation");
    return payload[name];
  }
  function isMotion(key, goal) {
    return ["survey_action", "photo_pose_service", "arm_service", "run_harvest_action"].includes(key)
      || key === "execute_action" && goal.mode !== 0
      || key === "control_service" && [1, 3].includes(goal.command)
      || key === "set_enables_service" && (goal.execution || goal.grasp || goal.tool);
  }
  async function debug(key, payload, options) {
    let goal;
    try { goal = goalFor(key, payload); }
    catch (error) { return reply(400, {accepted: false, message: error.message}); }
    const gate = await gates(options);
    if (!gate.debug_enabled) return reply(503, {accepted: false, message: "debug.enabled=false"});
    if (isMotion(key, goal) && !gate.motion_enabled) return reply(423, {accepted: false, message: "debug.motion_enabled=false"});
    if (options.signal?.aborted) throw fail("请求已取消", "abort");
    if (key === "cancel") {
      const active = goals.get(payload.target);
      if (!active) return reply(404, {accepted: false, message: "无本页在途目标"});
      try { active.cancel(); }
      catch (error) {
        if (error.code !== "SESSION_CHANGED") throw error;
        return reply(409, {accepted: false, message: "连接已更换，请从调度或技能取消入口核对并停止原请求"});
      }
      return reply(200, {accepted: true, result: {message: "已请求取消；终态以动作结果为准"}});
    }
    if (!config.endpoints[key] || !(actions[key] || services[key] || triggers.has(key))) return reply(404, {accepted: false, message: "未知调试端点"});
    if (!actions[key]) {
      const result = await client.callService(config.endpoints[key], services[key] || "std_srvs/srv/Trigger", goal, options);
      return reply(200, {accepted: true, result});
    }
    if (goals.has(key)) return reply(409, {accepted: false, message: "本页已有该动作目标；请查询状态或取消"});
    const handle = client.sendGoal(config.endpoints[key], `peach_interfaces/action/${actions[key]}`, goal, options);
    goals.set(key, handle);
    // If waiting fails, the remote outcome is unknown. Keep the handle and block resubmission.
    const {status, successful, value, error} = await handle.result;
    goals.delete(key);
    const result = {...(value && typeof value === "object" ? value : {message: String(value || "")}), action_status: status};
    if (!successful) result.success = false;
    if (key === "run_harvest_action" && result.summary) {
      for (const field of ["attempted", "succeeded", "failed"]) result[`summary_${field}`] = result.summary[field];
    }
    if (!result.message && status === 5) result.message = "动作已取消";
    if (!result.message && status === 6) result.message = "动作已中止";
    const terminal = [4, 5, 6].includes(status);
    return reply(terminal ? 200 : 409, {
      accepted: terminal, result,
      ...(!terminal ? {message: result.message || error || "动作目标未被接受"} : {}),
    });
  }

  async function request(path, options = {}) {
    const url = new URL(path, root.location.href);
    const key = url.pathname.split("/").pop();
    if (!["/api/state", "/api/force", "/api/trajectory", "/api/params"].includes(url.pathname) && !url.pathname.startsWith("/api/debug/")) return null;
    if (explicitMode === "http") return null;
    await waitForTransport(options.signal);
    if (mode() === "http") return null;
    if (stopped || options.signal?.aborted) throw fail("请求已取消", "abort");
    if (["state", "force", "trajectory"].includes(key)) return snapshot(key, options);
    if (url.pathname === "/api/params" && options.method !== "POST") {
      await waitFor(() => client.connected, options);
      return reply(200, await parameters(options));
    }
    if (!client.connected) throw fail("ROSbridge 未连接，命令未发送");
    const payload = options.body ? JSON.parse(options.body) : {};
    if (url.pathname === "/api/params") return setParameters(payload, options);
    return debug(decodeURIComponent(key), payload, options);
  }

  root.addEventListener("pagehide", (event) => {
    if (event.persisted) return;
    stopped = true;
    client?.dispose();  // Release local resources without canceling a remote goal.
    frames.clear();
    goals.clear();
  });
  root.PeachRosbridge = Object.freeze({request, get mode() { return mode(); }});
})(typeof window !== "undefined" ? window : globalThis);
