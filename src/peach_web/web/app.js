"use strict";

// 感知抓取过程页：/api/state（1s）+ /api/trajectory（0.4s）。
// 调试面转发既有动作/服务；动臂由 yaml debug.motion_enabled 放行。
// 全流程反馈：阶段时序（pipeline.fsm/arm 服务器侧，缓冲全量面板内滚动）
// + 节点日志（/api/state.logs，级别阈值 + 节点名包含）+ 批次账本
// + 感知节拍/重建进度 + 系统负载、参数、自检与诊断。

let labWorkbench = null;
let labVisualization = null;
let logsPaused = false;
const trajectoryFeed = {lastSuccessAt:0,error:''};

function labTrajectory() {
  const age = trajectoryFeed.lastSuccessAt ? (Date.now() - trajectoryFeed.lastSuccessAt) / 1000 : Infinity;
  const fresh = !trajectoryFeed.error && age <= 5;
  const frames = Object.fromEntries(Object.entries(trajectoryPayload?.tool_frames || {}).map(([name,tf]) => {
    const frameAge = numeric(tf.age_s) ? Number(tf.age_s) + age : null;
    const valid = fresh && tf.available === true && (frameAge === null || frameAge <= 1);
    return [name,{...tf,age_s:frameAge,available:valid,error:valid ? '' : tf.error || (fresh ? 'stale' : 'trajectory_unavailable')}];
  }));
  return {...(trajectoryPayload || {}),tool_frames:frames};
}

function labState() {
  const context = labWorkbench?.context?.() || {};
  return {...(monitorState || {}), ui: {fresh: stateIsFresh(), pending: [...pendingDebug],
    selected_target_id: context.target_id, selected_scene_epoch: context.scene_epoch}};
}

function updateLab() {
  const state = labState();
  labWorkbench?.update(state);
  labVisualization?.update(state, labTrajectory());
}

function stateIsFresh() {
  return stateFeed.lastSuccessAt > 0 && !stateFeed.error &&
    Date.now() - stateFeed.lastSuccessAt < 5000;
}

function updateConnectionView() {
  const fresh = stateIsFresh();
  const seen = stateFeed.lastSuccessAt > 0;
  const label = fresh ? "已连接" : stateFeed.error ? "连接中断" : seen ? "数据陈旧" : "正在连接";
  const connection = $("connection");
  connection.className = `connection ${fresh ? "online" : seen || stateFeed.error ? "offline" : ""}`;
  connection.querySelector("span").textContent = label;
  const age = seen ? Math.max(0, Math.floor((Date.now() - stateFeed.lastSuccessAt) / 1000)) : 0;
  setText("state-updated", seen
    ? `更新于 ${new Date(stateFeed.lastSuccessAt).toLocaleTimeString("zh-CN", {hour12: false})} · ${age}s 前`
    : "尚未收到状态快照");
  const alert = $("feed-alert");
  if (alert) {
    alert.hidden = fresh;
    alert.textContent = !seen
      ? "等待状态连接，操作将在数据就绪后开放。"
      : `当前显示最后一次快照（${age}s 前）。${stateFeed.error || "状态更新延迟"}；普通操作已暂停，可尝试取消在途动作。`;
  }
  document.body.classList.toggle("data-stale", !fresh);
  updateDebugControls();
}

async function pollTrajectory() {
  if (trajectoryInFlight || document.hidden) return;
  trajectoryInFlight = true;
  try {
    const {response, body} = await requestJson(`/api/trajectory?t=${Date.now()}`);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    trajectoryPayload = body;
    trajectoryFeed.lastSuccessAt = Date.now(); trajectoryFeed.error = '';
    renderTrajHud(body);
    renderTcpMini(body);
    labVisualization?.update(labState(), labTrajectory());
  } catch (_) {
    trajectoryFeed.error = 'trajectory_unavailable';
    labVisualization?.update(labState(), labTrajectory());
    setText("traj-status", "轨迹更新不可用 · 保留上次画面");
  } finally {
    trajectoryInFlight = false;
  }
}

async function pollState() {
  if (stateFeed.inFlight || document.hidden) return;
  stateFeed.inFlight = true;
  try {
    const {response, body: state} = await requestJson(`/api/state?t=${Date.now()}`);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    if (!state || typeof state !== "object" || Array.isArray(state)) {
      throw new Error("状态快照格式无效");
    }
    monitorState = state;
    renderMonitorSnapshot(state);
    stateFeed.lastSuccessAt = Date.now();
    stateFeed.error = "";
    renderDebugPanel(state.debug || {});
    updateLab();
  } catch (error) {
    stateFeed.error = error.message || "状态请求失败";
  } finally {
    stateFeed.inFlight = false;
    updateConnectionView();
  }
}

const debugView = {results: []};

function switchView(name) {
  document.querySelectorAll(".view-tabs button").forEach((el) => {
    const active = el.dataset.view === name;
    el.classList.toggle("active", active);
    el.setAttribute("aria-selected", String(active));
    el.tabIndex = active ? 0 : -1;
  });
  const monitorPanel = $("view-monitor");
  const debugPanel = $("view-debug");
  if (monitorPanel) monitorPanel.hidden = name !== "monitor";
  if (debugPanel) debugPanel.hidden = name !== "debug";
  document.body.dataset.view = name;
  const vizHost = $(name === "debug" && $("debug-visualization-host")
    ? "debug-visualization-host" : "monitor-visualization-host");
  if (vizHost) labVisualization?.moveTo(vizHost);
  if (name === "monitor" && trajectoryPayload) renderTcpMini(trajectoryPayload);
}

function debugRequestKey(action, payload) {
  if (action === "cancel") return `cancel:${payload.target || ""}`;
  if (action === "control_service" && payload.command === "CANCEL_NOW") return "control_service:CANCEL_NOW";
  return action;
}

function isCancelControl(el) {
  return isCancelRequest(el.dataset.debugCancel ? "cancel" : el.dataset.debug,
    JSON.parse(el.dataset.payload || "{}"));
}

function isCancelRequest(action, payload) {
  return action === "cancel" || (action === "control_service" && payload.command === "CANCEL_NOW");
}

// Mirror DebugBridge.is_motion; this is UI feedback, the backend remains authoritative.
function isMotionRequest(action, payload) {
  if (["survey_action", "photo_pose_service", "arm_service", "run_harvest_action"].includes(action)) return true;
  if (action === "execute_action") return String(payload.mode || "PREVIEW").toUpperCase() !== "PREVIEW";
  return action === "control_service" && ["RESUME", "EXIT_MAINTENANCE"].includes(String(payload.command || "").toUpperCase());
}

function debugBlockReason(action, payload, state = monitorState) {
  if (isMotionRequest(action, payload) && state?.debug?.motion_enabled !== true) return "后端动臂入口未放行";
  if (action === "photo_pose_service") {
    const arm = state?.manipulation?.status || {};
    if (arm.contact_recovery_required || state?.task_executor?.state?.recovery_required) return "待人工恢复确认；不会自动确认恢复";
    if (arm.running === true) return "臂周期正在运行，请等待结束或取消执行";
  }
  return "";
}

function updateDebugControls() {
  const enabled = monitorState?.debug?.enabled === true;
  document.querySelectorAll("[data-debug], [data-debug-cancel]").forEach((el) => {
    const action = el.dataset.debugCancel ? "cancel" : el.dataset.debug;
    const payload = el.dataset.debugCancel ? {target: el.dataset.debugCancel} :
      JSON.parse(el.dataset.payload || "{}");
    const busy = pendingDebug.has(debugRequestKey(action, payload));
    const block = debugBlockReason(action, action === "execute_action"
      ? {...payload, mode: $("exec-mode")?.value || "PREVIEW"} : payload);
    if (!el.dataset.label) el.dataset.label = el.textContent;
    el.disabled = !enabled || (!stateIsFresh() && !isCancelControl(el)) || busy || Boolean(block);
    el.setAttribute("aria-busy", String(busy));
    el.textContent = busy ? `${el.dataset.label} · 等待中` : el.dataset.label;
    el.title = busy ? "请求仍在等待结果，取消入口保持独立。" :
      !enabled ? "后端调试入口尚未就绪或已关闭。" :
        !stateIsFresh() && !isCancelControl(el) ? "状态快照已失联，请等待恢复。" : block;
  });
}

function debugNotice(message, tone = "") {
  const notice = $("debug-notice");
  if (!notice) return;
  notice.textContent = message;
  notice.className = `debug-notice ${tone}`;
}

async function debugPost(action, payload, confirmText, contextGuard) {
  const key = debugRequestKey(action, payload || {});
  if (pendingDebug.has(key)) {
    debugNotice("前一次请求仍在等待结果，请勿重复提交。", "warn");
    return null;
  }
  if (confirmText && !window.confirm(confirmText)) return null;
  const contextReason = typeof contextGuard === 'function' ? contextGuard() : '';
  if (contextReason) {
    debugNotice(`请求未发送：${contextReason}。`, 'warn');
    return null;
  }
  // A native confirmation can remain open while the last snapshot expires.
  if (monitorState?.debug?.enabled !== true || (!stateIsFresh() && !isCancelRequest(action, payload || {}))) {
    updateConnectionView();
    debugNotice("请求未发送：调试入口关闭或状态快照已陈旧，请等状态恢复后再操作。", "warn");
    return null;
  }
  const block = debugBlockReason(action, payload || {});
  if (block) {
    updateDebugControls();
    debugNotice(`请求未发送：${block}。`, "warn");
    return null;
  }
  pendingDebug.add(key);
  updateLab();
  updateDebugControls();
  debugNotice("请求已发送，等待后端结果；取消入口仍可使用。");
  let outcome;
  try {
    // Covers the usual 180s backend budget; longer ROS work can still continue.
    const {response, body} = await requestJson(`/api/debug/${encodeURIComponent(action)}`, {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload || {}),
    }, 195000);
    if (!body || typeof body !== "object" || Array.isArray(body)) throw new Error("后端结果格式无效");
    const {running, unknown, failed, rejected, tone, label} = classifyDebugResponse(response, body);
    const detailMessage = body.message || body.result?.message;
    const message = unknown
      ? `${detailMessage || "后端未返回操作终态"}。请核对状态或使用取消入口，勿直接重复发送。`
      : detailMessage || (failed ? "请求已受理，但服务返回失败。" :
        response.status === 423 ? "动臂未放行，后端授权门拒绝本次操作。" : label);
    debugView.results.unshift({ts: new Date().toLocaleTimeString("zh-CN", {hour12: false}),
      action, status: response.status, label, tone, message, detail: body});
    debugNotice(`${label} · ${message}`, tone);
    outcome = {ok: response.ok && body.result?.success === true && !failed && !rejected && !running && !unknown, status: response.status, body};
  } catch (error) {
    const message = `${error.message || "网络请求失败"}。ROS 操作结果未知，请核对状态或使用取消入口，勿直接重复发送。`;
    debugView.results.unshift({ts: new Date().toLocaleTimeString("zh-CN", {hour12: false}),
      action, status: null, label: "结果未知", tone: "warn", message, detail: {error: error.message}});
    debugNotice(message, "warn");
    outcome = {ok: false, status: null, body: {message}};
  } finally {
    if (debugView.results.length > 30) debugView.results.pop();
    pendingDebug.delete(key);
    renderDebugResults();
    updateDebugControls();
    updateLab();
  }
  return outcome;
}

function renderDebugResults() {
  const list = $("debug-results");
  if (!debugView.results.length) {
    list.innerHTML = '<p class="empty">尚无操作</p>';
    return;
  }
  list.innerHTML = debugView.results.map((entry, index) => {
    const detail = safe(JSON.stringify(entry.detail, null, 2));
    return `<details class="debug-result ${entry.tone}" ${index ? "" : "open"}>
      <summary><b>${safe(entry.ts)}</b> <span>${safe(entry.action)}</span>
      <span class="debug-status">${entry.status ?? "—"} ${safe(entry.label)}</span></summary>
      <p>${safe(entry.message)}</p><pre>${detail}</pre></details>`;
  }).join("");
  setText("debug-recent-count", `${debugView.results.length} 条`);
}

function renderDebugPanel(debug) {
  const gates = $("debug-gates");
  if (!gates) return;
  const enabled = debug.enabled === true;
  const motion = debug.motion_enabled === true;
  gates.innerHTML = `
    <span class="debug-gate ${enabled ? "ok" : "err"}">调试 ${enabled ? "已开启" : "已关闭"}</span>
    <span class="debug-gate ${motion ? "warn" : "ok"}">动臂 ${motion ? "已放行" : "未放行"}</span>`;
  updateDebugControls();
}

let debugControlsBound = false;
function bindDebugControls() {
  if (debugControlsBound) return;
  debugControlsBound = true;
  const tabs = [...document.querySelectorAll(".view-tabs button")];
  tabs.forEach((el, index) => {
    el.addEventListener("click", () => switchView(el.dataset.view));
    el.addEventListener("keydown", (event) => {
      const keys = {ArrowRight: (index + 1) % tabs.length,
        ArrowLeft: (index + tabs.length - 1) % tabs.length, Home: 0, End: tabs.length - 1};
      if (!(event.key in keys)) return;
      event.preventDefault();
      const next = tabs[keys[event.key]];
      switchView(next.dataset.view);
      next.focus();
    });
  });
  document.querySelectorAll("[data-debug]").forEach((el) => {
    el.addEventListener("click", async () => {
      try {
        const action = el.dataset.debug;
        const payload = buildDebugPayload(action, el);
        if (payload === null) return;
        await debugPost(action, payload, el.dataset.confirm === "1"
          ? el.dataset.confirmText || `确认发送 ${action}？\n${JSON.stringify(payload, null, 2)}` : null);
      } catch (error) {
        debugNotice(`请求未发送：${error.message}`, "err");
      }
    });
  });
  document.querySelectorAll("[data-debug-cancel]").forEach((el) => {
    el.addEventListener("click", () => debugPost("cancel", {target: el.dataset.debugCancel}, null));
  });
  switchView("monitor");
  $("exec-mode")?.addEventListener("change", updateDebugControls);
  updateDebugControls();
}

function debugField(id) {
  return $(id)?.value?.trim?.() || "";
}

function buildDebugPayload(action, el) {
  // 常用操作条只发送自带 payload 的命令；旧调试栅格的输入框可缺省。
  if (action === "control_service") {
    const command = el.dataset.payload ? JSON.parse(el.dataset.payload).command : "CANCEL_NOW";
    // 降险类命令（暂停/立即取消）不带序号防过期拦截；恢复/跳过/ACK 带
    // 最新镜像 state_seq，避免点错对象（过期会被调度拒并提示 mismatch）
    const seqUnchecked = command === "PAUSE" || command === "CANCEL_NOW";
    const stateSeq = Number(monitorState?.task_executor?.state?.state_seq) || 0;
    return {command, expected_state_seq: seqUnchecked ? 0 : stateSeq,
      reason: "web 调试"};
  }
  if (action === "arm_service") {
    const data = el.dataset.payload ? JSON.parse(el.dataset.payload).data : true;
    return {data: data === true};
  }
  if (action === "photo_pose_service" || action === "cancel") {
    return {};
  }
  const requestId = debugField("dbg-request-id");
  const sceneKey = debugField("dbg-scene-key") || "lab";
  const targetId = debugField("dbg-target-id");
  if (action === "run_harvest_action") {
    if (!requestId) { window.alert("request_id 必填（同时是账本目录名，不得复用）"); return null; }
    const intent = debugField("run-intent") || "PICK_ALL";
    const targets = debugField("run-target-ids").split(",").map((s) => s.trim()).filter(Boolean);
    return {request_id: requestId, scene_key: sceneKey, intent,
      selection_mode: targets.length ? "MANUAL" : "AUTO", target_ids: targets};
  }
  if (action === "begin_scene_service") {
    return {request_id: requestId || "dev", scene_key: sceneKey};
  }
  if (action === "survey_action") {
    return {request_id: requestId || "dev", scene_key: sceneKey};
  }
  if (action === "build_action") {
    if (!targetId) { window.alert("target_id 必填"); return null; }
    return {request_id: requestId || "dev", target_id: targetId,
      scene_epoch: Number(debugField("dbg-epoch")) || 0};
  }
  if (action === "execute_action") {
    if (!targetId) { window.alert("target_id 必填"); return null; }
    return {request_id: requestId || "dev", target_id: targetId,
      mode: debugField("exec-mode") || "PREVIEW",
      skip_observation: $("exec-skip-obs")?.checked === true};
  }
  const inline = el.dataset.payload ? JSON.parse(el.dataset.payload) : null;
  return inline && typeof inline === "object" ? inline : {};
}

bindDebugControls();
if (window.PeachWorkbench && $('workbench-host')) {
  labWorkbench = window.PeachWorkbench.create($('workbench-host'), {
    getState: labState, submit: debugPost, log: debugNotice,
    onContextChange: () => labVisualization?.update(labState(), labTrajectory()),
  });
}
if (window.PeachVisualization && $('monitor-visualization-host')) {
  labVisualization = window.PeachVisualization.create($('monitor-visualization-host'));
}
let logFiltersBound = false;
function bindLogFilters() {
  if (logFiltersBound) return;
  logFiltersBound = true;
  const refresh = () => renderLogs((monitorState && monitorState.logs) || []);
  const level = $("log-level");
  const node = $("log-node");
  if (level) level.addEventListener("change", refresh);
  if (node) node.addEventListener("input", refresh);
  $("log-keyword")?.addEventListener('input', refresh);
  $("log-pause")?.addEventListener('click', (event) => {
    logsPaused = !logsPaused;
    event.target.textContent = logsPaused ? '恢复日志' : '暂停日志';
    if (!logsPaused) refresh();
  });
}
bindLogFilters();
updateConnectionView();
let pollTimers = [];
function startPolling() {
  if (pollTimers.length) return;
  pollTimers = [setInterval(pollState, 1000), setInterval(pollTrajectory, 400),
    setInterval(updateConnectionView, 1000)];
}
function updateStickyHeaderHeight() {
  const header = $('page-header');
  if (header && typeof header.getBoundingClientRect === 'function') {
    document.documentElement?.style?.setProperty('--header-height', `${header.getBoundingClientRect().height}px`);
  }
}
updateStickyHeaderHeight();
startPolling();
window.addEventListener("resize", () => {
  updateStickyHeaderHeight();
  if (trajectoryPayload) renderTcpMini(trajectoryPayload);
});
document.addEventListener("visibilitychange", () => {
  labVisualization?.setActive(!document.hidden);
  if (!document.hidden) { pollState(); pollTrajectory(); }
});
window.addEventListener("pagehide", (event) => {
  pollTimers.forEach(clearInterval);
  pollTimers = [];
  fetchControllers.forEach((controller) => controller.abort());
  labVisualization?.setActive(false);
  if (!event?.persisted) { labVisualization?.dispose(); labWorkbench?.dispose(); }
});
window.addEventListener("pageshow", (event) => {
  if (!event.persisted) return;
  labVisualization?.setActive(true);
  startPolling();
  updateConnectionView();
  pollState();
  pollTrajectory();
});
pollState();
pollTrajectory();
