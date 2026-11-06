"use strict";

// Step-debug UI. Session state machine lives in debug-session.js; this file
// owns snapshot rendering, the parameter panel and one-shot bootstrapping.
(function () {
  const {$, safe} = Peach.ui;
  const api = PeachApi.api;
  let state = {};
  let paramData = null;
  let stateFeed = null;
  let session = null;
  let roundLoaded = false;
  let roundLoading = false;
  let settingsBusy = false;
  let paramsBusy = false;
  let paramsLoading = false;
  let evidence = null;
  let selectionEpoch = null;
  let targetKey = "";
  const picked = new Set();

  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

  function findParam(node, name) {
    const group = ((paramData && paramData.groups) || []).find((item) => item.node === node);
    return ((group && group.params) || []).find((item) => item.name === name);
  }

  function logRound(text) {
    const item = document.createElement("li");
    item.textContent = text;
    $("round-log").prepend(item);
  }

  function notify(message, tone = "info") {
    const status = $("session-status");
    if (status) {
      status.textContent = message;
      status.dataset.tone = tone;
    }
    PeachShell.notify(message, {tone: tone === "ok" ? "ok" : tone === "err" ? "err" : "info",
      timeout: tone === "err" ? 9000 : 5000});
    updateControls();
  }

  function setNote(id, text) {
    const node = $(id);
    if (node) node.textContent = text;
  }

  function taskState() {
    return state.task_executor?.state || {};
  }

  function reconstructionMode() {
    return PeachDebugEvidence.reconstructionMode(paramData, state.startup?.facts);
  }

  function recoveryRequired() {
    return taskState().recovery_required === true
      || state.manipulation?.status?.contact_recovery_required === true;
  }

  function canAcknowledge() {
    const task = taskState();
    return stateFeed?.isFresh() === true && state.debug?.enabled === true
      && [6, 8].includes(task.batch_state) && task.recovery_required === true
      && task.action_active === false && state.manipulation?.status?.running === false
      && Number.isInteger(task.state_seq) && task.state_seq > 0;
  }

  function ready() {
    return stateFeed?.isFresh() === true
      && state.debug?.enabled === true
      && !recoveryRequired()
      && !session?.snapshot().unknownRequest;
  }

  function renderGates() {
    const snap = session?.snapshot() || {};
    const task = taskState();
    const targets = state.perception?.targets || {};
    const chips = [
      ["调试", state.debug?.enabled === true ? "已开启" : "已关闭",
        state.debug?.enabled === true ? "ok" : "err"],
      ["动臂", state.debug?.motion_enabled === true ? "已放行" : "未放行",
        state.debug?.motion_enabled === true ? "warn" : "ok"],
      ["场景", targets.scene_epoch ? `epoch ${targets.scene_epoch}` : "未开",
        targets.scene_epoch ? "ok" : "idle"],
      ["目标", targets.target_set_locked === true
        ? `${(targets.observations || []).length} 颗已锁` : "收集中", targets.target_set_locked ? "ok" : "idle"],
      ["恢复", recoveryRequired() ? "待确认" : "无", recoveryRequired() ? "err" : "ok"],
      ["单轮", snap.singleArmed ? "保持中" : "未启动", snap.singleArmed ? "warn" : "idle"],
      ["多轮", snap.roundsBusy ? `运行 ${snap.queued.length ? `· 队列 ${snap.queued.length}` : ""}` : "空闲",
        snap.roundsBusy ? "warn" : "idle"],
    ];
    if (snap.unknownRequest) chips.push(["结果未知", snap.unknownRequest, "err"]);
    $("debug-chips").innerHTML = chips.map(([label, value, tone]) =>
      `<span class="debug-chip ${tone}">${safe(label)} <b>${safe(value)}</b></span>`).join("");
    $("ctx-scene").textContent = targets.scene_epoch ? String(targets.scene_epoch) : "—";
    $("ctx-targets").textContent = targets.target_set_locked
      ? `${(targets.observations || []).length} 颗` : "收集中";
    const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式",
      "已结算", "需要恢复", "已中断"];
    const phase = task.target_phase ?? 0;
    $("ctx-batch").textContent = task.run_id
      ? `${batchNames[task.batch_state] || task.batch_state} · ${task.target_id || "—"} · 阶段 ${phase}`
      : "空闲";
  }

  function paintSingle(snap) {
    $("single-panel").classList.toggle("live", snap.singleArmed);
    const queued = snap.queued.map((item) => item.ids.join("+"));
    if (!snap.singleArmed) {
      setNote("single-note", "先启动。启动后保持，点哪一颗就抓哪一颗，抓完继续等下一次点选。");
    } else if (snap.roundsBusy) {
      setNote("single-note", `正在抓。${queued.length ? `接下来 ${queued.join("，")}` : "抓完继续等下一次点选。"}`);
    } else {
      setNote("single-note", queued.length
        ? `接下来 ${queued.join("，")}` : "单轮保持。点顺序号，或勾选后抓已勾选。");
    }
    $("grasp-selected").disabled = !snap.singleArmed || !picked.size;
  }

  function updateControls() {
    const snap = session?.snapshot() || {singleArmed: false, roundsBusy: false, queued: [], unknownRequest: ""};
    const isReady = ready();
    const busy = snap.roundsBusy || settingsBusy || paramsBusy || paramsLoading || roundLoading;
    const armed = snap.singleArmed;
    const active = taskState().action_active === true || state.manipulation?.status?.running === true;
    const serverCancelable = stateFeed?.isFresh() === true && state.debug?.enabled === true
      && [1, 2, 3, 4, 5, 7].includes(taskState().batch_state);
    const recovery = recoveryRequired();
    const mode = reconstructionMode();
    const reason = !stateFeed?.isFresh() ? "等待状态连接"
      : state.debug?.enabled !== true ? "后端调试入口未开启"
        : recovery ? "等待人工恢复确认"
          : snap.unknownRequest ? "有一次请求结果未知，先取消/核对"
            : "";
    [
      ["round-apply", busy || armed || active || !isReady],
      ["start-single", busy || armed || active || !isReady || !mode.consistent],
      ["start-rounds", busy || armed || active || !isReady || !mode.consistent],
      ["stop-single", !snap.roundsBusy && !armed && !snap.unknownRequest && !serverCancelable],
      ["stop-rounds", !snap.roundsBusy && !snap.unknownRequest && !serverCancelable],
      ["ack-recovery", !canAcknowledge()],
      ["params-apply", busy || armed || active || !stateFeed?.isFresh() || state.debug?.enabled !== true],
      ["round-reload", busy || armed],
      ["params-reload", busy || armed],
      ["test-load", busy || armed || !roundLoaded],
    ].forEach(([id, disabled]) => {
      const node = $(id);
      if (!node) return;
      node.disabled = disabled;
      node.title = disabled && reason && !["stop-single", "stop-rounds", "ack-recovery"].includes(id)
        ? reason : "";
    });
    $("ack-recovery").title = recovery && ![6, 8].includes(taskState().batch_state)
      ? "先停止单轮或停止后续轮，确认旧批次已结束，再确认恢复" : "";
    document.querySelectorAll(".param-row input, .param-row select").forEach((node) => {
      node.disabled = busy || armed || active || !roundLoaded || !stateFeed?.isFresh() || state.debug?.enabled !== true
        || node.closest(".param-row").dataset.available !== "true";
    });
    for (const id of ["motion", "full", "ex", "gr", "tl", "reconstruction", "rounds", "round-mode"]) {
      $(id).disabled = busy || armed || active || !roundLoaded
        || (id === "reconstruction" && !mode.available);
    }
    setNote("reconstruction-note", `${mode.text}。${mode.available
      ? "本轮使用重建是待写选择；动作空闲时写入后生效，测试档保留该选择。"
      : "需要开启时，以 reconstruction_enabled:=true 重新部署；页面不能启动缺失节点。"}`);
    document.querySelectorAll("#target-picks button[data-target]").forEach((node) => {
      node.disabled = !armed || !isReady;
      node.title = !armed ? "先启动单轮点选" : !isReady ? reason : "抓取这一颗";
    });
    paintSingle(snap);
  }

  function observations() {
    return (state.perception?.targets?.observations || []).filter((item) => item && item.target_id);
  }

  function byPriority(list) {
    return [...list].sort((a, b) => (Number(a.priority) || 999) - (Number(b.priority) || 999));
  }

  function renderTargets() {
    const epoch = state.perception?.targets?.scene_epoch ?? null;
    if (epoch !== selectionEpoch) {
      selectionEpoch = epoch;
      picked.clear();
      targetKey = "";
      session?.setEpoch(epoch);
    }
    const items = byPriority(observations());
    const key = items.map((item) => `${item.target_id}:${item.priority}:${item.harvest_status}:${item.tracking_status}`).join("|");
    if (key === targetKey && $("target-picks").querySelector("button")) {
      updateControls();
      return;
    }
    targetKey = key;
    const host = $("target-picks");
    if (!items.length) {
      host.innerHTML = '<p class="empty">等待本场目标。开一轮后会重新锁定。</p>';
      updateControls();
      return;
    }
    host.innerHTML = items.map((item) => {
      const id = item.target_id;
      const order = Number(item.priority) || 0;
      const checked = picked.has(id) ? " checked" : "";
      const label = order ? `顺序 ${order}` : id;
      return `<span class="target-pick"><label><input type="checkbox" data-keep="${safe(id)}"${checked}> 点选</label>`
        + `<button type="button" data-target="${safe(id)}" aria-label="抓取 ${safe(label)} ${safe(id)}">${safe(label)}`
        + `<small>${safe(id)}</small></button></span>`;
    }).join("");
    host.querySelectorAll("input[data-keep]").forEach((box) => {
      box.addEventListener("change", () => {
        if (box.checked) picked.add(box.dataset.keep);
        else picked.delete(box.dataset.keep);
        updateControls();
      });
    });
    host.querySelectorAll("button[data-target]").forEach((button) => {
      button.addEventListener("click", () => session.enqueue([button.dataset.target]));
    });
    updateControls();
  }

  function renderState() {
    renderGates();
    renderTargets();
    if (!roundLoaded) {
      loadRound().catch((error) => setNote("round-note", `读取失败：${error.message}`));
    }
    const sessionStatus = $("session-status");
    if (sessionStatus && sessionStatus.textContent === "等待状态快照" && stateFeed?.isFresh()) {
      sessionStatus.textContent = "状态就绪 · 先写入本轮设置";
    }
    updateControls();
    evidence?.update(state, stateFeed?.status(), paramData);
  }

  async function loadParams() {
    if (paramsLoading) return;
    paramsLoading = true;
    updateControls();
    try {
      const {body} = await api.params();
      paramData = body;
      renderParams({preserveDrafts: true});
      setNote("params-note", body.debug_enabled ? "" : "调试入口已关闭：参数只读");
    } catch (error) {
      if (!paramData) $("param-groups").innerHTML = `<p class="empty">参数读取失败：${safe(error.message)}</p>`;
      setNote("params-note", `参数读取失败，当前值尚未刷新：${error.message}`);
    } finally {
      paramsLoading = false;
      updateControls();
    }
  }

  function renderParams({preserveDrafts = false} = {}) {
    const drafts = new Map();
    if (preserveDrafts) document.querySelectorAll(".param-row.changed").forEach((row) => {
      drafts.set(`${row.dataset.node}:${row.dataset.name}`, row.querySelector("input, select")?.value);
    });
    const groups = (paramData && paramData.groups) || [];
    if (!groups.length) {
      $("param-groups").innerHTML = '<p class="empty">参数面板未启用或无白名单参数。</p>';
      return;
    }
    $("param-groups").innerHTML = groups.map((group, index) => {
      const rows = (group.params || []).map((param) => {
        const id = `p-${index}-${String(param.name).replace(/[^a-zA-Z0-9_-]/g, "-")}`;
        const badge = (param.motion ? '<span class="badge motion">运动</span>' : "")
          + (param.static ? '<span class="badge static">重启</span>' : "");
        let input;
        if (param.type === "bool") {
          input = `<select id="${id}"><option value="true"${param.value === true ? " selected" : ""}>true</option>`
            + `<option value="false"${param.value === false ? " selected" : ""}>false</option></select>`;
        } else if (param.type === "string" && param.choices) {
          input = `<select id="${id}">${param.choices.map((choice) =>
            `<option value="${safe(choice)}"${param.value === choice ? " selected" : ""}>${safe(choice)}</option>`).join("")}</select>`;
        } else {
          const min = param.min ?? ""; const max = param.max ?? "";
          const step = param.type === "int" ? "1" : "any";
          input = `<input id="${id}" type="${param.type === "string" ? "text" : "number"}" step="${step}" min="${min}" max="${max}" value="${safe(param.value ?? "")}">`;
        }
        const note = !param.node_running ? '<div class="note">节点未运行</div>'
          : (!param.present ? `<div class="note">${safe(group.error || "参数未声明")}</div>` : "");
        return `<div class="param-row${param.node_running ? "" : " unavailable"}" data-node="${safe(group.node)}"`
          + ` data-name="${safe(param.name)}" data-type="${safe(param.type)}" data-orig="${safe(param.value)}"`
          + ` data-available="${param.node_running === true && param.present === true}"`
          + ` data-min="${param.min ?? ""}" data-max="${param.max ?? ""}">`
          + `<div><div class="name">${safe(param.name)}</div><div class="label">${safe(param.label)}${badge}</div></div>`
          + `<div><label class="param-input-label" for="${id}">待写值</label>${input}${note}</div>`
          + `<div class="muted">当前 <code>${safe(param.value)}</code><small>${safe(param.type)}`
          + `${param.min != null || param.max != null ? ` · ${safe(param.min ?? "不限")} ～ ${safe(param.max ?? "不限")}` : ""}</small></div></div>`;
      }).join("");
      return `<section class="param-group"><div class="param-head"><b>${safe(group.label)}</b>`
        + `<span class="muted">${safe(group.node)}</span><span class="muted">${group.running ? "运行中" : "未运行"}</span></div>`
        + `${rows}</section>`;
    }).join("");
    document.querySelectorAll(".param-row input, .param-row select").forEach((el) => {
      el.setAttribute("aria-label", `${el.closest(".param-row").dataset.name} 待写值`);
      const changed = () => {
        const row = el.closest(".param-row");
        const same = row.dataset.type === "double" || row.dataset.type === "int"
          ? el.value.trim() !== "" && Number(el.value) === Number(row.dataset.orig)
          : el.value === row.dataset.orig;
        row.classList.toggle("changed", !same);
        el.removeAttribute("aria-invalid");
        filterParams();
      };
      el.addEventListener("input", changed);
      el.addEventListener("change", changed);
      const row = el.closest(".param-row");
      const key = `${row.dataset.node}:${row.dataset.name}`;
      if (drafts.has(key)) { el.value = drafts.get(key); changed(); }
    });
    filterParams();
    updateControls();
  }

  function filterParams() {
    const query = $("params-search").value.trim().toLowerCase();
    const onlyChanged = $("params-changed").checked;
    let visible = 0;
    let changed = 0;
    document.querySelectorAll(".param-row").forEach((row) => {
      const dirty = row.classList.contains("changed");
      if (dirty) changed += 1;
      row.hidden = (onlyChanged && !dirty)
        || !`${row.dataset.node} ${row.textContent}`.toLowerCase().includes(query);
      if (!row.hidden) visible += 1;
    });
    document.querySelectorAll(".param-group").forEach((group) => {
      group.hidden = !group.querySelector(".param-row:not([hidden])");
    });
    setNote("params-count", `${visible} 项显示 · ${changed} 项待写入`);
  }

  function collectChanges() {
    const changes = [];
    const invalid = [];
    document.querySelectorAll(".param-row").forEach((row) => {
      const el = row.querySelector("input, select");
      if (!el || row.dataset.available !== "true") return;
      const type = row.dataset.type;
      let value;
      if (type === "bool") value = el.value === "true";
      else if (type === "string") value = el.value;
      else {
        if (String(el.value).trim() === "") { invalid.push(`${row.dataset.name} 为空`); return; }
        value = Number(el.value);
        if (!Number.isFinite(value)) { invalid.push(`${row.dataset.name} 不是数字`); return; }
        if (type === "int" && !Number.isSafeInteger(value)) {
          el.setAttribute("aria-invalid", "true");
          invalid.push(`${row.dataset.name} 必须是安全范围内的整数`); return;
        }
        const min = row.dataset.min === "" ? null : Number(row.dataset.min);
        const max = row.dataset.max === "" ? null : Number(row.dataset.max);
        if (min !== null && value < min) invalid.push(`${row.dataset.name} 小于 ${min}`);
        if (max !== null && value > max) invalid.push(`${row.dataset.name} 大于 ${max}`);
      }
      const original = row.dataset.orig;
      const same = type === "bool" ? String(value) === original
        : type === "string" ? String(value) === original
          : Number(value) === Number(original);
      if (!same) changes.push({node: row.dataset.node, name: row.dataset.name, value});
    });
    if (invalid.length) throw new Error(invalid.join("；"));
    return changes;
  }

  async function writeParams(changes) {
    let reply;
    try {
      reply = await api.setParams(changes);
    } catch (error) {
      const rows = error.body?.results;
      if (Array.isArray(rows)) {
        const details = rows.map((row) => `${row.node}/${row.name} ${
          row.accepted === true ? "已确认写入" : row.outcome === "not_sent"
            ? "未发送" : row.outcome === "unknown" ? "结果未知" : row.message || "未写入"}`);
        throw new Error([error.message, ...details].join("；"));
      }
      throw error;
    }
    const {response, body} = reply;
    const results = body?.results || [];
    if (!response.ok || body?.accepted !== true || results.length !== changes.length
      || results.some((row) => row.accepted !== true)) {
      const details = results.filter((row) => row.accepted !== true)
        .map((row) => `${row.node}/${row.name} ${row.message || "未写入"}`);
      throw new Error([body?.message, ...details].filter(Boolean).join("；") || `参数写入未确认（${response.status}）`);
    }
    const {body: fresh} = await api.params();
    paramData = fresh;
    for (const change of changes) {
      const actual = findParam(change.node, change.name);
      if (!actual?.present) throw new Error(`参数读回缺失：${change.name}`);
      const same = typeof change.value === "boolean"
        ? actual.value === change.value
        : (typeof change.value === "number" ? Number(actual.value) === change.value
          : String(actual.value) === String(change.value));
      if (!same) throw new Error(`参数读回不一致：${change.name}`);
    }
    return body;
  }

  function tieEnables(changed) {
    if ((changed === "ex" || changed === "gr" || changed === "tl") && $(changed).checked) {
      $("motion").checked = true;
    }
    if (changed === "tl" && $("tl").checked) {
      $("gr").checked = true;
      $("ex").checked = true;
    }
    if (changed === "gr" && $("gr").checked) $("ex").checked = true;
    if (changed === "gr" && !$("gr").checked) $("tl").checked = false;
    if (changed === "ex" && !$("ex").checked) {
      $("gr").checked = false;
      $("tl").checked = false;
    }
    if (changed === "motion" && !$("motion").checked) {
      $("ex").checked = false;
      $("gr").checked = false;
      $("tl").checked = false;
    }
  }

  async function loadRound() {
    if (roundLoading) return;
    roundLoading = true;
    updateControls();
    try {
      const {body} = await api.params();
      paramData = body;
      const motion = findParam("peach_observability", "debug.motion_enabled");
      const pregrasp = findParam("peach_supervisor", "execute_pregrasp_only");
      const skip = findParam("peach_supervisor", "skip_reconstruction");
      if (motion) $("motion").checked = motion.value === true;
      if (pregrasp) $("full").checked = pregrasp.value === false;
      if (skip) $("reconstruction").checked = skip.value === false && reconstructionMode().available;
      const task = taskState();
      $("ex").checked = task.execution_enabled === true;
      $("gr").checked = task.grasp_enabled === true;
      $("tl").checked = task.tool_enabled === true;
      roundLoaded = true;
      setNote("round-note", "已读取当前开关。");
      renderParams({preserveDrafts: true});
      updateControls();
    } finally { roundLoading = false; updateControls(); }
  }

  function roundSettings() {
    return {motion: $("motion").checked, full: $("full").checked, skip: !$("reconstruction").checked,
      enables: {execution: $("ex").checked, grasp: $("gr").checked, tool: $("tl").checked}};
  }

  async function pushSettings(settings = roundSettings()) {
    session.assertReady();
    const task = taskState();
    if (task.action_active === true || state.manipulation?.status?.running === true) {
      throw new Error("批次或臂周期正在运行，不能改本轮设置");
    }
    const generation = session.snapshot().sessionGeneration;
    const enables = settings.enables;
    const mode = reconstructionMode();
    if (!settings.skip && !mode.available) {
      throw new Error("重建节点未部署/参数不可用，不能开启本轮重建；请重新部署后读取");
    }
    const changes = [
      {node: "peach_observability", name: "debug.motion_enabled", value: settings.motion},
      {node: "peach_supervisor", name: "execute_pregrasp_only", value: !settings.full},
      {node: "peach_supervisor", name: "skip_reconstruction", value: settings.skip},
      {node: "peach_arm", name: "quality.allow_unrefined_geometry", value: settings.skip},
    ];
    if (mode.available) changes.push({node: "peach_target_reconstruction_node", name: "capture.auto_mode", value: !settings.skip});
    await writeParams(changes);
    if (!reconstructionMode().consistent) throw new Error("重建模式读回不一致，未应用使能");
    if (session.snapshot().sessionGeneration !== generation) {
      throw new Error("设置会话已停止，未应用使能");
    }
    await poll();
    session.assertReady();
    await session.submit("set_enables_service", {...enables, reason: "单步调试"});
    const deadline = Date.now() + 5000;
    while (Date.now() < deadline) {
      await poll();
      const current = taskState();
      const arm = state.manipulation?.status || {};
      if (Object.entries(enables).every(([name, value]) =>
        current[`${name}_enabled`] === value && arm[`${name}_enabled`] === value)) {
        return;
      }
      await sleep(150);
    }
    throw new Error("使能读回未确认，请核对当前状态");
  }

  async function applyRound() {
    if (settingsBusy || paramsBusy || paramsLoading || roundLoading) return;
    const snap = session.snapshot();
    if (snap.roundsBusy || snap.singleArmed) {
      notify("先停止当前单轮或多轮会话再改参数", "warn");
      return;
    }
    const settings = roundSettings();
    const summary = [
      `运动放行：${settings.motion ? "开" : "关"}`,
      `执行档：${settings.full ? "完整抓取（套入与剪切）" : "停在预抓取"}`,
      `重建：${settings.skip ? "跳过（使用感知结果）" : "完整重建"}`,
      `执行 ${settings.enables.execution ? "开" : "关"} · 抓取 ${settings.enables.grasp ? "开" : "关"} · 工具 ${settings.enables.tool ? "开" : "关"}`,
      "打开执行之后，启动批次会动臂。工具打开会走到剪切。",
    ].join("\n");
    const confirmed = await PeachShell.confirm({
      title: "写入本轮设置",
      message: summary,
      confirmText: "写入并应用",
      danger: settings.enables.execution || settings.enables.grasp || settings.enables.tool,
    });
    if (!confirmed) return;
    settingsBusy = true;
    updateControls();
    setNote("round-note", "写入中…");
    try {
      await pushSettings(settings);
      await loadRound();
      setNote("round-note", "本轮设置已写入并读回核对");
      notify("本轮设置已写入并读回核对", "ok");
    } catch (error) {
      setNote("round-note", `写入失败：${error.message}`);
      notify(`写入失败：${error.message}`, "err");
    } finally {
      settingsBusy = false;
      updateControls();
    }
  }

  async function applyParams() {
    if (settingsBusy || paramsBusy || paramsLoading || roundLoading) return;
    let changes;
    try {
      changes = collectChanges();
    } catch (error) {
      setNote("params-note", `未写入：${error.message}`);
      return;
    }
    if (!changes.length) {
      setNote("params-note", "没有修改");
      return;
    }
    setNote("params-note", "写入中…");
    paramsBusy = true;
    updateControls();
    try {
      if (session.snapshot().roundsBusy || session.snapshot().singleArmed) {
        throw new Error("先停止当前会话再改参数");
      }
      await writeParams(changes);
      setNote("params-note", `写入 ${changes.length} 项并读回一致`);
      renderParams();
      notify("参数已写入并读回一致", "ok");
    } catch (error) {
      setNote("params-note", `写入失败：${error.message}`);
      notify(`参数写入失败：${error.message}`, "err");
    } finally {
      paramsBusy = false;
      updateControls();
    }
  }

  async function poll() {
    await stateFeed.refresh();
  }

  function setPriority(ordinal) {
    return writeParams([{node: "peach_supervisor", name: "selection_priority", value: ordinal}]);
  }

  session = PeachDebugSession.create({
    api,
    getState: () => state,
    isFresh: () => stateFeed?.isFresh() === true,
    pushSettings,
    poll,
    setPriority,
    confirm: (options) => PeachShell.confirm(options),
    log: logRound,
    notify,
    onStatus: (snap) => { renderGates(); paintSingle(snap); updateControls(); },
  });

  stateFeed = PeachApi.createFeed({
    name: "debug-state", path: "/api/state", intervalMs: 1000, timeoutMs: 6000,
    staleAfterMs: 5000,
    onData: (next) => {
      state = next;
      session.syncState();
      renderState();
    },
    onError: (error) => {
      document.body.classList.add("data-stale");
      if (!state?.debug) setNote("session-status", `状态连接失败：${error.message}`);
      updateControls();
    },
    onStatus: (status) => {
      document.body.classList.toggle("data-stale", status.state !== "online");
      updateControls();
      evidence?.update(state, status, paramData);
    },
  });

  PeachShell.mount({
    active: "debug",
    title: "单步调试",
    eyebrow: "PEACH / STEP DEBUG",
    feed: stateFeed,
    footer: ["单步调试 · 只转发既有服务与动作 · 127.0.0.1:8090", "不会自动 ACK、武装或打开执行使能"],
  });

  evidence = PeachDebugEvidence.create({onDraft: (values) => {
    const snap = session.snapshot();
    if (snap.roundsBusy || snap.singleArmed || settingsBusy || paramsBusy) return;
    Object.entries(values).forEach(([id, value]) => { $(id).checked = value; });
    setNote("round-note", "测试档已载入草稿，尚未写入或应用使能");
    $("round-apply").focus();
  }});
  $("params-search").addEventListener("input", filterParams);
  $("params-changed").addEventListener("change", filterParams);

  ["motion", "full", "ex", "gr", "tl"].forEach((id) => {
    $(id).addEventListener("change", () => tieEnables(id));
  });
  $("round-apply").addEventListener("click", applyRound);
  $("round-reload").addEventListener("click", () => loadRound().catch((error) => {
    setNote("round-note", `读取失败：${error.message}`);
  }));
  $("params-reload").addEventListener("click", loadParams);
  $("params-apply").addEventListener("click", applyParams);
  $("start-single").addEventListener("click", () => {
    session.startSingle().catch((error) => notify(`未启动：${error.message}`, "err"));
  });
  $("stop-single").addEventListener("click", () => {
    session.stopSingle().catch((error) => notify(`停止失败：${error.message}`, "err"));
  });
  $("grasp-selected").addEventListener("click", () => session.enqueue([...picked]));
  $("start-rounds").addEventListener("click", () => {
    const count = Number($("rounds").value);
    if (!Number.isInteger(count) || count < 1 || count > 30) {
      notify("轮数必须是1至30的整数", "warn");
      $("rounds").focus();
      return;
    }
    const mode = $("round-mode").value;
    session.startRounds(count, mode).catch((error) => notify(`未启动：${error.message}`, "err"));
  });
  $("stop-rounds").addEventListener("click", () => {
    session.stopRounds().catch((error) => notify(`停止失败：${error.message}`, "err"));
  });
  $("ack-recovery").addEventListener("click", async () => {
    if (!canAcknowledge()) return;
    const sequence = taskState().state_seq;
    const confirmed = await PeachShell.confirm({title: "人工恢复确认",
      message: "请先在现场确认接触状态、工具和停驻位置。确认后解除恢复等待，允许重新下发任务；不会恢复旧轨迹。",
      confirmText: "现场已确认", danger: true});
    if (!confirmed) return;
    if (!canAcknowledge() || taskState().state_seq !== sequence) {
      notify("状态已变化或陈旧，请重新核对恢复条件", "warn");
      return;
    }
    session.acknowledgeRecovery().catch((error) => notify(`确认失败：${error.message}`, "err"));
  });

  loadParams();
  updateControls();
  stateFeed.start();
})();
