"use strict";

const $ = (id) => document.getElementById(id);
const numeric = (value) => value !== null && value !== undefined && value !== '' &&
  Number.isFinite(Number(value));
const fmt = (value, digits = 3, suffix = "") => numeric(value)
  ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const percent = (value) => numeric(value) ? `${(Number(value) * 100).toFixed(1)}%` : "—";
const safe = (value) => String(value ?? "—").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
})[char]);
const setText = (id, value) => { $(id).textContent = value ?? "—"; };

// 枚举映射与 peach_interfaces/HarvestState.msg 常量一致（2026-08 删除 FAULT）
const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式", "已完成", "需要恢复", "已中断"];
const phaseNames = ["空闲", "选择目标", "观测中", "完成观测", "质量校验", "靠近中", "工具动作", "撤退中", "收尾中", "目标成功", "目标跳过", "目标失败"];
const modeNames = ["自动", "已暂停", "维护"];
const stageOrder = ["ready", "photo", "lock", "observe", "validate", "approach", "tool", "retreat", "done"];

// 由编排器状态推导过程线当前环节：拍照前置/收齐在 DISCOVERY 内按消息区分；
// 目标周期内按 target_phase 映射；终局/异常单独归类。
function currentStage(state) {
  const batch = Number(state.batch_state ?? -1);
  const phase = Number(state.target_phase ?? 0);
  const message = String(state.message || "");
  if (batch === 6) return "done";
  if (batch === 0) return "ready";
  if (batch === 1) return message.includes("拍照") ? "photo" : "lock";
  if (batch === 2 || batch === 3) {
    if (phase <= 3) return "observe";
    if (phase === 4) return "validate";
    if (phase === 5) return "approach";
    if (phase === 6) return "tool";
    if (phase === 7 || phase === 8) return "retreat";
    return "observe";
  }
  return "ready";
}

// 复扫轮次：从最近一条 round_started/round_completed 事件文本解析“第N轮”，
// 总轮数取编排器参数镜像 harvest.max_rounds。
function renderRoundBadge(events, params) {
  const badge = $("round-badge");
  let round = null;
  (events || []).slice().reverse().some((ev) => {
    if (ev.code !== "round_started" && ev.code !== "round_completed") return false;
    const match = /第\s*(\d+)\s*轮/.exec(ev.message || "");
    if (!match) return false;
    round = Number(match[1]);
    return true;
  });
  if (!round) { badge.hidden = true; return; }
  const maxRounds = params?.["/peach_task_executor"]?.["harvest.max_rounds"];
  badge.textContent = numeric(maxRounds) ? `第 ${round}/${maxRounds} 轮` : `第 ${round} 轮`;
  badge.hidden = false;
}

function renderPipeline(state) {
  const active = currentStage(state);
  const activeIndex = stageOrder.indexOf(active);
  const batch = Number(state.batch_state ?? -1);
  const alert = batch === 7 || batch === 8 || state.recovery_required === true;
  document.querySelectorAll("#pipeline .step").forEach((el) => {
    const index = stageOrder.indexOf(el.dataset.stage);
    el.classList.toggle("done", index >= 0 && index < activeIndex);
    el.classList.toggle("active", index === activeIndex && !alert);
    el.classList.toggle("alert", index === activeIndex && alert);
  });
}

// 阶段耗时跟踪：阶段/周期/目标任一变化即结算上阶段耗时；records 按周期存明细。
let phaseTrack = {key: "", cycleKey: "", phase: -1, since: 0, records: []};
function trackPhaseDurations(state) {
  const phase = Number(state.target_phase ?? 0);
  const cycleKey = `${state.cycle_id ?? ""}|${state.target_id ?? ""}`;
  const key = `${state.batch_state ?? ""}|${phase}|${cycleKey}`;
  const now = Date.now();
  if (cycleKey !== phaseTrack.cycleKey) {
    phaseTrack = {key, cycleKey, phase, since: now, records: []};
    return;
  }
  if (key !== phaseTrack.key) {
    const elapsed = Math.max(0, (now - phaseTrack.since) / 1000);
    if (phaseTrack.phase >= 0 && phaseTrack.since > 0) {
      phaseTrack.records.push({
        name: phaseNames[phaseTrack.phase] || "—", seconds: elapsed});
    }
    phaseTrack.key = key;
    phaseTrack.phase = phase;
    phaseTrack.since = now;
  }
}
function phaseElapsedS() {
  return phaseTrack.since > 0 ? Math.max(0, (Date.now() - phaseTrack.since) / 1000) : 0;
}
function renderPhaseDurations(state) {
  const items = phaseTrack.records.map((r) =>
    `<span class="phase-chip">${safe(r.name)} ${r.seconds.toFixed(0)}s</span>`);
  if (Boolean(state.target_id) || Number(state.target_phase) > 0) {
    items.push(`<span class="phase-chip current">${phaseNames[state.target_phase] || "—"} ${phaseElapsedS().toFixed(0)}s</span>`);
  }
  $("phase-durations").innerHTML = items.length
    ? items.join("") : '<p class="empty">暂无阶段耗时</p>';
}

function renderFlow(orchestration, params) {
  const state = orchestration.state || {};
  const events = orchestration.events || [];
  trackPhaseDurations(state);
  setText("batch-state", batchNames[state.batch_state] || "等待编排器");
  $("batch-state").classList.toggle("completed", state.batch_state === 6);
  setText("batch-message", state.message || "尚未收到类型化状态");
  renderPipeline(state);
  renderRoundBadge(events, params);
  const hasTarget = Boolean(state.target_id);
  setText("cycle-target", state.target_id || "—");
  setText("cycle-id", state.cycle_id || "—");
  setText("cycle-phase", phaseNames[state.target_phase] || "—");
  setText("cycle-elapsed", hasTarget || Number(state.target_phase) > 0
    ? `${phaseElapsedS().toFixed(0)} s` : "—");
  renderPhaseDurations(state);
  const width = numeric(state.progress) ? Math.max(0, Math.min(100, Number(state.progress) * 100)) : 0;
  $("batch-progress-bar").style.width = `${width}%`;
  setText("batch-progress-value", percent(state.progress));
  renderEvents(events);
}

function renderEvents(events) {
  setText("event-count", `${events.length} 条`);
  if (!events.length) {
    $("event-list").innerHTML = '<p class="empty">等待编排器事件</p>';
    return;
  }
  $("event-list").innerHTML = events.slice().reverse().map((ev) => {
    const time = numeric(ev.stamp) && ev.stamp > 0
      ? new Date(ev.stamp * 1000).toLocaleTimeString("zh-CN", {hour12: false}) : "--:--:--";
    const severity = numeric(ev.severity) ? Number(ev.severity) : 0;
    const target = ev.target_id ? `<span class="target">[${safe(ev.target_id)}]</span>` : "";
    return `<div class="event-item sev-${severity}"><time>${time}</time><i class="dot" title="${safe(ev.severity_name)}"></i><div class="body"><span class="code">${safe(ev.code)}</span>${target}<p>${safe(ev.message)}</p></div></div>`;
  }).join("");
}

// 采摘/跟踪状态徽标配色（token 与 codec._TRACKING_NAMES 一致；
// OUT_OF_VIEW=出画（复扫无益，视同不可恢复）标红，DEPTH_VOID=深度空洞
// （质量类，可能随视角恢复）标黄）
const harvestChip = {HARVESTED: "ok", WAITING_QUALITY: "warn", SELECTED: "ok", PLANNED: ""};
const trackingChip = {
  OBSERVED: "ok", OCCLUDED: "warn", LOST: "err", INVALID: "err",
  OUT_OF_VIEW: "err", DEPTH_VOID: "warn",
};
const chip = (text, cls) => `<span class="status-chip ${cls}">${safe(text)}</span>`;

function renderPlan(perception, orchestration) {
  const targets = perception.targets || {};
  const harvest = perception.harvest || {};
  const state = orchestration.state || {};
  const observations = targets.observations || [];
  const harvested = observations.filter((item) => item.harvest_status === "HARVESTED");
  const pending = observations.filter((item) =>
    item.harvest_status === "PLANNED" || item.harvest_status === "WAITING_QUALITY");
  setText("plan-total", targets.target_count ?? harvest.target_count ?? observations.length);
  setText("plan-harvested", harvested.length || (harvest.completed_target_ids || []).length);
  setText("plan-pending", pending.length);
  setText("plan-selected", state.target_id || targets.selected_target_id || "—");
  setText("run-id", targets.harvest_run_id || harvest.harvest_run_id || state.run_id || "等待批次");

  const policies = [
    ["auto_start_enabled", "自动开始"],
    ["execution_enabled", "执行"],
    ["grasp_enabled", "抓取"],
    ["tool_enabled", "工具"],
  ];
  $("policy-badges").innerHTML = policies.map(([key, label]) => {
    const on = state[key] === true;
    return `<span class="badge ${on ? "on" : "off"}">${label} ${on ? "启" : "停"}</span>`;
  }).join("");

  const blockers = state.blockers || [];
  $("batch-blockers").hidden = !blockers.length;
  $("batch-blockers").innerHTML = blockers.map((item) =>
    `<span title="该就绪门未通过">${safe(item)}</span>`).join("");

  if (!observations.length) {
    $("target-list").innerHTML = '<tr><td colspan="7" class="empty">等待 target_observations</td></tr>';
  } else {
    const selectedId = state.target_id || targets.selected_target_id || "";
    $("target-list").innerHTML = observations.slice().sort((a, b) => a.priority - b.priority)
      .map((item) => {
        const rowClass = `${item.target_id === selectedId || item.selected ? "selected" : ""} ${item.harvest_status === "HARVESTED" ? "harvested" : ""}`;
        const flags = (item.diagnostic_flags || []).map((flag) => `<span class="flag">${safe(flag)}</span>`).join("");
        return `<tr class="${rowClass}">
          <td><b>${safe(item.target_id)}</b></td>
          <td>#${safe(item.priority)}</td>
          <td>${chip(item.harvest_status, harvestChip[item.harvest_status] ?? "")}</td>
          <td>${chip(item.tracking_status, trackingChip[item.tracking_status] ?? "")}</td>
          <td>${percent(item.confidence)}</td>
          <td>${fmt(item.camera_distance_m, 2, " m")}</td>
          <td>${flags || "—"}</td>
        </tr>`;
      }).join("");
  }
  const doneIds = harvest.completed_target_ids?.length
    ? harvest.completed_target_ids : harvested.map((item) => item.target_id);
  setText("harvested-ids", doneIds.length ? doneIds.join(", ") : "（暂无）");
}

// 话题新鲜度：>5s 黄、>15s 红、从未收到灰
function freshnessHtml(age) {
  if (age === undefined) return ["", "无数据"];
  if (age > 15) return ["err", `${age.toFixed(0)}s 前`];
  if (age > 5) return ["warn", `${age.toFixed(1)}s 前`];
  return ["ok", `${age.toFixed(1)}s 前`];
}

function setFreshness(nodeKey, age) {
  const el = document.querySelector(`.node-card[data-node="${nodeKey}"] [data-freshness]`);
  if (!el) return;
  const [cls, text] = freshnessHtml(age);
  el.className = `freshness ${cls}`;
  el.querySelector("b").textContent = text;
}

function pillClass(text) {
  const value = String(text || "").toUpperCase();
  if (/FAIL|ERROR|FAULT|RECOVERY/.test(value)) return "err";
  if (/WARN|PAUSE|REOBSERVE/.test(value)) return "warn";
  if (/READY|RUNNING|COMPLETE|SUCCEED|IDLE/.test(value)) return "ok";
  return "";
}

function setPill(id, text) {
  setText(id, text || "—");
  $(id).className = `state-pill ${pillClass(text)}`;
}

const yesNo = (value, yes = "是", no = "否") =>
  value === true || value === 1 ? yes : value === false || value === 0 ? no : "—";

function renderNodes(state) {
  const ages = state.system?.topic_age_s || {};
  setFreshness("perception", ages["perception.targets"]);
  setFreshness("reconstruction", ages["reconstruction.diagnostics"]);
  setFreshness("approach", ages["approach.status"]);
  setFreshness("orchestrator", ages["orchestration.state"]);
  setFreshness("robot", ages["robot.status"]);

  const targets = state.perception?.targets || {};
  setText("node-perception-count", targets.target_count ?? "—");
  setText("node-perception-locked", targets.target_set_locked === undefined
    ? "—" : targets.target_set_locked ? "已锁定" : "收集中");
  setText("node-perception-selected", targets.selected_target_id || "—");

  const diag = state.reconstruction?.diagnostics || {};
  const reconState = diag.state || state.reconstruction?.status?.state ||
    state.reconstruction?.status?.text;
  setPill("node-recon-state", reconState);
  setText("node-recon-target", diag.target_id || "—");
  setText("node-recon-views", diag.captured_views === undefined
    ? "—" : `${diag.captured_views} / ${diag.rejected_views ?? 0}`);

  const approach = state.approach?.status || {};
  setPill("node-approach-state", approach.state);
  setText("node-approach-message", approach.message || "—");
  setText("node-approach-arm", `${yesNo(approach.execution_enabled, "ON", "OFF")} / ${yesNo(approach.execution_armed, "ARM", "SAFE")}`);

  const orch = state.orchestration?.state || {};
  setPill("node-orch-state", batchNames[orch.batch_state]);
  setText("node-orch-mode", modeNames[orch.operation_mode] || "—");
  setText("node-orch-active", yesNo(orch.action_active));

  const robot = state.robot?.status || {};
  const hasRobot = Object.keys(robot).length > 0;
  setText("node-robot-power", yesNo(robot.drives_powered, "已上电", "未上电"));
  setText("node-robot-motion", `${yesNo(robot.motion_possible)} / ${yesNo(robot.in_motion)}`);
  const errorText = !hasRobot ? "—"
    : `${yesNo(robot.e_stopped, "急停", "正常")} / ${robot.in_error ? `错误(${robot.error_code})` : "无错误"}`;
  const errorEl = $("node-robot-error");
  errorEl.textContent = errorText;
  errorEl.style.color = hasRobot && (robot.e_stopped === 1 || robot.in_error === 1)
    ? "var(--err)" : "";
}

function gauge(id, value) {
  const el = $(id);
  const width = numeric(value) ? Math.max(0, Math.min(100, Number(value))) : 0;
  el.style.width = `${width}%`;
  el.classList.toggle("hot", width >= 70 && width < 90);
  el.classList.toggle("critical", width >= 90);
}

function renderMetrics(state) {
  const sample = state.metrics?.sample || {};
  const age = state.system?.topic_age_s?.["metrics.sample"];
  setText("metrics-age", age === undefined ? "采样未启动" : `采样 ${age.toFixed(1)}s 前`);
  gauge("sys-cpu-bar", sample.cpu_percent);
  gauge("sys-mem-bar", sample.memory_percent);
  setText("sys-cpu", numeric(sample.cpu_percent) ? `${Number(sample.cpu_percent).toFixed(0)}%` : "—");
  setText("sys-mem", numeric(sample.memory_percent)
    ? `${Number(sample.memory_percent).toFixed(0)}% (${fmt(sample.memory_used_mb, 0)}M)` : "—");
  setText("sys-load", numeric(sample.load1)
    ? `${fmt(sample.load1, 2)} / ${fmt(sample.load5, 2)} / ${fmt(sample.load15, 2)}` : "—");

  const gpu = sample.gpu;
  $("gpu-body").style.display = gpu ? "" : "none";
  $("gpu-empty").hidden = Boolean(gpu);
  if (gpu) {
    gauge("gpu-util-bar", gpu.utilization_percent);
    setText("gpu-util", `${fmt(gpu.utilization_percent, 0)}%`);
    setText("gpu-mem", `${fmt(gpu.memory_used_mb, 0)} / ${fmt(gpu.memory_total_mb, 0)} MB`);
  }

  const diag = state.reconstruction?.diagnostics || {};
  const timings = [
    ["TF 查询延迟", numeric(diag.tf_latency_ms) ? fmt(diag.tf_latency_ms, 1, " ms") : null],
    ["TSDF 积分耗时", numeric(diag.tsdf?.integrate_time_s) ? fmt(diag.tsdf.integrate_time_s, 3, " s") : null],
  ].filter(([, value]) => value !== null);
  $("timing-list").innerHTML = timings.length
    ? timings.map(([label, value]) =>
      `<div class="timing-row"><span>${label}</span><b>${value}</b></div>`).join("")
    : '<p class="empty">等待链路诊断数据</p>';

  const processes = sample.processes || [];
  $("process-list").innerHTML = processes.length
    ? processes.map((proc) => `<tr><td>${safe(proc.name)}</td><td>${proc.pid}</td><td>${fmt(proc.cpu_percent, 1, "%")}</td><td>${fmt(proc.rss_mb, 0, " MB")}</td></tr>`).join("")
    : '<tr><td colspan="4" class="empty">未匹配到受监控进程</td></tr>';
}

// 当前参数只读镜像：按节点分组的小表；值来自后端参数轮询。
function renderParams(params, ages) {
  const root = $("params-tables");
  const names = Object.keys(params || {}).sort();
  if (!names.length) {
    root.innerHTML = '<p class="empty">等待各节点参数服务</p>';
    return;
  }
  const shortName = (full) => full.replace(/^\//, "");
  root.innerHTML = names.map((nodeName) => {
    const values = params[nodeName] || {};
    const age = ages ? ages[`params.${nodeName}`] : undefined;
    const rows = Object.entries(values).map(([key, value]) => {
      const text = value === null || value === undefined ? "—" :
        Array.isArray(value) ? `[${value.join(", ")}]` :
        typeof value === "number" ? String(Math.round(value * 10000) / 10000) :
        String(value);
      return `<tr><td>${safe(key)}</td><td>${safe(text)}</td></tr>`;
    }).join("");
    const ageText = age === undefined ? "" : `${Number(age).toFixed(1)}s`;
    return `<div class="param-group"><h3 title="${safe(nodeName)}">${safe(shortName(nodeName))}<span>${ageText}</span></h3><table>${rows}</table></div>`;
  }).join("");
}

async function pollState() {
  try {
    const response = await fetch(`/api/state?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    const record = state.record?.info || {};
    setText("record-dir", record.enabled === false
      ? "记录已关闭" : record.directory || "等待首批数据");
    $("record-strip").classList.toggle("off",
      record.enabled === false || !record.directory);
    renderFlow(state.orchestration || {}, state.params || {});
    renderPlan(state.perception || {}, state.orchestration || {});
    renderNodes(state);
    renderMetrics(state);
    renderParams(state.params || {}, state.system?.topic_age_s || {});
    $("raw-json").textContent = JSON.stringify(state, null, 2);
    const uptime = Math.max(0, Number(state.system?.uptime_s) || 0);
    setText("server-uptime",
      `UP ${String(Math.floor(uptime / 60)).padStart(2, "0")}:${String(Math.floor(uptime % 60)).padStart(2, "0")}`);
    $("connection").className = "connection online";
    $("connection").querySelector("span").textContent = "数据 API 已连接";
  } catch (_) {
    $("connection").className = "connection offline";
    $("connection").querySelector("span").textContent = "网关连接中断";
  }
}

const tickClock = () => { $("clock").textContent = new Date().toLocaleTimeString("zh-CN", {hour12: false}); };
tickClock();
setInterval(tickClock, 500);
setInterval(pollState, 1000);
pollState();
