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
const pipelineClass = ["done", "active", "alert", "gated", "failed", "skipped"];

function renderPipeline(job, state) {
  const byId = {};
  (job && job.stages ? job.stages : []).forEach((stage) => {
    byId[stage.id] = stage;
  });
  const batch = Number((state || {}).batch_state ?? -1);
  document.querySelectorAll("#pipeline .step").forEach((el) => {
    pipelineClass.forEach((name) => el.classList.remove(name));
    const stage = byId[el.dataset.stage];
    if (!stage) return;
    let status = stage.status || "pending";
    if (status === "active" && (batch === 7 || batch === 8)) status = "alert";
    if (status !== "pending") el.classList.add(status);
    el.title = stage.detail || stage.label || "";
  });
}

// 复扫轮次：从最近一条 round_started/round_completed 事件文本解析“第N轮”。
function renderRoundBadge(events) {
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
  badge.textContent = `第 ${round} 轮`;
  badge.hidden = false;
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

function xyzCells(xyz) {
  if (!Array.isArray(xyz) || xyz.length < 3 || xyz.some((value) => !numeric(value))) {
    return ["—", "—", "—"];
  }
  return xyz.slice(0, 3).map((value) => Number(value).toFixed(3));
}

function renderTicket(job) {
  const banner = $("gate-banner");
  const why = (job && job.why) || "等待作业票";
  const grasp = (job && job.grasp) || {};
  const flags = (job && job.flags) || {};
  banner.textContent = why;
  banner.className = "gate-banner";
  if (grasp.allowed && flags.grasp_enabled) banner.classList.add("ok");
  else if (!flags.grasp_enabled || flags.execution_enabled === false) banner.classList.add("gated");
  else if (/失败|未许可|未收敛|未进入/.test(why)) banner.classList.add("fail");

  setText("ticket-title", job && job.target_id
    ? `${job.target_id} · ${(job.perception && job.perception.harvest_status) || "作业中"}`
    : "尚未锁定目标");
  const motion = (job && job.motion) || {};
  setText("ticket-skill", motion.state || "—");

  const coords = (job && job.coords) || {};
  const rows = [
    ["感知入口", coords.perception_entry, "单帧检测进入点"],
    ["感知袋底", coords.perception_bottom, ""],
    ["重建中心", coords.reconstruction_center, "绑定/TSDF 中心"],
    ["精化轴", coords.refined_axis, "单位向量"],
    ["预抓取", coords.grasp_pregrasp, "入口沿 −axis 后撤"],
    ["抓取进入", coords.grasp_entry, grasp.allowed
      ? "接触许可几何"
      : (coords.grasp_entry ? "预抓取几何（接触未许可）" : "无融合几何")],
    ["技能假设", coords.hypothesis_entry,
      numeric(coords.hypothesis_travel_m)
        ? `行程 ${Number(coords.hypothesis_travel_m).toFixed(3)} m` : "FULL 接触前发布"],
  ];
  const hasAny = rows.some(([, xyz]) => Array.isArray(xyz));
  $("coord-body").innerHTML = hasAny
    ? rows.map(([name, xyz, note]) => {
      const cell = xyzCells(xyz);
      return `<tr><td>${safe(name)}</td><td>${cell[0]}</td><td>${cell[1]}</td><td>${cell[2]}</td><td>${safe(note)}</td></tr>`;
    }).join("")
    : '<tr><td colspan="5" class="empty">等待感知/重建坐标（base_link，米）</td></tr>';

  const rec = (job && job.reconstruction) || {};
  const metrics = [
    numeric(coords.camera_distance_m) ? `相机距 ${Number(coords.camera_distance_m).toFixed(2)} m` : null,
    rec.captured_views !== undefined && rec.captured_views !== null ? `视角 ${rec.captured_views}` : null,
    numeric(rec.max_baseline_deg) ? `基线 ${Number(rec.max_baseline_deg).toFixed(1)}°` : null,
    numeric(grasp.diameter_m) ? `袋径 ${(Number(grasp.diameter_m) * 1000).toFixed(0)} mm` : null,
    numeric(grasp.inlier_ratio) ? `内点 ${(Number(grasp.inlier_ratio) * 100).toFixed(0)}%` : null,
  ].filter(Boolean);
  $("ticket-metrics").innerHTML = metrics.map((text) => `<span>${safe(text)}</span>`).join("");
}

function renderFlow(taskExecutor, job) {
  const state = taskExecutor.state || {};
  const events = taskExecutor.events || [];
  trackPhaseDurations(state);
  setText("batch-state", batchNames[state.batch_state] || "等待调度");
  $("batch-state").classList.toggle("completed", state.batch_state === 6);
  setText("batch-message", state.message || "尚未收到类型化状态");
  renderPipeline(job || {}, state);
  renderTicket(job || {});
  renderRoundBadge(events);
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
    $("event-list").innerHTML = '<p class="empty">等待调度事件</p>';
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

function renderPlan(perception, taskExecutor) {
  const targets = perception.targets || {};
  const harvest = perception.harvest || {};
  const state = taskExecutor.state || {};
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
    $("target-list").innerHTML = '<tr><td colspan="8" class="empty">等待 target_observations</td></tr>';
  } else {
    const selectedId = state.target_id || targets.selected_target_id || "";
    $("target-list").innerHTML = observations.slice().sort((a, b) => a.priority - b.priority)
      .map((item) => {
        const rowClass = `${item.target_id === selectedId || item.selected ? "selected" : ""} ${item.harvest_status === "HARVESTED" ? "harvested" : ""}`;
        const flags = (item.diagnostic_flags || []).map((flag) => `<span class="flag">${safe(flag)}</span>`).join("");
        const entry = xyzCells((item.candidate || {}).entry_position);
        return `<tr class="${rowClass}">
          <td><b>${safe(item.target_id)}</b></td>
          <td>#${safe(item.priority)}</td>
          <td>${chip(item.harvest_status, harvestChip[item.harvest_status] ?? "")}</td>
          <td>${chip(item.tracking_status, trackingChip[item.tracking_status] ?? "")}</td>
          <td>${percent(item.confidence)}</td>
          <td>${fmt(item.camera_distance_m, 2, " m")}</td>
          <td class="mono">${entry[0]}, ${entry[1]}, ${entry[2]}</td>
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
  setFreshness("manipulation", ages["manipulation.status"]);
  setFreshness("task_executor", ages["task_executor.state"]);
  setFreshness("robot", ages["robot.status"]);

  const targets = state.perception?.targets || {};
  const harvest = state.perception?.harvest || {};
  const epoch = Number(targets.scene_epoch ?? harvest.scene_epoch ?? 0);
  const collecting = Number(
    targets.collecting_count ?? harvest.collecting_count ?? 0);
  const locked = targets.target_set_locked === true
    || harvest.target_set_locked === true;
  const lockKnown = targets.target_set_locked !== undefined
    || harvest.target_set_locked !== undefined
    || targets.scene_epoch !== undefined
    || harvest.scene_epoch !== undefined;
  let lockLabel = "—";
  if (lockKnown) {
    if (!epoch) lockLabel = "未 Begin";
    else if (locked) lockLabel = "已锁定";
    else lockLabel = collecting ? `收齐中 ${collecting}` : "收齐中";
  }
  setText("node-perception-count", targets.target_count ?? harvest.target_count ?? "—");
  setText("node-perception-locked", lockLabel);
  setText("node-perception-selected", targets.selected_target_id || harvest.selected_target_id || "—");

  const diag = state.reconstruction?.diagnostics || {};
  const reconState = diag.state || state.reconstruction?.status?.state ||
    state.reconstruction?.status?.text;
  setPill("node-recon-state", reconState);
  setText("node-recon-target", diag.target_id || "—");
  setText("node-recon-views", diag.captured_views === undefined
    ? "—" : `${diag.captured_views} / ${diag.rejected_views ?? 0}`);
  const decision = state.reconstruction?.grasp_decision || {};
  setText("node-recon-grasp", decision.allowed === true
    ? "允许" : (decision.reason || (decision.allowed === false ? "未许可" : "—")));
  $("node-recon-grasp").style.color = decision.allowed === true
    ? "var(--ok)" : (decision.reason ? "var(--warn)" : "");

  const manipulation = state.manipulation?.status || {};
  setPill("node-manipulation-state", manipulation.state);
  setText("node-manipulation-message", manipulation.message || "—");
  setText("node-manipulation-arm", `${yesNo(manipulation.execution_enabled, "ON", "OFF")} / ${yesNo(manipulation.execution_armed, "ARM", "SAFE")}`);

  const executor = state.task_executor?.state || {};
  setPill("node-executor-state", batchNames[executor.batch_state]);
  setText("node-executor-mode", modeNames[executor.operation_mode] || "—");
  setText("node-executor-active", yesNo(executor.action_active));

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
  const tcp = state.robot?.tcp || {};
  const tcpXyz = tcp.xyz;
  setText("node-robot-tcp", Array.isArray(tcpXyz) && tcpXyz.length >= 3
    ? tcpXyz.map((value) => Number(value).toFixed(3)).join(", ")
    : (tcp.tf_ok === false ? "TF 不可用" : "—"));
  const ratio = tcp.detour_ratio;
  const pathBits = [
    numeric(tcp.path_length_m) ? `${Number(tcp.path_length_m).toFixed(3)} m` : null,
    numeric(tcp.chord_m) ? `${Number(tcp.chord_m).toFixed(3)} m` : null,
    numeric(ratio) ? `${Number(ratio).toFixed(2)}×` : null,
  ].filter(Boolean);
  setText("node-robot-path", pathBits.length ? pathBits.join(" / ") : "—");
  $("node-robot-path").style.color = numeric(ratio) && Number(ratio) >= 1.4
    ? "var(--warn)" : "";
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

const PHASE_RGB = {
  2: [79, 163, 224],
  5: [224, 169, 62],
  6: [196, 125, 255],
  7: [63, 191, 114],
};
const LANDMARKS = [
  ["perception_entry", "感知入口", [63, 191, 114], 5],
  ["reconstruction_center", "重建中心", [196, 125, 255], 5],
  ["grasp_pregrasp", "预抓取", [224, 169, 62], 7],
  ["grasp_entry", "抓取入口", [224, 92, 92], 7],
];

function unpackXyz(flat) {
  const points = [];
  if (!Array.isArray(flat)) return points;
  for (let i = 0; i + 2 < flat.length; i += 3) {
    const x = Number(flat[i]); const y = Number(flat[i + 1]); const z = Number(flat[i + 2]);
    if ([x, y, z].every(Number.isFinite)) points.push([x, y, z]);
  }
  return points;
}

function finiteXyz(value) {
  return Array.isArray(value) && value.length >= 3 &&
    value.slice(0, 3).every((item) => Number.isFinite(Number(item)));
}

function quatRotate(quat, vec) {
  const qx = quat[0]; const qy = quat[1]; const qz = quat[2]; const qw = quat[3];
  const ix = qw * vec[0] + qy * vec[2] - qz * vec[1];
  const iy = qw * vec[1] + qz * vec[0] - qx * vec[2];
  const iz = qw * vec[2] + qx * vec[1] - qy * vec[0];
  const iw = -qx * vec[0] - qy * vec[1] - qz * vec[2];
  return [
    ix * qw + iw * -qx + iy * -qz - iz * -qy,
    iy * qw + iw * -qy + iz * -qx - ix * -qz,
    iz * qw + iw * -qz + ix * -qy - iy * -qx,
  ];
}

function add3(a, b, scale = 1) {
  return [a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2]];
}

function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross3(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
function norm3(a) {
  const n = Math.hypot(a[0], a[1], a[2]);
  return n < 1e-12 ? [0, 0, 0] : [a[0] / n, a[1] / n, a[2] / n];
}

const tcpView = {
  yaw: 0.85,
  pitch: 0.42,
  distance: 1.6,
  look: [0.25, -0.45, 0.45],
  fitted: false,
  dragging: null,
  last: {x: 0, y: 0},
  hoverHits: [],
  follow: false,
  showChord: true,
  scrub: -1,
  payload: null,
};

function collectScenePoints(payload, points) {
  const out = points.slice();
  const marks = (payload && payload.landmarks) || {};
  LANDMARKS.forEach(([key]) => {
    if (finiteXyz(marks[key])) out.push(marks[key].slice(0, 3).map(Number));
  });
  if (finiteXyz(marks.axis) && finiteXyz(marks.grasp_entry)) {
    const axis = norm3(marks.axis.map(Number));
    const entry = marks.grasp_entry.map(Number);
    out.push(add3(entry, axis, -0.12), add3(entry, axis, 0.22));
  }
  return out;
}

function sceneBounds(points) {
  if (!points.length) {
    return {min: [-0.2, -0.8, 0], max: [0.8, 0.2, 0.9], center: [0.3, -0.3, 0.45], span: 1.2};
  }
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  points.forEach((p) => {
    for (let i = 0; i < 3; i += 1) {
      min[i] = Math.min(min[i], p[i]);
      max[i] = Math.max(max[i], p[i]);
    }
  });
  const center = [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2];
  const span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2], 0.25);
  return {min, max, center, span};
}

function fitTcpView(payload, points) {
  const bounds = sceneBounds(collectScenePoints(payload, points));
  tcpView.look = bounds.center;
  tcpView.distance = Math.max(0.45, bounds.span * 2.15);
  tcpView.fitted = true;
}

function projectPoint(point, width, height) {
  const cosP = Math.cos(tcpView.pitch);
  const eye = [
    tcpView.look[0] + tcpView.distance * cosP * Math.cos(tcpView.yaw),
    tcpView.look[1] + tcpView.distance * cosP * Math.sin(tcpView.yaw),
    tcpView.look[2] + tcpView.distance * Math.sin(tcpView.pitch),
  ];
  const forward = norm3(sub3(tcpView.look, eye));
  let right = cross3(forward, [0, 0, 1]);
  if (Math.hypot(right[0], right[1], right[2]) < 1e-6) right = cross3(forward, [0, 1, 0]);
  right = norm3(right);
  const up = norm3(cross3(right, forward));
  const rel = sub3(point, eye);
  const depth = dot3(rel, forward);
  if (depth < 0.04) return null;
  const fov = 0.62;
  const u = dot3(rel, right) / (depth * Math.tan(fov));
  const v = dot3(rel, up) / (depth * Math.tan(fov));
  return {
    x: (u * 0.5 + 0.5) * width,
    y: (1 - (v * 0.5 + 0.5)) * height,
    depth,
  };
}

function resizeTcpCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, canvas.clientWidth);
  const height = Math.max(1, canvas.clientHeight);
  const tw = Math.round(width * dpr);
  const th = Math.round(height * dpr);
  if (canvas.width !== tw || canvas.height !== th) {
    canvas.width = tw;
    canvas.height = th;
  }
  return {width, height, dpr};
}

function strokePath(ctx, projected, phases) {
  if (projected.length < 2) return;
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  let current = null;
  const flush = () => {
    if (!current || current.pts.length < 2) return;
    ctx.beginPath();
    ctx.strokeStyle = `rgb(${current.rgb.join(",")})`;
    ctx.moveTo(current.pts[0].x, current.pts[0].y);
    for (let i = 1; i < current.pts.length; i += 1) ctx.lineTo(current.pts[i].x, current.pts[i].y);
    ctx.stroke();
  };
  projected.forEach((pt, index) => {
    if (!pt) {
      flush();
      current = null;
      return;
    }
    const phase = Number((phases && phases[index]) || 0);
    const rgb = PHASE_RGB[phase] || [223, 230, 236];
    if (!current || current.phase !== phase) {
      flush();
      current = {phase, rgb, pts: [pt]};
    } else current.pts.push(pt);
  });
  flush();
}

function dashChord(ctx, a, b, width, height) {
  const pa = projectPoint(a, width, height);
  const pb = projectPoint(b, width, height);
  if (!pa || !pb) return;
  ctx.save();
  ctx.setLineDash([6, 5]);
  ctx.strokeStyle = "rgba(223,230,236,0.55)";
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
  ctx.restore();
}

function drawGrid(ctx, bounds, width, height) {
  const step = bounds.span > 1.2 ? 0.2 : 0.1;
  const pad = bounds.span * 0.35;
  const x0 = Math.floor((bounds.min[0] - pad) / step) * step;
  const x1 = Math.ceil((bounds.max[0] + pad) / step) * step;
  const y0 = Math.floor((bounds.min[1] - pad) / step) * step;
  const y1 = Math.ceil((bounds.max[1] + pad) / step) * step;
  ctx.lineWidth = 1;
  for (let x = x0; x <= x1 + 1e-9; x += step) {
    const a = projectPoint([x, y0, 0], width, height);
    const b = projectPoint([x, y1, 0], width, height);
    if (!a || !b) continue;
    ctx.strokeStyle = Math.abs(x) < 1e-6 ? "rgba(224,92,92,0.55)" : "rgba(42,51,61,0.9)";
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }
  for (let y = y0; y <= y1 + 1e-9; y += step) {
    const a = projectPoint([x0, y, 0], width, height);
    const b = projectPoint([x1, y, 0], width, height);
    if (!a || !b) continue;
    ctx.strokeStyle = Math.abs(y) < 1e-6 ? "rgba(63,191,114,0.5)" : "rgba(42,51,61,0.9)";
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }
}

function drawAxes(ctx, origin, length, width, height) {
  const axes = [
    [add3(origin, [length, 0, 0]), "X", "rgb(224,92,92)"],
    [add3(origin, [0, length, 0]), "Y", "rgb(63,191,114)"],
    [add3(origin, [0, 0, length]), "Z", "rgb(79,163,224)"],
  ];
  const po = projectPoint(origin, width, height);
  if (!po) return;
  ctx.lineWidth = 1.6;
  ctx.font = "11px ui-monospace, monospace";
  axes.forEach(([tip, label, color]) => {
    const pt = projectPoint(tip, width, height);
    if (!pt) return;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.moveTo(po.x, po.y); ctx.lineTo(pt.x, pt.y); ctx.stroke();
    ctx.fillText(label, pt.x + 4, pt.y - 2);
  });
}

function drawSphere(ctx, point, rgb, radiusPx, label, width, height) {
  const pt = projectPoint(point, width, height);
  if (!pt) return pt;
  ctx.beginPath();
  ctx.fillStyle = `rgba(${rgb.join(",")},0.92)`;
  ctx.arc(pt.x, pt.y, radiusPx, 0, Math.PI * 2);
  ctx.fill();
  if (label) {
    ctx.fillStyle = "rgba(223,230,236,0.9)";
    ctx.font = '11px "PingFang SC","Noto Sans CJK SC",sans-serif';
    ctx.fillText(label, pt.x + 8, pt.y - 6);
  }
  return pt;
}

function renderTcpCanvas() {
  const canvas = $("tcp-canvas");
  if (!canvas) return;
  const {width, height} = resizeTcpCanvas(canvas);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(window.devicePixelRatio || 1, 0, 0, window.devicePixelRatio || 1, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const payload = tcpView.payload || {};
  const all = unpackXyz(payload.xyz);
  const phases = payload.phase || [];
  const until = tcpView.scrub >= 0 ? tcpView.scrub + 1 : all.length;
  const points = all.slice(0, until);
  const usedPhases = phases.slice(0, points.length);
  if (!tcpView.fitted && (points.length || Object.keys(payload.landmarks || {}).length)) {
    fitTcpView(payload, points);
  }
  if (tcpView.follow && points.length) {
    tcpView.look = points[points.length - 1].slice();
  }
  const bounds = sceneBounds(collectScenePoints(payload, points));
  drawGrid(ctx, bounds, width, height);
  drawAxes(ctx, [0, 0, 0], Math.max(0.12, bounds.span * 0.18), width, height);
  const projected = points.map((p) => projectPoint(p, width, height));
  strokePath(ctx, projected, usedPhases);
  if (tcpView.showChord && points.length >= 2) {
    dashChord(ctx, points[0], points[points.length - 1], width, height);
  }
  const marks = payload.landmarks || {};
  if (finiteXyz(marks.axis) && finiteXyz(marks.grasp_entry)) {
    const axis = norm3(marks.axis.map(Number));
    const entry = marks.grasp_entry.map(Number);
    const a = projectPoint(add3(entry, axis, -0.12), width, height);
    const b = projectPoint(add3(entry, axis, 0.22), width, height);
    if (a && b) {
      ctx.strokeStyle = "rgb(51, 196, 232)";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
  }
  LANDMARKS.forEach(([key, label, rgb, radius]) => {
    if (finiteXyz(marks[key])) {
      drawSphere(ctx, marks[key].map(Number), rgb, radius, label, width, height);
    }
  });
  if (points.length) {
    const tip = points[points.length - 1];
    drawSphere(ctx, tip, [245, 245, 245], 6, "tcp", width, height);
    const quat = (payload.metrics || {}).quat;
    if (Array.isArray(quat) && quat.length >= 4) {
      [["X", [0.05, 0, 0], "rgb(224,92,92)"],
        ["Y", [0, 0.05, 0], "rgb(63,191,114)"],
        ["Z", [0, 0, 0.08], "rgb(79,163,224)"]].forEach(([, vec, color]) => {
        const tip2 = add3(tip, quatRotate(quat.map(Number), vec));
        const pa = projectPoint(tip, width, height);
        const pb = projectPoint(tip2, width, height);
        if (!pa || !pb) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.8;
        ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();
      });
    }
  }
  tcpView.hoverHits = projected.map((pt, index) => (pt ? {...pt, index, world: points[index]} : null));
}

function renderTrajHud(payload) {
  const metrics = (payload && payload.metrics) || {};
  const enabled = payload && payload.enabled !== false;
  const tfOk = payload && payload.tf_ok;
  const n = unpackXyz(payload && payload.xyz).length;
  let status = "等待 TF base_link←tcp";
  if (!enabled) status = "轨迹采样已关闭";
  else if (tfOk === false) status = `TF 失败 ${payload.tf_failures || 0} 次`;
  else if (n) status = `${payload.frame_id || "base_link"} ← ${payload.tip_frame || "tcp"} · ${n} 点`;
  setText("traj-status", status);
  const ratio = metrics.detour_ratio;
  const chips = [
    ["点数", metrics.count ?? n],
    ["路径", numeric(metrics.path_length_m) ? `${Number(metrics.path_length_m).toFixed(3)} m` : "—"],
    ["弦长", numeric(metrics.chord_m) ? `${Number(metrics.chord_m).toFixed(3)} m` : "—"],
    ["绕行比", numeric(ratio) ? `${Number(ratio).toFixed(2)}×` : "—"],
    ["偏弦", numeric(metrics.max_dev_m) ? `${Number(metrics.max_dev_m).toFixed(3)} m` : "—"],
    ["Δz", numeric(metrics.dz_m) ? `${Number(metrics.dz_m).toFixed(3)} m` : "—"],
  ];
  $("traj-metrics").innerHTML = chips.map(([label, value]) => {
    const hot = label === "绕行比" && numeric(ratio) && Number(ratio) >= 1.4 ? " hot" : "";
    return `<span class="${hot}">${label} <b>${safe(String(value))}</b></span>`;
  }).join("");
}

function bindTcpCanvas() {
  const canvas = $("tcp-canvas");
  if (!canvas || canvas.dataset.bound === "1") return;
  canvas.dataset.bound = "1";
  const tooltip = $("traj-tooltip");
  canvas.addEventListener("pointerdown", (event) => {
    canvas.setPointerCapture(event.pointerId);
    tcpView.dragging = event.button === 2 || event.shiftKey ? "pan" : "orbit";
    tcpView.last = {x: event.clientX, y: event.clientY};
  });
  canvas.addEventListener("pointerup", () => { tcpView.dragging = null; });
  canvas.addEventListener("pointermove", (event) => {
    if (tcpView.dragging) {
      const dx = event.clientX - tcpView.last.x;
      const dy = event.clientY - tcpView.last.y;
      tcpView.last = {x: event.clientX, y: event.clientY};
      if (tcpView.dragging === "orbit") {
        tcpView.follow = false;
        $("traj-follow").checked = false;
        tcpView.yaw += dx * 0.008;
        tcpView.pitch = Math.max(-1.2, Math.min(1.2, tcpView.pitch + dy * 0.006));
      } else {
        const right = [-Math.sin(tcpView.yaw), Math.cos(tcpView.yaw), 0];
        const up = [0, 0, 1];
        const scale = tcpView.distance * 0.0022;
        tcpView.look = add3(tcpView.look, right, -dx * scale);
        tcpView.look = add3(tcpView.look, up, dy * scale);
      }
      renderTcpCanvas();
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const mx = event.clientX - rect.left;
    const my = event.clientY - rect.top;
    let best = null;
    (tcpView.hoverHits || []).forEach((hit) => {
      if (!hit) return;
      const dist = Math.hypot(hit.x - mx, hit.y - my);
      if (dist < 10 && (!best || dist < best.dist)) best = {dist, hit};
    });
    if (!best) { tooltip.hidden = true; return; }
    const w = best.hit.world;
    tooltip.hidden = false;
    tooltip.style.left = `${mx}px`;
    tooltip.style.top = `${my}px`;
    tooltip.textContent =
      `tcp  ${w[0].toFixed(3)}, ${w[1].toFixed(3)}, ${w[2].toFixed(3)}`;
  });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    tcpView.distance = Math.max(0.25, Math.min(8, tcpView.distance * (event.deltaY > 0 ? 1.08 : 0.92)));
    renderTcpCanvas();
  }, {passive: false});
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());
  $("traj-fit").addEventListener("click", () => {
    const points = unpackXyz((tcpView.payload || {}).xyz);
    fitTcpView(tcpView.payload || {}, points);
    tcpView.follow = false;
    $("traj-follow").checked = false;
    renderTcpCanvas();
  });
  $("traj-follow").addEventListener("change", (event) => {
    tcpView.follow = event.target.checked;
    renderTcpCanvas();
  });
  $("traj-show-chord").addEventListener("change", (event) => {
    tcpView.showChord = event.target.checked;
    renderTcpCanvas();
  });
  $("traj-scrub").addEventListener("input", (event) => {
    const max = Number(event.target.max || 0);
    const value = Number(event.target.value);
    tcpView.scrub = max <= 0 || value >= max ? -1 : value;
    $("traj-scrub-label").textContent = tcpView.scrub < 0 ? "实时" : `#${tcpView.scrub + 1}`;
    renderTcpCanvas();
  });
}

async function pollTrajectory() {
  bindTcpCanvas();
  try {
    const response = await fetch(`/api/trajectory?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    tcpView.payload = payload;
    const count = unpackXyz(payload.xyz).length;
    const scrub = $("traj-scrub");
    scrub.max = String(Math.max(0, count - 1));
    scrub.disabled = count < 2;
    if (tcpView.scrub < 0) scrub.value = scrub.max;
    renderTrajHud(payload);
    renderTcpCanvas();
  } catch (_) {
    setText("traj-status", "轨迹 API 不可用");
  }
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
    renderFlow(state.task_executor || {}, state.job || {});
    renderPlan(state.perception || {}, state.task_executor || {});
    renderNodes(state);
    renderMetrics(state);
    renderParams(state.params || {}, state.system?.topic_age_s || {});
    $("raw-json").textContent = JSON.stringify(state, null, 2);
    renderDebugPanel(state.debug || {});
    const uptime = Math.max(0, Number(state.system?.uptime_s) || 0);
    setText("server-uptime",
      `UP ${String(Math.floor(uptime / 60)).padStart(2, "0")}:${String(Math.floor(uptime % 60)).padStart(2, "0")}`);
    $("connection").className = "connection online";
    $("connection").querySelector("span").textContent = "数据 API 已连接";
  } catch (_) {
    $("connection").className = "connection offline";
    $("connection").querySelector("span").textContent = "监控连接中断";
  }
}

const tickClock = () => { $("clock").textContent = new Date().toLocaleTimeString("zh-CN", {hour12: false}); };
tickClock();
setInterval(tickClock, 500);
setInterval(pollState, 1000);
setInterval(pollTrajectory, 400);
window.addEventListener("resize", renderTcpCanvas);
pollState();

// ============ 手动调试操作面（决策 0007 推翻条款：融合 8090，鉴权+门控） ============
// Web 只是另一个 ROS 客户端：技能 ExecutionAuthority 与调度/重建门照常复核。
const debugView = {results: []};

function switchView(name) {
  document.querySelectorAll(".view-tabs button").forEach((el) => {
    el.classList.toggle("active", el.dataset.view === name);
  });
  $("view-monitor").hidden = name !== "monitor";
  $("view-debug").hidden = name !== "debug";
}

function debugToken() {
  return $("debug-token").value.trim();
}

async function debugPost(action, payload, confirmText) {
  if (confirmText && !window.confirm(confirmText)) {
    return null;
  }
  const response = await fetch(`/api/debug/${encodeURIComponent(action)}`, {
    method: "POST",
    headers: {"Content-Type": "application/json", "X-Debug-Token": debugToken()},
    body: JSON.stringify(payload || {}),
  });
  let body = {};
  try { body = await response.json(); } catch (_) { /* 非 JSON 响应 */ }
  debugView.results.unshift({
    ts: new Date().toLocaleTimeString("zh-CN", {hour12: false}),
    action, status: response.status, accepted: body.accepted === true,
    message: body.message || "", detail: body,
  });
  if (debugView.results.length > 30) debugView.results.pop();
  renderDebugResults();
  return {ok: response.ok, status: response.status, body};
}

function renderDebugResults() {
  const list = $("debug-results");
  if (!debugView.results.length) {
    list.innerHTML = '<p class="empty">尚无操作</p>';
    return;
  }
  list.innerHTML = debugView.results.map((entry, index) => {
    const cls = entry.accepted ? (entry.status < 300 ? "ok" : "warn") : "err";
    const detail = safe(JSON.stringify(entry.detail));
    return `<details class="debug-result ${cls}" ${index ? "" : "open"}>
      <summary><b>${safe(entry.ts)}</b> ${safe(entry.action)}
      <span class="debug-status">${entry.status} ${entry.accepted ? "已受理" : "被拒/未成"}</span></summary>
      <p>${safe(entry.message)}</p><pre>${detail}</pre></details>`;
  }).join("");
  $("debug-recent-count").textContent = `${debugView.results.length} 条（会话内）`;
}

// 门控状态横幅（/api/state 每秒刷新；令牌本身绝不下发）
function renderDebugPanel(debug) {
  const gates = $("debug-gates");
  if (!gates) return;
  const enabled = debug.enabled === true;
  const motion = debug.motion_enabled === true;
  const tokenRequired = debug.token_required === true;
  gates.innerHTML = `
    <span class="debug-gate ${enabled ? "ok" : "err"}">操作面 ${enabled ? "已启用" : "未启用（debug.enabled=false，POST 全拒）"}</span>
    <span class="debug-gate ${motion ? "warn" : "ok"}">运动类 ${motion ? "已放行" : "默认拒绝（423）"}</span>
    <span class="debug-gate ${tokenRequired ? "warn" : "err"}">令牌 ${tokenRequired ? "必填" : "未配置（全拒）"}</span>`;
  document.querySelectorAll("[data-debug], [data-debug-cancel]").forEach((el) => {
    el.disabled = !enabled;
  });
}

function bindDebugControls() {
  document.querySelectorAll(".view-tabs button").forEach((el) => {
    el.addEventListener("click", () => switchView(el.dataset.view));
  });
  document.querySelectorAll("[data-debug]").forEach((el) => {
    el.addEventListener("click", async () => {
      const action = el.dataset.debug;
      const payload = buildDebugPayload(action, el);
      if (payload === null) return;
      const motionControl = action === "control_service" &&
        (payload.command === "RESUME" || payload.command === "EXIT_MAINTENANCE");
      const needsConfirm = el.dataset.confirm === "1" || motionControl;
      const outcome = await debugPost(action, payload, needsConfirm
        ? `确认发送【${action}】？\n${JSON.stringify(payload, null, 2)}\n\n运动类操作：技能侧安全门仍会独立复核。`
        : null);
      if (outcome && !outcome.ok && outcome.status === 401) {
        window.alert("401：令牌缺失或不匹配（服务端 debug.token）");
      } else if (outcome && outcome.status === 423) {
        window.alert("423：运动类操作被拒（服务端 debug.motion_enabled=false）");
      }
    });
  });
  document.querySelectorAll("[data-debug-cancel]").forEach((el) => {
    el.addEventListener("click", () => debugPost("cancel", {target: el.dataset.debugCancel}, null));
  });
  $("debug-ping").addEventListener("click", async () => {
    const outcome = await debugPost("recon_query_service", {}, null);
    if (outcome) window.alert(outcome.ok ? "令牌有效，操作面可用" : `失败：HTTP ${outcome.status}`);
  });
}

// 从页面控件收集各端点 payload；返回 null 表示输入不合法
function buildDebugPayload(action, el) {
  if (action === "manage_nodes_service") {
    return {command: el.dataset.payload ? JSON.parse(el.dataset.payload).command : ""};
  }
  if (action === "run_harvest_action") {
    const requestId = $("run-request-id").value.trim();
    if (!requestId) { window.alert("request_id 必填（同时是账本目录名，不得复用）"); return null; }
    const intent = $("run-intent").value;
    const targets = $("run-target-ids").value.split(",").map((s) => s.trim()).filter(Boolean);
    return {request_id: requestId, scene_key: "lab", intent,
      selection_mode: targets.length ? "MANUAL" : "AUTO", target_ids: targets};
  }
  if (action === "control_service") {
    return {command: $("ctl-command").value,
      expected_state_seq: Number($("ctl-seq").value) || 0,
      reason: $("ctl-reason").value.trim() || "web 手动调试"};
  }
  if (action === "begin_scene_service") {
    return {request_id: $("scene-request-id").value.trim() || "dev",
      scene_key: $("scene-key").value.trim() || "lab"};
  }
  if (action === "survey_action") {
    return {request_id: $("scene-request-id").value.trim() ||
      $("run-request-id").value.trim() || "dev",
      scene_key: $("scene-key").value.trim() || "lab"};
  }
  if (action === "build_action") {
    const targetId = $("build-target-id").value.trim();
    if (!targetId) { window.alert("target_id 必填（须与 HarvestState.target_id 一致）"); return null; }
    return {request_id: $("scene-request-id").value.trim() || "dev", target_id: targetId,
      scene_epoch: Number($("build-epoch").value) || 0};
  }
  if (action === "execute_action") {
    const targetId = $("exec-target-id").value.trim();
    if (!targetId) { window.alert("target_id 必填"); return null; }
    return {request_id: $("scene-request-id").value.trim() || "dev", target_id: targetId,
      mode: $("exec-mode").value, skip_observation: $("exec-skip-obs").checked};
  }
  if (action === "arm_service") {
    return {data: $("arm-data").value === "true"};
  }
  if (action === "check_reachability_service") {
    const parts = $("reach-xyz").value.split(",").map((s) => Number(s.trim()));
    if (parts.length !== 3 || parts.some((v) => !Number.isFinite(v))) {
      window.alert("位姿格式：x,y,z（米）"); return null;
    }
    return {timeout_s: 0.5, tcp_poses: [{frame_id: "base_link",
      position: {x: parts[0], y: parts[1], z: parts[2]},
      orientation: {x: 0, y: 0, z: 0, w: 1}}]};
  }
  return {};  // Trigger 形服务无字段
}

bindDebugControls();
pollTrajectory();
