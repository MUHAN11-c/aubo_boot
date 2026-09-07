"use strict";

// 采摘流程驾驶舱前端：/api/state（1s）+ /api/trajectory（0.4s）轮询渲染；
// 手动调试操作面（决策 0013）为既有动作/服务的纯转发客户端。
// 后端契约：/api/state 区段见 docs/io.md §5.3；调试端点见 config/observability_parameters.yaml。

const $ = (id) => document.getElementById(id);
const numeric = (value) => value !== null && value !== undefined && value !== "" &&
  Number.isFinite(Number(value));
const fmt = (value, digits = 3, suffix = "") => numeric(value)
  ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const percent = (value) => numeric(value) ? `${(Number(value) * 100).toFixed(1)}%` : "—";
const safe = (value) => String(value ?? "—").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
})[char]);
const setText = (id, value) => { const el = $(id); if (el) el.textContent = value ?? "—"; };

// 枚举映射与 peach_interfaces/HarvestState.msg 常量一致
const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式", "已完成", "需要恢复", "已中断"];
const phaseNames = ["空闲", "选择目标", "观测中", "完成观测", "质量校验", "靠近中", "工具动作", "撤退中", "收尾中", "目标成功", "目标跳过", "目标失败"];
const modeNames = ["自动", "已暂停", "维护"];
const pipelineClass = ["done", "active", "alert", "gated", "failed", "skipped"];

// ── 批次流程 ────────────────────────────────────────────────

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

// 复扫轮次：从最近一条 round_started/round_completed 事件文本解析「第N轮」。
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

// 阶段耗时跟踪：阶段/周期/目标任一变化即结算上阶段耗时。
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

// ── 目标与选果 ──────────────────────────────────────────────

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
  setText("run-id", targets.harvest_run_id || harvest.harvest_run_id || state.run_id || "等待批次");
  $("plan-summary-inline").innerHTML = [
    ["锁定", targets.target_count ?? harvest.target_count ?? observations.length],
    ["已抓", harvested.length || (harvest.completed_target_ids || []).length],
    ["待抓", pending.length],
    ["选中", state.target_id || targets.selected_target_id || "—"],
  ].map(([label, value]) => `<span>${label} <b>${safe(String(value))}</b></span>`).join("");

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

// ── 就绪区（节点灯 + 档位 + 验收门条）───────────────────────

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
  const el = $(id);
  if (!el) return;
  setText(id, text || "—");
  el.className = `state-pill ${pillClass(text)}`;
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
  const count = targets.target_count ?? harvest.target_count ?? "—";
  const lockedKnown = targets.target_set_locked !== undefined
    || harvest.target_set_locked !== undefined;
  const lockedLabel = lockedKnown
    ? (targets.target_set_locked === true || harvest.target_set_locked === true
      ? "已锁定" : "收齐中") : "";
  setText("node-perception-count", lockedLabel ? `${count} · ${lockedLabel}` : count);
  setText("node-perception-selected", targets.selected_target_id || harvest.selected_target_id || "—");

  const diag = state.reconstruction?.diagnostics || {};
  const reconState = diag.state || state.reconstruction?.status?.state ||
    state.reconstruction?.status?.text;
  setPill("node-recon-state", [reconState, diag.target_id].filter(Boolean).join(" · "));
  const decision = state.reconstruction?.grasp_decision || {};
  const graspEl = $("node-recon-grasp");
  setText("node-recon-grasp", decision.allowed === true
    ? "允许" : (decision.reason || (decision.allowed === false ? "未许可" : "—")));
  graspEl.style.color = decision.allowed === true
    ? "var(--ok)" : (decision.reason ? "var(--warn)" : "");

  const manipulation = state.manipulation?.status || {};
  setPill("node-manipulation-state", manipulation.state);
  setText("node-manipulation-arm", `${yesNo(manipulation.execution_enabled, "ON", "OFF")} / ${yesNo(manipulation.execution_armed, "ARM", "SAFE")}`);

  const executor = state.task_executor?.state || {};
  setPill("node-executor-state", batchNames[executor.batch_state]);
  setText("node-executor-active", yesNo(executor.action_active));

  const robot = state.robot?.status || {};
  setText("node-robot-power", `${yesNo(robot.drives_powered, "已上电", "未上电")} / ${yesNo(robot.e_stopped, "急停", "正常")}`);
  const tcp = state.robot?.tcp || {};
  setText("node-robot-tcp", Array.isArray(tcp.xyz) && tcp.xyz.length >= 3
    ? tcp.xyz.map((value) => Number(value).toFixed(3)).join(", ")
    : (tcp.tf_ok === false ? "TF 不可用" : "—"));
}

function renderReadiness(state) {
  const executor = state.task_executor?.state || {};
  const policies = [
    ["auto_start_enabled", "自动开始"],
    ["execution_enabled", "执行"],
    ["grasp_enabled", "抓取"],
    ["tool_enabled", "工具"],
  ];
  $("policy-badges").innerHTML = policies.map(([key, label]) => {
    const on = executor[key] === true;
    return `<span class="badge ${on ? "on" : "off"}">${label} ${on ? "启" : "停"}</span>`;
  }).join("");

  // 验收门条（testing.md 验收门的可自动判定项；对错判定仍以现场为准）
  const ages = state.system?.topic_age_s || {};
  const freshCount = ["perception.targets", "reconstruction.diagnostics",
    "manipulation.status", "task_executor.state", "robot.status"]
    .filter((key) => numeric(ages[key]) && Number(ages[key]) <= 15).length;
  const robot = state.robot?.status || {};
  const cabinetOk = robot.drives_powered === 1 && robot.motion_possible === 1 &&
    robot.e_stopped === 0;
  const tcp = state.robot?.tcp || {};
  const tfFailures = tcp.tf_failures;
  const ratio = tcp.detour_ratio;
  const gates = [
    {label: "五话题新鲜", value: `${freshCount}/5`,
     cls: freshCount >= 5 ? "ok" : freshCount > 0 ? "warn" : "err"},
    {label: "柜侧就绪", value: Object.keys(robot).length
      ? yesNo(cabinetOk, "就绪", "未就绪") : "—",
     cls: Object.keys(robot).length ? (cabinetOk ? "ok" : "err") : ""},
    {label: "TF 失败", value: numeric(tfFailures) ? String(tfFailures) : "—",
     cls: numeric(tfFailures) ? (Number(tfFailures) === 0 ? "ok" : "err") : ""},
    {label: "绕行比 ≤2.2", value: numeric(ratio) ? `${Number(ratio).toFixed(2)}×` : "—",
     cls: numeric(ratio) ? (Number(ratio) <= 1.4 ? "ok" : Number(ratio) <= 2.2 ? "warn" : "err") : ""},
  ];
  const fps = state.perception?.harvest?.timing?.fps;
  gates.push({label: "相机 fps", value: numeric(fps) ? Number(fps).toFixed(2) : "—",
    cls: numeric(fps) ? (Number(fps) >= 2 ? "ok" : "warn") : ""});
  $("accept-gates").innerHTML = gates.map((gate) =>
    `<span class="gate ${gate.cls}">${safe(gate.label)} <b>${safe(gate.value)}</b></span>`).join("");
}

// ── 末端轨迹：指标 + 俯视投影（非交互）──────────────────────

const LANDMARKS = [
  ["perception_entry", "感知入口", [63, 191, 114], 4],
  ["reconstruction_center", "重建中心", [196, 125, 255], 4],
  ["grasp_pregrasp", "预抓取", [224, 169, 62], 5],
  ["grasp_entry", "抓取入口", [224, 92, 92], 5],
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
    ["路径", numeric(metrics.path_length_m) ? `${Number(metrics.path_length_m).toFixed(3)} m` : "—"],
    ["弦长", numeric(metrics.chord_m) ? `${Number(metrics.chord_m).toFixed(3)} m` : "—"],
    ["绕行比", numeric(ratio) ? `${Number(ratio).toFixed(2)}×` : "—"],
    ["偏弦", numeric(metrics.max_dev_m) ? `${Number(metrics.max_dev_m).toFixed(3)} m` : "—"],
  ];
  $("traj-metrics").innerHTML = chips.map(([label, value]) => {
    const hot = label === "绕行比" && numeric(ratio) && Number(ratio) >= 1.4 ? " hot" : "";
    return `<span class="${hot}">${label} <b>${safe(String(value))}</b></span>`;
  }).join("");
}

// 俯视（base_link X 向右 / Y 向上）投影：框选路点+路标，等比缩放留边。
function renderTcpMini(payload) {
  const canvas = $("tcp-canvas");
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, canvas.clientWidth);
  const height = Math.max(1, canvas.clientHeight);
  if (canvas.width !== Math.round(width * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const points = unpackXyz(payload && payload.xyz);
  const marks = (payload && payload.landmarks) || {};
  const scene = points.slice();
  LANDMARKS.forEach(([key]) => {
    if (finiteXyz(marks[key])) scene.push(marks[key].slice(0, 3).map(Number));
  });
  if (!scene.length) {
    ctx.fillStyle = "rgba(223,230,236,0.45)";
    ctx.font = '13px "PingFang SC","Noto Sans CJK SC",sans-serif';
    ctx.fillText("等待末端轨迹数据（latest TF base_link←tcp）", 16, height / 2);
    return;
  }
  const pad = 34;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  scene.forEach(([x, y]) => {
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minY = Math.min(minY, y); maxY = Math.max(maxY, y);
  });
  const spanX = Math.max(maxX - minX, 0.2);
  const spanY = Math.max(maxY - minY, 0.2);
  const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
  const toPx = ([x, y]) => [
    pad + (x - minX) * scale + (width - pad * 2 - spanX * scale) / 2,
    height - pad - (y - minY) * scale - (height - pad * 2 - spanY * scale) / 2,
  ];

  // 网格参考原点（base_link 投影）
  const origin = toPx([0, 0]);
  ctx.strokeStyle = "rgba(42,51,61,0.9)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, origin[1]); ctx.lineTo(width, origin[1]);
  ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], height);
  ctx.stroke();

  if (points.length >= 2) {
    ctx.strokeStyle = "rgba(79,163,224,0.95)";
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.beginPath();
    points.forEach((p, index) => {
      const [px, py] = toPx(p);
      if (index === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
    const [ax, ay] = toPx(points[0]);
    const [bx, by] = toPx(points[points.length - 1]);
    ctx.save();
    ctx.setLineDash([6, 5]);
    ctx.strokeStyle = "rgba(223,230,236,0.55)";
    ctx.lineWidth = 1.4;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    ctx.restore();
  }
  LANDMARKS.forEach(([key, label, rgb, radius]) => {
    if (!finiteXyz(marks[key])) return;
    const [px, py] = toPx(marks[key].slice(0, 3).map(Number));
    ctx.beginPath();
    ctx.fillStyle = `rgba(${rgb.join(",")},0.92)`;
    ctx.arc(px, py, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "rgba(223,230,236,0.9)";
    ctx.font = '11px "PingFang SC","Noto Sans CJK SC",sans-serif';
    ctx.fillText(label, px + 7, py - 5);
  });
  if (points.length) {
    const [tx, ty] = toPx(points[points.length - 1]);
    ctx.beginPath();
    ctx.fillStyle = "rgb(245,245,245)";
    ctx.arc(tx, ty, 4.5, 0, Math.PI * 2);
    ctx.fill();
  }
}

async function pollTrajectory() {
  try {
    const response = await fetch(`/api/trajectory?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    renderTrajHud(payload);
    renderTcpMini(payload);
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
    renderReadiness(state);
    renderNodes(state);
    renderFlow(state.task_executor || {}, state.job || {});
    renderPlan(state.perception || {}, state.task_executor || {});
    renderDebugPanel(state.debug || {});
    $("connection").className = "connection online";
    $("connection").querySelector("span").textContent = "数据 API 已连接";
  } catch (_) {
    $("connection").className = "connection offline";
    $("connection").querySelector("span").textContent = "监控连接中断";
  }
}

setInterval(pollState, 1000);
setInterval(pollTrajectory, 400);
window.addEventListener("resize", pollTrajectory);
pollState();
pollTrajectory();

// ============ 手动调试操作面（决策 0013：融合 8090，鉴权+门控+审计） ============
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
