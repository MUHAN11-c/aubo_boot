"use strict";

// 感知抓取过程页：/api/state（1s）+ /api/trajectory（0.4s）。
// 调试面转发既有动作/服务；动臂由 yaml debug.motion_enabled 放行。
// 全流程反馈：阶段时序（pipeline.fsm/arm 服务器侧）+ 批次账本（ledger.live）
// + 感知节拍/重建进度 + 系统负载与参数镜像。

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

const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式", "已完成", "需要恢复", "已中断"];
const phaseNames = ["空闲", "选择目标", "观测中", "完成观测", "质量校验", "靠近中", "工具动作", "撤退中", "收尾中", "目标成功", "目标跳过", "目标失败"];
const pipelineClass = ["done", "active", "alert", "gated", "failed", "skipped"];

// CanonicalEvent.code → 人读标签（对齐 CanonicalEvent.msg 头注词表；未知码回退原码）
const eventCodeNames = {
  target_dispatched: "派发目标", target_succeeded: "目标成功",
  target_skipped: "目标跳过", target_failed: "目标失败",
  target_canceled: "目标取消", target_operator_skipped: "人工跳过",
  survey_failed: "扫场失败", begin_scene_failed: "开场景失败",
  navigate_failed: "到位失败（预留）", photo_pose_reached: "到达拍照位",
  round_locked: "锁定本轮", target_timeout: "单果时限超限",
  targets_filtered: "选果过滤", batch_paused: "批次暂停",
  batch_resumed: "批次恢复", recovery_required: "需要恢复",
  recovery_acknowledged: "恢复已确认", enables_changed: "使能变更",
  batch_policy_updated: "批次策略变更", fire_step: "单步调试",
  ledger_restored: "账本断点恢复", observe_build_view_race: "观察/建模视角竞态",
};
// HarvestState.blockers 词表（harvest_fsm.BLOCKER_*）
const blockerNames = {
  stack_not_ready: "托管栈未就绪", recovery_required: "待人工恢复确认",
  mode_paused: "批次已暂停", mode_maintenance: "维护模式",
  ledger_write_failed: "账本写失败",
};
// FailureCode 数值 → 常量名（IDL 0-23；对齐 docs/io.md 排障表）
const failureCodeNames = {
  0: "NONE", 1: "OBSERVE_FAILED", 2: "BUILD_FAILED", 3: "DECISION_REJECTED",
  4: "PREGRASP_RESIDUAL", 5: "SLEEVE_PLAN_FAILED", 6: "CUT_COMMAND_FAILED",
  7: "CUT_FEEDBACK_TIMEOUT", 8: "RETREAT_FAILED", 9: "RECOVERY_REQUIRED",
  10: "VEHICLE_NOT_STATIONARY", 11: "EXACT_TF_MISSING",
  12: "DYNAMIC_BUDGET_NEGATIVE", 13: "UNBAGGED_NOT_IN_SCOPE",
  14: "DEGRADED_CONTACT_FORBIDDEN", 15: "MODEL_STALE", 16: "CORRIDOR_BLOCKED",
  17: "MODEL_IDENTITY_INCOMPLETE", 18: "MODEL_EXPIRED",
  19: "TOOL_STATE_UNKNOWN", 20: "PLAN_MISMATCH", 21: "TRANSIT_FAILED",
  22: "START_NOT_READY", 23: "CANCELED",
};
const failureCodeLabel = (n) =>
  failureCodeNames[Number(n)] || `CODE_${n}`;

// 最新一次 /api/state（调试面取 state_seq；渲染器共用）
let monitorState = null;

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
    const sevLabel = ["信息", "警告", "错误", "审计"][severity] || "信息";
    const codeLabel = eventCodeNames[ev.code] || ev.code;
    const target = ev.target_id ? `<span class="target">[${safe(ev.target_id)}]</span>` : "";
    const detailKeys = ev.details && Object.keys(ev.details).length
      ? Object.keys(ev.details) : [];
    const details = detailKeys.length
      ? `<details class="event-details"><summary>${detailKeys.length} 项详情</summary><pre>${safe(JSON.stringify(ev.details))}</pre></details>`
      : "";
    return `<div class="event-item sev-${severity}"><time>${time}</time><i class="dot" title="${safe(ev.severity_name)}"></i><div class="body"><span class="code">${safe(codeLabel)}</span><span class="sev-badge sev-${severity}">${sevLabel}</span>${target}<p>${safe(ev.message)}</p>${details}</div></div>`;
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
    `<span title="该就绪门未通过">${safe(blockerNames[item] || item)}</span>`).join("");

  if (!observations.length) {
    $("target-list").innerHTML = '<tr><td colspan="5" class="empty">等待 target_observations</td></tr>';
    return;
  }
  const selectedId = state.target_id || targets.selected_target_id || "";
  $("target-list").innerHTML = observations.slice().sort((a, b) => a.priority - b.priority)
    .map((item) => {
      const rowClass = `${item.target_id === selectedId || item.selected ? "selected" : ""} ${item.harvest_status === "HARVESTED" ? "harvested" : ""}`;
      const entry = xyzCells((item.candidate || {}).entry_position);
      return `<tr class="${rowClass}">
        <td><b>${safe(item.target_id)}</b></td>
        <td>${chip(item.harvest_status, harvestChip[item.harvest_status] ?? "")}</td>
        <td>${chip(item.tracking_status, trackingChip[item.tracking_status] ?? "")}</td>
        <td>${percent(item.confidence)}</td>
        <td class="mono">${entry[0]}, ${entry[1]}, ${entry[2]}</td>
      </tr>`;
    }).join("");
}

/* ---------- 批次账本（runs/<request_id>/ledger.json 直播） ---------- */

const ledgerChip = {
  SUCCEEDED: "ok", SKIPPED_QUALITY: "warn", SKIPPED_UNREACHABLE: "warn",
  FAILED: "err", CANCELED: "warn",
};
const ledgerNames = {
  SUCCEEDED: "成功", SKIPPED_QUALITY: "跳过·质量", SKIPPED_UNREACHABLE: "跳过·不可达",
  FAILED: "失败", CANCELED: "取消",
};

function fmtDur(sec) {
  if (!numeric(sec)) return "—";
  return `${Number(sec) >= 10 ? Number(sec).toFixed(0) : Number(sec).toFixed(1)}s`;
}

function renderLedger(ledger) {
  const live = (ledger && ledger.live) || {};
  const note = $("ledger-note");
  const rows = live.rows || [];
  if (!live.request_id) {
    note.textContent = "等待批次账本（runs/<request_id>/ledger.json，随终局入账）";
    $("ledger-summary").innerHTML = "";
    $("ledger-body").innerHTML = '<tr><td colspan="6" class="empty">尚未开批</td></tr>';
    return;
  }
  note.textContent = live.error
    ? `${live.request_id} · ${live.error}`
    : `${live.request_id} · ${live.path}`;
  const totals = live.totals || {};
  const pending = Math.max(0, Number(totals.claimed || 0) - Number(totals.attempted || 0));
  $("ledger-summary").innerHTML = [
    ["入账", totals.attempted ?? rows.length],
    ["成功", totals.SUCCEEDED ?? 0],
    ["跳过", (totals.SKIPPED_QUALITY ?? 0) + (totals.SKIPPED_UNREACHABLE ?? 0)],
    ["失败", totals.FAILED ?? 0],
    ["取消", totals.CANCELED ?? 0],
    ["待完成", pending],
  ].map(([label, value]) => `<span>${label} <b>${safe(String(value))}</b></span>`).join("");

  if (!rows.length) {
    $("ledger-body").innerHTML = '<tr><td colspan="6" class="empty">本批尚无终局目标</td></tr>';
    return;
  }
  $("ledger-body").innerHTML = rows.slice().reverse().map((row) => {
    const cls = ledgerChip[row.outcome_name] || "";
    const name = ledgerNames[row.outcome_name] || row.outcome_name || "—";
    const failure = row.failure_code !== undefined && row.failure_code !== null
      ? ` <span class="mono">[${safe(String(row.failure_code))}${row.failure_code_n !== undefined && row.failure_code_n !== null ? `/${failureCodeLabel(row.failure_code_n)}` : ""}]</span>` : "";
    const stages = (row.stages || []);
    const stageChips = stages.length
      ? stages.map((s) => `<span class="phase-chip">${safe(s.name)} ${s.dur_s === null ? "—" : s.dur_s.toFixed(1)}s</span>`).join("")
      : '<p class="empty">无阶段记录</p>';
    const stageTotal = stages.length
      ? ` ${stages.reduce((acc, s) => acc + (Number(s.dur_s) || 0), 0).toFixed(1)}s` : "";
    return `<tr>
      <td><b>${safe(row.target_id)}</b></td>
      <td>${chip(name, cls)}</td>
      <td class="reason-cell">${safe(row.reason || "—")}${failure}</td>
      <td class="mono">${row.elapsed_s === null || row.elapsed_s === undefined ? "—" : Number(row.elapsed_s).toFixed(1)}</td>
      <td class="mono">${row.build_view_count ?? "—"}</td>
      <td><details class="stage-details"><summary>${stages.length} 段${stageTotal}</summary><div class="phase-durations">${stageChips}</div></details></td>
    </tr>`;
  }).join("");
}

/* ---------- 阶段时序（服务器侧 pipeline.fsm / pipeline.arm） ---------- */

function timelineTime(t) {
  return numeric(t) && t > 0
    ? new Date(t * 1000).toLocaleTimeString("zh-CN", {hour12: false}) : "--:--:--";
}

function renderStages(pipelineSection) {
  const fsm = (pipelineSection && pipelineSection.fsm) || [];
  const arm = (pipelineSection && pipelineSection.arm) || [];
  setText("fsm-count", `${fsm.length} 条`);
  setText("arm-count", `${arm.length} 条`);

  if (!fsm.length) {
    $("fsm-list").innerHTML = '<p class="empty">等待调度状态转移</p>';
  } else {
    $("fsm-list").innerHTML = fsm.slice().reverse().slice(0, 40).map((item) => {
      const batch = batchNames[item.batch_state] || item.batch_state;
      const phase = phaseNames[item.target_phase] || item.target_phase;
      const flags = [
        item.recovery_required ? "恢复" : null,
        item.execution_enabled ? null : "执行关",
        item.grasp_enabled ? null : "抓取关",
      ].filter(Boolean).join("·") || null;
      const target = item.target_id ? ` <span class="target">[${safe(item.target_id)}]</span>` : "";
      return `<div class="tl-item${item.current ? " current" : ""}">
        <time>${timelineTime(item.t)}</time>
        <div class="tl-main"><span class="tl-pill">${safe(batch)} → ${safe(phase)}</span>${target}
        ${flags ? `<span class="tl-flag">${safe(flags)}</span>` : ""}
        <p>${safe(item.message || "")}</p></div>
        <b class="tl-dur">${fmtDur(item.dur_s)}</b></div>`;
    }).join("");
  }

  if (!arm.length) {
    $("arm-list").innerHTML = '<p class="empty">等待技能状态（/peach_arm/status）</p>';
  } else {
    $("arm-list").innerHTML = arm.slice().reverse().slice(0, 40).map((item) => {
      const target = item.target_id ? ` <span class="target">[${safe(item.target_id)}]</span>` : "";
      return `<div class="tl-item${item.current ? " current" : ""}">
        <time>${timelineTime(item.t)}</time>
        <div class="tl-main"><span class="state-pill ${pillClass(item.state)}">${safe(item.state || "—")}</span>${target}${item.recovery ? '<span class="tl-flag">恢复</span>' : ""}
        <p>${safe(item.message || "")}</p></div>
        <b class="tl-dur">${fmtDur(item.dur_s)}</b></div>`;
    }).join("");
  }
}

/* ---------- 感知节拍 / 重建进度 ---------- */

function metricChip(label, value, cls = "") {
  return value === null || value === undefined || value === ""
    ? "" : `<span class="${cls}">${label} <b>${safe(String(value))}</b></span>`;
}

function renderVision(state) {
  const harvest = state.perception?.harvest || {};
  const perBody = $("perception-metrics");
  if (!harvest || !Object.keys(harvest).length) {
    perBody.innerHTML = '<p class="empty">等待 /peach/perception/harvest_state</p>';
  } else {
    const timing = harvest.timing || {};
    const fpsHot = numeric(timing.fps) && Number(timing.fps) < 2.0 ? "hot" : "";
    const dropped = (harvest.dropped_target_ids || []).length;
    const stale = (harvest.anchor_stale_target_ids || []).length;
    perBody.innerHTML = [
      metricChip("FPS", numeric(timing.fps) ? Number(timing.fps).toFixed(2) : "—", fpsHot),
      metricChip("检测", numeric(timing.detect_ms) ? `${Number(timing.detect_ms).toFixed(0)}ms` : "—"),
      metricChip("分割", numeric(timing.segment_ms) ? `${Number(timing.segment_ms).toFixed(0)}ms` : "—"),
      metricChip("几何", numeric(timing.geometry_ms) ? `${Number(timing.geometry_ms).toFixed(0)}ms` : "—"),
      metricChip("整帧", numeric(timing.total_ms) ? `${Number(timing.total_ms).toFixed(0)}ms` : "—"),
      metricChip("光照", harvest.lighting),
      metricChip("低光", harvest.low_light_quality === true ? "是" : (harvest.low_light_quality === false ? "否" : null), harvest.low_light_quality === true ? "hot" : ""),
      metricChip("掉锚", dropped || null, dropped ? "hot" : ""),
      metricChip("陈旧锚", stale || null, stale ? "hot" : ""),
      metricChip("epoch", harvest.scene_epoch ?? null),
      metricChip("收齐/待收", harvest.target_count !== undefined ? `${harvest.collecting_count ?? "—"}/${harvest.pending_count ?? "—"}` : null),
    ].filter(Boolean).join("") || '<p class="empty">等待节拍字段</p>';
  }

  const diag = state.reconstruction?.diagnostics || {};
  const decision = state.reconstruction?.grasp_decision || {};
  const reconBody = $("recon-metrics");
  if (!diag || !Object.keys(diag).length) {
    reconBody.innerHTML = '<p class="empty">等待 /peach/reconstruction/diagnostics</p>';
  } else {
    const coverage = diag.view_coverage || {};
    const tfFail = Number(diag.tf_failures ?? 0);
    const serverNow = state.system?.server_time || 0;
    const validLeft = numeric(decision.valid_until) && decision.valid_until > 0 && serverNow
      ? decision.valid_until - serverNow : null;
    const validText = decision.allowed === undefined
      ? null
      : (validLeft === null
        ? (decision.allowed ? "允许" : "未许可")
        : `${decision.allowed ? "允许" : "未许可"} · 剩 ${validLeft > 0 ? validLeft.toFixed(1) : "0.0"}s`);
    reconBody.innerHTML = [
      metricChip("状态", diag.state || "—"),
      metricChip("目标", diag.target_id || diag.selected_target_id || null),
      metricChip("机位", Number(diag.captured_views ?? -1) >= 0 ? diag.captured_views : "—"),
      metricChip("拒帧", Number(diag.rejected_views ?? -1) > 0 ? diag.rejected_views : null,
        Number(diag.rejected_views ?? 0) > 0 ? "hot" : ""),
      metricChip("TF 失败", tfFail > 0 ? tfFail : null, tfFail > 0 ? "hot" : ""),
      metricChip("TF 延迟", numeric(diag.tf_latency_ms) ? `${Number(diag.tf_latency_ms).toFixed(0)}ms` : "—"),
      metricChip("最大基线", numeric(coverage.max_baseline_deg) ? `${Number(coverage.max_baseline_deg).toFixed(1)}°` : "—"),
      metricChip("许可", validText, decision.allowed ? "" : "hot"),
      metricChip("原因", decision.reason || null),
    ].filter(Boolean).join("") || '<p class="empty">等待重建心跳</p>';
  }
}

/* ---------- 系统负载与参数镜像 ---------- */

function renderSystem(state) {
  const sample = state.metrics?.sample || {};
  const gpu = sample.gpu || {};
  setText("system-summary", numeric(sample.cpu_percent)
    ? `CPU ${Number(sample.cpu_percent).toFixed(0)}% · 内存 ${Math.round(Number(sample.memory_used_mb || 0))}/${Math.round(Number(sample.memory_total_mb || 0))} MB${gpu.utilization_percent !== undefined ? ` · GPU ${Number(gpu.utilization_percent).toFixed(0)}%` : ""}`
    : "—");
  $("system-metrics").innerHTML = [
    metricChip("CPU", numeric(sample.cpu_percent) ? `${Number(sample.cpu_percent).toFixed(0)}%` : "—",
      numeric(sample.cpu_percent) && Number(sample.cpu_percent) > 85 ? "hot" : ""),
    metricChip("内存", numeric(sample.memory_used_mb)
      ? `${Number(sample.memory_used_mb).toFixed(0)}/${Number(sample.memory_total_mb).toFixed(0)} MB` : "—"),
    metricChip("load1", sample.load1),
    metricChip("GPU", gpu.utilization_percent !== undefined && gpu.utilization_percent !== null
      ? `${Number(gpu.utilization_percent).toFixed(0)}% · ${Number(gpu.memory_used_mb || 0).toFixed(0)} MB` : null),
    ...(sample.processes || []).map((p) => metricChip(
      p.name, numeric(p.cpu_percent)
        ? `cpu ${Number(p.cpu_percent).toFixed(0)}% · ${Number(p.rss_mb || 0).toFixed(0)} MB` : "—")),
  ].filter(Boolean).join("") || '<p class="empty">等待性能采样</p>';

  const params = state.params || {};
  const nodes = Object.keys(params).filter((n) => params[n] && Object.keys(params[n]).length);
  $("params-mirror").innerHTML = nodes.length
    ? nodes.map((node) => {
      const rows = Object.entries(params[node]).map(([k, v]) =>
        `<div class="node-row"><span>${safe(k)}</span><b>${safe(String(v))}</b></div>`).join("");
      return `<div class="param-node"><h3>${safe(node)}</h3>${rows}</div>`;
    }).join("")
    : '<p class="empty">等待参数轮询</p>';

  const report = state.selfcheck?.report;
  const selfcheckRows = report
    ? `<div class="param-node"><h3>${safe(report.status || "—")} · ${safe(report.checked_at || "")}</h3>` +
      `<div class="node-row"><span>结论</span><b>${safe(report.summary || "")}</b></div>` +
      (report.checks || []).map((c) =>
        `<div class="node-row"><span>${safe(c.name)}</span>` +
        `<b class="${pillClass(c.status)}">${safe(c.status)}${c.detail ? " · " + safe(c.detail) : ""}</b></div>`).join("") +
      `</div>`
    : "";
  $("selfcheck-panel").innerHTML = selfcheckRows || '<p class="empty">等待首次自检</p>';

  // 启动事实摘要（startup.json 同源；回答"当前跑的什么配置"）
  const facts = state.startup?.facts || {};
  const factsRow = [
    facts.hardware_mode ? `模式 ${String(facts.hardware_mode) === "mock" ? "mock" : "真机"}` : null,
    facts.tool_profile ? `末端 ${safe(facts.tool_profile)}` : null,
    facts.camera_enabled !== undefined ? `相机 ${String(facts.camera_enabled) === "true" ? "开" : "关"}` : null,
    (facts.git && facts.git.commit && facts.git.commit !== "unknown") ? `git ${safe(String(facts.git.commit).slice(0, 8))}` : null,
    facts.ros_domain_id !== undefined && facts.ros_domain_id !== "" ? `域 ${safe(String(facts.ros_domain_id))}` : null,
  ].filter(Boolean);
  const startupEl = $("startup-facts");
  if (startupEl) {
    startupEl.hidden = !factsRow.length;
    startupEl.innerHTML = factsRow.map((text) => `<span>${safe(text)}</span>`).join("");
  }

  const diagNodes = state.diagnostics?.nodes || {};
  const levelText = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"};
  const diagRows = Object.entries(diagNodes).map(([node, entry]) =>
    `<div class="param-node"><h3 class="${pillClass(levelText[entry.level] || "")}">${safe(node)}</h3>` +
    Object.entries(entry.tasks || {}).map(([name, task]) =>
      `<div class="node-row"><span>${safe(name)}</span>` +
      `<b class="${pillClass(levelText[task.level] || "")}">${safe(levelText[task.level] || task.level)} · ${safe(task.message || "")}</b></div>`).join("") +
    `</div>`).join("");
  $("diagnostics-panel").innerHTML = diagRows || '<p class="empty">等待诊断聚合</p>';
}

function freshnessHtml(age) {
  if (age === undefined) return ["", "无数据"];
  if (age > 15) return ["err", `${age.toFixed(0)}s 前`];
  if (age > 5) return ["warn", `${age.toFixed(1)}s 前`];
  return ["ok", `${age.toFixed(1)}s 前`];
}

function pillClass(text) {
  const value = String(text || "").toUpperCase();
  if (/FAIL|ERROR|FAULT|RECOVERY/.test(value)) return "err";
  if (/WARN|PAUSE|REOBSERVE/.test(value)) return "warn";
  if (/READY|RUNNING|COMPLETE|SUCCEED|IDLE/.test(value)) return "ok";
  return "";
}

const yesNo = (value, yes = "是", no = "否") =>
  value === true || value === 1 ? yes : value === false || value === 0 ? no : "—";

function nodeCard(label, age, rows) {
  const [cls, fresh] = freshnessHtml(age);
  const body = rows.map(([k, v]) =>
    `<div class="node-row"><span>${safe(k)}</span><b>${v}</b></div>`).join("");
  return `<article class="node-card"><div class="node-head"><h2>${safe(label)}</h2>
    <span class="freshness ${cls}"><i></i><b>${safe(fresh)}</b></span></div>
    <div class="node-body">${body}</div></article>`;
}

function renderStatus(state) {
  const ages = state.system?.topic_age_s || {};
  const executor = state.task_executor?.state || {};
  $("policy-badges").innerHTML = [
    ["execution_enabled", "执行"],
    ["grasp_enabled", "抓取"],
    ["tool_enabled", "工具"],
  ].map(([key, label]) => {
    const on = executor[key] === true;
    return `<span class="badge ${on ? "on" : "off"}">${label} ${on ? "启" : "停"}</span>`;
  }).join("");

  const targets = state.perception?.targets || {};
  const harvest = state.perception?.harvest || {};
  const count = targets.target_count ?? harvest.target_count ?? "—";
  const lockedKnown = targets.target_set_locked !== undefined
    || harvest.target_set_locked !== undefined;
  const lockedLabel = lockedKnown
    ? (targets.target_set_locked === true || harvest.target_set_locked === true
      ? "已锁定" : "收齐中") : "—";
  const diag = state.reconstruction?.diagnostics || {};
  const reconState = diag.state || state.reconstruction?.status?.state ||
    state.reconstruction?.status?.text || "—";
  const decision = state.reconstruction?.grasp_decision || {};
  const graspText = decision.allowed === true
    ? "允许" : (decision.reason || (decision.allowed === false ? "未许可" : "—"));
  const manipulation = state.manipulation?.status || {};
  const robot = state.robot?.status || {};

  $("status-strip").innerHTML = [
    nodeCard("感知", ages["perception.targets"], [
      ["目标 / 锁定", `${safe(String(count))} · ${safe(String(lockedLabel))}`],
      ["选中", safe(targets.selected_target_id || harvest.selected_target_id || "—")],
    ]),
    nodeCard("重建", ages["reconstruction.diagnostics"], [
      ["状态", `<span class="state-pill ${pillClass(reconState)}">${safe(reconState)}</span>`],
      ["许可", safe(graspText)],
    ]),
    nodeCard("技能", ages["manipulation.status"], [
      ["周期", `<span class="state-pill ${pillClass(manipulation.state)}">${safe(manipulation.state || "—")}</span>`],
      ["执行", `${yesNo(manipulation.execution_enabled, "ON", "OFF")} / ${yesNo(manipulation.execution_armed, "ARM", "SAFE")}`],
    ]),
    nodeCard("调度", ages["task_executor.state"], [
      ["批次", `<span class="state-pill ${pillClass(batchNames[executor.batch_state])}">${safe(batchNames[executor.batch_state] || "—")}</span>`],
      ["动作中", yesNo(executor.action_active)],
    ]),
    nodeCard("机械臂", ages["robot.status"], [
      ["上电 / 急停", `${yesNo(robot.drives_powered, "已上电", "未上电")} / ${yesNo(robot.e_stopped, "急停", "正常")}`],
      ["可动 / 故障", `${yesNo(robot.motion_possible, "可接轨", "不可")} / ${yesNo(robot.in_error, "故障", "正常")}`],
    ]),
  ].join("");
  renderHardware(state);
}

function radDeg(value) {
  return numeric(value) ? (Number(value) * 180 / Math.PI).toFixed(2) : "—";
}

function rpyDeg(quat) {
  if (!Array.isArray(quat) || quat.length < 4 || quat.some((item) => !numeric(item))) {
    return null;
  }
  const x = Number(quat[0]); const y = Number(quat[1]);
  const z = Number(quat[2]); const w = Number(quat[3]);
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw].map((rad) => (rad * 180 / Math.PI).toFixed(1));
}

function renderHardware(state) {
  const robot = state.robot?.status || {};
  const tcp = state.robot?.tcp || {};
  const joints = state.robot?.joints || {};
  const ages = state.system?.topic_age_s || {};
  const flags = [
    [robot.drives_powered === 1 ? "已上电" : "未上电", robot.drives_powered === 1, robot.drives_powered === 0],
    [robot.e_stopped === 1 ? "急停" : "无急停", robot.e_stopped === 0, robot.e_stopped === 1],
    [robot.motion_possible === 1 ? "可接轨" : "不可接轨", robot.motion_possible === 1, robot.motion_possible === 0],
    [robot.in_motion === 1 ? "运动中" : "静止", robot.in_motion === 1, false],
    [robot.in_error === 1 ? "故障" : "无故障", robot.in_error === 0, robot.in_error === 1],
  ];
  $("hw-flags").innerHTML = flags.map(([label, ok, bad]) =>
    `<span class="badge ${ok ? "on" : (bad ? "off" : "")}">${safe(label)}</span>`
  ).join("") +
    `<span class="badge">错误码 <b>${safe(String(robot.error_code ?? "—"))}</b></span>` +
    `<span class="badge">柜侧 ${safe(freshnessHtml(ages["robot.status"])[1])}</span>` +
    `<span class="badge">关节 ${safe(freshnessHtml(ages["robot.joints"])[1])}</span>`;

  const xyz = Array.isArray(tcp.xyz) && tcp.xyz.length >= 3
    ? tcp.xyz.map((value) => Number(value).toFixed(3)) : null;
  const rpy = rpyDeg(tcp.quat);
  const pose = [];
  if (xyz) {
    pose.push(`TCP x <b>${xyz[0]}</b>`, `y <b>${xyz[1]}</b>`, `z <b>${xyz[2]}</b>`);
  } else {
    pose.push(`TCP <b>${tcp.tf_ok === false ? "TF 不可用" : "—"}</b>`);
  }
  if (rpy) {
    pose.push(`rpy <b>${rpy[0]}, ${rpy[1]}, ${rpy[2]}</b>`);
  }
  $("hw-pose").innerHTML = pose.map((item) => `<span>${item}</span>`).join("");

  const rows = joints.rows || [];
  if (!rows.length) {
    $("hw-joint-body").innerHTML =
      '<tr><td colspan="7" class="empty">等待 /joint_states 与 /aubo_io_controller/joint_status</td></tr>';
    return;
  }
  $("hw-joint-body").innerHTML = rows.map((row) => {
    const err = Number(row.error_code || 0);
    const name = String(row.name || "").replace(/_joint$/, "");
    return `<tr class="${err ? "alert-row" : ""}">
      <td><b>${safe(name)}</b></td>
      <td class="mono">${radDeg(row.position)}</td>
      <td class="mono">${radDeg(row.velocity)}</td>
      <td class="mono">${numeric(row.current) ? Number(row.current).toFixed(1) : "—"}</td>
      <td class="mono">${numeric(row.temperature) ? Number(row.temperature).toFixed(1) : "—"}</td>
      <td class="mono">${numeric(row.following_error) ? Number(row.following_error).toFixed(4) : "—"}</td>
      <td>${err ? `<span class="status-chip err">${err}</span>` : "0"}</td>
    </tr>`;
  }).join("");
}

const LANDMARKS = [
  ["perception_entry", "感知入口", [63, 191, 114], 4],
  ["reconstruction_center", "重建中心", [196, 125, 255], 4],
  ["grasp_pregrasp", "预抓取", [224, 169, 62], 5],
  ["grasp_entry", "抓取入口", [224, 92, 92], 5],
];
const AXIS_NAME = ["X", "Y", "Z"];
const PHASE_RGB = {
  2: [79, 163, 224],
  5: [224, 169, 62],
  6: [196, 125, 255],
  7: [64, 191, 115],
};

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

function principalAxes(points) {
  const span = [0, 0, 0];
  if (!points.length) return [0, 1];
  for (let axis = 0; axis < 3; axis += 1) {
    let min = Infinity;
    let max = -Infinity;
    points.forEach((xyz) => {
      const value = xyz[axis];
      if (value < min) min = value;
      if (value > max) max = value;
    });
    span[axis] = max - min;
  }
  const order = [0, 1, 2].sort((a, b) => span[b] - span[a]);
  return [order[0], order[1]];
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
    ["Δz", numeric(metrics.dz_m) ? `${Number(metrics.dz_m).toFixed(3)} m` : "—"],
  ];
  $("traj-metrics").innerHTML = chips.map(([label, value]) => {
    const hot = label === "绕行比" && numeric(ratio) && Number(ratio) >= 1.4 ? " hot" : "";
    return `<span class="${hot}">${label} <b>${safe(String(value))}</b></span>`;
  }).join("");
}

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
  const phases = Array.isArray(payload && payload.phase) ? payload.phase : [];
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
  const [ii, jj] = principalAxes(scene);
  const title = $("traj-title");
  if (title) title.textContent = `末端轨迹（${AXIS_NAME[ii]}–${AXIS_NAME[jj]}）`;
  const pad = 34;
  let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity;
  scene.forEach((xyz) => {
    minU = Math.min(minU, xyz[ii]); maxU = Math.max(maxU, xyz[ii]);
    minV = Math.min(minV, xyz[jj]); maxV = Math.max(maxV, xyz[jj]);
  });
  const spanU = Math.max(maxU - minU, 0.2);
  const spanV = Math.max(maxV - minV, 0.2);
  const scale = Math.min((width - pad * 2) / spanU, (height - pad * 2) / spanV);
  const toPx = (xyz) => [
    pad + (xyz[ii] - minU) * scale + (width - pad * 2 - spanU * scale) / 2,
    height - pad - (xyz[jj] - minV) * scale - (height - pad * 2 - spanV * scale) / 2,
  ];

  ctx.strokeStyle = "rgba(42,51,61,0.9)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  const origin = toPx([0, 0, 0]);
  ctx.moveTo(0, origin[1]); ctx.lineTo(width, origin[1]);
  ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], height);
  ctx.stroke();

  if (points.length >= 2) {
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    let current = -1;
    for (let index = 0; index < points.length; index += 1) {
      const phase = Number.isFinite(Number(phases[index])) ? Number(phases[index]) : 0;
      const [px, py] = toPx(points[index]);
      if (index === 0 || phase !== current) {
        if (index > 0) ctx.stroke();
        current = phase;
        const rgb = PHASE_RGB[phase] || [222, 230, 236];
        ctx.strokeStyle = `rgba(${rgb.join(",")},0.95)`;
        ctx.beginPath();
        if (index > 0) {
          const prev = toPx(points[index - 1]);
          ctx.moveTo(prev[0], prev[1]);
          ctx.lineTo(px, py);
        } else {
          ctx.moveTo(px, py);
        }
      } else {
        ctx.lineTo(px, py);
      }
    }
    ctx.stroke();
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
    monitorState = state;
    const record = state.record?.info || {};
    setText("record-dir", record.enabled === false
      ? "记录已关闭" : record.directory || "等待首批数据");
    $("record-strip").classList.toggle("off",
      record.enabled === false || !record.directory);
    renderStatus(state);
    renderFlow(state.task_executor || {}, state.job || {});
    renderPlan(state.perception || {}, state.task_executor || {});
    renderStages(state.pipeline || {});
    renderLedger(state.ledger || {});
    renderVision(state);
    renderSystem(state);
    renderDebugPanel(state.debug || {});
    $("connection").className = "connection online";
    $("connection").querySelector("span").textContent = "已连接";
  } catch (_) {
    $("connection").className = "connection offline";
    $("connection").querySelector("span").textContent = "连接中断";
  }
}

setInterval(pollState, 1000);
setInterval(pollTrajectory, 400);
window.addEventListener("resize", pollTrajectory);
pollState();
pollTrajectory();

const debugView = {results: []};

function switchView(name) {
  document.querySelectorAll(".view-tabs button").forEach((el) => {
    el.classList.toggle("active", el.dataset.view === name);
  });
  $("view-monitor").hidden = name !== "monitor";
  $("view-debug").hidden = name !== "debug";
}

async function debugPost(action, payload, confirmText) {
  if (confirmText && !window.confirm(confirmText)) {
    return null;
  }
  const response = await fetch(`/api/debug/${encodeURIComponent(action)}`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
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
  $("debug-recent-count").textContent = `${debugView.results.length} 条`;
}

function renderDebugPanel(debug) {
  const gates = $("debug-gates");
  if (!gates) return;
  const enabled = debug.enabled === true;
  const motion = debug.motion_enabled === true;
  gates.innerHTML = `
    <span class="debug-gate ${enabled ? "ok" : "err"}">调试 ${enabled ? "开" : "关（debug.enabled=false）"}</span>
    <span class="debug-gate ${motion ? "warn" : "ok"}">动臂 ${motion ? "已放行" : "未放行（423）"}</span>`;
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
      const outcome = await debugPost(action, payload, el.dataset.confirm === "1"
        ? `确认发送 ${action}？\n${JSON.stringify(payload, null, 2)}`
        : null);
      if (outcome && outcome.status === 423) {
        window.alert("423：动臂未放行（yaml debug.motion_enabled: true）");
      }
    });
  });
  document.querySelectorAll("[data-debug-cancel]").forEach((el) => {
    el.addEventListener("click", () => debugPost("cancel", {target: el.dataset.debugCancel}, null));
  });
}

function buildDebugPayload(action, el) {
  const requestId = $("dbg-request-id").value.trim();
  const sceneKey = $("dbg-scene-key").value.trim() || "lab";
  const targetId = $("dbg-target-id").value.trim();
  if (action === "run_harvest_action") {
    if (!requestId) { window.alert("request_id 必填（同时是账本目录名，不得复用）"); return null; }
    const intent = $("run-intent").value;
    const targets = $("run-target-ids").value.split(",").map((s) => s.trim()).filter(Boolean);
    return {request_id: requestId, scene_key: sceneKey, intent,
      selection_mode: targets.length ? "MANUAL" : "AUTO", target_ids: targets};
  }
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
  if (action === "begin_scene_service") {
    return {request_id: requestId || "dev", scene_key: sceneKey};
  }
  if (action === "survey_action") {
    return {request_id: requestId || "dev", scene_key: sceneKey};
  }
  if (action === "build_action") {
    if (!targetId) { window.alert("target_id 必填"); return null; }
    return {request_id: requestId || "dev", target_id: targetId,
      scene_epoch: Number($("dbg-epoch").value) || 0};
  }
  if (action === "execute_action") {
    if (!targetId) { window.alert("target_id 必填"); return null; }
    return {request_id: requestId || "dev", target_id: targetId,
      mode: $("exec-mode").value, skip_observation: $("exec-skip-obs").checked};
  }
  const inline = el.dataset.payload ? JSON.parse(el.dataset.payload) : null;
  return inline && typeof inline === "object" ? inline : {};
}

bindDebugControls();
pollTrajectory();
