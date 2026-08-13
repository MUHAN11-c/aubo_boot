"use strict";

const $ = (id) => document.getElementById(id);
const numeric = (value) => value !== null && value !== undefined && value !== '' &&
  Number.isFinite(Number(value));
const fmt = (value, digits = 3, suffix = "") => numeric(value)
  ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const intFmt = (value) => numeric(value) ? Number(value).toLocaleString() : "0";
const percent = (value) => numeric(value) ? `${(Number(value) * 100).toFixed(1)}%` : "—";
const vector = (value, digits = 4) => Array.isArray(value) && value.length
  ? `[${value.map((item) => numeric(item) ? Number(item).toFixed(digits) : "—").join(", ")}]`
  : "—";
const safe = (value) => String(value ?? "—").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
})[char]);
// 批次状态与目标阶段枚举映射，与 peach_harvest_msgs/HarvestState.msg 常量保持一致
const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式", "已完成", "故障", "需要恢复", "已中断"];
const phaseNames = ["空闲", "选择目标", "观测中", "完成观测", "质量校验", "靠近中", "工具动作", "撤退中", "收尾中", "目标成功", "目标跳过", "目标失败"];
// 过程线阶段顺序（与 index.html data-stage 一致）
const stageOrder = ["ready", "photo", "lock", "observe", "validate", "approach", "tool", "retreat", "done"];

// 由编排器状态推导当前过程线阶段：拍照前置/收齐在 DISCOVERY 内按消息区分；
// 目标周期内按 target_phase 映射；终局/异常单独归类。
function currentStage(state) {
  const batch = Number(state.batch_state ?? -1);
  const phase = Number(state.target_phase ?? 0);
  const message = String(state.message || "");
  if (batch === 6) return "done";
  if (batch === 0) return "ready";
  if (batch === 1) return message.includes("拍照") ? "photo" : "lock";
  if (batch === 2 || batch === 3) {
    if (phase >= 1 && phase <= 3) return "observe";
    if (phase === 4) return "validate";
    if (phase === 5) return "approach";
    if (phase === 6) return "tool";
    if (phase === 7 || phase === 8) return "retreat";
    return "observe";
  }
  return "ready";
}

function renderPipeline(state) {
  const active = currentStage(state);
  const activeIndex = stageOrder.indexOf(active);
  const batch = Number(state.batch_state ?? -1);
  const alert = batch === 7 || batch === 8 || state.recovery_required === true;
  document.querySelectorAll("#pipeline .step").forEach((el) => {
    const index = stageOrder.indexOf(el.dataset.stage);
    el.classList.toggle("done", index >= 0 && index < activeIndex);
    el.classList.toggle("active", index === activeIndex);
    el.classList.toggle("alert", index === activeIndex && alert);
  });
}

function renderOperations(orchestration) {
  const state = orchestration.state || {};
  setText("batch-state", batchNames[state.batch_state] || "等待编排器");
  $("batch-state").classList.toggle("completed", state.batch_state === 6);
  setText("batch-message", state.message || "尚未收到类型化状态");
  setText("target-phase", phaseNames[state.target_phase] || "—");
  setText("batch-progress-value", percent(state.progress));
  setBar("batch-progress-bar", state.progress);
  $("batch-blockers").innerHTML = (state.blockers || []).map((item) =>
    "<span title=\"该健康门未通过\">" + safe(item) + "</span>").join("");
  renderPipeline(state);
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

function qualityClass(status) {
  return String(status || "").toLowerCase();
}

function setText(id, value) {
  $(id).textContent = value ?? "—";
}

function setBar(id, value) {
  const width = numeric(value) ? Math.max(0, Math.min(100, Number(value) * 100)) : 0;
  $(id).style.width = `${width}%`;
}

function renderIdentity(targets, diagnostics, refined, decision) {
  const perceptionId = targets.selected_target_id || "";
  const reconstructionId = diagnostics.target_id || decision.target_id || "";
  const refinedId = refined.pose?.candidates?.[0]?.target_id || "";
  setText("perception-id", perceptionId || "—");
  setText("reconstruction-id", reconstructionId || "—");
  setText("refined-id", refinedId || "等待 finalize");
  const bus = $("identity-bus");
  bus.classList.remove("match", "mismatch");
  if (!perceptionId || !reconstructionId) {
    setText("identity-result", "链路未建立");
    setText("identity-hint", "等待感知选中与重建绑定");
    return;
  }
  const baseMatch = perceptionId === reconstructionId;
  const refinedMatch = !refinedId || refinedId === reconstructionId;
  if (baseMatch && refinedMatch) {
    bus.classList.add("match");
    setText("identity-result", refinedId ? "三级 ID 一致" : "绑定一致 · 等待精化");
    setText("identity-hint", refinedId ? "可追溯链路完整" : "当前重建没有串目标");
  } else {
    bus.classList.add("mismatch");
    setText("identity-result", "目标 ID 不一致");
    setText("identity-hint", "必须停止融合并核对绑定");
  }
}

function renderTargets(targets) {
  const observations = targets.observations || [];
  setText("target-count", targets.target_count ?? observations.length);
  setText("snapshot-id", `SNAP ${targets.snapshot_id ?? "—"}`);
  setText("completed-count", `DONE ${observations.filter((item) => item.harvest_status === "HARVESTED").length}`);
  setText("run-id", targets.harvest_run_id || "等待目标计划");
  $("plan-lock").textContent = targets.target_set_locked ? "计划已锁定" : "未锁定";
  $("plan-lock").className = `tag ${targets.target_set_locked ? "locked" : "neutral"}`;
  if (!observations.length) {
    $("target-list").innerHTML = '<tr><td colspan="5" class="empty">等待 target_observations</td></tr>';
    return null;
  }
  $("target-list").innerHTML = observations.slice().sort((a, b) => a.priority - b.priority)
    .map((item) => {
      const quality = item.fitting?.status || item.candidate?.status || "—";
      const rowClass = `${item.selected ? "selected" : ""} ${item.harvest_status === "HARVESTED" ? "harvested" : ""}`;
      return `<tr class="${rowClass}">
        <td class="rank">#${safe(item.priority)}</td>
        <td class="target-name"><b>${safe(item.target_id)}</b><span>${safe(item.harvest_status)} · ${safe(item.tracking_status)}</span></td>
        <td><span class="quality ${qualityClass(quality)}">${safe(quality)}</span></td>
        <td class="number">${fmt(item.camera_distance_m, 2, " m")}</td>
        <td class="number">${percent(item.fitting?.valid_depth_ratio)}</td>
      </tr>`;
    }).join("");
  return observations.find((item) => item.selected) || null;
}

function renderSelected(selected, frameId) {
  const candidate = selected?.candidate || {};
  const fitting = selected?.fitting || {};
  const position = candidate.entry_position || [];
  const quaternion = candidate.entry_quaternion_xyzw || [];
  setText("tracking-state", selected?.tracking_status || "NO DATA");
  setText("quality-state", fitting.status || candidate.status || "—");
  $("quality-state").className = qualityClass(fitting.status || candidate.status);
  setText("pose-frame", `FRAME ${frameId || "—"}`);
  ["x", "y", "z"].forEach((axis, index) => setText(`entry-${axis}`, fmt(position[index], 5)));
  ["x", "y", "z", "w"].forEach((axis, index) => setText(`quat-${axis}`, fmt(quaternion[index], 6)));
  setText("translation-vector", vector(candidate.translation_direction));
  setText("bottom-vector", vector(candidate.bag_bottom));
  setText("neck-vector", vector(candidate.bag_neck));
  setText("camera-distance", fmt(selected?.camera_distance_m, 3, " m"));
  setText("confidence", percent(selected?.confidence));
  setText("diameter", fmt(candidate.diameter_m || fitting.diameter_m, 4, " m"));
  setText("travel", fmt(candidate.travel_m, 4, " m"));
  setText("depth-ratio", percent(fitting.valid_depth_ratio));
  setText("fit-points", intFmt(fitting.n_points));
  setText("axis-confidence-value", percent(fitting.axis_confidence));
  setText("inlier-value", percent(fitting.inlier_ratio));
  setBar("axis-confidence-bar", fitting.axis_confidence);
  setBar("inlier-bar", fitting.inlier_ratio);
  const flags = [...new Set([
    ...(selected?.diagnostic_flags || []),
    ...(candidate.diagnostic_flags || []),
    ...(fitting.diagnostic_flags || [])
  ])];
  $("target-flags").innerHTML = flags.length
    ? flags.map((flag) => `<span class="flag">${safe(flag)}</span>`).join("")
    : '<span class="flag neutral">暂无诊断标志</span>';
  setText("strategy-id", candidate.strategy_id || "—");
  setText("model-version", candidate.model_version || "—");
  setText("calibration-version", candidate.calibration_version || "—");
  setText("tool-version", candidate.tool_version || "—");
}

function renderReconstruction(reconstruction, refined) {
  const diagnostics = reconstruction.diagnostics || {};
  const status = diagnostics.state || reconstruction.status?.state || reconstruction.status?.text || "IDLE";
  setText("recon-state", status);
  $("recon-state").className = `state-pill ${String(status).toLowerCase()}`;
  setText("captured-views", intFmt(diagnostics.captured_views));
  setText("rejected-views", intFmt(diagnostics.rejected_views));
  setText("tf-failures", intFmt(diagnostics.tf_failures));
  setText("tf-latency", fmt(diagnostics.tf_latency_ms, 2, " ms"));
  setText("target-center", vector(diagnostics.target_center_base));
  setText("relative-translation", fmt(diagnostics.last_rel_translation_m, 4, " m"));
  setText("relative-rotation", fmt(diagnostics.last_rel_rotation_deg, 2, "°"));
  setText("mask-cache", intFmt(diagnostics.target_mask_cache_size));

  const registration = diagnostics.registration || {};
  const latest = registration.latest || {};
  setText("reg-mode", String(latest.mode || "WARMUP").toUpperCase());
  setText("reg-reason", latest.reason || "等待模型");
  setText("reg-accepted", `${intFmt(registration.accepted)} ACCEPTED`);
  setText("fitness", fmt(latest.fitness, 4));
  setText("rmse", fmt(latest.rmse_m, 5, " m"));
  setText("correction-translation", fmt(latest.translation_m, 5, " m"));
  setText("correction-rotation", fmt(latest.rotation_deg, 3, "°"));

  const tsdf = diagnostics.tsdf || {};
  setText("tsdf-status", diagnostics.tsdf ? "ONLINE" : "NO DATA");
  setText("tsdf-points", intFmt(tsdf.points));
  setText("integrated-frames", intFmt(tsdf.integrated_frames));
  setText("voxel-length", fmt(tsdf.voxel_length, 4, " m"));
  setText("integrate-time", fmt(tsdf.integrate_time_s, 3, " s"));
  setText("roi-center", vector(tsdf.roi_center));
  setText("cloud-points", intFmt(diagnostics.cloud_points));

  const decision = reconstruction.grasp_decision || diagnostics.grasp_decision || {};
  const allowed = decision.allowed === true;
  $("decision-card").classList.toggle("allowed", allowed);
  setText("decision-title", allowed ? "视觉允许抓取" : "禁止抓取");
  setText("decision-reason", decision.reason || "reconstruction_not_ready");
  document.querySelector(".decision-symbol").textContent = allowed ? "✓" : "×";
  const refinedFit = refined.diagnostics?.fittings?.[0] || {};
  const refinedCandidate = refined.pose?.candidates?.[0] || {};
  setText("refined-diameter", fmt(decision.diameter_m ?? refinedFit.diameter_m, 4, " m"));
  setText("refined-rmse", fmt(decision.rmse_m ?? refinedFit.cylinder_rms_m ?? refinedFit.sphere_rms_m, 5, " m"));
  setText("refined-inlier", percent(decision.inlier_ratio ?? refinedFit.inlier_ratio));
  setText("refined-entry", vector(decision.entry || refinedCandidate.entry_position));
  setText("refined-axis", vector(decision.axis || refined.axis?.xyz));
  return {diagnostics, decision};
}

function renderApproach(approach) {
  const status = approach.status || {};
  const quality = status.quality || {};
  const state = status.state || "IDLE";
  setText("approach-state", state);
  $("approach-state").className = `state-pill ${String(state).toLowerCase()}`;
  setText("approach-message", status.message || "等待节点");
  setText("approach-target", status.target_id || quality.selected_target_id || "—");
  setText("approach-arm", `${status.execution_enabled ? "ON" : "OFF"} / ${status.execution_armed ? "ARM" : "SAFE"}`);
  setText("approach-views", intFmt(quality.captured_views));
  setText("approach-baseline", `${fmt(quality.max_baseline_deg, 1, "°")} / ${fmt(quality.mean_nearest_baseline_deg, 1, "°")}`);
  setText("approach-depth", percent(quality.mean_depth_ratio));
  setText("approach-rmse", fmt(quality.refined_rmse_m, 5, " m"));
  setText("approach-inlier", percent(quality.refined_inlier_ratio));
  setText("approach-allowed", quality.grasp_allowed ? "YES" : "NO");
  setText("approach-grasp", status.grasp_enabled ? "ENABLED" : "DISABLED");
}

function renderHealth(system) {
  setText("revision", `REV ${system.revision || 0}`);
  const uptime = Math.max(0, Number(system.uptime_s) || 0);
  setText("server-uptime", `UP ${String(Math.floor(uptime / 60)).padStart(2, "0")}:${String(Math.floor(uptime % 60)).padStart(2, "0")}`);
  const labels = {
    "perception.targets": "目标快照",
    "perception.harvest": "采摘计划",
    "reconstruction.status": "重建状态",
    "reconstruction.diagnostics": "重建诊断",
    "reconstruction.grasp_decision": "抓取许可",
    "refined.pose": "精化位姿",
    "refined.diagnostics": "精化质量",
    "approach.status": "靠近抓取"
  };
  const ages = system.topic_age_s || {};
  $("topic-health").innerHTML = Object.entries(labels).map(([key, label]) => {
    const age = ages[key];
    const cls = age === undefined ? "" : age < 3 ? "fresh" : age < 15 ? "stale" : "";
    return `<article class="health-item ${cls}"><div><i></i><span>${label}</span></div><b>${age === undefined ? "NO DATA" : `${age.toFixed(1)} s`}</b></article>`;
  }).join("");
}

async function pollState() {
  try {
    const response = await fetch(`/api/state?t=${Date.now()}`, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    const targets = state.perception?.targets || {};
    const selected = renderTargets(targets);
    renderSelected(selected, targets.frame_id);
    const result = renderReconstruction(state.reconstruction || {}, state.refined || {});
    renderApproach(state.approach || {});
    renderOperations(state.orchestration || {});
    renderParams(state.params || {}, state.system?.topic_age_s || {});
    renderIdentity(targets, result.diagnostics, state.refined || {}, result.decision);
    renderHealth(state.system || {});
    $("raw-json").textContent = JSON.stringify(state, null, 2);
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
setInterval(pollState, 500);
pollState();
