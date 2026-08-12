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
let operationRevision = 0;
let policyDirty = false;
const batchNames = ["等待就绪", "发现目标", "运行中", "等待安全暂停点", "已暂停", "维护模式", "已完成", "故障", "需要恢复", "已中断"];

function renderOperations(orchestration) {
  const state = orchestration.state || {};
  operationRevision = Number(state.revision || 0);
  setText("batch-state", batchNames[state.batch_state] || "等待编排器");
  setText("batch-message", state.message || "尚未收到类型化状态");
  $("batch-blockers").innerHTML = (state.blockers || []).map((item) =>
    "<span title=\"该健康门未通过\">" + safe(item) + "</span>").join("");
  const maintenance = state.operation_mode === 2;
  document.querySelectorAll("[data-debug]").forEach((button) => {
    button.disabled = !maintenance;
  });
  if (!policyDirty) {
    $("policy-auto").checked = state.auto_start_enabled === true;
    $("policy-execution").checked = state.execution_enabled === true;
    $("policy-grasp").checked = state.grasp_enabled === true;
    $("policy-tool").checked = state.tool_enabled === true;
  }
}

async function apiPost(path, payload) {
  const response = await fetch(path, {
    method: "POST", credentials: "same-origin",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({...payload, expected_revision: operationRevision})
  });
  const result = await response.json();
  setText("operation-result", result.message || ("HTTP " + response.status));
  if (!response.ok) throw new Error(result.message || ("HTTP " + response.status));
  return result;
}

document.querySelectorAll("[data-control]").forEach((button) => {
  button.addEventListener("click", async () => {
    const confirmed = !button.dataset.confirm || window.confirm(button.dataset.confirm);
    if (!confirmed) return;
    try {
      await apiPost("/api/control", {
        command: Number(button.dataset.control),
        request_id: crypto.randomUUID(), confirmed,
        reason: "web_task_center"
      });
    } catch (_) { /* API 已显示明确原因 */ }
  });
});

document.querySelectorAll("[data-debug]").forEach((button) => {
  button.addEventListener("click", async () => {
    const confirmed = !button.dataset.confirm || window.confirm(button.dataset.confirm);
    if (!confirmed) return;
    try {
      await apiPost("/api/debug", {
        action: button.dataset.debug,
        armed: button.dataset.armed === "true", confirmed
      });
    } catch (_) { /* API 已显示明确原因 */ }
  });
});

["policy-auto", "policy-execution", "policy-grasp", "policy-tool"].forEach((id) => {
  $(id).addEventListener("change", () => { policyDirty = true; });
});

$("apply-policy").addEventListener("click", async () => {
  const payload = {
    request_id: crypto.randomUUID(),
    auto_start_enabled: $("policy-auto").checked,
    execution_enabled: $("policy-execution").checked,
    grasp_enabled: $("policy-grasp").checked,
    tool_enabled: $("policy-tool").checked
  };
  const enabling = payload.execution_enabled || payload.grasp_enabled || payload.tool_enabled;
  payload.confirmed = !enabling || window.confirm("确认修改运动、抓取和工具使能？真机必须保持低速并由现场人员上电。");
  if (!payload.confirmed) return;
  try { await apiPost("/api/policy", payload); policyDirty = false; } catch (_) {}
});

$("save-profile").addEventListener("click", async () => {
  try {
    await apiPost("/api/profiles/save", {
      name: $("profile-name").value,
      values: {orchestrator: {
        auto_start_enabled: $("policy-auto").checked,
        execution_enabled: $("policy-execution").checked,
        grasp_enabled: $("policy-grasp").checked,
        tool_enabled: $("policy-tool").checked
      }}
    });
  } catch (_) {}
});

$("load-profile").addEventListener("click", async () => {
  try {
    const result = await apiPost("/api/profiles/load", {name: $("profile-name").value});
    const policy = result.values?.orchestrator || {};
    $("policy-auto").checked = policy.auto_start_enabled === true;
    $("policy-execution").checked = policy.execution_enabled === true;
    $("policy-grasp").checked = policy.grasp_enabled === true;
    $("policy-tool").checked = policy.tool_enabled === true;
    policyDirty = true;
  } catch (_) {}
});

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
