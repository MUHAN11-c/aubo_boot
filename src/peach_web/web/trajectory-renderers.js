"use strict";

// TCP projection and historical trajectory rendering, with no network ownership.
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
    ctx.fillStyle = "rgba(70,95,108,0.8)";
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
        const rgb = PHASE_RGB[phase] || [38, 84, 112];
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
    ctx.fillStyle = "rgba(33,52,62,0.95)";
    ctx.font = '11px "PingFang SC","Noto Sans CJK SC",sans-serif';
    ctx.fillText(label, px + 7, py - 5);
  });
  if (points.length) {
    const [tx, ty] = toPx(points[points.length - 1]);
    ctx.beginPath();
    ctx.fillStyle = "rgb(8,127,115)";
    ctx.arc(tx, ty, 4.5, 0, Math.PI * 2);
    ctx.fill();
  }
}

