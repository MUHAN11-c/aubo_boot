// trajectory_history.js — 轨迹选中后渲染 3D + 时序图 (无模块依赖)
(function() {
var viewer = null, fullData = null, latteData = [];

// 拉花段色板
var C = ['#ff6b6b','#ffd93d','#6bcb77','#4d96ff','#ff922b','#845ef7','#f06595','#20c997'];

// 图表色板 — 暖灰蓝基调
var POS_C = ['#3e5a73','#c8873b','#dc2626'];         // X Y Z
var RPY_C = ['#dc2626','#16a34a','#3e5a73'];          // R P Y
var JNT_C = ['#3e5a73','#c8873b','#16a34a','#dc2626','#d97706','#845ef7'];

/* ── Init ─────────────────────────────────────────────── */

var _initDone = false;

function init() {
  if (_initDone) return;
  _initDone = true;

  var sel = document.getElementById('traj-select');
  if (sel) sel.addEventListener('change', onSelect);
  var btn = document.getElementById('traj-refresh');
  if (btn) btn.addEventListener('click', function() { location.reload(); });
  var cb = document.getElementById('traj-latte-only');
  if (cb) cb.addEventListener('change', function() { render3D(); });

  // Collapse/expand
  var hdr = document.getElementById('traj-toggle');
  if (hdr) hdr.addEventListener('click', function() {
    var body = document.getElementById('traj-body');
    var icon = hdr.querySelector('.traj-header-icon');
    if (body) {
      var hidden = body.style.display === 'none';
      body.style.display = hidden ? 'flex' : 'none';
      if (icon) icon.innerHTML = hidden ? '▼' : '▶';
      if (hidden && viewer) { setTimeout(function() { viewer._resize(); }, 50); }
    }
  });

  // Charts <details> toggle → redraw (fix pixelated charts after expand)
  // Ref: https://developer.mozilla.org/en-US/docs/Web/API/HTMLDetailsElement/toggle_event
  var chartDetails = document.getElementById('traj-charts-details');
  if (chartDetails) chartDetails.addEventListener('toggle', function() {
    if (chartDetails.open && fullData) renderCharts();
  });
}

/* ── Data loading ────────────────────────────────────── */

async function onSelect() {
  var sel = document.getElementById('traj-select');
  if (!sel) return;
  var ts = sel.value;
  var emptyEl = document.getElementById('traj-3d-empty');
  if (!ts) {
    if (viewer) viewer.clearTrajectories();
    fullData = null; latteData = [];
    if (emptyEl) emptyEl.style.display = 'flex';
    updateInfo(); return;
  }
  if (emptyEl) emptyEl.style.display = 'none';
  document.getElementById('traj-info').innerHTML =
    '<span style="opacity:0.6;">加载中…</span>';

  try {
    var resp = await fetch('/api/v1/trajectories/' + ts + '/full');
    // fetch() 不 reject HTTP 错误状态码, 必须手动检查 resp.ok
    // Ref: https://developer.mozilla.org/en-US/docs/Web/API/fetch
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    fullData = await resp.json();

    // Fetch latte segments
    latteData = [];
    var lr = await fetch('/api/v1/trajectories');
    if (lr.ok) {
      var list = await lr.json();
      var item = (list.trajectories || []).find(function(t) { return t.ts === ts; });
      if (item && item.latte_segments > 0) {
        for (var i = 1; i <= item.latte_segments; i++) {
          try {
            var sr = await fetch('/api/v1/trajectories/' + ts + '/latte/' + i);
            if (sr.ok) latteData.push(await sr.json());
          } catch(e) { console.warn('[traj] latte segment ' + i + ' load failed:', e.message || e); }
        }
      }
    }

    // Lazy init viewer
    if (!viewer && window.TrajectoryStandaloneViewer) {
      viewer = new TrajectoryStandaloneViewer();
      viewer.init('traj-3d');
    }

    render3D();
    renderCharts();
    updateInfo();
  } catch(e) {
    console.error('[traj]', e);
    document.getElementById('traj-info').innerHTML =
      '<span style="color:var(--err);">加载失败</span>';
  }
}

/* ── 3D rendering ────────────────────────────────────── */

function render3D() {
  if (!viewer || !fullData) return;
  var latteOnly = document.getElementById('traj-latte-only');
  if (latteOnly && latteOnly.checked) {
    viewer.clearTrajectories();
    if (latteData.length) viewer.loadLatteSegments(latteData, C);
  } else {
    viewer.loadTrajectory(fullData, '#4d96ff');
    if (latteData.length) viewer.loadLatteSegments(latteData, C);
  }
}

/* ── Chart rendering ─────────────────────────────────── */

function renderCharts() {
  var d = fullData;
  if (!d || !d.t || d.t.length < 2) return;

  drawOne('traj-chart-xyz', d.t, [
    { data: d.x, color: POS_C[0], label: 'X' },
    { data: d.y, color: POS_C[1], label: 'Y' },
    { data: d.z, color: POS_C[2], label: 'Z' }
  ]);

  drawOne('traj-chart-rpy', d.t, [
    { data: d.roll.map(function(v) { return v * 180 / Math.PI; }), color: RPY_C[0], label: 'R' },
    { data: d.pitch.map(function(v) { return v * 180 / Math.PI; }), color: RPY_C[1], label: 'P' },
    { data: d.yaw.map(function(v) { return v * 180 / Math.PI; }), color: RPY_C[2], label: 'Y' }
  ]);

  var js = [];
  for (var i = 1; i <= 6; i++)
    if (d['j'+i]) js.push({ data: d['j'+i], color: JNT_C[i-1], label: 'J'+i });
  if (js.length) drawOne('traj-chart-joint', d.t, js);
}

/**
 * Draw a multi-series time-series chart on a canvas.
 * Features: gradient area fills, latte segment bands, grid,
 *           y-axis labels, legend.
 */
function drawOne(id, t, series) {
  var c = document.getElementById(id);
  if (!c) return;

  var box = c.parentElement;
  var W = box ? Math.max(box.clientWidth - 2, 200) : 320;
  var H = 100;
  var dpr = Math.min(window.devicePixelRatio || 1, 2);
  c.width  = W * dpr;
  c.height = H * dpr;
  c.style.width  = '100%';
  c.style.height = H + 'px';

  var ctx = c.getContext('2d');
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, W, H);

  // Margins
  var ml = 40, mr = 8, mt = 18, mb = 12;
  var pw = W - ml - mr;
  var ph = H - mt - mb;
  if (pw < 20) return;

  // ── Data range ──
  var mn = Infinity, mx = -Infinity;
  series.forEach(function(s) {
    s.data.forEach(function(v) {
      if (v < mn) mn = v; if (v > mx) mx = v;
    });
  });
  if (!isFinite(mn)) { mn = -1; mx = 1; }
  var pad = Math.max((mx - mn) * 0.12, 0.08);
  mn -= pad; mx += pad;

  var t0 = t[0], tr = (t[t.length - 1] - t0) || 1;
  var tx = function(v) { return ml + ((v - t0) / tr) * pw; };
  var ty = function(v) { return mt + (1 - (v - mn) / (mx - mn || 1)) * ph; };

  // ── Grid (horizontal) ──
  ctx.strokeStyle = 'rgba(157,150,140,0.10)';
  ctx.lineWidth = 0.5;
  for (var i = 0; i <= 4; i++) {
    var gy = mt + ph * i / 4;
    ctx.beginPath(); ctx.moveTo(ml, gy); ctx.lineTo(W - mr, gy); ctx.stroke();
  }
  // Baseline accent
  ctx.strokeStyle = 'rgba(157,150,140,0.18)';
  ctx.beginPath(); ctx.moveTo(ml, ty(0)); ctx.lineTo(W - mr, ty(0)); ctx.stroke();

  // ── Latte segment bands ──
  if (latteData.length) {
    latteData.forEach(function(lat, li) {
      if (!lat.t || lat.t.length < 2) return;
      var lx0 = tx(lat.t[0]), lx1 = tx(lat.t[lat.t.length - 1]);
      var ww = Math.max(lx1 - lx0, 1);
      var grad = ctx.createLinearGradient(0, mt, 0, mt + ph);
      grad.addColorStop(0, C[li % C.length] + '0F');
      grad.addColorStop(0.5, C[li % C.length] + '08');
      grad.addColorStop(1, C[li % C.length] + '03');
      ctx.fillStyle = grad;
      ctx.fillRect(lx0, mt, ww, ph);
    });
  }

  // ── Area fills under each series ──
  series.forEach(function(s) {
    if (s.data.length < 2) return;
    // Gradient area
    var ag = ctx.createLinearGradient(0, mt, 0, mt + ph);
    ag.addColorStop(0, s.color + '14');
    ag.addColorStop(1, s.color + '02');
    ctx.fillStyle = ag;
    ctx.beginPath();
    ctx.moveTo(tx(t[0]), ty(0));
    for (var i = 0; i < s.data.length; i++)
      ctx.lineTo(tx(t[i]), ty(s.data[i]));
    ctx.lineTo(tx(t[t.length - 1]), ty(0));
    ctx.closePath();
    ctx.fill();
  });

  // ── Series lines ──
  series.forEach(function(s) {
    if (s.data.length < 2) return;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    for (var i = 0; i < s.data.length; i++) {
      var sx = tx(t[i]), sy = ty(s.data[i]);
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  });

  // ── Y-axis labels ──
  ctx.fillStyle = 'rgba(120,115,106,0.65)';
  ctx.font = '7.5px "Noto Sans Mono","DejaVu Sans Mono",monospace';
  ctx.textAlign = 'right';
  for (var i = 0; i <= 4; i++) {
    var v = mn + (mx - mn) * i / 4;
    var ly = mt + ph * (1 - i / 4);
    ctx.fillText(v.toFixed(2), ml - 6, ly + 3);
  }

  // ── Legend ──
  var lx = ml;
  series.forEach(function(s, i) {
    var lw = ctx.measureText(s.label).width + 16;
    ctx.fillStyle = s.color;
    ctx.fillRect(lx, 2, 8, 8);
    ctx.fillStyle = '#5d564e';
    ctx.font = '7px "Noto Sans SC","PingFang SC",sans-serif';
    ctx.fillText(s.label, lx + 11, 10);
    lx += lw;
  });
}

/* ── Info bar ────────────────────────────────────────── */

function updateInfo() {
  var d = fullData, el = document.getElementById('traj-info');
  if (!el) return;
  if (!d) { el.innerHTML = ''; return; }
  var dur = (d.t[d.t.length-1] - d.t[0]).toFixed(1);
  el.innerHTML =
    '<span>● 点位 <b>' + (d._original_n || d.t.length) + '</b>' +
    (d._downsampled ? ' <span style="opacity:0.6;">降采样</span>' : '') + '</span>' +
    '<span>● 时长 <b>' + dur + 's</b></span>' +
    '<span>● 拉花段 <b>' + latteData.length + '</b></span>';
}

/* ── Bootstrap ───────────────────────────────────────── */

document.addEventListener('DOMContentLoaded', init);
})();
