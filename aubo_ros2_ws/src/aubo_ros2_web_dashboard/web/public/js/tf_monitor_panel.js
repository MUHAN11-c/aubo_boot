// tf_monitor_panel.js — 监控面板：仿真/真实识别、末端位姿、关节角复制、数据导出
import { $, escapeHtml } from './core/utils.js';
import { loadRecords, saveRecords, clearRecords } from './core/record_store.js';
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, MODE_TOPIC, MODE_TYPE } from './core/topics.js';
import { ros } from './core/ros.js';
import { ivgTransport } from './ivg_transport.js';
import { ivgPorts } from './ivg_runtime.js';

var TAG = '[monitor]';

var JOINT_NAMES = [
  'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
  'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
];

var latestMsg = null;
var records = loadRecords();
var isSimulation = null;

var toastTimer = 0;
function showToast(text) {
  var el = $('toast');
  if (!el) return;
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(function () { el.classList.remove('show'); }, 2000);
}

function setConnStatus(text, ok) {
  var el = $('conn-status');
  if (!el) return;
  el.textContent = text;
  el.className = ok === true ? 'status ok' : ok === false ? 'status off' : 'status pending';
}

// ── 模式检测 (通过 /aubo/mode topic) ──────────────────────────────
function detectMode() {
  ivgTransport.subscribe({ topic: MODE_TOPIC, msgType: MODE_TYPE, maxHz: 1 });
  ivgTransport.onRosJson(MODE_TOPIC, function (msg) {
    var raw = (msg && msg.data) ? String(msg.data) : '';
    if (raw === 'simulation') isSimulation = true;
    else if (raw === 'real') isSimulation = false;
    else isSimulation = null;
    updateModeBadge();
  }, 'tf_monitor');
}

function updateModeBadge() {
  var badge = $('mode-badge');
  if (!badge) return;
  if (isSimulation === true) {
    badge.className = 'mode-badge sim';
    badge.innerHTML = '<span class="mode-dot"></span>仿真模式';
  } else if (isSimulation === false) {
    badge.className = 'mode-badge real';
    badge.innerHTML = '<span class="mode-dot"></span>真实机械臂';
  } else {
    badge.className = 'mode-badge unknown';
    badge.innerHTML = '<span class="mode-dot"></span>模式未知';
  }
}

// ── 状态概览 ─────────────────────────────────────────────────────
function updateStatusSummary(msg) {
  var el = $('status-summary');
  if (!el) return;
  var chips = [
    { label: '在线', key: 'is_online', on: msg.is_online },
    { label: '使能', key: 'enable', on: msg.enable },
    { label: '运动', key: 'in_motion', on: msg.in_motion },
  ];
  var planLabel = '规划', planOn = true;
  if (msg.planning_status === 'error') { planLabel = '规划:错误'; planOn = false; }
  else if (msg.planning_status === 'idle') planLabel = '规划:空闲';
  else planLabel = '规划:' + (msg.planning_status || '0.000');

  el.innerHTML = chips.map(function (c) {
    return '<span class="status-chip ' + (c.on ? 'on' : 'off') + '">' + c.label + ': ' + (c.on ? '✓' : '✗') + '</span>';
  }).join('') + '<span class="status-chip ' + (planOn ? 'on' : 'off') + '">' + planLabel + '</span>';
}

// ── 末端位姿 ─────────────────────────────────────────────────────
function updatePose(msg) {
  var pos = msg.cartesian_position_xyz || {};
  var ori = msg.cartesian_position && msg.cartesian_position.orientation || {};
  var rpy = msg.cartesian_rpy || {};

  function setVal(id, val) { var el = $(id); if (el) el.textContent = val; }

  setVal('pose-x',   pos.x != null ? pos.x.toFixed(3) : '0.000');
  setVal('pose-y',   pos.y != null ? pos.y.toFixed(3) : '0.000');
  setVal('pose-z',   pos.z != null ? pos.z.toFixed(3) : '0.000');
  setVal('pose-qx',  ori.x != null ? ori.x.toFixed(3) : '0.000');
  setVal('pose-qy',  ori.y != null ? ori.y.toFixed(3) : '0.000');
  setVal('pose-qz',  ori.z != null ? ori.z.toFixed(3) : '0.000');
  setVal('pose-qw',  ori.w != null ? ori.w.toFixed(3) : '0.000');
  setVal('pose-roll',  rpy.x != null ? (rpy.x * 180 / Math.PI).toFixed(1) + '°' : '0.0°');
  setVal('pose-pitch', rpy.y != null ? (rpy.y * 180 / Math.PI).toFixed(1) + '°' : '0.0°');
  setVal('pose-yaw',   rpy.z != null ? (rpy.z * 180 / Math.PI).toFixed(1) + '°' : '0.0°');
}

// ── 关节角 ───────────────────────────────────────────────────────
function buildJointRows(msg) {
  var degs = msg.joint_position_deg || [];
  var rads = msg.joint_position_rad || [];
  var html = '';
  for (var i = 0; i < JOINT_NAMES.length; i++) {
    html += '<tr><td>' + JOINT_NAMES[i] + '</td><td>'
      + (degs[i] != null ? degs[i].toFixed(3) : '0.000') + '°</td><td>'
      + (rads[i] != null ? rads[i].toFixed(3) : '0.000') + '</td></tr>';
  }
  return html;
}

function updateJoints(msg) {
  var tbody = $('joint-tbody');
  if (!tbody) return;
  tbody.innerHTML = buildJointRows(msg);
}

// ── XML / JSON 快照 ──────────────────────────────────────────────
function buildGroupStateXml(msg) {
  var rads = msg.joint_position_rad || [];
  var lines = ['<group_state name="current_pose" group="manipulator">'];
  for (var i = 0; i < JOINT_NAMES.length; i++)
    lines.push('        <joint name="' + JOINT_NAMES[i] + '" value="' + (rads[i] != null ? rads[i] : 0) + '"/>');
  lines.push('    </group_state>');
  return lines.join('\n');
}

function buildSnapshot(msg) {
  var pos = msg.cartesian_position_xyz || {};
  var ori = msg.cartesian_position && msg.cartesian_position.orientation || {};
  var rpy = msg.cartesian_rpy || {};
  var rads = msg.joint_position_rad || [];
  var degs = msg.joint_position_deg || [];
  return {
    timestamp: new Date().toISOString(),
    mode: isSimulation === true ? 'simulation' : isSimulation === false ? 'real' : 'unknown',
    end_effector: {
      position_m: { x: pos.x, y: pos.y, z: pos.z },
      orientation_quaternion: { x: ori.x, y: ori.y, z: ori.z, w: ori.w },
      euler_rpy_deg: {
        roll: rpy.x != null ? +(rpy.x * 180 / Math.PI).toFixed(3) : null,
        pitch: rpy.y != null ? +(rpy.y * 180 / Math.PI).toFixed(3) : null,
        yaw: rpy.z != null ? +(rpy.z * 180 / Math.PI).toFixed(3) : null,
      },
    },
    joints: JOINT_NAMES.map(function (name, i) {
      return { name: name, position_rad: rads[i] != null ? rads[i] : null, position_deg: degs[i] != null ? degs[i] : null };
    }),
  };
}

// ── 复制 ─────────────────────────────────────────────────────────
async function copyText(text, label) {
  try { await navigator.clipboard.writeText(text); showToast('已复制 ' + label); } catch (_) {
    var ta = document.createElement('textarea');
    ta.value = text; ta.style.position = 'fixed'; ta.style.opacity = '0';
    document.body.appendChild(ta); ta.select();
    document.execCommand('copy'); document.body.removeChild(ta);
    showToast('已复制 ' + label);
  }
}

function onCopyGroupState() {
  if (!latestMsg) { showToast('暂无数据'); return; }
  copyText(buildGroupStateXml(latestMsg), 'group_state XML');
}
function onCopyJson() {
  if (!latestMsg) { showToast('暂无数据'); return; }
  copyText(JSON.stringify(buildSnapshot(latestMsg), null, 2), 'JSON');
}

// ── 记录管理 ─────────────────────────────────────────────────────
function updateHistoryUI() {
  var countEl = $('record-count');
  var histEl = $('export-history');
  if (countEl) countEl.textContent = '已记录: ' + records.length + ' 条';
  if (!histEl) return;
  if (records.length === 0) {
    histEl.innerHTML = '<span style="color:var(--text-muted);">暂无记录，点击「记录当前快照」开始。</span>';
    return;
  }
  var recent = records.slice(-20);
  var offset = records.length - recent.length;
  histEl.innerHTML = recent.map(function (r, i) {
    var idx = offset + i;
    var t = (r.timestamp || '').replace('T', ' ').slice(0, 19);
    var name = r.name || ('快照 #' + (idx + 1));
    return '<div class="entry">'
      + '<div class="entry-name-row">'
      + '<span class="record-name" data-idx="' + idx + '" title="点击编辑名称">' + escapeHtml(name) + '</span>'
      + '<button class="record-delete-btn" data-idx="' + idx + '" title="删除此记录">✕</button>'
      + '</div>'
      + '<div class="entry-info">#' + (idx + 1) + ' ' + t + ' — ' + (r.mode || 'unknown') + '</div>'
      + '</div>';
  }).join('');

  histEl.querySelectorAll('.record-name').forEach(function (el) {
    el.addEventListener('click', function () { onRenameRecord(parseInt(el.dataset.idx)); });
  });
  histEl.querySelectorAll('.record-delete-btn').forEach(function (el) {
    el.addEventListener('click', function () { onDeleteRecord(parseInt(el.dataset.idx)); });
  });
}

function onRenameRecord(index) {
  var r = records[index];
  if (!r) return;
  var oldName = r.name || ('快照 #' + (index + 1));
  var nameEl = document.querySelector('.record-name[data-idx="' + index + '"]');
  if (!nameEl) return;

  var input = document.createElement('input');
  input.type = 'text';
  input.className = 'record-name-edit';
  input.value = oldName;
  input.maxLength = 60;

  function commit() {
    var newName = input.value.trim();
    if (!newName) newName = '快照 #' + (index + 1);
    records[index].name = newName;
    saveRecords(records);
    updateHistoryUI();
    showToast('已重命名为: ' + newName);
  }

  function cancel() {
    updateHistoryUI();
  }

  input.addEventListener('keydown', function (e) {
    if (e.key === 'Enter') { e.preventDefault(); commit(); }
    else if (e.key === 'Escape') { e.preventDefault(); cancel(); }
  });
  input.addEventListener('blur', function () {
    if (document.body.contains(input)) commit();
  });

  nameEl.replaceWith(input);
  input.focus();
  input.select();
}

function onDeleteRecord(index) {
  if (index < 0 || index >= records.length) return;
  var name = records[index].name || ('快照 #' + (index + 1));
  if (!confirm('删除记录「' + name + '」？')) return;
  records.splice(index, 1);
  saveRecords(records);
  updateHistoryUI();
  showToast('已删除: ' + name);
}

function onRecordSnapshot() {
  if (!latestMsg) { showToast('暂无数据可记录'); return; }
  var snap = buildSnapshot(latestMsg);
  snap.name = '快照 #' + (records.length + 1);
  records.push(snap);
  if (!saveRecords(records)) {
    records.pop();
    showToast('存储空间不足，请清理旧记录');
    return;
  }
  updateHistoryUI();
  showToast('已记录: ' + snap.name);
}

function onExportJson() {
  if (records.length === 0) { showToast('无记录可导出'); return; }
  var blob = new Blob([JSON.stringify(records, null, 2)], { type: 'application/json' });
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url; a.download = 'ivg_monitor_' + new Date().toISOString().slice(0, 10) + '.json';
  a.click(); URL.revokeObjectURL(url);
  showToast('JSON 已导出');
}

function onExportXml() {
  if (records.length === 0) { showToast('无记录可导出'); return; }
  var xmlEntries = records.map(function (r, i) {
    var jointsXml = r.joints.map(function (j) {
      return '        <joint name="' + j.name + '" value="' + (j.position_rad != null ? j.position_rad : 0) + '"/>';
    }).join('\n');
    return '    <!-- Snapshot #' + (i + 1) + ' — ' + r.timestamp + ' -->\n    <group_state name="snapshot_' + (i + 1) + '" group="manipulator">\n' + jointsXml + '\n    </group_state>';
  }).join('\n');
  var xml = '<?xml version="1.0" encoding="utf-8"?>\n<group_states>\n' + xmlEntries + '\n</group_states>\n';
  var blob = new Blob([xml], { type: 'application/xml' });
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url; a.download = 'ivg_joints_' + new Date().toISOString().slice(0, 10) + '.xml';
  a.click(); URL.revokeObjectURL(url);
  showToast('XML 已导出');
}

function onClearHistory() {
  if (records.length === 0) { showToast('无记录可清空'); return; }
  if (!confirm('确定清空全部 ' + records.length + ' 条记录？此操作不可恢复。')) return;
  records.length = 0;
  clearRecords();
  updateHistoryUI();
  showToast('记录已清空');
}

// ── 数据到达 ─────────────────────────────────────────────────────
function onRobotStatus(msg) {
  if (!msg) return;
  latestMsg = msg;
  updateStatusSummary(msg);
  updatePose(msg);
  updateJoints(msg);
}

// ── 订阅 ─────────────────────────────────────────────────────────
function subscribeStatus() {
  ivgTransport.clearRosHandlersByOwner('tf_monitor');
  ivgTransport.onRosJson(ROBOT_STATUS_TOPIC, onRobotStatus, 'tf_monitor');
  var ok = ivgTransport.subscribe({ topic: ROBOT_STATUS_TOPIC, msgType: ROBOT_STATUS_TYPE, maxHz: 10 });
  console.log(TAG, ok ? '已订阅 ' + ROBOT_STATUS_TOPIC : '订阅失败');
}

// ── 连接 (统一使用 ros.js) ──────────────────────────────────────
ros.onStatusChange(setConnStatus);
ros.onConnected(function () {
  subscribeStatus();
  setTimeout(detectMode, 500);
});

// ── 按钮绑定 ─────────────────────────────────────────────────────
function bindButtons() {
  $('btn-copy-group-state') && $('btn-copy-group-state').addEventListener('click', onCopyGroupState);
  $('btn-copy-json') && $('btn-copy-json').addEventListener('click', onCopyJson);
  $('btn-record-snapshot') && $('btn-record-snapshot').addEventListener('click', onRecordSnapshot);
  $('btn-export-json') && $('btn-export-json').addEventListener('click', onExportJson);
  $('btn-export-xml') && $('btn-export-xml').addEventListener('click', onExportXml);
  $('btn-clear-history') && $('btn-clear-history').addEventListener('click', onClearHistory);
}

// ── 入口 ─────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', function () {
  (async function () {
    await ivgPorts.loadRuntime();
    bindButtons();
    ros.wireOnlineReconnect();
    ros.connect();
  })();
});
