// tf_monitor_panel.js — 监控面板：仿真/真实识别、末端位姿、关节角复制、数据导出
import { ivgPorts } from './ivg_runtime.js';
import { ivgTransport } from './ivg_transport.js';

const TAG = '[monitor]';
const ROBOT_STATUS_TOPIC = '/aubo_driver/robot_status';
const ROBOT_STATUS_TYPE = 'demo_interface/msg/RobotStatus';
const RECONNECT_MAX = 12;

const JOINT_NAMES = [
  'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
  'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
];

// ── 状态 ───────────────────────────────────────────────────────────────
let latestMsg = null;
let records = [];
let isSimulation = null; // true=仿真, false=真实, null=未知

// ── DOM ────────────────────────────────────────────────────────────────
function $(id) { return document.getElementById(id); }

// ── Toast ──────────────────────────────────────────────────────────────
let toastTimer = 0;
function showToast(text) {
  const el = $('toast');
  if (!el) return;
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2000);
}

// ── 连接状态 ───────────────────────────────────────────────────────────
function setConnStatus(text, ok) {
  const el = $('conn-status');
  if (!el) return;
  el.textContent = text;
  el.className = ok === true ? 'status ok' : ok === false ? 'status off' : 'status pending';
}

// ── 模式检测 ───────────────────────────────────────────────────────────
async function detectMode() {
  // 方法1: 直接查 use_fake_hardware 参数
  try {
    const value = await new Promise((resolve, reject) => {
      if (!ivgTransport.ros || !ivgTransport.ros.getParam) {
        reject(new Error('no_getParam')); return;
      }
      ivgTransport.ros.getParam('use_fake_hardware', v => resolve(v));
      setTimeout(() => reject(new Error('timeout')), 4000);
    });
    isSimulation = value === true || String(value).toLowerCase() === 'true';
    updateModeBadge();
    return;
  } catch (_) { /* 继续尝试 */ }

  // 方法2: 通过 service call 查 rosapi
  try {
    const result = await ivgTransport.callService({
      service: '/rosapi/get_param',
      type: 'rosapi/GetParam',
      request: { name: 'use_fake_hardware' },
      timeoutMs: 3000,
    });
    const v = result && result.value;
    isSimulation = v === true || String(v).toLowerCase() === 'true';
    updateModeBadge();
    return;
  } catch (_e2) { /* 继续尝试 */ }

  // 方法3: 从 robot_description 参数判断
  try {
    const desc = await new Promise((resolve, reject) => {
      if (!ivgTransport.ros || !ivgTransport.ros.getParam) {
        reject(new Error('no_getParam')); return;
      }
      ivgTransport.ros.getParam('robot_description', v => resolve(v));
      setTimeout(() => reject(new Error('timeout')), 4000);
    });
    if (typeof desc === 'string') {
      if (desc.includes('mock_components') || desc.includes('GenericSystem')) isSimulation = true;
      else if (desc.includes('aubo')) isSimulation = false;
    }
  } catch (_e3) {
    isSimulation = null;
  }
  updateModeBadge();
}

function updateModeBadge() {
  const badge = $('mode-badge');
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

// ── 状态概览更新 ───────────────────────────────────────────────────────
function updateStatusSummary(msg) {
  const el = $('status-summary');
  if (!el) return;
  const chips = [
    { label: '在线', key: 'is_online', on: msg.is_online },
    { label: '使能', key: 'enable', on: msg.enable },
    { label: '运动', key: 'in_motion', on: msg.in_motion },
  ];
  let planLabel = '规划';
  let planOn = true;
  if (msg.planning_status === 'error') { planLabel = '规划:错误'; planOn = false; }
  else if (msg.planning_status === 'idle') planLabel = '规划:空闲';
  else planLabel = '规划:' + (msg.planning_status || '—');

  const parts = chips.map(c =>
    `<span class="status-chip ${c.on ? 'on' : 'off'}">${c.label}: ${c.on ? '✓' : '✗'}</span>`
  );
  parts.push(`<span class="status-chip ${planOn ? 'on' : 'off'}">${planLabel}</span>`);
  el.innerHTML = parts.join('');
}

// ── 末端位姿更新 ──────────────────────────────────────────────────────
function updatePose(msg) {
  const pos = msg.cartesian_position_xyz || {};
  const ori = msg.cartesian_position && msg.cartesian_position.orientation || {};
  const rpy = msg.cartesian_rpy || {};

  const setVal = (id, val) => {
    const el = $(id); if (el) el.textContent = val;
  };

  setVal('pose-x',   pos.x != null ? pos.x.toFixed(4) : '—');
  setVal('pose-y',   pos.y != null ? pos.y.toFixed(4) : '—');
  setVal('pose-z',   pos.z != null ? pos.z.toFixed(4) : '—');
  setVal('pose-qx',  ori.x != null ? ori.x.toFixed(4) : '—');
  setVal('pose-qy',  ori.y != null ? ori.y.toFixed(4) : '—');
  setVal('pose-qz',  ori.z != null ? ori.z.toFixed(4) : '—');
  setVal('pose-qw',  ori.w != null ? ori.w.toFixed(4) : '—');
  setVal('pose-roll',  rpy.x != null ? (rpy.x * 180 / Math.PI).toFixed(1) + '°' : '—');
  setVal('pose-pitch', rpy.y != null ? (rpy.y * 180 / Math.PI).toFixed(1) + '°' : '—');
  setVal('pose-yaw',   rpy.z != null ? (rpy.z * 180 / Math.PI).toFixed(1) + '°' : '—');
}

// ── 关节角更新 ────────────────────────────────────────────────────────
function buildJointRows(msg) {
  const degs = msg.joint_position_deg || [];
  const rads = msg.joint_position_rad || [];
  let html = '';
  for (let i = 0; i < JOINT_NAMES.length; i++) {
    const d = degs[i] != null ? degs[i].toFixed(3) : '—';
    const r = rads[i] != null ? rads[i].toFixed(6) : '—';
    html += `<tr><td>${JOINT_NAMES[i]}</td><td>${d}°</td><td>${r}</td></tr>`;
  }
  return html;
}

function updateJoints(msg) {
  const tbody = $('joint-tbody');
  if (!tbody) return;
  tbody.innerHTML = buildJointRows(msg);
}

// ── group_state XML 生成 ──────────────────────────────────────────────
function buildGroupStateXml(msg) {
  const rads = msg.joint_position_rad || [];
  const lines = ['<group_state name="current_pose" group="manipulator">'];
  for (let i = 0; i < JOINT_NAMES.length; i++) {
    const val = rads[i] != null ? rads[i] : 0;
    lines.push(`        <joint name="${JOINT_NAMES[i]}" value="${val}"/>`);
  }
  lines.push('    </group_state>');
  return lines.join('\n');
}

// ── JSON 快照生成 ─────────────────────────────────────────────────────
function buildSnapshot(msg) {
  const pos = msg.cartesian_position_xyz || {};
  const ori = msg.cartesian_position && msg.cartesian_position.orientation || {};
  const rpy = msg.cartesian_rpy || {};
  const rads = msg.joint_position_rad || [];
  const degs = msg.joint_position_deg || [];

  const joints = JOINT_NAMES.map((name, i) => ({
    name,
    position_rad: rads[i] != null ? rads[i] : null,
    position_deg: degs[i] != null ? degs[i] : null,
  }));

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
    joints,
  };
}

// ── 复制 ───────────────────────────────────────────────────────────────
async function copyText(text, label) {
  try {
    await navigator.clipboard.writeText(text);
    showToast(`已复制 ${label}`);
  } catch (_) {
    // 回退方案
    const ta = document.createElement('textarea');
    ta.value = text;
    ta.style.position = 'fixed'; ta.style.opacity = '0';
    document.body.appendChild(ta); ta.select();
    document.execCommand('copy');
    document.body.removeChild(ta);
    showToast(`已复制 ${label}`);
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

// ── 记录管理 ──────────────────────────────────────────────────────────
function updateHistoryUI() {
  const countEl = $('record-count');
  const histEl = $('export-history');
  if (countEl) countEl.textContent = `已记录: ${records.length} 条`;
  if (!histEl) return;
  if (records.length === 0) {
    histEl.innerHTML = '<span style="color:var(--text-muted);">暂无记录，点击「记录当前快照」开始。</span>';
    return;
  }
  histEl.innerHTML = records.slice(-20).map((r, i) => {
    const t = r.timestamp.replace('T', ' ').slice(0, 19);
    return `<div class="entry">#${records.length - records.slice(-20).length + i + 1} ${t} — ${r.mode}</div>`;
  }).join('');
}

function onRecordSnapshot() {
  if (!latestMsg) { showToast('暂无数据可记录'); return; }
  records.push(buildSnapshot(latestMsg));
  updateHistoryUI();
  showToast('已记录当前快照');
}

function onExportJson() {
  if (records.length === 0) { showToast('无记录可导出'); return; }
  const blob = new Blob([JSON.stringify(records, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = `ivg_monitor_${new Date().toISOString().slice(0,10)}.json`;
  a.click();
  URL.revokeObjectURL(url);
  showToast('JSON 已导出');
}

function onExportXml() {
  if (records.length === 0) { showToast('无记录可导出'); return; }
  const xmlEntries = records.map((r, i) => {
    const jointsXml = r.joints.map(j =>
      `        <joint name="${j.name}" value="${j.position_rad != null ? j.position_rad : 0}"/>`
    ).join('\n');
    return `    <!-- Snapshot #${i + 1} — ${r.timestamp} -->
    <group_state name="snapshot_${i + 1}" group="manipulator">
${jointsXml}
    </group_state>`;
  }).join('\n');
  const xml = `<?xml version="1.0" encoding="utf-8"?>\n<group_states>\n${xmlEntries}\n</group_states>\n`;
  const blob = new Blob([xml], { type: 'application/xml' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = `ivg_joints_${new Date().toISOString().slice(0,10)}.xml`;
  a.click();
  URL.revokeObjectURL(url);
  showToast('XML 已导出');
}

function onClearHistory() {
  records.length = 0;
  updateHistoryUI();
  showToast('记录已清空');
}

// ── 数据到达 ──────────────────────────────────────────────────────────
function onRobotStatus(msg) {
  if (!msg) return;
  latestMsg = msg;
  updateStatusSummary(msg);
  updatePose(msg);
  updateJoints(msg);
}

// ── 连接与订阅 ────────────────────────────────────────────────────────
const rosReconnect = ivgPorts.createRosReconnectState();
let connectInFlight = false;

function subscribeStatus() {
  ivgTransport.clearRosHandlers();
  ivgTransport.unsubscribeAll();

  ivgTransport.onRosJson(ROBOT_STATUS_TOPIC, onRobotStatus);
  const ok = ivgTransport.subscribe({
    topic: ROBOT_STATUS_TOPIC,
    msgType: ROBOT_STATUS_TYPE,
    maxHz: 10,
  });
  if (ok) console.log(TAG, '已订阅', ROBOT_STATUS_TOPIC);
  else console.warn(TAG, '订阅失败');
}

function connect() {
  if (connectInFlight) return;
  connectInFlight = true;
  ivgPorts.clearRosReconnectTimer(rosReconnect);
  const myGen = ivgPorts.bumpRosReconnectGen(rosReconnect);
  setConnStatus('正在连接…', null);
  ivgTransport.close();

  void (async () => {
    try {
      await ivgTransport.loadRuntime();
      await ivgTransport.connectControl();
      if (myGen !== rosReconnect.gen) return;
      rosReconnect.attempts = 0;
      ivgPorts.clearRosReconnectTimer(rosReconnect);

      ivgTransport.clearControlJsonHandlers();
      ivgTransport.onControlJson(o => {
        if (!o || typeof o !== 'object') return;
        if (o.op === 'error') setConnStatus('已连接但发生错误', false);
        if (o.op === 'close') {
          setConnStatus('连接已断开', false);
          ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
            maxAttempts: RECONNECT_MAX,
            onSchedule(delayMs, attempt, max) {
              setConnStatus(`${Math.round(delayMs/1000)}s 后重连 (${attempt}/${max})`, false);
            },
            onExhausted() {
              setConnStatus('已达重连上限，请刷新页面', false);
            }
          });
        }
      });

      setConnStatus('已连接', true);
      subscribeStatus();
      // 连接成功后检测模式
      setTimeout(detectMode, 500);
    } catch (e) {
      if (myGen !== rosReconnect.gen) return;
      setConnStatus('连接错误', false);
      ivgPorts.scheduleRosReconnect(rosReconnect, connect, {
        maxAttempts: RECONNECT_MAX,
        onSchedule(delayMs, attempt, max) {
          setConnStatus(`${Math.round(delayMs/1000)}s 后重连 (${attempt}/${max})`, false);
        },
        onExhausted() {
          setConnStatus('已达重连上限，请刷新页面', false);
        }
      });
    } finally {
      connectInFlight = false;
    }
  })();
}

// ── 按钮绑定 ──────────────────────────────────────────────────────────
function bindButtons() {
  $('btn-copy-group-state')?.addEventListener('click', onCopyGroupState);
  $('btn-copy-json')?.addEventListener('click', onCopyJson);
  $('btn-record-snapshot')?.addEventListener('click', onRecordSnapshot);
  $('btn-export-json')?.addEventListener('click', onExportJson);
  $('btn-export-xml')?.addEventListener('click', onExportXml);
  $('btn-clear-history')?.addEventListener('click', () => {
    if (records.length === 0) { showToast('无记录可清空'); return; }
    if (confirm('确定清空全部记录？')) onClearHistory();
  });
}

// ── 入口 ───────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  void (async () => {
    await ivgPorts.loadRuntime();
    bindButtons();
    ivgPorts.wireOnlineRosReconnect(rosReconnect, connect);
    connect();
  })();
});
