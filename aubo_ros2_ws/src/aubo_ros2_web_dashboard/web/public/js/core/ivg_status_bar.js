// ivg_status_bar — 底部状态栏
// ROS 连接(WebSocket) + Bridge 选择器 + /robot_status + /aubo/mode
import { ivgTransport } from '../ivg_transport.js';
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, MODE_TOPIC, MODE_TYPE } from './topics.js';
import { logBus } from './log-bus.js';

// 异步加载 TransportHub (不阻塞 status_bar 首次渲染)
import('../transport/hub.js').then(() => {
  _bindHubEvents();
}).catch(() => {});

const g = globalThis;

const TAG = '[ivg_status_bar]';

const STATUS_BAR_ID = 'ivg-status-bar';

const LABELS = {
  ros_connected: 'ROS',
  is_online: '在线',
  enable: '使能',
  in_motion: '运动',
  planning_status: '规划',
  driver_mode: '模式',
};

const VALUE_MAP = {
  ros_connected: v => (v ? '已连接' : '已断开'),
  is_online: v => (v ? '在线' : '离线'),
  enable: v => (v ? '已使能' : '未使能'),
  in_motion: v => (v ? '运动中' : '静止'),
  planning_status: v => {
    const m = {
      idle: '空闲', planning: '规划中', executing: '执行中', error: '错误',
    };
    return m[v] || v || '未知';
  },
  driver_mode: v => {
    if (v === 'real') return '真实';
    if (v === 'simulation') return '仿真';
    return v || '—';
  },
};

const DOT_CLASS = {
  ros_connected: v => (v ? 'ivg-status-bar__dot--on' : 'ivg-status-bar__dot--err'),
  is_online: v => (v ? 'ivg-status-bar__dot--on' : 'ivg-status-bar__dot--off'),
  enable: v => (v ? 'ivg-status-bar__dot--on' : 'ivg-status-bar__dot--off'),
  in_motion: v => (v ? 'ivg-status-bar__dot--warn' : 'ivg-status-bar__dot--off'),
  planning_status: v => {
    if (v === 'executing') return 'ivg-status-bar__dot--warn';
    if (v === 'error') return 'ivg-status-bar__dot--err';
    if (v === 'planning') return 'ivg-status-bar__dot--warn';
    return 'ivg-status-bar__dot--on';
  },
  driver_mode: v => {
    if (v === 'real') return 'ivg-status-bar__dot--on';
    if (v === 'simulation') return 'ivg-status-bar__dot--warn';
    return 'ivg-status-bar__dot--off';
  },
};

const ITEM_ORDER = ['ros_connected', 'is_online', 'enable', 'in_motion', 'planning_status', 'driver_mode'];

function buildBar() {
  const bar = document.createElement('div');
  bar.id = STATUS_BAR_ID;
  bar.className = 'ivg-status-bar';

  // ── Bridge 选择器 (最左侧) ──
  const bridgeSel = buildBridgeSelector();
  bar.appendChild(bridgeSel);
  bar.appendChild(document.createTextNode(' '));

  const spacer = document.createElement('div');
  spacer.className = 'ivg-status-bar__spacer';
  bar.appendChild(spacer);

  for (const key of ITEM_ORDER) {
    const item = document.createElement('div');
    item.className = 'ivg-status-bar__item';
    if (key === 'ros_connected') {
      item.classList.add('ivg-status-bar__item--ros');
    }
    item.dataset.key = key;

    const dot = document.createElement('span');
    dot.className = 'ivg-status-bar__dot ivg-status-bar__dot--off';

    const label = document.createElement('span');
    label.className = 'ivg-status-bar__label';
    label.textContent = LABELS[key];

    const val = document.createElement('span');
    val.className = 'ivg-status-bar__value';
    val.textContent = '—';

    item.appendChild(dot);
    item.appendChild(label);
    item.appendChild(val);
    bar.insertBefore(item, spacer);
  }

  // ── Foxglove 统计 (最右侧, spacer 之后) ──
  const foxStats = buildFoxgloveStats();
  bar.appendChild(foxStats);

  return bar;
}

// ── Bridge 选择器 ───────────────────────────────────────────────────────────

function buildBridgeSelector() {
  const container = document.createElement('div');
  container.className = 'ivg-bridge-selector';

  const label = document.createElement('span');
  label.className = 'ivg-bridge-selector__label';
  label.textContent = 'Bridge';

  const select = document.createElement('select');
  select.className = 'ivg-bridge-selector__select';
  select.id = 'ivg-bridge-select';

  const options = [
    { value: 'auto', label: 'Auto' },
    { value: 'foxglove', label: 'Foxglove' },
    { value: 'rosbridge', label: 'Rosbridge' },
  ];
  for (const opt of options) {
    const el = document.createElement('option');
    el.value = opt.value;
    el.textContent = opt.label;
    select.appendChild(el);
  }

  // 连接状态点
  const dot = document.createElement('span');
  dot.className = 'ivg-bridge-selector__dot ivg-bridge-selector__dot--off';
  dot.id = 'ivg-bridge-dot';

  container.appendChild(dot);
  container.appendChild(label);
  container.appendChild(select);

  // 事件: mode 切换 → TransportHub.setMode() (统一入口)
  select.addEventListener('change', () => {
    const mode = select.value;
    const hub = g.__transportHub;
    if (hub) {
      hub.setMode(mode);
    } else {
      localStorage.setItem('ivg_bridge_mode', mode);
    }
  });

  return container;
}

function updateBridgeDot() {
  var dot = document.getElementById('ivg-bridge-dot');
  if (!dot) return;
  var hub = g.__transportHub;
  var activeId = hub ? hub.activeId : null;
  var mode = hub ? hub.mode : 'auto';

  dot.className = 'ivg-bridge-selector__dot';

  // 错误: 模式锁定但 bridge 不可达
  if ((mode === 'foxglove' && !activeId) || (mode === 'rosbridge' && !activeId)) {
    dot.classList.add('ivg-bridge-selector__dot--err');
    dot.title = mode + ' mode — bridge unreachable (locked, no fallback)';
    return;
  }

  if (activeId === 'foxglove') {
    dot.classList.add('ivg-bridge-selector__dot--foxglove');
  } else if (activeId === 'rosbridge') {
    if (mode === 'auto') {
      dot.classList.add('ivg-bridge-selector__dot--degraded');
      dot.title = 'AUTO degraded to rosbridge (foxglove unreachable)';
    } else {
      dot.classList.add('ivg-bridge-selector__dot--rosbridge');
    }
  } else {
    dot.classList.add('ivg-bridge-selector__dot--off');
  }
}

// ── Foxglove 统计 ───────────────────────────────────────────────────────────

function buildFoxgloveStats() {
  const container = document.createElement('div');
  container.className = 'ivg-bridge-stats';
  container.id = 'ivg-bridge-stats';
  container.style.display = 'none';  // 默认隐藏, foxglove 活跃时显示

  const chLabel = document.createElement('span');
  chLabel.className = 'ivg-bridge-stats__item';
  chLabel.id = 'ivg-bridge-stats-ch';

  container.appendChild(chLabel);
  return container;
}

function updateFoxgloveStats() {
  var container = document.getElementById('ivg-bridge-stats');
  if (!container) return;

  var bridgeLogger = g.__bridgeLogger;
  var stats = bridgeLogger ? bridgeLogger.getStats() : null;
  var hub = g.__transportHub;
  var activeId = hub ? hub.activeId : null;

  if (activeId && stats && stats[activeId]) {
    container.style.display = '';
    var s = stats[activeId];
    var chEl = document.getElementById('ivg-bridge-stats-ch');
    if (chEl) {
      var parts = [];
      if (s.subs > 0) parts.push(s.subs + ' subs');
      if (s.pubs > 0) parts.push(s.pubs + ' pubs');
      if (s.svcs > 0) parts.push(s.svcs + ' svcs');
      if (s.errors > 0) parts.push(s.errors + ' err');
      chEl.textContent = parts.join(' ') || activeId;
    }
    if (s.errors > 0) {
      container.classList.add('ivg-bridge-stats--has-errors');
    } else {
      container.classList.remove('ivg-bridge-stats--has-errors');
    }
  } else {
    container.style.display = 'none';
  }
}

// 周期性刷新 Bridge 统计
var _bridgeStatsTimer = null;
function startBridgeStatsTimer() {
  if (_bridgeStatsTimer) return;
  _bridgeStatsTimer = setInterval(() => {
    updateBridgeDot();
    updateFoxgloveStats();
  }, 2000);
}

function updateBar(msg) {
  const bar = document.getElementById(STATUS_BAR_ID);
  if (!bar) return;

  for (const key of ['is_online', 'enable', 'in_motion', 'planning_status']) {
    const item = bar.querySelector(`[data-key="${key}"]`);
    if (!item) continue;

    let raw = msg[key];
    const dot = item.querySelector('.ivg-status-bar__dot');
    const val = item.querySelector('.ivg-status-bar__value');

    if (dot) {
      dot.className = 'ivg-status-bar__dot ' + DOT_CLASS[key](raw);
    }
    if (val) {
      val.textContent = VALUE_MAP[key](raw);
    }
  }
}

var _lastKnownMode = '';

function updateRosConnection(connected) {
  const bar = document.getElementById(STATUS_BAR_ID);
  if (!bar) return;
  const item = bar.querySelector('[data-key="ros_connected"]');
  if (!item) return;
  const dot = item.querySelector('.ivg-status-bar__dot');
  const val = item.querySelector('.ivg-status-bar__value');
  if (dot) {
    dot.className = 'ivg-status-bar__dot ' + DOT_CLASS.ros_connected(connected);
  }
  if (val) {
    val.textContent = VALUE_MAP.ros_connected(connected);
  }
  if (connected) ensureStatus();
}

function updateDriverMode(raw) {
  const bar = document.getElementById(STATUS_BAR_ID);
  if (!bar) return;

  let mode = '';
  if (raw && typeof raw.data === 'string') mode = raw.data;
  else if (typeof raw === 'string') mode = raw;

  if (mode && mode !== _lastKnownMode) {
    _lastKnownMode = mode;
    if (ivgTransport) { ivgTransport._driverMode = mode; }
    logBus.addLog('info', 'system', '驱动模式 = ' + mode
      + (mode === 'real' ? ' (真实硬件)' : ' (仿真模式)'));
  }

  const item = bar.querySelector('[data-key="driver_mode"]');
  if (!item) return;

  const dot = item.querySelector('.ivg-status-bar__dot');
  const val = item.querySelector('.ivg-status-bar__value');

  if (dot) {
    dot.className = 'ivg-status-bar__dot ' + DOT_CLASS.driver_mode(mode);
  }
  if (val) {
    val.className = 'ivg-status-bar__value';
    if (mode === 'real') {
      val.classList.add('ivg-status-bar__value--real');
    } else if (mode === 'simulation') {
      val.classList.add('ivg-status-bar__value--sim');
    }
    val.textContent = VALUE_MAP.driver_mode(mode);
  }
}

function mountBar() {
  if (document.getElementById(STATUS_BAR_ID)) return;
  const bar = buildBar();
  document.body.appendChild(bar);
  document.body.classList.add('has-ivg-status-bar');
}

var _statusSetupDone = false;
var _rosHandlersRegistered = false;

// 命名的 ros 消息处理器，通过 owner='status_bar' 注册，不受其他 owner 清除影响喵~
function _onRobotStatus(msg) { updateBar(msg); }
function _onDriverMode(msg) { updateDriverMode(msg); }

function _registerRosHandlers() {
  if (_rosHandlersRegistered) return;
  ivgTransport.onRosJson(ROBOT_STATUS_TOPIC, _onRobotStatus, 'status_bar');
  ivgTransport.onRosJson(MODE_TOPIC, _onDriverMode, 'status_bar');
  _rosHandlersRegistered = true;
}

function subscribeStatus() {
  ivgTransport.subscribe({
    topic: ROBOT_STATUS_TOPIC,
    msgType: ROBOT_STATUS_TYPE,
    maxHz: 10,
  });
  ivgTransport.subscribe({
    topic: MODE_TOPIC,
    msgType: MODE_TYPE,
    maxHz: 1,
  });
}

function setupStatus() {
  _registerRosHandlers();
  subscribeStatus();
  _statusSetupDone = true;
}

function ensureStatus() {
  if (!(ivgTransport.ros && ivgTransport.ros.isConnected)) return;
  if (!_statusSetupDone) {
    setupStatus();
  }
}

function init() {
  mountBar();

  // Bridge 选择器默认值
  var storedMode = localStorage.getItem('ivg_bridge_mode');
  if (!storedMode) storedMode = 'auto';
  var sel = document.getElementById('ivg-bridge-select');
  if (sel) sel.value = storedMode;

  var _rosConnected = !!(ivgTransport.ros && ivgTransport.ros.isConnected);
  updateRosConnection(_rosConnected);

  function _onControl(ctrl) {
    if (ctrl && ctrl.op === 'connection') {
      var connected = !!(ctrl.connected || (ivgTransport.ros && ivgTransport.ros.isConnected));
      updateRosConnection(connected);
      _statusSetupDone = false;
      if (connected) ensureStatus();
    }
  }
  ivgTransport.onControlJson(_onControl, 'status_bar');

    // TransportHub 事件监听 (EventTarget, 标准事件)
  _bindHubEvents();

  startBridgeStatsTimer();
  ensureStatus();
}

/** TransportHub 加载完成后绑定事件 (零轮询) */
function _bindHubEvents() {
  var hub = g.__transportHub;
  if (!hub || hub._statusBarBound) return;
  hub._statusBarBound = true;

  // modechange: 用户/系统切换 Bridge 模式
  hub.addEventListener('modechange', function(e) {
    var sel = document.getElementById('ivg-bridge-select');
    if (sel && sel.value !== e.detail.mode) sel.value = e.detail.mode;
    updateBridgeDot();
    updateFoxgloveStats();
    updateRosConnection(true);
  });

  // bridgechange: 实际活跃桥接变更 (自动回退等)
  hub.addEventListener('bridgechange', function() {
    updateBridgeDot();
    updateFoxgloveStats();
  });

  // stats: 定期统计更新
  hub.addEventListener('stats', function() {
    updateFoxgloveStats();
  });

  // bridgedegraded: AUTO 模式降级到 rosbridge
  hub.addEventListener('bridgedegraded', function(e) {
    updateBridgeDot();
    updateFoxgloveStats();
    logBus.addLog('warn', 'bridge',
      'AUTO mode degraded to rosbridge (foxglove unreachable)', e.detail, 'bridge');
  });

  // bridgerestored: foxglove 恢复
  hub.addEventListener('bridgerestored', function() {
    updateBridgeDot();
    updateFoxgloveStats();
    logBus.addLog('info', 'bridge',
      'AUTO mode restored to foxglove', {}, 'bridge');
  });

  // bridgeerror: 模式锁定但 bridge 不可达
  hub.addEventListener('bridgeerror', function(e) {
    updateBridgeDot();
    logBus.addLog('error', 'bridge',
      'Bridge error (mode: ' + (e.detail && e.detail.mode) + '): ' + (e.detail && e.detail.error),
      e.detail, 'bridge');
  });

  // 初始状态
  updateBridgeDot();
  updateFoxgloveStats();
}

// DOM ready 后初始化
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
