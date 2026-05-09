// ivg_status_bar — 底部机械臂状态栏
// 订阅 /aubo_driver/robot_status + /aubo/mode，显示在线/使能/运动/规划/模式
import { ivgTransport } from '../ivg_transport.js';
import { ROBOT_STATUS_TOPIC, ROBOT_STATUS_TYPE, MODE_TOPIC, MODE_TYPE } from './topics.js';

const TAG = '[ivg_status_bar]';

const STATUS_BAR_ID = 'ivg-status-bar';
// ROBOT_STATUS_TOPIC/ROBOT_STATUS_TYPE/MODE_TOPIC/MODE_TYPE 已从 topics.js 导入

const LABELS = {
  is_online: '在线',
  enable: '使能',
  in_motion: '运动',
  planning_status: '规划',
  driver_mode: '模式',
};

const VALUE_MAP = {
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

const ITEM_ORDER = ['is_online', 'enable', 'in_motion', 'planning_status', 'driver_mode'];

function buildBar() {
  const bar = document.createElement('div');
  bar.id = STATUS_BAR_ID;
  bar.className = 'ivg-status-bar';

  const spacer = document.createElement('div');
  spacer.className = 'ivg-status-bar__spacer';
  bar.appendChild(spacer);

  for (const key of ITEM_ORDER) {
    const item = document.createElement('div');
    item.className = 'ivg-status-bar__item';
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

  return bar;
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

function updateDriverMode(raw) {
  const bar = document.getElementById(STATUS_BAR_ID);
  if (!bar) return;

  let mode = '';
  if (raw && typeof raw.data === 'string') mode = raw.data;
  else if (typeof raw === 'string') mode = raw;

  if (mode && mode !== _lastKnownMode) {
    _lastKnownMode = mode;
    // 同步到全局 transport，供 services 等模块读取
    if (ivgTransport) { ivgTransport._driverMode = mode; }
    console.log(TAG, '驱动模式 =', mode,
      mode === 'real' ? '(真实硬件)' : '(仿真模式)');
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

var _statusSetupDone = false;

function setupStatus() {
  ivgTransport.onRosJson(ROBOT_STATUS_TOPIC, function (msg) { updateBar(msg); });
  ivgTransport.onRosJson(MODE_TOPIC, function (msg) { updateDriverMode(msg); });
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
  ensureStatus();

  // 重连后 rosbridge 侧订阅已清空，需要重新注册 handler + subscribe
  ivgTransport.onControlJson(function (ctrl) {
    if (ctrl && ctrl.op === 'connection') {
      _statusSetupDone = false;
      ensureStatus();
    }
  });
}

// DOM ready 后初始化
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
