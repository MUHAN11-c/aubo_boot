// ivg_status_bar — 底部机械臂状态栏
// 订阅 /aubo_driver/robot_status，显示 is_online / enable / in_motion / planning_status
import { ivgTransport } from '../ivg_transport.js';

const TAG = '[ivg_status_bar]';

const STATUS_BAR_ID = 'ivg-status-bar';
const TOPIC_NAME = '/aubo_driver/robot_status';
const MSG_TYPE = 'demo_interface/msg/RobotStatus';

const LABELS = {
  is_online: '在线',
  enable: '使能',
  in_motion: '运动',
  planning_status: '规划',
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
};

function buildBar() {
  const bar = document.createElement('div');
  bar.id = STATUS_BAR_ID;
  bar.className = 'ivg-status-bar';

  const spacer = document.createElement('div');
  spacer.className = 'ivg-status-bar__spacer';
  bar.appendChild(spacer);

  for (const key of ['is_online', 'enable', 'in_motion', 'planning_status']) {
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
    // planning_status is string, others are bool
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

function mountBar() {
  if (document.getElementById(STATUS_BAR_ID)) return;
  const bar = buildBar();
  document.body.appendChild(bar);
  document.body.classList.add('has-ivg-status-bar');
}

function subscribeStatus() {
  const ok = ivgTransport.subscribe({
    topic: TOPIC_NAME,
    msgType: MSG_TYPE,
    maxHz: 10,
  });
  if (!ok) {
    console.warn(TAG, '订阅失败，等待重连...');
    return;
  }
  console.log(TAG, '已订阅', TOPIC_NAME);
}

function init() {
  mountBar();

  // 监听 ROS 消息
  ivgTransport.onRosJson(TOPIC_NAME, (msg) => {
    updateBar(msg);
  });

  // 连接成功后自动订阅
  ivgTransport.onControlJson((ctrl) => {
    if (ctrl.op === 'connection') {
      subscribeStatus();
    }
  });

  // 如果已连接则立即订阅
  if (ivgTransport.isConnected()) {
    subscribeStatus();
  }
}

// DOM ready 后初始化
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}
