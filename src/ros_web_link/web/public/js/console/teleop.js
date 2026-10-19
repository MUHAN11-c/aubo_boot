// console/teleop.js — 键盘遥控面板（官方 TB3 仿真 /cmd_vel；仅限仿真）
//
// 交互语义（Foxglove Teleop / teleop_twist_keyboard 折中）：
//   - WASD/方向键合键 → mixTeleopCommand 纯核（viz_core）→ Twist
//   - 50ms 节流发布；松键（空集）立即发零速急停
//   - 使能开关 + 红色停止钮；卡片失焦自动停止
// 纯核 mixTeleopCommand 在 viz_core.js（vitest 覆盖），本模块只做 I/O 接线。

import { mixTeleopCommand, teleopMessage } from './viz_core.js';

const $ = (id) => document.getElementById(id);
const PUB_MS = 50;

/** 遥控键盘集合（仅当使能且指针悬停在卡片上时采集，避免全局抢键） */
export function initTeleop(getClient) {
  const keys = new Set();
  let armed = false;
  let timer = null;
  let last = '';

  const card = $('sec-teleop');
  const state = () => $('teleop-state');
  const params = () => ({
    maxLin: parseFloat($('teleop-lin').value) || 0.2,
    maxAng: parseFloat($('teleop-ang').value) || 1.0,
  });

  const publish = (twist) => {
    const c = getClient();
    if (!c?.isConnected) {
      state().textContent = '未连接 · 未发布';
      return;
    }
    // 方言自动识别：果园仿真桥 /cmd_vel 是 Twist；nav2/TB3 常为 TwistStamped。
    // 选择器可强制；auto 时读图上 /cmd_vel 的 schema，再回退 Twist。
    const sel = $('teleop-msg-type')?.value || 'auto';
    let type = sel;
    if (sel === 'auto') {
      const schema = c.player?.channels?.get?.('/cmd_vel')?.schemaName || '';
      type = schema.endsWith('TwistStamped') ? 'TwistStamped' : 'Twist';
    }
    const msg = teleopMessage(type, twist, 'base_link');
    const ok = c.publish('/cmd_vel', 'geometry_msgs/msg/' + type, msg);
    state().textContent = (ok ? '发布中' : '发布失败')
      + ` · ${type} · vx=${twist.linear.x} wz=${twist.angular.z}`;
  };

  const tick = () => {
    const { maxLin, maxAng } = params();
    const { active, twist } = mixTeleopCommand(keys, maxLin, maxAng);
    if (active || last !== '') publish(twist);
    last = active ? 'run' : '';
    if (!active && timer) {
      // 已发零速急停，停表
      clearInterval(timer);
      timer = null;
    }
  };

  const ensureTimer = () => {
    if (!timer) {
      timer = setInterval(tick, PUB_MS);
      tick();
    }
  };

  const stop = () => {
    keys.clear();
    if (timer) { clearInterval(timer); timer = null; }
    publish({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
    last = '';
    state().textContent = armed ? '已使能 · 已急停归零' : '未使能 · 未发布';
  };

  // 键盘采集：仅当使能且指针悬停在卡片上时，避免全局抢键
  card.addEventListener('pointerenter', () => { card.dataset.hover = '1'; });
  card.addEventListener('pointerleave', () => { card.dataset.hover = ''; });

  window.addEventListener('keydown', (e) => {
    if (!armed || card.dataset.hover !== '1') return;
    const tag = e.target?.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
    const k = e.key.toLowerCase();
    const hit = ['w', 'a', 's', 'd', 'q', 'e', 'arrowup', 'arrowdown',
      'arrowleft', 'arrowright'].includes(k);
    if (!hit) return;
    e.preventDefault();
    keys.add(k);
    ensureTimer();
  });
  window.addEventListener('keyup', (e) => {
    keys.delete(e.key.toLowerCase());
    if (armed && keys.size === 0 && timer) tick();  // 触发零速急停 + 停表
  });
  window.addEventListener('blur', () => { if (armed) stop(); });

  $('teleop-arm').addEventListener('change', (e) => {
    armed = e.target.checked;
    if (!armed) stop();
    else state().textContent = '已使能 · 悬停本卡按 WASD 驱动';
  });
  $('teleop-stop').addEventListener('click', stop);
  $('teleop-lin').addEventListener('input', () => {
    $('teleop-lin-v').textContent = (+$('teleop-lin').value).toFixed(2);
  });
  $('teleop-ang').addEventListener('input', () => {
    $('teleop-ang-v').textContent = (+$('teleop-ang').value).toFixed(1);
  });
}
