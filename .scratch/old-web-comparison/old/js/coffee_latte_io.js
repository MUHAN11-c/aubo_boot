// coffee_latte_io.js — 咖啡拉花 DI 反馈灯 + DO 开关（纯前端展示）
// 依赖: ivgTransport (全局单例)
// 链路: 浏览器 ↔ rosbridge ← /latte_di_status (String)
// 注: DO 按钮仅前端切换 UI 状态，暂不调用后端服务
import { ivgTransport } from './ivg_transport.js';

const TAG = '[latte_io]';

// ── 话题名（可从 settings localStorage 覆盖）──
const STORAGE_KEY = 'ivg_vision_grasp_topics_v3';

function getSetting(key, fallback) {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const obj = JSON.parse(raw);
      if (obj && obj[key] && obj[key].trim()) return obj[key].trim();
    }
  } catch (_) { /* ignore */ }
  return fallback;
}

// ── DI 反馈灯 ──────────────────────────────────────────────────────────────

function setLamp(elId, on) {
  const el = document.getElementById(elId);
  if (!el) return;
  el.setAttribute('data-state', on ? 'on' : 'off');
}

function parseDiMessage(msg) {
  let text = '';
  if (typeof msg === 'string') {
    text = msg;
  } else if (msg && typeof msg.data === 'string') {
    text = msg.data;
  } else if (msg && msg.data !== undefined) {
    text = String(msg.data);
  } else {
    return;
  }

  // JSON: {"di2":1, "di3":0, "di4":1}
  try {
    const obj = JSON.parse(text);
    if (obj && typeof obj === 'object') {
      setLamp('latte-di2-lamp', !!obj.di2);
      setLamp('latte-di3-lamp', !!obj.di3);
      setLamp('latte-di4-lamp', !!obj.di4);
      return;
    }
  } catch (_) { /* fall through */ }

  // CSV: "1,0,1"
  const parts = text.split(',').map(s => s.trim());
  if (parts.length >= 3) {
    setLamp('latte-di2-lamp', parts[0] === '1');
    setLamp('latte-di3-lamp', parts[1] === '1');
    setLamp('latte-di4-lamp', parts[2] === '1');
  }
}

// ── DO 开关（纯 UI 切换，不调后端）─────────────────────────────────────────

function toggleDoButton(btnId) {
  const btn = document.getElementById(btnId);
  if (!btn) return;
  const pressed = btn.getAttribute('aria-pressed') === 'true';
  const next = !pressed;
  btn.setAttribute('aria-pressed', next ? 'true' : 'false');
  btn.textContent = next ? '开' : '关';
  btn.classList.toggle('is-on', next);
}

// ── 订阅 ───────────────────────────────────────────────────────────────────

function subscribeDi() {
  const topic = getSetting('latte-di-topic', '/latte_di_status');

  ivgTransport.onRosJson(topic, msg => parseDiMessage(msg));

  ivgTransport.subscribe({
    topic: topic,
    msgType: 'std_msgs/msg/String',
    maxHz: 10,
  });

  console.log(TAG, '已订阅 DI 状态:', topic);
}

// ── 连接感知 ───────────────────────────────────────────────────────────────

function wireConnection() {
  ivgTransport.onControlJson(ctrl => {
    if (ctrl.op === 'connection' && ivgTransport.isConnected()) {
      subscribeDi();
    }
  });
  if (ivgTransport.isConnected()) {
    subscribeDi();
  }
}

// ── 入口 ───────────────────────────────────────────────────────────────────

(function init() {
  // DO 按钮：纯前端显示切换
  const do2 = document.getElementById('latte-do2-toggle');
  const do4 = document.getElementById('latte-do4-toggle');
  if (do2) do2.addEventListener('click', () => toggleDoButton('latte-do2-toggle'));
  if (do4) do4.addEventListener('click', () => toggleDoButton('latte-do4-toggle'));

  wireConnection();
  console.log(TAG, '咖啡拉花 IO 显示就绪（DI 订阅 + DO 纯前端切换）');
})();
