// SPDX-License-Identifier: Apache-2.0
// minimal_client.js — ros_web_link 浏览器最小调用示例（无构建 ES Modules）
import {
  foxgloveRosClient,
  createActionClient,
} from '/js/ros_web_link.js';

const $ = (id) => document.getElementById(id);
const client = foxgloveRosClient();
let sub = null;
let action = null;

function log(el, ...parts) {
  const line = parts.map((p) => (typeof p === 'string' ? p : JSON.stringify(p)))
    .join(' ');
  el.textContent = (el.textContent === '—' ? '' : el.textContent + '\n')
    + `[${new Date().toLocaleTimeString()}] ${line}`;
  el.scrollTop = el.scrollHeight;
}

function setState() {
  const on = client.isConnected;
  $('state').classList.toggle('on', on);
  $('state-text').textContent = on ? '已连接' : '未连接';
  $('connect').disabled = on;
  $('disconnect').disabled = !on;
}

$('url').value = `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws/foxglove`;

client.onStateChange((s) => {
  setState();
  log($('op-log'), `state: ${s}`);
});
setState();

$('connect').addEventListener('click', async () => {
  try {
    await client.connect($('url').value.trim());
    log($('op-log'), 'connected, capabilities =', client.player?.capabilities || []);
  } catch (e) {
    log($('op-log'), 'connect failed:', e.message || String(e));
  }
  setState();
});

$('disconnect').addEventListener('click', () => {
  sub?.cancel?.();
  sub = null;
  client.disconnect();
  setState();
});

$('subscribe').addEventListener('click', async () => {
  if (!client.isConnected) return log($('sub-log'), '先连接');
  sub?.cancel?.();
  try {
    sub = client.subscribe($('topic').value.trim(), 'std_msgs/msg/String',
      (msg) => log($('sub-log'), 'message:', msg.data));
    log($('sub-log'), 'subscribed:', $('topic').value.trim());
  } catch (e) {
    log($('sub-log'), 'subscribe failed:', e.message || String(e));
  }
});

$('unsubscribe').addEventListener('click', () => {
  sub?.cancel?.();
  sub = null;
  log($('sub-log'), 'unsubscribed');
});

$('publish').addEventListener('click', () => {
  if (!client.isConnected) return log($('op-log'), '先连接');
  const ok = client.publish('/rwl_demo/echo', 'std_msgs/msg/String',
    { data: $('echo').value });
  log($('op-log'), ok ? 'published /rwl_demo/echo' : 'publish failed');
});

$('service').addEventListener('click', async () => {
  if (!client.isConnected) return log($('op-log'), '先连接');
  try {
    const r = await client.callService('/rwl_demo/add_two_ints',
      { a: 20, b: 22 }, { timeoutMs: 8000 });
    log($('op-log'), 'service response:', { sum: String(r.sum) });
  } catch (e) {
    log($('op-log'), 'service failed:', e.message || String(e));
  }
});

$('params').addEventListener('click', async () => {
  if (!client.isConnected) return log($('op-log'), '先连接');
  try {
    const p = await client.getParams($('node').value.trim(), [$('param').value.trim()]);
    log($('op-log'), 'params:', p);
  } catch (e) {
    log($('op-log'), 'params failed:', e.message || String(e));
  }
});

$('action-send').addEventListener('click', async () => {
  if (!client.isConnected) return log($('action-log'), '先连接');
  try {
    action = createActionClient($('action').value.trim());
    const { status, result } = await action.sendGoal({ order: 8 }, {
      onFeedback: (fb) => log($('action-log'), 'feedback:', fb.sequence),
      resultTimeoutMs: 30000,
    });
    log($('action-log'), 'result:', { status, result });
  } catch (e) {
    log($('action-log'), 'action failed:', e.message || String(e));
  } finally {
    action?.dispose?.();
    action = null;
  }
});

window.addEventListener('beforeunload', () => {
  sub?.cancel?.();
  client.disconnect();
});
