// ros_health_monitor.js — ROS 连接健康监视器（心跳/延迟/图谱/消息率）
//
// 挂在状态栏右侧，实时显示：
//   连接状态（● 绿=已连接 / ● 黄=重连中 / ● 红=断开）
//   心跳延迟（对 /rosout 或 /joint_states 的最新消息到达间隔做 EMA）
//   消息率（player.stats.messagesReceived 差分）
//   图谱计数（topics/services/nodes，经 connectionGraph）
//   重连次数与最近重连时间
//
// 2026-10-13 新增，配合 foxglove 唯一传输面。

import { foxgloveRosClient } from '../transport/foxglove/ros_client.js';

const g = globalThis;

function createRosHealthMonitor() {
  const client = foxgloveRosClient();
  let _el = null;
  let _timer = null;
  let _lastMsgCount = 0;
  let _lastCheckAt = Date.now();
  let _msgRate = 0;
  let _latencyMs = 0;
  let _lastMsgAt = 0;
  let _reconnectCount = 0;
  let _lastReconnectAt = null;
  let _graphSubscribed = false;
  let _mounted = false;

  function _ensureEl() {
    if (_el) return _el;
    const existing = document.getElementById('ros-health-monitor');
    if (existing) { _el = existing; return _el; }
    _el = document.createElement('div');
    _el.id = 'ros-health-monitor';
    _el.className = 'ros-health-monitor';
    _el.innerHTML = `
      <span class="rhm-dot rhm-dot--off" id="rhm-dot"></span>
      <span class="rhm-item" id="rhm-status">连接中…</span>
      <span class="rhm-item rhm-sep">|</span>
      <span class="rhm-item" id="rhm-latency">— ms</span>
      <span class="rhm-sep">|</span>
      <span class="rhm-item" id="rhm-rate">0 msg/s</span>
      <span class="rhm-sep">|</span>
      <span class="rhm-item" id="rhm-graph">0T 0S 0N</span>
      <span class="rhm-sep">|</span>
      <span class="rhm-item" id="rhm-reconnect">重连 0 次</span>
    `;
    return _el;
  }

  function _update() {
    if (!_el) return;
    const state = client.state;
    const dot = _el.querySelector('#rhm-dot');
    const statusEl = _el.querySelector('#rhm-status');
    const latencyEl = _el.querySelector('#rhm-latency');
    const rateEl = _el.querySelector('#rhm-rate');
    const graphEl = _el.querySelector('#rhm-graph');
    const reconnectEl = _el.querySelector('#rhm-reconnect');

    // 状态与颜色
    let cls = 'rhm-dot--off';
    let text = '断开';
    if (state === 'connected') { cls = 'rhm-dot--on'; text = '已连接'; }
    else if (state === 'reconnecting') { cls = 'rhm-dot--warn'; text = '重连中'; }
    else if (state === 'connecting') { cls = 'rhm-dot--warn'; text = '连接中'; }
    if (dot) dot.className = 'rhm-dot ' + cls;
    if (statusEl) statusEl.textContent = text;

    // 消息率（差分）
    const now = Date.now();
    const dt = (now - _lastCheckAt) / 1000;
    if (dt > 0.5) {
      const current = client.player.stats.messagesReceived;
      _msgRate = Math.round((current - _lastMsgCount) / dt);
      _lastMsgCount = current;
      _lastCheckAt = now;
    }
    if (rateEl) rateEl.textContent = _msgRate + ' msg/s';

    // 心跳延迟（最新消息距今的时间，作为链路活跃度指标）
    if (_lastMsgAt > 0) {
      const silent = now - _lastMsgAt;
      if (latencyEl) {
        latencyEl.textContent = (silent < 2000 ? silent + ' ms' : '>2s (静默)');
        latencyEl.style.color = silent < 1000 ? '' : silent < 3000 ? '#fbbf24' : '#f87171';
      }
    } else if (latencyEl) {
      latencyEl.textContent = state === 'connected' ? '等待消息' : '—';
    }

    // 图谱
    if (!_graphSubscribed && state === 'connected') {
      try {
        client.player.subscribeConnectionGraph();
        _graphSubscribed = true;
      } catch (_) { /* capability 未开启 */ }
    }
    if (_graphSubscribed) {
      const graph = client.getGraph();
      if (graphEl) graphEl.textContent = `${graph.topics.length}T ${graph.services.length}S ${graph.nodes.length}N`;
    }

    // 重连
    const reconnects = client.player.stats.reconnects;
    if (reconnects > _reconnectCount) {
      _reconnectCount = reconnects;
      _lastReconnectAt = now;
    }
    if (reconnectEl) {
      reconnectEl.textContent = _reconnectCount > 0
        ? `重连 ${_reconnectCount} 次${_lastReconnectAt ? ' (' + Math.round((now - _lastReconnectAt) / 1000) + 's前)' : ''}`
        : '稳定';
    }
  }

  // 监听消息到达（心跳源）
  function _onMessage() {
    _lastMsgAt = Date.now();
  }

  function mount(parentSelector) {
    if (_mounted) return;
    const parent = document.querySelector(parentSelector || '.ivg-status-bar') || document.body;
    const el = _ensureEl();
    // 插到状态栏（或 body）尾部
    parent.appendChild(el);
    _mounted = true;

    // 注入样式
    if (!document.getElementById('ros-health-monitor-style')) {
      const style = document.createElement('style');
      style.id = 'ros-health-monitor-style';
      style.textContent = `
        .ros-health-monitor {
          display: inline-flex; align-items: center; gap: 4px;
          font-size: 11px; color: #aaa; padding: 0 8px;
          font-family: monospace, monospace;
        }
        .rhm-dot {
          width: 8px; height: 8px; border-radius: 50%; display: inline-block;
          transition: background 0.3s;
        }
        .rhm-dot--on { background: #4ade80; box-shadow: 0 0 4px #4ade80; }
        .rhm-dot--warn { background: #fbbf24; box-shadow: 0 0 4px #fbbf24; animation: rhm-pulse 1s infinite; }
        .rhm-dot--off { background: #f87171; }
        .rhm-item { white-space: nowrap; }
        .rhm-sep { color: #555; margin: 0 2px; }
        @keyframes rhm-pulse { 50% { opacity: 0.5; } }
      `;
      document.head.appendChild(style);
    }

    // 订阅一条高频消息作为心跳源
    client.subscribe('/joint_states', 'sensor_msgs/msg/JointState', _onMessage, { dispatchMode: 'latest' });
    client.onStateChange((s) => {
      if (s === 'connected') {
        // 重连后重新订阅图谱
        _graphSubscribed = false;
      }
    });

    _timer = setInterval(_update, 1000);
    _update();
  }

  function unmount() {
    if (_timer) { clearInterval(_timer); _timer = null; }
    if (_el) { _el.remove(); _el = null; }
    _mounted = false;
  }

  function getSnapshot() {
    return {
      state: client.state,
      msgRate: _msgRate,
      latencyMs: _lastMsgAt > 0 ? Date.now() - _lastMsgAt : -1,
      reconnects: _reconnectCount,
      stats: { ...client.player.stats },
    };
  }

  return { mount, unmount, getSnapshot };
}

export { createRosHealthMonitor };
