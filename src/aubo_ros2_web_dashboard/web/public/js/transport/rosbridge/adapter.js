// transport/rosbridge/adapter.js — RosbridgeAdapter implements MessageAdapter
//
// 包装现有 ivgTransport (ROSLIB.Ros) 到 MessageAdapter 接口,
// JSON 消息直接透传 (data 已为 JS 对象, 无需反序列化)。
//
// 职责:
//   - 桥接 ivgTransport API ↔ MessageAdapter 接口
//   - 统一 TypedMessage 输出 (bridge='rosbridge')

import { ivgTransport } from '../../ivg_transport.js';
import { rosbridgeWebSocketUrl } from '../../ivg_runtime.js';
import { MessageAdapter } from '../adapter.js';

const g = globalThis;

class RosbridgeAdapter extends MessageAdapter {
  constructor() {
    super();
    this._transport = ivgTransport;
    this._messageCbs = new Set();
    this._statusCbs = new Set();
    this._status = { state: 'disconnected', since: Date.now() };
    this._stats = { channelsCount: 0, servicesCount: 0, bytesReceived: 0, messagesReceived: 0, connectedAt: null };
    this._connected = false;

    // 监听 ivgTransport 的 control 消息
    this._transport.onControlJson((ctrl) => {
      if (ctrl && ctrl.op === 'connection') {
        const prev = this._connected;
        this._connected = !!(ctrl.connected || (this._transport.ros && this._transport.ros.isConnected));
        if (this._connected !== prev) {
          this._stats.connectedAt = this._connected ? Date.now() : null;
          this._setStatus(this._connected ? 'connected' : 'disconnected');
        }
      }
    }, 'rosbridge_adapter');
  }

  get id() { return 'rosbridge'; }
  get isConnected() { return this._transport.isConnected(); }

  // ── 连接管理 ──────────────────────────────────────────────────────────────

  async connect(url) {
    this._setStatus('connecting');
    try {
      await this._transport.loadRuntime();
      await this._transport.connectControl();
      this._connected = true;
      this._stats.connectedAt = Date.now();
      this._setStatus('connected');
    } catch (e) {
      this._setStatus('error', e.message);
      throw e;
    }
  }

  disconnect() {
    this._transport.close();
    this._connected = false;
    this._setStatus('disconnected');
  }

  // ── 话题订阅 ──────────────────────────────────────────────────────────────

  subscribe(topic, msgType, callback) {
    // 注册 topic handler (owner='rosbridge_adapter')
    this._transport.onRosJson(topic, (msg) => {
      this._stats.messagesReceived++;
      const typedMsg = {
        topic,
        type: msgType,
        data: msg,
        timestamp: Date.now(),
        bridge: 'rosbridge',
      };
      try { callback(typedMsg); } catch (_) {}
      for (const cb of this._messageCbs) {
        try { cb(typedMsg); } catch (_) {}
      }
    }, 'rosbridge_adapter');

    // 实际订阅
    const ok = this._transport.subscribe({ topic, msgType });
    if (!ok) return null;

    return { cancel: () => this.unsubscribe(topic) };
  }

  unsubscribe(topic) {
    this._transport.unsubscribe(topic);
  }

  // ── 发布 + 服务调用 (委托) ───────────────────────────────────────────────

  async publish(topic, msgType, msg) {
    return this._transport.publish({ topic, type: msgType, msg });
  }

  async callService(name, type, request, timeoutMs = 60000) {
    return this._transport.callService({
      service: name,
      type,
      request,
      timeoutMs,
    });
  }

  // ── 事件 ──────────────────────────────────────────────────────────────────

  onMessage(callback) {
    this._messageCbs.add(callback);
    return () => this._messageCbs.delete(callback);
  }

  onStatusChange(callback) {
    this._statusCbs.add(callback);
    return () => this._statusCbs.delete(callback);
  }

  getCapabilities() {
    // rosbridge 支持 publish + service call + topic list
    return new Set(['clientPublish']);
  }

  getStats() { return { ...this._stats }; }

  // ── 内部 ──────────────────────────────────────────────────────────────────

  _setStatus(state, error) {
    this._status = { state, error, since: Date.now() };
    for (const cb of this._statusCbs) {
      try { cb({ ...this._status }); } catch (_) {}
    }
  }
}

const rosbridgeAdapter = new RosbridgeAdapter();
g.rosbridgeAdapter = rosbridgeAdapter;

export { RosbridgeAdapter, rosbridgeAdapter };
