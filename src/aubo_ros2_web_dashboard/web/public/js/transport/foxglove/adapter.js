// transport/foxglove/adapter.js — FoxgloveAdapter implements MessageAdapter
//
// 职责:
//   - CDR 消息流水线: binary → MessageReader (官方 @foxglove/rosmsg2-serialization) → TypedMessage → dispatch
//   - 客户端发布 (clientPublish): JS → MessageWriter → CDR binary
//   - 服务调用: 通过 foxglove_bridge services capability
//
// 参考:
//   test_all.mjs — 官方 @foxglove 包用法 (parse, MessageReader, MessageWriter)
//   VALIDATION_REPORT.md — AUBO E5 实测数据

import { parse, MessageReader, MessageWriter } from './vendor/foxglove_cdr.js';
import { foxgloveClient } from './client.js';
import { MessageAdapter } from '../adapter.js';

const g = globalThis;

class FoxgloveAdapter extends MessageAdapter {
  constructor() {
    super();
    this._client = foxgloveClient;
    this._readers = new Map();       // topic → { reader, schemaText } (schema变更时重建)
    this._writers = new Map();       // topic → { writer, schemaText }
    this._subs = new Map();          // topic → { subId, msgType, callbacks: Set }
    this._messageCbs = new Set();
    this._statusCbs = new Set();
    this._status = { state: 'disconnected', since: Date.now() };
    this._stats = { channelsCount: 0, servicesCount: 0, bytesReceived: 0, messagesReceived: 0, connectedAt: null };
    this._clientChanIds = new Map(); // topic → channelId (for clientPublish)
  }

  get id() { return 'foxglove'; }
  get isConnected() { return this._client.isConnected; }

  // ── 连接管理 ──────────────────────────────────────────────────────────────

  async connect(url) {
    this._setStatus('connecting');
    this._client.onConnect((serverInfo) => {
      this._stats.connectedAt = Date.now();
      this._stats.channelsCount = this._client.channels.size;
      this._setStatus('connected');
    });
    this._client.onClose(() => {
      this._setStatus('disconnected');
      this._stats.connectedAt = null;
    });
    this._client.onError(() => {
      this._setStatus('error');
    });
    try {
      await this._client.connect(url);
    } catch (e) {
      this._setStatus('error', e.message);
      throw e;
    }
  }

  disconnect() {
    this._client.disconnect();
    this._setStatus('disconnected');
  }

  // ── 话题订阅 ──────────────────────────────────────────────────────────────

  subscribe(topic, msgType, callback) {
    if (this._subs.has(topic)) {
      const sub = this._subs.get(topic);
      sub.callbacks.add(callback);
      return { cancel: () => this.unsubscribe(topic) };
    }

    const callbacks = new Set([callback]);
    const self = this;

    const clientSubId = this._client.subscribe(topic, msgType, (payload, meta) => {
      self._stats.messagesReceived++;
      self._stats.bytesReceived += payload.byteLength;

      const ch = self._client.getChannel(topic);
      const isJson = ch && ch.encoding === 'json';

      let data;
      if (isJson) {
        try {
          data = JSON.parse(new TextDecoder().decode(payload));
        } catch (_) { return; }
      } else {
        try {
          const reader = self._getReader(topic);
          data = reader.readMessage(payload);
        } catch (e) {
          console.warn(`[FoxgloveAdapter] CDR deserialize failed for ${topic}:`, e);
          return;
        }
      }

      const typedMsg = {
        topic,
        type: msgType,
        data,
        timestamp: meta.tsMs,
        bridge: 'foxglove',
      };

      for (const cb of callbacks) {
        try { cb(typedMsg); } catch (_) {}
      }
      for (const cb of self._messageCbs) {
        try { cb(typedMsg); } catch (_) {}
      }
    });

    this._subs.set(topic, {
      subId: clientSubId,
      msgType,
      callbacks,
    });

    return { cancel: () => this.unsubscribe(topic) };
  }

  unsubscribe(topic) {
    const sub = this._subs.get(topic);
    if (!sub) return;
    if (sub.subId !== null) {
      this._client.unsubscribe(sub.subId);
    }
    this._subs.delete(topic);
  }

  // ── 客户端发布 ────────────────────────────────────────────────────────────

  async publish(topic, msgType, msg) {
    if (!this._client.isConnected) throw new Error('foxglove_not_connected');

    if (!this._clientChanIds.has(topic)) {
      const ch = this._client.getChannel(topic);
      const schema = ch ? ch.schema : '';
      const channelId = this._client.clientAdvertise(topic, msgType, schema);
      this._clientChanIds.set(topic, channelId);
    }

    const writer = this._getWriter(topic);
    const cdrData = writer.writeMessage(msg);

    const channelId = this._clientChanIds.get(topic);
    this._client.sendMessage(channelId, cdrData);
  }

  // ── 服务调用 ──────────────────────────────────────────────────────────────

  async callService(name, type, request, timeoutMs = 60000) {
    if (!this._client.isConnected) throw new Error('foxglove_not_connected');

    const callId = this._client._callIdCounter + 1;
    this._client._callIdCounter = callId;

    const serializer = this._getServiceWriter(name, type);
    const cdrData = serializer.writeMessage(request);

    return new Promise((resolve, reject) => {
      this._client.registerServiceCall(callId, resolve, reject, timeoutMs);
      this._client.sendServiceCallRequest(name, callId, cdrData);
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
    return new Set(this._client.getCapabilities());
  }

  getStats() {
    this._stats.channelsCount = this._client.channels.size;
    this._stats.servicesCount = this._client.services.size;
    return { ...this._stats };
  }

  // ── 内部: MessageReader/MessageWriter 缓存 ────────────────────────────────

  /** 获取或创建 MessageReader (仅 schema 变更时重建) */
  _getReader(topic) {
    const ch = this._client.getChannel(topic);
    const schema = ch ? ch.schema : '';
    if (!schema) throw new Error(`No schema for topic ${topic}`);

    const cached = this._readers.get(topic);
    if (cached && cached.schemaText === schema) return cached.reader;

    const defs = parse(schema, { ros2: true });
    const reader = new MessageReader(defs);
    this._readers.set(topic, { reader, schemaText: schema });
    return reader;
  }

  /** 获取或创建 MessageWriter (仅 schema 变更时重建) */
  _getWriter(topic) {
    const ch = this._client.getChannel(topic);
    const schema = ch ? ch.schema : '';
    if (!schema) throw new Error(`No schema for topic ${topic}`);

    const cached = this._writers.get(topic);
    if (cached && cached.schemaText === schema) return cached.writer;

    const defs = parse(schema, { ros2: true });
    const writer = new MessageWriter(defs);
    this._writers.set(topic, { writer, schemaText: schema });
    return writer;
  }

  /** 服务请求序列化: 从 foxglove services 注册表获取 schema */
  _getServiceWriter(serviceName, type) {
    for (const [, svc] of this._client.services) {
      if (svc.name === serviceName && svc.schema) {
        const defs = parse(svc.schema, { ros2: true });
        return new MessageWriter(defs);
      }
    }
    throw new Error(`No schema found for service ${serviceName}`);
  }

  // ── 内部 ──────────────────────────────────────────────────────────────────

  _setStatus(state, error) {
    this._status = { state, error, since: Date.now() };
    for (const cb of this._statusCbs) {
      try { cb({ ...this._status }); } catch (_) {}
    }
  }
}

const foxgloveAdapter = new FoxgloveAdapter();
g.foxgloveAdapter = foxgloveAdapter;

export { FoxgloveAdapter, foxgloveAdapter };
