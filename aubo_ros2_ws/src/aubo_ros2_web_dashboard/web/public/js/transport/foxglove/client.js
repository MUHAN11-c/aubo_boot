// transport/foxglove/client.js — Foxglove WebSocket 客户端 (foxglove.sdk.v1 协议)
//
// 职责:
//   - WebSocket 连接管理 (子协议: foxglove.sdk.v1)
//   - JSON 控制帧 (serverInfo/advertise/unadvertise/advertiseServices)
//   - CDR 二进制帧 (MESSAGE_DATA / SERVICE_CALL_RESPONSE)
//   - Channel 注册表 (topic ↔ channelId)
//   - 服务调用帧
//
// 参考:
//   PROTOCOL.md: foxglove.sdk.v1 子协议确认
//   VALIDATION_REPORT.md: AUBO E5 实测 45 channels (44 CDR+1 JSON)
//   test_all.mjs: channel ID 动态重建 200ms 轮询策略
//   refs/ws-protocol/typescript/ws-protocol/src/FoxgloveClient.ts

const g = globalThis;

// ── 协议常量 ──────────────────────────────────────────────────────────────────

const SUBPROTOCOL = 'foxglove.sdk.v1';

const OP_MESSAGE_DATA          = 0x01;  // 话题数据 (CDR binary)
const OP_TIME                  = 0x02;  // 时间同步
const OP_SERVICE_CALL_RESPONSE = 0x03;  // 服务调用响应 (CDR binary)
const OP_FETCH_ASSET_RESPONSE  = 0x04;  // 资产数据

// ── FoxgloveClient ────────────────────────────────────────────────────────────

class FoxgloveClient {
  constructor() {
    this._ws = null;
    this._url = '';
    this._subIdCounter = 0;
    this._callIdCounter = 0;

    // 订阅: subId → { channelId, topic, callback }
    this._subscriptions = new Map();

    // Channel 注册表: channelId → { topic, schemaName, schema, encoding, schemaEncoding }
    this._channels = new Map();

    // 服务注册表: serviceId → { name, type, schema }
    this._services = new Map();

    // 服务调用: callId → { resolve, reject, timer }
    this._pendingCalls = new Map();

    // 等待 channel advertise 的订阅: topic → [{ msgType, callback }]
    this._pendingSubs = new Map();

    // retry 定时器
    this._retryTimer = null;

    // 状态
    this._isConnected = false;
    this._serverInfo = null;

    // 连接 Promise (去重)
    this._connectPromise = null;

    // 事件
    this._onConnectHandlers = [];
    this._onCloseHandlers = [];
    this._onErrorHandlers = [];
  }

  // ── 查询接口 ──────────────────────────────────────────────────────────────

  get isConnected() { return this._isConnected; }
  get serverInfo() { return this._serverInfo; }
  get channels() { return this._channels; }
  get services() { return this._services; }

  getChannel(topic) {
    for (const [, ch] of this._channels) {
      if (ch.topic === topic) return ch;
    }
    return null;
  }

  getChannelId(topic) {
    for (const [id, ch] of this._channels) {
      if (ch.topic === topic) return id;
    }
    return null;
  }

  getCapabilities() {
    return this._serverInfo?.capabilities ?? [];
  }

  // ── 连接管理 ──────────────────────────────────────────────────────────────

  connect(url) {
    if (this._connectPromise) return this._connectPromise;
    this._url = url;

    const self = this;
    this._connectPromise = new Promise((resolve, reject) => {
      let settled = false;
      function ok() { if (!settled) { settled = true; resolve(); } }
      function fail(e) { if (!settled) { settled = true; reject(e); } }

      try {
        if (self._ws) {
          try { self._ws.close(); } catch (_) { /* ignore */ }
          self._ws = null;
        }

        const ws = new WebSocket(url, [SUBPROTOCOL]);
        ws.binaryType = 'arraybuffer';
        self._ws = ws;

        ws.onopen = () => {
          // 等待 serverInfo 才算就绪
        };

        ws.onmessage = (event) => {
          if (typeof event.data === 'string') {
            try {
              const obj = JSON.parse(event.data);
              self._handleJson(obj);
              if (obj.op === 'serverInfo') {
                self._isConnected = true;
                self._onConnectHandlers.forEach(fn => { try { fn(self._serverInfo); } catch (_) {} });
                ok();
              }
            } catch (_) { /* not JSON */ }
          } else if (event.data instanceof ArrayBuffer) {
            self._handleBinary(new Uint8Array(event.data));
          }
        };

        ws.onerror = (err) => {
          self._isConnected = false;
          self._onErrorHandlers.forEach(fn => { try { fn(err); } catch (_) {} });
          if (!settled) fail(new Error('foxglove_ws_error'));
        };

        ws.onclose = () => {
          self._isConnected = false;
          self._ws = null;
          self._onCloseHandlers.forEach(fn => { try { fn(); } catch (_) {} });
          if (!settled) fail(new Error('foxglove_ws_closed'));
        };
      } catch (e) {
        fail(e);
      }
    }).finally(() => {
      self._connectPromise = null;
    });

    return this._connectPromise;
  }

  disconnect() {
    this._clearRetry();
    this._connectPromise = null;
    this._subscriptions.clear();
    this._channels.clear();
    this._services.clear();
    this._pendingSubs.clear();
    this._isConnected = false;
    this._serverInfo = null;
    try { if (this._ws) this._ws.close(); } catch (_) { /* */ }
    this._ws = null;
  }

  // ── JSON 控制帧处理 ──────────────────────────────────────────────────────

  _handleJson(obj) {
    switch (obj.op) {
      case 'serverInfo':
        this._serverInfo = {
          name: obj.name || '',
          capabilities: obj.capabilities || [],
          supportedEncodings: obj.supportedEncodings || [],
          metadata: obj.metadata || {},
        };
        break;

      case 'advertise':
        if (Array.isArray(obj.channels)) {
          for (const ch of obj.channels) {
            this._channels.set(ch.id, {
              topic: ch.topic,
              schemaName: ch.schemaName || '',
              schema: ch.schema || '',
              encoding: ch.encoding || 'cdr',
              schemaEncoding: ch.schemaEncoding || 'ros2msg',
            });
            // 补发等待中的订阅
            this._fulfillPending(ch);
          }
        }
        break;

      case 'unadvertise':
        if (Array.isArray(obj.channelIds)) {
          for (const cid of obj.channelIds) {
            this._channels.delete(cid);
          }
        }
        break;

      case 'advertiseServices':
        if (Array.isArray(obj.services)) {
          for (const svc of obj.services) {
            this._services.set(svc.id, {
              name: svc.name,
              type: svc.type,
              schema: svc.schema || '',
            });
          }
        }
        break;

      case 'unadvertiseServices':
        if (Array.isArray(obj.serviceIds)) {
          for (const sid of obj.serviceIds) {
            this._services.delete(sid);
          }
        }
        break;

      case 'serviceCallResponse':
        this._resolveServiceCall(obj.callId, obj);
        break;

      case 'serviceCallFailure':
        this._rejectServiceCall(obj.callId, new Error(obj.message || 'service_call_failed'));
        break;

      case 'parameterValues':
        // 参数读取结果 — 由上层 adapter 处理
        break;

      case 'connectionGraphUpdate':
        // 拓扑更新 — 由上层 adapter 处理
        break;
    }
  }

  // ── 二进制帧处理 ──────────────────────────────────────────────────────────

  _handleBinary(buf) {
    if (buf.length < 5) return;
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    const opcode = buf[0];

    switch (opcode) {
      case OP_MESSAGE_DATA: {
        // 格式: [opcode:1] [subId:uint32LE] [timestamp:uint64LE] [data...]
        // timestamp = uint32 seconds + uint32 nanoseconds
        const subId = dv.getUint32(1, true);
        const tsSec = dv.getUint32(5, true);
        const tsNsec = dv.getUint32(9, true);
        const tsMs = tsSec * 1000 + Math.floor(tsNsec / 1e6);
        const payload = new Uint8Array(buf.buffer, buf.byteOffset + 13, buf.length - 13);

        const sub = this._subscriptions.get(subId);
        if (sub && sub.callback) {
          try { sub.callback(payload, { subId, channelId: sub.channelId, topic: sub.topic, tsMs }); } catch (_) {}
        }
        break;
      }

      case OP_SERVICE_CALL_RESPONSE: {
        // 格式: [opcode:1] [callId:uint32LE] [data...]
        const callId = dv.getUint32(1, true);
        const payload = new Uint8Array(buf.buffer, buf.byteOffset + 5, buf.length - 5);
        const pending = this._pendingCalls.get(callId);
        if (pending) {
          pending.resolve({ callId, data: payload });
        }
        break;
      }

      case OP_TIME:
      case OP_FETCH_ASSET_RESPONSE:
        // 当前不需要处理
        break;
    }
  }

  // ── 话题订阅 ──────────────────────────────────────────────────────────────

  subscribe(topic, msgType, callback) {
    const channelId = this.getChannelId(topic);
    if (channelId !== null) {
      return this._doSubscribe(channelId, topic, msgType, callback);
    }
    // channel 尚未 advertise — 加入等待队列
    if (!this._pendingSubs.has(topic)) {
      this._pendingSubs.set(topic, []);
    }
    this._pendingSubs.get(topic).push({ msgType, callback });
    this._scheduleRetry();
    return null;  // 返回 null 表示 pending
  }

  _doSubscribe(channelId, topic, msgType, callback) {
    const subId = ++this._subIdCounter;
    this._subscriptions.set(subId, {
      channelId,
      topic,
      msgType,
      callback,
    });
    this._sendJson({
      op: 'subscribe',
      subscriptions: [{ id: subId, channelId }],
    });
    return subId;
  }

  unsubscribe(subId) {
    const sub = this._subscriptions.get(subId);
    if (!sub) return;
    this._subscriptions.delete(subId);
    this._sendJson({
      op: 'unsubscribe',
      subscriptionIds: [subId],
    });
  }

  unsubscribeAll() {
    const ids = Array.from(this._subscriptions.keys());
    if (ids.length > 0) {
      this._sendJson({ op: 'unsubscribe', subscriptionIds: ids });
    }
    this._subscriptions.clear();
  }

  // ── 客户端发布 (clientPublish capability) ─────────────────────────────────

  clientAdvertise(topic, schemaName, schema) {
    const channelId = this._nextClientChanId();
    this._sendJson({
      op: 'advertise',
      channels: [{
        id: channelId,
        topic,
        encoding: 'cdr',
        schemaName,
        schema,
      }],
    });
    return channelId;
  }

  clientUnadvertise(channelId) {
    this._sendJson({
      op: 'unadvertise',
      channelIds: [channelId],
    });
  }

  sendMessage(channelId, cdrData) {
    // 格式: [opcode:1] [channelId:uint32LE] [cdrData...]
    const header = new Uint8Array(5);
    header[0] = OP_MESSAGE_DATA;
    new DataView(header.buffer).setUint32(1, channelId, true);
    this._sendBinary(new Uint8Array([...header, ...cdrData]));
  }

  // ── 服务调用 ──────────────────────────────────────────────────────────────

  sendServiceCallRequest(serviceName, callId, cdrData) {
    this._sendJson({
      op: 'serviceCallRequest',
      service: serviceName,
      callId,
      encoding: 'cdr',
    });
    // 服务请求体也是 CDR binary frame (opcode 0x01 MESSAGE_DATA)
    this._sendBinary(cdrData);
  }

  registerServiceCall(callId, resolve, reject, timeoutMs) {
    const timer = setTimeout(() => {
      this._pendingCalls.delete(callId);
      reject(new Error('service_call_timeout'));
    }, timeoutMs || 60000);
    this._pendingCalls.set(callId, { resolve, reject, timer });
  }

  _resolveServiceCall(callId, response) {
    const pending = this._pendingCalls.get(callId);
    if (!pending) return;
    clearTimeout(pending.timer);
    this._pendingCalls.delete(callId);
    pending.resolve(response);
  }

  _rejectServiceCall(callId, error) {
    const pending = this._pendingCalls.get(callId);
    if (!pending) return;
    clearTimeout(pending.timer);
    this._pendingCalls.delete(callId);
    pending.reject(error);
  }

  // ── 事件 ──────────────────────────────────────────────────────────────────

  onConnect(fn) { this._onConnectHandlers.push(fn); }
  onClose(fn) { this._onCloseHandlers.push(fn); }
  onError(fn) { this._onErrorHandlers.push(fn); }

  // ── 内部 ──────────────────────────────────────────────────────────────────

  _sendJson(obj) {
    if (!this._ws || this._ws.readyState !== 1) return;
    try { this._ws.send(JSON.stringify(obj)); } catch (_) {}
  }

  _sendBinary(data) {
    if (!this._ws || this._ws.readyState !== 1) return;
    try { this._ws.send(data); } catch (_) {}
  }

  _fulfillPending(channel) {
    const pending = this._pendingSubs.get(channel.topic);
    if (!pending || pending.length === 0) return;
    this._pendingSubs.delete(channel.topic);
    for (const { msgType, callback } of pending) {
      this._doSubscribe(channel.id, channel.topic, msgType, callback);
    }
  }

  _scheduleRetry() {
    if (this._retryTimer) return;
    let retries = 0;
    const maxRetries = 5;
    const poll = () => {
      retries++;
      let hasPending = false;
      for (const [topic, subs] of this._pendingSubs) {
        const channelId = this.getChannelId(topic);
        if (channelId !== null) {
          const ch = this._channels.get(channelId);
          for (const { msgType, callback } of [...subs]) {
            this._doSubscribe(channelId, topic, msgType, callback);
          }
          this._pendingSubs.delete(topic);
        } else {
          hasPending = true;
        }
      }
      if (hasPending && retries < maxRetries) {
        this._retryTimer = setTimeout(poll, 200);
      } else {
        this._retryTimer = null;
      }
    };
    this._retryTimer = setTimeout(poll, 200);
  }

  _clearRetry() {
    if (this._retryTimer) {
      clearTimeout(this._retryTimer);
      this._retryTimer = null;
    }
  }

  _nextClientChanId() {
    return 100000 + Math.floor(Math.random() * 900000);
  }
}

// ── 单例 ──────────────────────────────────────────────────────────────────────

const foxgloveClient = new FoxgloveClient();
g.foxgloveClient = foxgloveClient;

export { FoxgloveClient, foxgloveClient, SUBPROTOCOL,
  OP_MESSAGE_DATA, OP_TIME, OP_SERVICE_CALL_RESPONSE, OP_FETCH_ASSET_RESPONSE };
