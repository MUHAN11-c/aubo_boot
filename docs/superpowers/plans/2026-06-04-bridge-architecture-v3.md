# Bridge 架构 v3.1 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 Bridge 模式锁定 + BridgeLogger 全链路可观测 + 四大 Store 状态管理，消除 TransportHub 的静默回退行为。

**Architecture:** 三阶段渐进迁移。Phase 1 新建 BridgeLogger + 四大 Store（不影响现有行为）；Phase 2 修改 TransportHub/TopicRouter 实现模式锁定；Phase 3 集成 Store 到 RosManager 并清理废弃代码。

**Tech Stack:** vanilla JS (ES modules, EventTarget, Map, WebSocket), importmap, 零框架依赖。

**基础路径:** `aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/`

---

### Task 1: 新建 BridgeLogger — 全链路可观测核心

**Files:**
- Create: `js/transport/bridge_logger.js`

- [ ] **Step 1: 创建 bridge_logger.js**

```js
// transport/bridge_logger.js — BridgeLogger 全链路可观测
//
// 每条 sub/pub/svc 记录 {ts, direction, topic, msgType, bridge, mode, bytes, status}
// 3 条输出路径: LogEventBus(持久化) + ivg_status_bar(指示器) + 调试面板(过滤)
//
// 用法:
//   import { bridgeLogger } from './transport/bridge_logger.js';
//   bridgeLogger.log({ direction:'sub', topic:'/joint_states', msgType:'...', bridge:'foxglove', mode:'FOXGLOVE', bytes:1234, status:'ok' });
//   bridgeLogger.getStats();         // → { foxglove:{subs, pubs, svcs, bytes, msgs, errors}, rosbridge:{...} }
//   bridgeLogger.getActiveSubs();    // → [{topic, bridge, msgType}]

const g = globalThis;

const MAX_RECORDS = 2000;

class BridgeLogger extends EventTarget {
  constructor() {
    super();
    this._records = [];
    this._stats = {
      foxglove:  { subs: 0, pubs: 0, svcs: 0, bytesRx: 0, bytesTx: 0, msgsRx: 0, msgsTx: 0, errors: 0 },
      rosbridge: { subs: 0, pubs: 0, svcs: 0, bytesRx: 0, bytesTx: 0, msgsRx: 0, msgsTx: 0, errors: 0 },
    };
    this._activeSubs = new Map();  // topic → { bridge, msgType }
    this._logBus = null;           // lazy init
  }

  /**
   * 记录一条流量日志
   * @param {{direction:'sub'|'pub'|'svc', topic:string, msgType?:string, bridge:'foxglove'|'rosbridge', mode:string, bytes?:number, latencyMs?:number, status:'ok'|'error'|'degraded'}} entry
   */
  log(entry) {
    entry.ts = Date.now();
    entry.status = entry.status || 'ok';

    // 环形缓冲
    if (this._records.length >= MAX_RECORDS) this._records.shift();
    this._records.push(entry);

    // 累计统计
    const s = this._stats[entry.bridge];
    if (s) {
      if (entry.direction === 'sub') {
        s.subs++;
        s.msgsRx++;
        s.bytesRx += entry.bytes || 0;
      } else if (entry.direction === 'pub') {
        s.pubs++;
        s.msgsTx++;
        s.bytesTx += entry.bytes || 0;
      } else if (entry.direction === 'svc') {
        s.svcs++;
      }
      if (entry.status === 'error') s.errors++;
    }

    // 活跃订阅追踪
    if (entry.direction === 'sub' && entry.status === 'ok') {
      this._activeSubs.set(entry.topic, { bridge: entry.bridge, msgType: entry.msgType || '?' });
    }

    // dispatch event → 3 条输出
    this.dispatchEvent(new CustomEvent('bridge:traffic', { detail: entry }));

    // → LogEventBus (lazy init 避免循环依赖)
    if (!this._logBus) {
      try {
        const busMod = g.__logBus;
        if (busMod) this._logBus = busMod;
      } catch (_) {}
    }
    if (this._logBus) {
      const level = entry.status === 'error' ? 'error' : (entry.status === 'degraded' ? 'warn' : 'info');
      this._logBus.addLog(level, 'bridge',
        `[${entry.bridge}] ${entry.direction} ${entry.topic}${entry.status !== 'ok' ? ' ' + entry.status : ''}${entry.bytes ? ' ' + entry.bytes + 'B' : ''}`,
        entry, 'bridge');
    }
  }

  /** 获取累计统计 */
  getStats() {
    return {
      foxglove: { ...this._stats.foxglove },
      rosbridge: { ...this._stats.rosbridge },
    };
  }

  /** 获取当前活跃订阅及其 bridge 归属 */
  getActiveSubs() {
    return Array.from(this._activeSubs.entries()).map(([topic, info]) => ({
      topic, bridge: info.bridge, msgType: info.msgType,
    }));
  }

  /** 按条件过滤历史记录 */
  getRecords(filter = {}) {
    let result = this._records;
    if (filter.bridge)    result = result.filter(r => r.bridge === filter.bridge);
    if (filter.direction) result = result.filter(r => r.direction === filter.direction);
    if (filter.status)    result = result.filter(r => r.status === filter.status);
    if (filter.since)     result = result.filter(r => r.ts >= filter.since);
    return result;
  }
}

const bridgeLogger = new BridgeLogger();
g.__bridgeLogger = bridgeLogger;

export { BridgeLogger, bridgeLogger };
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/transport/bridge_logger.js
git commit -m "feat: add BridgeLogger — per-topic bridge traffic observability"
```

---

### Task 2: 新建 ConnectionStore

**Files:**
- Create: `js/entities/connection_store.js`

- [ ] **Step 1: 创建 connection_store.js**

```js
// entities/connection_store.js — 连接状态管理 (BehaviorSubject 模式)
//
// 单一数据源: connectionStatus / activeBridge / bridgeMode / bridgeStats / reconnectAttempts
// 新订阅者立即收到当前状态 (BehaviorSubject)

class ConnectionStore extends EventTarget {
  constructor() {
    super();
    this._state = {
      connectionStatus: 'disconnected', // 'disconnected' | 'connecting' | 'connected' | 'error'
      activeBridge: null,              // 'foxglove' | 'rosbridge' | null
      bridgeMode: 'auto',              // 'auto' | 'foxglove' | 'rosbridge'
      bridgeStats: {},                 // TransportHub.getStats() 快照
      reconnectAttempts: 0,
      connectedAt: null,
      lastError: null,
    };
  }

  getState() { return { ...this._state }; }

  /**
   * 订阅状态变更 (BehaviorSubject: 立即回调当前值)
   * @param {function(Object): void} fn
   * @returns {function} unsubscribe
   */
  subscribe(fn) {
    try { fn({ ...this._state }); } catch (_) {}
    const handler = (e) => { try { fn(e.detail); } catch (_) {} };
    this.addEventListener('connectionchange', handler);
    return () => this.removeEventListener('connectionchange', handler);
  }

  setConnectionStatus(status, error) {
    this._state.connectionStatus = status;
    this._state.lastError = error || null;
    if (status === 'connected') this._state.connectedAt = Date.now();
    this._emit();
  }

  setActiveBridge(bridgeId) {
    if (this._state.activeBridge === bridgeId) return;
    this._state.activeBridge = bridgeId;
    this._emit();
  }

  setBridgeMode(mode) {
    if (this._state.bridgeMode === mode) return;
    this._state.bridgeMode = mode;
    this._emit();
  }

  setBridgeStats(stats) {
    this._state.bridgeStats = stats;
    this.dispatchEvent(new CustomEvent('statsupdate', { detail: this._state }));
  }

  setReconnectAttempts(n) {
    this._state.reconnectAttempts = n;
    this._emit();
  }

  _emit() {
    this.dispatchEvent(new CustomEvent('connectionchange', { detail: { ...this._state } }));
  }
}

const connectionStore = new ConnectionStore();
globalThis.__connectionStore = connectionStore;

export { ConnectionStore, connectionStore };
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/entities/connection_store.js
git commit -m "feat: add ConnectionStore — connection lifecycle state management"
```

---

### Task 3: 新建 RosDataStore

**Files:**
- Create: `js/entities/ros_data_store.js`

- [ ] **Step 1: 创建 ros_data_store.js**

```js
// entities/ros_data_store.js — ROS 话题数据缓存 (SWR + BehaviorSubject)
//
// 每个 topic 缓存最新值 (SWR stale-while-revalidate)
// 新订阅者立即收到缓存值 (BehaviorSubject)
// 高频节流: /joint_states 50ms debounce
// 大消息不缓存: PointCloud2/Image

const g = globalThis;

const TTL_MS = 30000;       // SWR: 30s 后标记 stale (不清除)
const MAX_CACHE = 200;       // LRU cap
const DEBOUNCE_MS = 50;      // 高频话题节流
const HIGH_FREQ_TOPICS = new Set(['/joint_states', '/tf', '/dynamic_joint_states']);
const NO_CACHE_TOPICS = new Set([
  '/camera/depth/color/points', '/camera/depth/points',
  '/camera/depth_registered/points', '/camera/color/image_raw', '/camera/depth/image_raw',
]);

class RosDataStore extends EventTarget {
  constructor() {
    super();
    this._cache = new Map();       // topic → { data, ts, bridge, stale }
    this._accessOrder = [];        // LRU 访问顺序
    this._debounceTimers = new Map(); // topic → timeoutId
    this._subscribers = new Map(); // topic → Set<fn>
  }

  get(topic) {
    const entry = this._cache.get(topic);
    if (!entry) return undefined;
    this._touchAccessOrder(topic);
    if (Date.now() - entry.ts > TTL_MS) entry.stale = true;
    return { data: entry.data, ts: entry.ts, bridge: entry.bridge, stale: entry.stale };
  }

  subscribe(topic, fn) {
    const cached = this.get(topic);
    if (cached) {
      try { fn(cached.data); } catch (_) {}
    }
    if (!this._subscribers.has(topic)) this._subscribers.set(topic, new Set());
    this._subscribers.get(topic).add(fn);
    return () => {
      const subs = this._subscribers.get(topic);
      if (subs) {
        subs.delete(fn);
        if (subs.size === 0) this._subscribers.delete(topic);
      }
    };
  }

  set(topic, data, bridge) {
    if (NO_CACHE_TOPICS.has(topic)) {
      this._notifySubs(topic, data);
      return;
    }
    if (HIGH_FREQ_TOPICS.has(topic)) {
      if (this._debounceTimers.has(topic)) {
        this._debounceTimers.get(topic)._pending = { data, bridge };
        return;
      }
      const timerInfo = { _pending: null };
      this._debounceTimers.set(topic, setTimeout(() => {
        const d = timerInfo._pending || { data, bridge };
        this._debounceTimers.delete(topic);
        this._doSet(topic, d.data, d.bridge);
      }, DEBOUNCE_MS));
      return;
    }
    this._doSet(topic, data, bridge);
  }

  _doSet(topic, data, bridge) {
    if (!this._cache.has(topic) && this._cache.size >= MAX_CACHE) {
      const oldest = this._accessOrder.shift();
      if (oldest) this._cache.delete(oldest);
    }
    this._cache.set(topic, { data, ts: Date.now(), bridge, stale: false });
    this._touchAccessOrder(topic);
    this._notifySubs(topic, data);
  }

  _notifySubs(topic, data) {
    const subs = this._subscribers.get(topic);
    if (!subs) return;
    for (const fn of subs) {
      try { fn(data); } catch (_) {}
    }
    this.dispatchEvent(new CustomEvent('data:' + topic, { detail: data }));
  }

  _touchAccessOrder(topic) {
    const idx = this._accessOrder.indexOf(topic);
    if (idx > -1) this._accessOrder.splice(idx, 1);
    this._accessOrder.push(topic);
  }
}

const rosDataStore = new RosDataStore();
g.__rosDataStore = rosDataStore;

export { RosDataStore, rosDataStore };
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/entities/ros_data_store.js
git commit -m "feat: add RosDataStore — SWR cache + BehaviorSubject for ROS topic data"
```

---

### Task 4: 新建 ServiceStore

**Files:**
- Create: `js/entities/service_store.js`

- [ ] **Step 1: 创建 service_store.js**

```js
// entities/service_store.js — 服务调用状态机
//
// 状态机: IDLE → STARTING → IN_PROGRESS → COMPLETED / FAILED
// 5s 心跳防超时

const g = globalThis;
const HEARTBEAT_MS = 5000;

class ServiceStore extends EventTarget {
  constructor() {
    super();
    this._lastResults = new Map();  // service → { data, ts }
    this._callHistory = [];         // [{service, status, durationMs, bridge, error}]
    this._nextCallId = 1;
    this._executor = null;          // (name, type, request, timeoutMs) => Promise<result>
  }

  setExecutor(fn) { this._executor = fn; }

  async callService(name, type, request, timeoutMs = 60000) {
    const callId = this._nextCallId++;
    const startTime = Date.now();

    this._emit('service:start', { callId, service: name, type, request });

    const progressTimer = setInterval(() => {
      this._emit('service:progress', {
        callId, service: name, elapsedMs: Date.now() - startTime,
      });
    }, HEARTBEAT_MS);

    try {
      if (!this._executor) throw new Error('ServiceStore: executor not set');
      const result = await this._executor(name, type, request, timeoutMs);
      clearInterval(progressTimer);

      const durationMs = Date.now() - startTime;
      this._lastResults.set(name, { data: result, ts: Date.now() });
      this._callHistory.push({
        service: name, status: 'completed', durationMs,
        bridge: result && result._bridge ? result._bridge : 'unknown',
      });

      this._emit('service:complete', { callId, service: name, result, durationMs });
      return result;
    } catch (e) {
      clearInterval(progressTimer);
      const durationMs = Date.now() - startTime;

      this._callHistory.push({
        service: name, status: 'failed', durationMs,
        bridge: 'unknown', error: e.message,
      });

      this._emit('service:error', { callId, service: name, error: e.message, durationMs });
      throw e;
    }
  }

  getLastResult(service) {
    return this._lastResults.get(service);
  }

  getCallHistory() {
    return this._callHistory.slice(-50);
  }

  _emit(eventType, detail) {
    this.dispatchEvent(new CustomEvent(eventType, { detail }));
  }
}

const serviceStore = new ServiceStore();
g.__serviceStore = serviceStore;

export { ServiceStore, serviceStore };
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/entities/service_store.js
git commit -m "feat: add ServiceStore — service call state machine with heartbeat"
```

---

### Task 5: 新建 UIStateStore

**Files:**
- Create: `js/entities/ui_state_store.js`

- [ ] **Step 1: 创建 ui_state_store.js**

```js
// entities/ui_state_store.js — UI 状态持久化 (localStorage 同步)

const PREFIX = 'ivg_ui_';

class UIStateStore extends EventTarget {
  get(key) {
    try {
      const raw = localStorage.getItem(PREFIX + key);
      if (raw === null) return undefined;
      return JSON.parse(raw);
    } catch (_) {
      return undefined;
    }
  }

  set(key, val) {
    try {
      localStorage.setItem(PREFIX + key, JSON.stringify(val));
    } catch (_) {}
    this.dispatchEvent(new CustomEvent('uisettings:change', { detail: { key, val } }));
  }

  remove(key) {
    localStorage.removeItem(PREFIX + key);
    this.dispatchEvent(new CustomEvent('uisettings:change', { detail: { key, val: undefined } }));
  }
}

const uiStateStore = new UIStateStore();
globalThis.__uiStateStore = uiStateStore;

export { UIStateStore, uiStateStore };
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/entities/ui_state_store.js
git commit -m "feat: add UIStateStore — localStorage-backed UI state persistence"
```

---

### Task 6: TransportHub 集成 BridgeLogger (日志记录，不改变路由)

**Files:**
- Modify: `js/transport/hub.js:19-24` (imports), `:228-265` (subscribe/callService/publish)

- [ ] **Step 1: 添加 BridgeLogger import**

在 `hub.js` 顶部 import 块 (L18-24) 末尾添加:

```js
import { bridgeLogger } from './bridge_logger.js';
```

- [ ] **Step 2: subscribe() 添加日志**

修改 `subscribe()` (L228-237):

```js
subscribe(topic, msgType, callback) {
  const adapter = this._router.resolve(topic, msgType, this._mode,
    Object.fromEntries(this._adapters));
  if (!adapter) {
    bridgeLogger.log({
      direction: 'sub', topic, msgType, bridge: 'none', mode: this._mode, status: 'error'
    });
    console.warn(`[TransportHub] no adapter for ${topic}`);
    return null;
  }
  this._subRegistry.register(topic, msgType, adapter, callback);
  bridgeLogger.log({
    direction: 'sub', topic, msgType, bridge: adapter.id, mode: this._mode, status: 'ok'
  });
  return adapter.subscribe(topic, msgType, callback);
}
```

- [ ] **Step 3: callService() 添加日志 (保留 fallback pending Phase 2)**

修改 `callService()` (L249-258):

```js
async callService(name, type, request, timeoutMs = 60000) {
  const adapter = this.activeAdapter;
  if (!adapter) {
    bridgeLogger.log({
      direction: 'svc', topic: name, msgType: type, bridge: 'none', mode: this._mode, status: 'error'
    });
    for (const a of this._adapters.values()) {
      if (a.isConnected) {
        bridgeLogger.log({
          direction: 'svc', topic: name, msgType: type, bridge: a.id, mode: this._mode, status: 'degraded'
        });
        return a.callService(name, type, request, timeoutMs);
      }
    }
    throw new Error('no_connected_adapter');
  }
  bridgeLogger.log({
    direction: 'svc', topic: name, msgType: type, bridge: adapter.id, mode: this._mode, status: 'ok'
  });
  try {
    return await adapter.callService(name, type, request, timeoutMs);
  } catch (e) {
    bridgeLogger.log({
      direction: 'svc', topic: name, msgType: type, bridge: adapter.id, mode: this._mode, status: 'error'
    });
    throw e;
  }
}
```

- [ ] **Step 4: publish() 添加日志**

修改 `publish()` (L261-265):

```js
async publish(topic, type, msg) {
  const adapter = this.activeAdapter;
  if (!adapter) {
    bridgeLogger.log({
      direction: 'pub', topic, msgType: type, bridge: 'none', mode: this._mode, status: 'error'
    });
    throw new Error('no_connected_adapter');
  }
  bridgeLogger.log({
    direction: 'pub', topic, msgType: type, bridge: adapter.id, mode: this._mode, status: 'ok'
  });
  return adapter.publish(topic, type, msg);
}
```

- [ ] **Step 5: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/transport/hub.js
git commit -m "feat: integrate BridgeLogger into TransportHub sub/pub/svc (log-only, no routing change)"
```

---

### Task 7: 模式锁定 — 修改 _reconcile() + callService() + setMode()

**Files:**
- Modify: `js/transport/hub.js`

- [ ] **Step 1: 重写 _reconcile() — 模式锁定**

替换 `_reconcile()` (L201-221):

```js
_reconcile() {
  const prev = this._activeId;
  let nextId = null;
  const fox = this._adapters.get('foxglove');
  const rb = this._adapters.get('rosbridge');

  if (this._mode === BridgeMode.FOXGLOVE) {
    nextId = (fox && fox.isConnected) ? 'foxglove' : null;
    if (!nextId) {
      bridgeLogger.log({
        direction: 'sub', topic: '(mode)', msgType: '',
        bridge: 'foxglove', mode: this._mode, status: 'error'
      });
      this.dispatchEvent(new CustomEvent('bridge:error', {
        detail: { mode: this._mode, error: 'foxglove unreachable' }
      }));
    }
  } else if (this._mode === BridgeMode.ROSBRIDGE) {
    nextId = (rb && rb.isConnected) ? 'rosbridge' : null;
    if (!nextId) {
      bridgeLogger.log({
        direction: 'sub', topic: '(mode)', msgType: '',
        bridge: 'rosbridge', mode: this._mode, status: 'error'
      });
      this.dispatchEvent(new CustomEvent('bridge:error', {
        detail: { mode: this._mode, error: 'rosbridge unreachable' }
      }));
    }
  } else {
    if (fox && fox.isConnected) {
      nextId = 'foxglove';
    } else if (rb && rb.isConnected) {
      nextId = 'rosbridge';
      bridgeLogger.log({
        direction: 'sub', topic: '(mode)', msgType: '',
        bridge: 'rosbridge', mode: this._mode, status: 'degraded'
      });
      this.dispatchEvent(new CustomEvent('bridge:degraded', {
        detail: { mode: this._mode, active: 'rosbridge', reason: 'foxglove unreachable' }
      }));
    }
  }

  this._activeId = nextId;
  if (prev !== this._activeId) {
    this.dispatchEvent(new CustomEvent('bridgechange', {
      detail: { active: this._activeId, prevActive: prev }
    }));
  }
}
```

- [ ] **Step 2: 重写 callService() — 移除 fallback**

```js
async callService(name, type, request, timeoutMs = 60000) {
  const adapter = this.activeAdapter;
  if (!adapter) {
    bridgeLogger.log({
      direction: 'svc', topic: name, msgType: type,
      bridge: 'none', mode: this._mode, status: 'error'
    });
    throw new Error('no active adapter for mode: ' + this._mode);
  }
  bridgeLogger.log({
    direction: 'svc', topic: name, msgType: type,
    bridge: adapter.id, mode: this._mode, status: 'ok'
  });
  try {
    return await adapter.callService(name, type, request, timeoutMs);
  } catch (e) {
    bridgeLogger.log({
      direction: 'svc', topic: name, msgType: type,
      bridge: adapter.id, mode: this._mode, status: 'error'
    });
    throw e;
  }
}
```

- [ ] **Step 3: setMode() 添加 BridgeLogger 日志**

在 `setMode()` (L177 L178 `if (mode === this._mode) return;` 之后) 添加:

```js
bridgeLogger.log({
  direction: 'svc', topic: '(setMode)', msgType: '',
  bridge: this._activeId || 'none', mode, status: 'ok'
});
```

- [ ] **Step 4: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/transport/hub.js
git commit -m "feat: mode-locked routing — no silent fallback in FOXGLOVE/ROSBRIDGE, explicit AUTO degradation"
```

---

### Task 8: 简化 TopicRouter

**Files:**
- Modify: `js/transport/topic_router.js`

- [ ] **Step 1: 简化 resolve()**

将整个 `topic_router.js` 替换为:

```js
// transport/topic_router.js — 话题级路由 (模式锁定)
//
// 规则:
//   FOXGLOVE  → FoxgloveAdapter ONLY (不可用返回 null)
//   ROSBRIDGE → RosbridgeAdapter ONLY (不可用返回 null)
//   AUTO      → Foxglove 优先, 不可用降级 Rosbridge

import { BridgeMode } from './adapter.js';

class TopicRouter {
  resolve(topic, msgType, mode, adapters) {
    if (mode === BridgeMode.FOXGLOVE) {
      return adapters.foxglove?.isConnected ? adapters.foxglove : null;
    }
    if (mode === BridgeMode.ROSBRIDGE) {
      return adapters.rosbridge?.isConnected ? adapters.rosbridge : null;
    }
    if (adapters.foxglove?.isConnected) return adapters.foxglove;
    if (adapters.rosbridge?.isConnected) return adapters.rosbridge;
    return null;
  }
}

const topicRouter = new TopicRouter();

export { TopicRouter, topicRouter };
```

删除的内容:
- `LARGE_MSG_TOPICS` Set 常量
- `fox._client?.getChannel(topic)` 检查
- 大消息话题强制 foxglove 逻辑
- topic 不在 foxglove channels 中回退 rosbridge 的逻辑

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/transport/topic_router.js
git commit -m "refactor: simplify TopicRouter — mode-locked, remove whitelist and channel checks"
```

---

### Task 9: 更新 ivg_status_bar.js — BridgeLogger 统计 + 降级/错误状态

**Files:**
- Modify: `js/core/ivg_status_bar.js`

- [ ] **Step 1: 更新 updateBridgeDot() — 添加降级/错误状态**

替换 `updateBridgeDot()` (L160-174):

```js
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
```

- [ ] **Step 2: 更新 updateFoxgloveStats() — 使用 BridgeLogger 数据**

替换 `updateFoxgloveStats()` (L192-209):

```js
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
```

- [ ] **Step 3: 在 _bindHubEvents() 中添加 bridge:degraded + bridge:error 监听**

在 `_bindHubEvents()` 中 `hub.addEventListener('stats', ...)` 之后添加:

```js
  hub.addEventListener('bridge:degraded', function(e) {
    updateBridgeDot();
    logBus.addLog('warn', 'bridge',
      'AUTO mode degraded to rosbridge (foxglove unreachable)', e.detail, 'bridge');
  });

  hub.addEventListener('bridge:error', function(e) {
    updateBridgeDot();
    logBus.addLog('error', 'bridge',
      'Bridge error (mode: ' + (e.detail && e.detail.mode) + '): ' + (e.detail && e.detail.error),
      e.detail, 'bridge');
  });
```

- [ ] **Step 4: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/core/ivg_status_bar.js
git commit -m "feat: status bar shows BridgeLogger stats + degraded/error states"
```

---

### Task 10: RosManager 集成 Stores + 移除 ivgTransport fallback

**Files:**
- Modify: `js/core/ros.js`

- [ ] **Step 1: 添加 Store import**

在 `ros.js` import 块 (约 L22) 后添加:

```js
import { rosDataStore } from '../entities/ros_data_store.js';
import { serviceStore } from '../entities/service_store.js';
import { connectionStore } from '../entities/connection_store.js';
```

- [ ] **Step 2: subscribe() → RosDataStore**

修改 `subscribe()` (L163-174):

```js
subscribe(topic, msgType, callback) {
  var name = canonicalRosTopic(topic);
  this._rosHandlers.push({ topic: name, fn: callback });

  var hub = g.__transportHub;
  if (hub && hub.initialized) {
    var result = hub.subscribe(name, msgType, function(typedMsg) {
      rosDataStore.set(name, typedMsg.data, typedMsg.bridge || 'unknown');
      try { callback(typedMsg.data); } catch (_) {}
    });
    return !!result;
  }
  return this._transport.subscribe({ topic: name, msgType });
}
```

- [ ] **Step 3: callService() → ServiceStore**

修改 `callService()` (L189-197):

```js
async callService(service, type, request, timeoutMs) {
  timeoutMs = timeoutMs || 60000;
  if (!serviceStore._executor) {
    var self = this;
    serviceStore.setExecutor(function(name, svcType, svcReq, timeout) {
      var hub = g.__transportHub;
      if (hub && hub.initialized) {
        return hub.callService(name, svcType, svcReq, timeout);
      }
      return self._transport.callService({
        service: name, type: svcType, request: svcReq, timeoutMs: timeout,
      });
    });
  }
  return serviceStore.callService(service, type, request, timeoutMs);
}
```

- [ ] **Step 4: 扩展 _wireHubBridgeChange() → ConnectionStore**

修改 `_wireHubBridgeChange()` (L144-153):

```js
_wireHubBridgeChange(hub) {
  if (hub._rosManagerBridgeWired) return;
  hub._rosManagerBridgeWired = true;

  hub.addEventListener('bridgechange', function(e) {
    connectionStore.setActiveBridge(e.detail.active);
    if (!e.detail.active && !this._paused) {
      this._log('transport', 'all bridges disconnected, scheduling reconnect...');
      this._scheduleReconnect();
    }
  }.bind(this));

  hub.addEventListener('modechange', function(e) {
    connectionStore.setBridgeMode(e.detail.mode);
    connectionStore.setActiveBridge(e.detail.active);
  });

  hub.addEventListener('stats', function() {
    connectionStore.setBridgeStats(hub.getStats());
  });
}
```

- [ ] **Step 5: 移除 connect() 中的 ivgTransport fallback**

修改 `connect()` (L97-103):

旧:
```js
if (hub) {
  await hub.init();
} else {
  await this._transport.loadRuntime();
  await this._transport.connectControl();
}
```

新:
```js
if (hub) {
  await hub.init();
} else {
  throw new Error('TransportHub not available');
}
```

- [ ] **Step 6: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/core/ros.js
git commit -m "refactor: integrate Stores into RosManager; remove ivgTransport fallback"
```

---

### Task 11: 废弃 ros_connector.js

**Files:**
- Modify: `js/core/ros_connector.js` (添加 deprecated 注释)

- [ ] **Step 1: 添加 deprecated 注释**

在 `ros_connector.js` L1 之前添加:

```js
// ⚠️ DEPRECATED (2026-06-04): 请使用 RosManager (core/ros.js) + TransportHub 替代。
// 此文件绕过 TransportHub 直接操作 ivgTransport，Bridge 模式切换后订阅不会迁移。
// 迁移:
//   旧: import { createRosConnector } from './core/ros_connector.js';
//       const rc = createRosConnector({ setConnStatus, onConnected });
//       rc.connect();
//   新: import { ros } from './core/ros.js';
//       ros.onStatusChange(setConnStatus);
//       ros.onConnected(onConnected);
//       await ros.connect();
// 此文件将在后续版本删除。
//
```

- [ ] **Step 2: 提交**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/core/ros_connector.js
git commit -m "docs: mark ros_connector.js as deprecated — migrate to RosManager + TransportHub"
```

---

### Task 12: 验证 — 浏览器 console 测试

- [ ] **Step 1: 启动系统**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
SKIP_BUILD=1 SKIP_ROSBAG=1 SKIP_RVIZ=1 ./start_aubo_new_driver.sh
```

- [ ] **Step 2: 浏览器 console 验证 BridgeLogger**

打开 Web Dashboard → F12 console:

```js
// 1. BridgeLogger 实例存在
__bridgeLogger

// 2. 获取统计 (应看到 foxglove/rosbridge 各自统计数据)
__bridgeLogger.getStats()

// 3. 获取活跃订阅 (每个 topic 标注其 bridge 归属)
__bridgeLogger.getActiveSubs()

// 4. TransportHub 模式
__transportHub.mode       // → 'auto' | 'foxglove' | 'rosbridge'
__transportHub.activeId   // → 'foxglove' | 'rosbridge' | null

// 5. ConnectionStore 状态
__connectionStore.getState()
```

- [ ] **Step 3: 验证模式锁定**

```js
// 切换到 Foxglove 模式
__transportHub.setMode('foxglove')

// 确认 activeId
__transportHub.activeId  // → 'foxglove' (foxglove 可用时)

// 查看 BridgeLogger (所有记录 bridge='foxglove')
__bridgeLogger.getRecords({bridge:'foxglove'})
```

- [ ] **Step 4: 提交**

```bash
cd /home/mu/aubo_boot
git commit --allow-empty -m "test: verify bridge mode-locked routing and BridgeLogger observability"
```

---

## 回滚方案

每个 Task 独立可回滚:
- **Task 1-5** (新文件): `git revert` 对应 commit，删除新文件
- **Task 6** (BridgeLogger 集成, log-only): `git revert` 单 commit
- **Task 7-8** (模式锁定 + TopicRouter): `git revert` 两个 commit，恢复 fallback
- **Task 9-11** (Store 集成 + 清理): 逐 commit revert
