# Bridge 架构 v3.2 设计文档

**日期**: 2026-06-04
**状态**: 设计完成，待实现
**范围**: `aubo_ros2_web_dashboard/web/public/js/transport/` + `js/core/ros.js` + `js/entities/` (新建)

---

## 1. 目标

1. **模式锁定** — BridgeMode 决定使用哪个通道，FOXGLOVE/ROSBRIDGE 模式下禁止自动回退
2. **全链路可观测** — 每条 sub/pub/svc 记录 `{topic, bridge, mode, status}`，可追溯到具体 bridge
3. **分层架构** — FSD 四层 (pages→features→entities→shared)，严格单向依赖
4. **状态管理** — 四大 Store (ConnectionStore/RosDataStore/ServiceStore/UIStateStore)，BehaviorSubject 模式

## 2. 核心设计决策

### 2.1 模式锁定 (禁止静默回退)

| 模式 | Foxglove 可用 | Foxglove 不可用 |
|------|-------------|----------------|
| **FOXGLOVE** | FoxgloveAdapter | **报错，不回退 rosbridge** |
| **ROSBRIDGE** | 不使用 | RosbridgeAdapter |
| **AUTO** | FoxgloveAdapter | **显式降级** → RosbridgeAdapter + `bridgedegraded` 事件 |

**为什么禁止自动回退**: 静默回退导致问题难以排查。选了 Foxglove 但数据不通 → 一定是 Foxglove 的问题。如果自动切 rosbridge，用户看到数据正常但实际走的是 rosbridge JSON（点云/图像带宽爆炸），难以定位根因。

### 2.2 AUTO 模式显式降级与恢复

仅 AUTO 模式允许降级，且降级时:
- `BridgeLogger.log({status:'degraded'})` 记录
- `TransportHub.dispatchEvent('bridgedegraded')` 通知 UI
- 状态栏显示降级状态 (黄色指示器)

Foxglove 恢复后自动切回:
- `TransportHub.dispatchEvent('bridgerestored')` 通知 UI
- 状态栏恢复绿色指示器

### 2.3 兼容性保留

rosbridge 作为独立模式 (BridgeMode.ROSBRIDGE) 完整保留。用户可手动切换。不自动回退 ≠ 删除 rosbridge。

### 2.4 WebSocket 重连策略 (指数退避 + 抖动)

基于 2025 年工业最佳实践:
- 基础延迟 1s，×2 递增，上限 30s
- ±20% 随机抖动（防惊群效应）
- 区分关闭原因: code 1000 不重连，1006/1011 重连
- `navigator.onLine` 感知: offline 暂停重连，online 重置计数立即重连
- 原子状态锁防重复重连

## 3. 分层架构 (FSD 四层)

基于项目零框架 MPA 特点，采用精简四层架构（参考 nanostores/Campfire Core 的单向数据流）:

```
pages/          → HTML + importmap + 全局 Provider 初始化
features/       → vision_grasp/latte/view3d 业务逻辑 + Web Components (widgets 合并)
entities/       → 领域模型 + 四大 Store + RosManager (只能 import shared)
shared/         → transport/ + lib/ (不依赖任何上层)
  ├── transport/   → MessageAdapter, TransportHub, TopicRouter, FoxgloveAdapter, RosbridgeAdapter, BridgeLogger
  └── lib/         → utils.js, tf-math.js, topics.js, log-bus.js, ivg_runtime.js
```

**严格单向依赖**: 上层可依赖下层，同层禁止互引，下层不知道上层。

**为何不用 FSD 六层**: 本项目是无路由、无组件树、无 DI 容器的 MPA。widgets/ 只有 3 个 Web Components，features/ 和 pages/ 几乎 1:1 对应。六层中的 widgets/features/pages 在本项目中就是同一个文件。四层已足够表达依赖方向。

## 4. 状态管理 (entities/)

四个 Store，全部基于 `EventTarget` 子类 + BehaviorSubject 模式实现 (vanilla JS，零框架依赖)。

设计参考: nanostores atomic store 模式 + Campfire Core EventTarget 模式 + OpenTelemetry 结构化日志。

### 4.1 ConnectionStore (BehaviorSubject)

```js
class ConnectionStore extends EventTarget {
  getState()           // → { connectionStatus, activeBridge, bridgeMode, bridgeStats, reconnectAttempts }
  subscribe(fn)        // → unsubscribe (立即回调当前值 — BehaviorSubject)
  // 内部事件: 'connectionchange', 'statsupdate'
}
```

**BehaviorSubject 模式**: `subscribe(fn)` 注册时立即用当前状态调用 `fn`，之后每次变更再次调用。参考 nanostores `atom.subscribe()`。

### 4.2 RosDataStore (SWR 缓存)

```js
class RosDataStore extends EventTarget {
  get(topic)           // → { data, ts, bridge } | undefined (SWR: 返回缓存即使 stale)
  subscribe(topic, fn) // → unsubscribe (BehaviorSubject: 新订阅者立即收到缓存值)
  // 内部: set(topic, data) → dispatchEvent('data:'+topic)
  // SWR: TTL 30s (不清除，仅标记 stale), LRU cap 200, PointCloud2/Image 不缓存
  // 高频节流: /joint_states 50ms debounce (合并而非丢弃)
}
```

**为何用 SWR 而非单纯缓存**: 页面切换时立即展示旧数据（即使 stale），避免 UI 闪烁。新数据到达后自动刷新。

### 4.3 ServiceStore (状态机)

```js
class ServiceStore extends EventTarget {
  async callService(name, type, request, timeoutMs)
  // 状态机: IDLE → STARTING → IN_PROGRESS → COMPLETED/FAILED
  // 事件: 'service:start', 'service:progress', 'service:complete', 'service:error'
}
```

### 4.4 UIStateStore

```js
class UIStateStore extends EventTarget {
  get(key) / set(key, val)  // localStorage 同步，带 ivg_ui_ 前缀
  // 事件: 'uisettings:change'
}
```

## 5. 通信层 (shared/transport/)

### 5.1 数据流 (FOXGLOVE 模式)

```
ROS2 Node → foxglove_bridge (CDR) → FoxgloveClient → FoxgloveAdapter (MessageReader)
  → TypedMessage {topic, type, data, timestamp, bridge:'foxglove'}
  → BridgeLogger.log({direction:'sub', bridge:'foxglove', ...})   ← 每条消息记录
  → TransportHub.dispatchEvent('message', {detail: typedMsg})
  → RosDataStore.set(topic, data)
  → RosDataStore.dispatchEvent('data:'+topic)
  → UI 订阅者回调
```

### 5.2 TransportHub — 模式锁定版

核心变更: `callService()` 和 `publish()` 不再有 fallback 逻辑。

```js
class TransportHub extends EventTarget {
  async setMode(mode) {
    // 1. 更新 _mode
    // 2. _reconcile() → 根据 mode 选择 activeAdapter (不回退)
    // 3. 如果 activeId 变更 → SubRegistry.migrateAll() (先建后拆)
    // 4. dispatchEvent('modechange')
  }

  _reconcile() {
    if (mode === FOXGLOVE)  → activeId = foxglove.isConnected ? 'foxglove' : null
                              → 不可用时 dispatchEvent('bridgeerror')
    if (mode === ROSBRIDGE) → activeId = rosbridge.isConnected ? 'rosbridge' : null
                              → 不可用时 dispatchEvent('bridgeerror')
    if (mode === AUTO)      → foxglove可用 → 'foxglove'
                              → foxglove不可用 + rosbridge可用 → 'rosbridge' + bridgedegraded
                              → 都不行 → null
  }

  subscribe(topic, type, cb) {
    adapter = TopicRouter.resolve(topic, type, this._mode, adapters)
    // mode=FOXGLOVE → 只返回 foxglove adapter，不可用返回 null
    // mode=ROSBRIDGE → 只返回 rosbridge adapter，不可用返回 null
    // adapter=null → BridgeLogger 记录 error，抛出异常
  }

  async callService(name, type, request, timeoutMs) {
    adapter = this.activeAdapter   // 模式决定，不回退
    if (!adapter) throw new Error('no active adapter for mode: ' + this._mode)
    BridgeLogger.log({direction:'svc', bridge:adapter.id, ...})
    return adapter.callService(...)
  }

  async publish(topic, type, msg) {
    adapter = this.activeAdapter   // 模式决定，不回退
    if (!adapter) throw new Error('no active adapter for mode: ' + this._mode)
    BridgeLogger.log({direction:'pub', bridge:adapter.id, ...})
    return adapter.publish(...)
  }
}
```

### 5.3 TopicRouter — 模式锁定 + channel warn

```js
class TopicRouter {
  resolve(topic, msgType, mode, adapters) {
    if (mode === FOXGLOVE) {
      if (!adapters.foxglove?.isConnected) return null
      // channel 不存在时 warn (不回退，保持模式锁定)
      if (!adapters.foxglove._client?.getChannel(topic)) {
        console.warn(`[TopicRouter] ${topic} not in foxglove channels (mode locked)`)
      }
      return adapters.foxglove
    }
    if (mode === ROSBRIDGE) {
      return adapters.rosbridge?.isConnected ? adapters.rosbridge : null
    }
    // AUTO: foxglove 优先，不可用降级 rosbridge
    if (adapters.foxglove?.isConnected) return adapters.foxglove
    if (adapters.rosbridge?.isConnected) return adapters.rosbridge
    return null
  }
}
```

移除旧的 "大消息白名单强制" 逻辑 — TopicRouter 不关心消息大小，只关心模式 + 可用性。大消息优化属于 AUTO 模式的自动行为（foxglove CDR 优先），不是硬编码白名单。

### 5.4 BridgeLogger — 全链路可观测 (v3.2 精简版)

**v3.2 核心变更**: BridgeLogger 不维护自己的日志缓冲，所有持久化委托给 LogEventBus。BridgeLogger 只维护累加统计 + 活跃订阅追踪。

```
BridgeLogger (统计引擎)
  ├── _stats: { foxglove:{subs,pubs,svcs,msgs,errors}, rosbridge:{...} }
  ├── _activeSubs: Map<topic, {bridge, msgType}>
  └── output → LogEventBus.addLog('bridge', ...)  (持久化 + BroadcastChannel)
                → dispatchEvent('bridge:traffic')   (实时监听器)
```

```js
class BridgeLogger extends EventTarget {
  _stats = {
    foxglove:  { subs:0, pubs:0, svcs:0, msgsRx:0, msgsTx:0, errors:0 },
    rosbridge: { subs:0, pubs:0, svcs:0, msgsRx:0, msgsTx:0, errors:0 }
  }
  _activeSubs = new Map()  // topic → { bridge, msgType }

  log({direction, topic, msgType, bridge, mode, status})
  // → _stats[bridge] 累加
  // → _activeSubs 追踪
  // → LogEventBus.addLog() 持久化
  // → dispatchEvent('bridge:traffic')

  getStats()            // 快照统计
  getActiveSubs()       // 活跃订阅 + bridge 归属
}
```

**bytes 统计延后到 Phase 2**: 当前 `TypedMessage` 接口不包含原始字节数。Phase 2 在适配器层增加 `onRawMessage` 回调后启用。

**日志流向 (3 条输出)**:
1. `LogEventBus.addLog('bridge', ...)` → IndexedDB + BroadcastChannel (唯一持久化路径)
2. `ivg_status_bar` 实时指示器 (监听 bridge:traffic)
3. 调试面板 "Bridge 流量" 标签页 (查询 LogEventBus)

**状态栏展示**:
- 正常: `🟢 Foxglove | 45 subs 2 pubs 3 svcs`
- 降级: `🟡 AUTO → Rosbridge (foxglove unreachable)`
- 错误: `🔴 Foxglove | CONNECTION REFUSED | Mode locked`

## 6. 与 LogEventBus 的集成关系

```
BridgeLogger                LogEventBus (已有)
────────────                ──────────────────
_stats 累加器               _entries[] 环形缓冲
_activeSubs 追踪            IndexedDB 持久化
  │                         BroadcastChannel 跨页面
  │                         _rateTrackers 频率折叠
  └──addLog('bridge')──→    CATEGORIES.bridge
                            FEATURES.bridge
```

**职责分离**:
- BridgeLogger = **领域语义** (sub/pub/svc 计数、bridge 归属、错误统计)
- LogEventBus = **基础设施** (存储、持久化、跨页面同步、频率控制)

**不在 BridgeLogger 中重复建设**: 环形缓冲、IndexedDB、BroadcastChannel、频率折叠 — 这些 LogEventBus 已有。

**LogEventBus 增强** (v3.2):
- `g.__logBus = logBus` — 暴露实例到 globalThis (BridgeLogger 通过此引用访问，避免循环 import)
- `CATEGORIES` 新增 `bridge: { color: '#818cf8', label: 'bridge' }`
- 日志条目可选 `sessionId` 字段 (从 `sessionStorage` 读取，用于跨页面日志关联)

## 7. 文件变更清单

### 7.1 新建文件

| 路径 | 说明 |
|------|------|
| `js/transport/bridge_logger.js` | BridgeLogger — 全链路统计引擎 |
| `js/entities/connection_store.js` | ConnectionStore — BehaviorSubject 连接状态 |
| `js/entities/ros_data_store.js` | RosDataStore — SWR 数据缓存 |
| `js/entities/service_store.js` | ServiceStore — 服务调用状态机 |
| `js/entities/ui_state_store.js` | UIStateStore — UI 状态持久化 |

### 7.2 修改文件

| 路径 | 变更 |
|------|------|
| `js/transport/hub.js` | `_reconcile()` 模式锁定；`callService()`/`publish()` 移除 fallback；集成 BridgeLogger；bridgedegraded/bridgerestored/bridgeerror 事件 |
| `js/transport/topic_router.js` | 简化 `resolve()`，移除大消息白名单，保留 channel 检查改为 warn |
| `js/core/log-bus.js` | 新增 `bridge` 分类；暴露 `g.__logBus = logBus`；日志条目增加可选 `sessionId` |
| `js/core/ros.js` | `subscribe()`/`callService()` 集成 RosDataStore/ServiceStore；移除 ivgTransport fallback 路径 |
| `js/core/ivg_status_bar.js` | 去掉 rosbridge 强制覆盖；BridgeLogger 统计展示；降级/恢复/错误状态渲染 |
| `js/core/log-ros-bridge.js` | 接收 `bridge:traffic` 事件 → LogEventBus |

### 7.3 废弃/删除

| 路径 | 说明 |
|------|------|
| `js/core/ros_connector.js` | 标记 deprecated，不再使用，所有页面统一走 RosManager |
| `js/ivg_transport.js` 直连调用 | 外部不再直接调用 ivgTransport，仅 RosbridgeAdapter 内部使用 |

## 8. 迁移策略 (三阶段)

### Phase 1: 基础设施 (不改变外部行为)
1. LogEventBus 增强: 暴露 `g.__logBus` + bridge 分类 + sessionId
2. 新建 `bridge_logger.js`
3. 新建 `entities/` 四个 Store
4. TransportHub 集成 BridgeLogger (日志记录不影响路由决策)
5. 验证: 浏览器 console `__bridgeLogger.getStats()` 有数据

### Phase 2: 模式锁定 (核心变更)
6. 修改 `_reconcile()` + `callService()` + `publish()` 移除 fallback
7. 简化 `TopicRouter.resolve()`，保留 channel warn
8. 状态栏更新: 去掉 rosbridge 强制覆盖 + BridgeLogger 数据源 + 降级/恢复/错误状态
9. RosManager 监听 bridgedegraded/bridgerestored/bridgeerror 事件
10. 验证: FOXGLOVE 模式下断掉 foxglove_bridge → 红色报错，不回退

### Phase 3: Store 集成 + 旧代码清理
11. RosManager.subscribe() 集成 RosDataStore (SWR + BehaviorSubject)
12. RosManager.callService() 集成 ServiceStore (状态机)
13. RosManager 连接生命周期 → ConnectionStore
14. ros_connector.js 标记 deprecated
15. 移除 RosManager 中的 ivgTransport fallback 路径

## 9. 回滚方案

每个 Phase 独立可回滚:
- Phase 1: 新文件不影响现有逻辑，删除即可
- Phase 2: `git revert` 单 commit，恢复 fallback 逻辑
- Phase 3: 逐文件 revert，RosManager 保留 fallback 路径直到最后阶段

## 10. 测试要点

1. **FOXGLOVE 模式 + foxglove_bridge 正常** → 所有 sub/pub/svc 走 foxglove，BridgeLogger 记录 bridge='foxglove'
2. **FOXGLOVE 模式 + foxglove_bridge 断开** → 报错，不回退 rosbridge，状态栏红色
3. **ROSBRIDGE 模式 + rosbridge 正常** → 所有 sub/pub/svc 走 rosbridge
4. **AUTO 模式 + foxglove 不可用** → 显式降级 rosbridge，dispatchEvent 'bridgedegraded'，状态栏黄色
5. **AUTO 模式 + foxglove 恢复** → 自动切回 foxglove，dispatchEvent 'bridgerestored'，状态栏绿色
6. **模式切换** → SubRegistry 先建后拆，迁移窗口 <500ms
7. **BridgeLogger 统计** → `__bridgeLogger.getStats()` 有计数；`__bridgeLogger.getActiveSubs()` 标注 bridge 归属
8. **LogEventBus 集成** → `logBus.getLogs({source:'bridge'})` 有记录
9. **RosDataStore SWR** → 页面切换后再切回，立即看到缓存数据（stale 标记）
10. **ConnectionStore BehaviorSubject** → 新订阅者立即收到当前连接状态

## 11. 参考

- nanostores atomic store: https://github.com/nanostores/nanostores (265B, vanilla JS BehaviorSubject)
- Campfire Core EventTarget Store: https://jsr.io/@campfire/core/doc/~/Store
- OpenTelemetry JS SDK: https://opentelemetry.io/docs/languages/js/
- WebSocket 指数退避 + 抖动: https://dev.to/hexshift/robust-websocket-reconnection-strategies-in-javascript-with-exponential-backoff-40n1
- "Reactivity Without a Framework" (2025): https://blog.openreplay.com/reactivity-without-framework-native-js/
