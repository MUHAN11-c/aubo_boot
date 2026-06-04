# Bridge 架构 v3.1 设计文档

**日期**: 2026-06-04
**状态**: 设计完成，待评审
**范围**: `aubo_ros2_web_dashboard/web/public/js/transport/` + `js/core/ros.js` + `js/entities/` (新建)

---

## 1. 目标

1. **模式锁定** — BridgeMode 决定使用哪个通道，FOXGLOVE/ROSBRIDGE 模式下禁止自动回退
2. **全链路可观测** — 每条 sub/pub/svc 记录 `{topic, bridge, mode, bytes, status}`，可追溯到具体 bridge
3. **分层架构** — FSD 六层 (app→pages→widgets→features→entities→shared)，严格单向依赖
4. **状态管理** — 四大 Store (ConnectionStore/RosDataStore/ServiceStore/UIStateStore)，EventTarget 驱动

## 2. 核心设计决策

### 2.1 模式锁定 (禁止静默回退)

| 模式 | Foxglove 可用 | Foxglove 不可用 |
|------|-------------|----------------|
| **FOXGLOVE** | FoxgloveAdapter | **报错，不回退 rosbridge** |
| **ROSBRIDGE** | 不使用 | RosbridgeAdapter |
| **AUTO** | FoxgloveAdapter | **显式降级** → RosbridgeAdapter + `dispatchEvent('bridge:degraded')` |

**为什么禁止自动回退**: 静默回退导致问题难以排查。选了 Foxglove 但数据不通 → 一定是 Foxglove 的问题。如果自动切 rosbridge，用户看到数据正常但实际走的是 rosbridge JSON（点云/图像带宽爆炸），难以定位根因。

### 2.2 AUTO 模式显式降级

仅 AUTO 模式允许降级，且降级时:
- `BridgeLogger.log({status:'degraded'})` 记录
- `TransportHub.dispatchEvent('bridge:degraded')` 通知 UI
- 状态栏显示降级状态 (黄色指示器)
- Foxglove 恢复后自动切回，同样 dispatchEvent 通知

### 2.3 兼容性保留

rosbridge 作为独立模式 (BridgeMode.ROSBRIDGE) 完整保留。用户可手动切换。不自动回退 ≠ 删除 rosbridge。

## 3. 分层架构 (FSD 六层)

```
app/            → HTML + importmap + 全局 Provider 初始化
pages/          → debug/vision/latte/log/tf 页面 (只能 import widgets/features/entities/shared)
widgets/        → Web Components (ivg_status_bar/pose-card/joint-chart) (只能 import features/entities/shared)
features/       → vision_grasp/latte/view3d 业务逻辑 (只能 import entities/shared)
entities/       → 领域模型 + 四大 Store + RosManager (只能 import shared)
shared/         → transport/ + lib/ (不依赖任何上层)
  ├── transport/   → MessageAdapter, TransportHub, TopicRouter, FoxgloveAdapter, RosbridgeAdapter, BridgeLogger
  └── lib/         → utils.js, tf-math.js, topics.js, log-bus.js, ivg_runtime.js
```

**严格单向依赖**: 上层可依赖下层，同层禁止互引，下层不知道上层。

## 4. 状态管理 (entities/)

四个 Store，全部基于 `EventTarget` 子类 + `Map` 缓存实现 (vanilla JS，无框架依赖)。

### 4.1 ConnectionStore

```js
class ConnectionStore extends EventTarget {
  getState()           // → { connectionStatus, activeBridge, bridgeMode, bridgeStats, reconnectAttempts }
  subscribe(fn)        // → unsubscribe
  // 内部事件: 'connectionchange', 'modechange', 'statsupdate'
}
```

### 4.2 RosDataStore (SWR 缓存)

```js
class RosDataStore extends EventTarget {
  get(topic)           // → { data, ts, bridge } | undefined (SWR: 返回缓存即使 stale)
  subscribe(topic, fn) // → unsubscribe (BehaviorSubject: 新订阅者立即收到缓存值)
  // 内部: set(topic, data) → dispatchEvent('data:'+topic)
  // SWR: TTL 30s, LRU cap 200, PointCloud2/Image 不缓存
  // 高频节流: /joint_states 50ms debounce
}
```

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
  get(key) / set(key, val)  // localStorage 同步
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
    // 2. _reconcile() → 根据 mode 选择 activeAdapter
    // 3. 如果 activeId 变更 → SubRegistry.migrateAll()
    // 4. dispatchEvent('modechange')
  }

  _reconcile() {
    if (mode === FOXGLOVE)  → activeId = foxglove.isConnected ? 'foxglove' : null
    if (mode === ROSBRIDGE) → activeId = rosbridge.isConnected ? 'rosbridge' : null
    if (mode === AUTO)      → foxglove可用 ? 'foxglove' : (rosbridge可用 ? 'rosbridge' : null)
  }

  subscribe(topic, type, cb) {
    adapter = TopicRouter.resolve(topic, type, this._mode, adapters)
    // mode=FOXGLOVE → 只返回 foxglove adapter
    // mode=ROSBRIDGE → 只返回 rosbridge adapter
    // adapter=null → 报错，不回退
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

### 5.3 TopicRouter — 简化版

```js
class TopicRouter {
  resolve(topic, msgType, mode, adapters) {
    if (mode === FOXGLOVE)  return adapters.foxglove?.isConnected ? adapters.foxglove : null
    if (mode === ROSBRIDGE) return adapters.rosbridge?.isConnected ? adapters.rosbridge : null
    // AUTO: foxglove 优先，不可用降级 rosbridge (显式降级 + 日志)
    if (adapters.foxglove?.isConnected) return adapters.foxglove
    if (adapters.rosbridge?.isConnected) return adapters.rosbridge
    return null
  }
}
```

移除旧的 "大消息白名单强制" 和 "channel 存在性检查" 逻辑 — 这些在 AUTO 模式下的细分路由增加了排查难度。

### 5.4 BridgeLogger — 新增

```js
class BridgeLogger extends EventTarget {
  _records = []     // 环形缓冲 2000 条
  _stats = {         // 累计 per-bridge
    foxglove:  { subs:0, pubs:0, svcs:0, bytes:0, msgs:0, errors:0 },
    rosbridge: { subs:0, pubs:0, svcs:0, bytes:0, msgs:0, errors:0 }
  }

  log({direction, topic, msgType, bridge, mode, bytes, latencyMs, status})
  // → _records.push() + _stats[bridge]++ + dispatchEvent('bridge:traffic')

  getStats()            // 返回累计统计
  getActiveSubs()       // 当前活跃订阅 + bridge 归属
  getRecords(filter)    // 按条件过滤历史
}
```

**日志流向 (3 条输出)**:
1. `LogEventBus.addLog('bridge', ...)` → IndexedDB + BroadcastChannel
2. `ivg_status_bar` 实时指示器
3. 调试面板 "Bridge 流量" 标签页

**状态栏展示**:
- 正常: `🟢 Foxglove | subs:45 pubs:2 svcs:3 | ↓3.2MB/s ↑12KB/s`
- 降级: `🟡 AUTO(degraded) → Rosbridge | Foxglove unreachable`
- 错误: `🔴 Foxglove | CONNECTION REFUSED | Mode locked, no fallback`

## 6. 文件变更清单

### 6.1 新建文件

| 路径 | 说明 |
|------|------|
| `js/entities/connection_store.js` | ConnectionStore — 连接状态管理 |
| `js/entities/ros_data_store.js` | RosDataStore — SWR 数据缓存 |
| `js/entities/service_store.js` | ServiceStore — 服务调用状态机 |
| `js/entities/ui_state_store.js` | UIStateStore — UI 状态持久化 |
| `js/transport/bridge_logger.js` | BridgeLogger — 全链路可观测 |

### 6.2 修改文件

| 路径 | 变更 |
|------|------|
| `js/transport/hub.js` | `_reconcile()` 模式锁定；`callService()`/`publish()` 移除 fallback；集成 BridgeLogger |
| `js/transport/topic_router.js` | 简化 `resolve()`，移除大消息白名单和 channel 检查；AUTO 模式显式降级 |
| `js/transport/adapter.js` | `TypedMessage` 不变；`MessageAdapter` 接口不变 |
| `js/core/ros.js` | `subscribe()`/`callService()` 集成 RosDataStore/ServiceStore；移除 ivgTransport fallback 路径 |
| `js/core/ivg_status_bar.js` | 新增 BridgeLogger 统计展示；错误/降级状态渲染 |
| `js/core/log-ros-bridge.js` | 接收 `bridge:traffic` 事件 → LogEventBus |

### 6.3 废弃/删除

| 路径 | 说明 |
|------|------|
| `js/core/ros_connector.js` | 不再使用，所有页面统一走 RosManager |
| `js/ivg_transport.js` 直连调用 | 外部不再直接调用 ivgTransport，仅 RosbridgeAdapter 内部使用 |

## 7. 迁移策略

**渐进式，三个阶段**:

### Phase 1: 基础设施 (不改变外部行为)
1. 新建 `entities/` 四个 Store + `bridge_logger.js`
2. TransportHub 集成 BridgeLogger (日志记录不影响路由决策)
3. 验证: 日志面板出现 bridge:traffic 事件

### Phase 2: 模式锁定 (核心变更)
4. 修改 `_reconcile()` + `callService()` + `publish()` 移除 fallback
5. 简化 `TopicRouter.resolve()`
6. 状态栏更新为 BridgeLogger 数据源
7. 验证: FOXGLOVE 模式下断掉 foxglove_bridge → 红色报错，不回退

### Phase 3: Store 集成 + 旧代码清理
8. RosManager 集成 RosDataStore/ServiceStore
9. ros_connector.js 标记 deprecated，逐步迁移调用方
10. 移除 RosManager 中的 ivgTransport fallback 路径

## 8. 回滚方案

每个 Phase 独立可回滚:
- Phase 1: 新文件不影响现有逻辑，删除即可
- Phase 2: `git revert` 单 commit，恢复 fallback 逻辑
- Phase 3: 逐文件 revert，RosManager 保留 fallback 路径直到最后阶段

## 9. 测试要点

1. **FOXGLOVE 模式 + foxglove_bridge 正常** → 所有 sub/pub/svc 走 foxglove，BridgeLogger 记录 bridge='foxglove'
2. **FOXGLOVE 模式 + foxglove_bridge 断开** → 报错，不回退 rosbridge，状态栏红色
3. **ROSBRIDGE 模式 + rosbridge 正常** → 所有 sub/pub/svc 走 rosbridge
4. **AUTO 模式 + foxglove 不可用** → 显式降级 rosbridge，dispatchEvent 'bridge:degraded'
5. **AUTO 模式 + foxglove 恢复** → 自动切回 foxglove，dispatchEvent 通知
6. **模式切换** → SubRegistry 先建后拆，迁移窗口 <500ms
7. **BridgeLogger 日志** → LogEventBus 收到 bridge:traffic 事件，调试面板可见
