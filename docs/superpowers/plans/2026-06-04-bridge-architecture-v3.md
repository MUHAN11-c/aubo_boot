# Bridge 架构 v3.2 实现记录

> **实现日期**: 2026-06-04
> **设计文档**: `docs/superpowers/specs/2026-06-04-bridge-architecture-v3-design.md`
> **状态**: Phase 1-3 全部完成

## 变更汇总

### Phase 1: 基础设施 (已完成)

| 文件 | 操作 | 说明 |
|------|------|------|
| `js/core/log-bus.js` | 修改 | 新增 `bridge` 分类；暴露 `g.__logBus = logBus`；日志条目增加 `sessionId` |
| `js/transport/bridge_logger.js` | **新建** | BridgeLogger 统计引擎 — per-bridge 累加统计 + 活跃订阅追踪，持久化委托 LogEventBus |
| `js/entities/connection_store.js` | **新建** | ConnectionStore — BehaviorSubject 连接状态管理 |
| `js/entities/ros_data_store.js` | **新建** | RosDataStore — SWR 缓存 + LRU 淘汰 + 高频节流 |
| `js/entities/service_store.js` | **新建** | ServiceStore — 服务调用状态机 (5s 心跳) |
| `js/entities/ui_state_store.js` | **新建** | UIStateStore — localStorage 持久化 |
| `js/transport/hub.js` | 修改 | subscribe/callService/publish 集成 BridgeLogger (log-only) |

### Phase 2: 模式锁定 (已完成)

| 文件 | 操作 | 说明 |
|------|------|------|
| `js/transport/hub.js` | 修改 | `_reconcile()` 重写 — FOXGLOVE/ROSBRIDGE 模式锁定 + bridgedegraded/bridgerestored/bridgeerror 事件；`callService()` 移除 fallback |
| `js/transport/topic_router.js` | 重写 | 简化 `resolve()` — 移除大消息白名单和 channel 回退；保留 channel 检查改为 warn |
| `js/core/ivg_status_bar.js` | 修改 | 去掉 rosbridge 强制覆盖；updateBridgeDot() 新增降级/错误状态；updateFoxgloveStats() 使用 BridgeLogger 数据源；_bindHubEvents() 监听 bridgedegraded/bridgerestored/bridgeerror |

### Phase 3: Store 集成 + 清理 (已完成)

| 文件 | 操作 | 说明 |
|------|------|------|
| `js/core/ros.js` | 修改 | subscribe() 集成 RosDataStore；callService() 集成 ServiceStore；_wireHubBridgeChange() 集成 ConnectionStore；移除 ivgTransport fallback |
| `js/core/ros_connector.js` | 修改 | 标记 DEPRECATED |

## 关键设计决策

1. **BridgeLogger v3.2 精简**: 不维护自己的 _records 缓冲，持久化完全委托 LogEventBus。BridgeLogger 只维护 _stats 累加器和 _activeSubs 追踪
2. **FSD 四层** (非六层): pages→features→entities→shared，匹配项目零框架 MPA 特点
3. **bytes 统计延后**: 当前 TypedMessage 接口不包含原始字节数，Phase 2 适配器层增加 onRawMessage 后启用
4. **AUTO 模式完整降级/恢复链**: AUTO → bridgedegraded → rosbridge → bridgerestored → foxglove

## 文件统计

- 新建: 5 个文件 (1 bridge_logger + 4 stores)
- 修改: 6 个文件 (log-bus, hub, topic_router, ros, status_bar, ros_connector)
- 总计: 11 个文件
- 删除: 0 个文件 (渐进迁移，旧代码标记 deprecated)

## 验证要点

1. 浏览器 console: `__bridgeLogger.getStats()` 有数据
2. `__bridgeLogger.getActiveSubs()` 标注每个 topic 的 bridge 归属
3. `logBus.getLogs({source:'bridge'})` 有记录
4. FOXGLOVE 模式断 foxglove_bridge → 红色报错，不回退
5. AUTO 模式断 foxglove_bridge → 黄色降级，恢复后切回
6. status_bar 状态栏显示正确的 subs/pubs/svcs 计数
7. `__connectionStore.getState()` 返回当前连接状态
8. `__rosDataStore.get('/joint_states')` 返回缓存数据
