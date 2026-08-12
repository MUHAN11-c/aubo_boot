# 桃子采摘商业化联动实施计划

1. 固化架构、接口迁移和安全边界。
2. 创建 `peach_harvest_msgs` 类型化消息、服务和动作接口。
3. 用测试先行创建 `peach_harvest_orchestrator` 纯状态核与 Lifecycle/Action 节点。
4. 将感知改为最新帧有界 worker，将重建改为目标会话的串行 worker。
5. 将 `peach_approach_grasp` 收敛为可取消的单目标 Action 能力，并保留兼容服务。
6. 将 Web 升级为任务中心、维护调试面板、动态参数档案和审计界面。
7. 增加统一 launch、单元/集成测试、运维与接口文档，完成 sim 模式验收。
