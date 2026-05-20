# IVG2.0 上下文地图

> 本文档指向项目各领域的上下文文档。多上下文布局 — 不同角色按需查阅对应文档。

---

## 文档索引

### 使用与运维

| 文档 | 面向 | 说明 |
|------|------|------|
| [USAGE.md](./docs/USAGE.md) | 操作者 | 系统启动/停止、Web 界面入口、日常操作流程、状态监控、常见问题 |
| [DEPLOYMENT.md](./DEPLOYMENT.md) | 运维 | 环境准备、依赖安装、编译构建、配置调优、首次部署验证 |

### 开发与架构

| 文档 | 面向 | 说明 |
|------|------|------|
| [architecture.md](./docs/architecture.md) | 架构师/开发者 | 6 层架构蓝图、组件职责、数据流、技术栈总表、设计决策、审计追踪 |
| [PROCESS-FLOW.md](./docs/PROCESS-FLOW.md) | 开发者/调试者 | 16 步启动流程时序、每步详细说明、感知管道、工具快换链路、调试命令 |
| [frontend-migration-plan.md](./docs/frontend-migration-plan.md) | 前端开发者 | Vue 3 迁移方案、技术栈、代码映射、4 阶段计划、npm 依赖清单 |
| [ZERO-BASIS-PREREQUISITES.md](./aubo_ros2_ws/docs/ZERO-BASIS-PREREQUISITES.md) | 零基础学习者 | 分阶段前置知识：Linux/ROS2/MoveIt/视觉/Web/GPU + 官方资料索引 |
| [ZERO-BASIS-REPLICATION.md](./aubo_ros2_ws/docs/ZERO-BASIS-REPLICATION.md) | 零基础部署者 | 从零复刻 8 阶段：环境→编译→GraspNet→前端→硬件→分步验收 |

### 项目参考

| 文档 | 说明 |
|------|------|
| [large-files-inventory.md](./docs/large-files-inventory.md) | 被 .gitignore 排除的大文件清单 |
| [CLAUDE.md](./CLAUDE.md) | Claude Code 协作指令：构建规则、关键知识点、常见报错速查 |

---

## 快速导航

```
我想...                           → 看这个
─────────────────────────────────────────────
启动系统                          → docs/USAGE.md §二
排查启动失败                      → docs/PROCESS-FLOW.md §八
在新机器上部署                    → DEPLOYMENT.md（或零基础: aubo_ros2_ws/docs/ZERO-BASIS-REPLICATION.md）
从零学习本项目技术栈              → aubo_ros2_ws/docs/ZERO-BASIS-PREREQUISITES.md
理解系统架构                      → docs/architecture.md
追踪一条抓取请求的完整链路         → docs/PROCESS-FLOW.md §五
修改前端代码                      → docs/frontend-migration-plan.md
了解为什么选这个技术栈             → docs/architecture.md §五
查看已知架构问题                   → docs/architecture.md §七
理解 AUBO SDK 双连接               → CLAUDE.md §1
```

---

*最后更新: 2026-05-15*
