# `peach_interfaces` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包。系统级架构在 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。**源码与本文互相更新，改一边须同一轮改另一边**。

---

### `peach_interfaces` — 跨包唯一契约

**作用：** 各能力包之间唯一允许的消息/服务/动作类型。感知两节点之间、技能、调度、监控都只依赖本包，禁止互相 `import` 业务模块传结构体。

**含什么：** 无节点、无 launch、无运行参数。`msg/` `srv/` `action/`（文件头写话题/谁发谁订，字段行内注释）+ `config/interface_manifest.yaml`（名称/类型/QoS/生产消费方；57 active + 5 reserved——预留第五个为 `peach_sim/joint_states` gz 桥登记，非导航；`scripts/check_interface_manifest.py` 双向核对）。管子与字段含义：[README.md](../../peach_interfaces/README.md)。

**对外提供：**

| 种类 | 名字 | 谁当服务端 | 语义 |
|------|------|------------|------|
| 动作 | `RunHarvest` | 调度 | 显式开一批 |
| 动作 | `NavigateToWorksite` | （预留，导航包已归档） | 走到作业位；调度直通 `NAV_OK`，无现行服务端 |
| 动作 | `SurveyScene` | 技能 | 去拍照位姿并复核关节；DISCOVERY 首巡在 Begin 之前 |
| 动作 | `BuildTargetModel` | 重建 | 绑定目标、等视角、finalize |
| 动作 | `ExecuteTarget` | 技能 | PREVIEW / OBSERVE_ONLY / FULL / PREGRASP_ONLY |
| 动作 | `MoveTo` | 技能 | 视点/拍照位/命名位/位姿转移移动（KIND_JOINTS 预留未实现）；与接触周期互斥 |
| 服务 | `CheckReachability` | 技能 | 选果：预抓取 IK；FULL 再检套入终点 IK 与沿轴笛卡尔；不动臂 |
| 服务 | `BeginScene` | 场景感知 | 重启收齐窗、推进 `scene_epoch`；换场才清身份 |
| 服务 | `ControlTask` | 调度 | 暂停/跳过/取消/ACK 恢复 |
| 服务 | `SetEnables` | 调度 | 操作台使能开关；广播 `/peach/batch/enables` + 审计 |
| 服务 | `SetBatchPolicy` | 调度 | 运行期改批次策略（当前批或下批默认） |
| 服务 | `FireStep` | 调度 | 操作台单步；现行仅 PHOTO 已接线（MoveTo KIND_NAMED），其余档位占位调用即拒；未来运动类档位过臂侧命令门 |
| 服务 | `ManageLifecycleNodes` | lifecycle 管理器（仅随包内独立 launch；现行整栈同名服务由 nav2_lm 以 `nav2_msgs` 类型提供） | 整栈 STARTUP…SHUTDOWN；不发 `RunHarvest` |

主要消息：观测 `PeachTargetObservation*`；几何初值 `BagGraspCandidate*` / `BagFitting*`；批次 `HarvestState` / `HarvestSummary` / `TargetOutcome` / `CanonicalEvent`；重建 `GraspDecision` / `ReconstructionStatus` / `TargetModel`；执行合同 `Enables` / `Clearance` / `ToolState`。契约预留、节点尚未全部接线：`ShapeHypothesis`、`GraspHypothesis`。导航预留（manifest `reserved_interfaces` 区）：`NavigateToWorksite`、`HarvestTargetReport`、`VehicleState`、`HarvestOperationStatus`。已删类型（e2f37ff，勿再造）：`JobIntent`（常量并入 `RunHarvest` 的 `INTENT_*`）、`HarvestEvent`、`MatchStatus`。

**禁止：** 跑节点、设算法默认值、写 launch、夹带视觉/运动实现。
