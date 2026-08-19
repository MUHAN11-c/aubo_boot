# peach_interfaces

采摘链路**唯一跨包 IDL**。感知、重建、技能、执行器、监控都只依赖本包消息，不再另起 `peach_*_msgs`。

工作区总览见仓库根 [README.md](../../README.md)；流程见 [docs/flow.md](../../docs/flow.md)。

## 职责

定义批次、场景、目标、重建、抓取结果的类型。改字段只改这里，各节点一起跟。

## 不负责

运行时逻辑、参数默认值、launch。

## 动作（4）

| 动作 | 服务端 | 作用 |
|------|--------|------|
| `RunHarvest` | `peach_task_executor` | 显式开一批：BeginScene → SurveyScene → 逐目标 ExecuteTarget |
| `SurveyScene` | `peach_manipulation_skills` | 去拍照位姿，返回快照引用与是否降级 |
| `BuildTargetModel` | `peach_target_reconstruction` | 绑定目标、等合格视角、finalize TSDF/精化 |
| `ExecuteTarget` | `peach_manipulation_skills` | 单目标：观察扫描 / 再确认 / MTC / 工具 / 同轴撤退 |

`RunHarvest` 的 goal 带 `request_id`、`scene_key`、`profile_id`、`intent`、`selection_mode`、可选 `target_ids`。结果是 `HarvestSummary`（含 `TargetOutcome` 账本）。

`ExecuteTarget` 模式：`PREVIEW=0`、`OBSERVE_ONLY=1`、`FULL=2`。终局：`SUCCEEDED` / `SKIPPED_*` / `FAILED` / `CANCELED`。

## 服务（2）

| 服务 | 服务端 | 作用 |
|------|--------|------|
| `BeginScene` | `peach_scene_perception` | 新场景：清身份表、推进 `scene_epoch` |
| `ControlTask` | `peach_task_executor` | PAUSE / RESUME / 维护 / CANCEL_NOW / SKIP_TARGET / ACKNOWLEDGE_RECOVERY；`expected_state_seq` 防乱序 |

## 主要消息

- **观测**：`PeachTargetObservation` / `Array`（稳定 `target_id`、确认、锁定集、选中字段）
- **几何**：`BagGraspCandidate`、`BagFitting`、`BagGrasp2D` 及数组
- **批次**：`HarvestState`、`HarvestSummary`、`TargetOutcome`、`CanonicalEvent`
- **重建**：`ReconstructionStatus`、`GraspDecision`、`TargetModel`、`TargetQuality`
- **契约预留**（节点尚未全部接线）：`SceneSnapshot`、`JobIntent`、`ShapeHypothesis`、`GraspHypothesis`、`HarvestEvent`

`MatchStatus`：`OK` / `NEW` / `AMBIGUOUS` / `REJECTED`。身份歧义标 AMBIGUOUS，不强制合并。

## 约定

C++17 生成；依赖 `geometry_msgs`、`sensor_msgs`、`std_msgs`、`diagnostic_msgs`、`action_msgs`。改 IDL 后需先编本包再编下游。
