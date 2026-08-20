# peach_interfaces

采摘链路**唯一跨包 IDL**。感知、重建、技能、执行器、监控都只依赖本包消息。

- 流程与阅读地图：[docs/flow.md](../../docs/flow.md)
- 契约清单（名称 / 类型 / QoS / 生产消费方）：[`config/interface_manifest.yaml`](config/interface_manifest.yaml)
- 漂移检查：`python3 scripts/check_interface_manifest.py`

## 职责

定义批次、场景、目标、重建、抓取结果的类型。改字段只改这里，各节点一起跟。

## 不负责

运行时逻辑、参数默认值、launch。

## 从哪读

| 路径 | 内容 |
|------|------|
| `action/RunHarvest.action` | 开批 goal / feedback / `HarvestSummary` |
| `action/SurveyScene.action` | 拍照位姿 + 快照引用 |
| `action/BuildTargetModel.action` | 绑定目标、视角进度、finalize |
| `action/ExecuteTarget.action` | PREVIEW / OBSERVE_ONLY / FULL；`HarvestResult` 等 |
| `srv/BeginScene.srv` | `scene_epoch` |
| `srv/ControlTask.srv` | 人工命令 + `expected_state_seq` |
| `msg/HarvestState.msg` | `batch_state` / `target_phase` / `target_id`（由 `harvest_fsm` 填） |
| `msg/CanonicalEvent.msg` | 记录器事件 |
| `msg/PeachTargetObservation*.msg` | 身份与锁定集 |
| `config/interface_manifest.yaml` | 话题动作服务清单 |

## 动作（4）

| 动作 | 服务端 | 作用 |
|------|--------|------|
| `RunHarvest` | `peach_task_executor` | 显式开一批 |
| `SurveyScene` | `peach_manipulation_skills` | 去拍照位姿 |
| `BuildTargetModel` | `peach_target_reconstruction` | 等合格视角后 finalize |
| `ExecuteTarget` | `peach_manipulation_skills` | 单目标周期 |

`RunHarvest` goal：`request_id`、`scene_key`、`profile_id`、`intent`、`selection_mode`、可选 `target_ids`。

`ExecuteTarget` 模式：`PREVIEW=0`、`OBSERVE_ONLY=1`、`FULL=2`。终局：`SUCCEEDED` / `SKIPPED_*` / `FAILED` / `CANCELED`。

## 服务（2）

| 服务 | 服务端 | 作用 |
|------|--------|------|
| `BeginScene` | `peach_scene_perception` | 清身份、推进世代 |
| `ControlTask` | `peach_task_executor` | PAUSE / RESUME / 维护 / CANCEL_NOW / SKIP_TARGET / ACKNOWLEDGE_RECOVERY |

## 主要消息

- **观测**：`PeachTargetObservation` / `Array`（稳定 `target_id`、确认、锁定集）
- **几何**：`BagGraspCandidate`、`BagFitting`、`BagGrasp2D` 及数组
- **批次**：`HarvestState`、`HarvestSummary`、`TargetOutcome`、`CanonicalEvent`
- **事件码**：`target_dispatched`、`target_succeeded`、`target_skipped`、`target_failed`、`target_canceled`、`target_operator_skipped`、`round_locked`
- **重建**：`ReconstructionStatus`、`GraspDecision`、`TargetModel`、`TargetQuality`
- **契约预留**（节点尚未全部接线）：`JobIntent`、`ShapeHypothesis`、`GraspHypothesis`、`HarvestEvent`

`MatchStatus`：`OK` / `NEW` / `AMBIGUOUS` / `REJECTED`。歧义不强制合并。

C++17 生成。改 IDL 后先编本包再编下游。
