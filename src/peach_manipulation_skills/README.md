# peach_manipulation_skills

能力端：拍照观察、主动视点、质量门、再确认、MTC 接近/插入、工具 GPIO、同轴撤退。

**Lifecycle**：仅 **Active** 才允许运动类入口。总览：[docs/flow.md](../../docs/flow.md)。

## 从哪读

| 文件 | 职责 |
|------|------|
| `src/approach_grasp_node.cpp` + `approach_grasp_node_impl.hpp` | 节点外壳：Lifecycle、`createSubscriptions/Services/Actions`、预规划槽 |
| `src/cycle_action.cpp` | `ExecuteTarget` / `SurveyScene` 接受、执行、取消 |
| `config/harvest_tree.xml` | 主树 `PeachHarvest` 与 SubTree 阶段 |
| `src/bt_nodes.cpp` | 树节点实现（观察、质量、再确认、MTC、工具、撤离） |
| `include/.../grasp_task.hpp` + `src/grasp_task.cpp` | MTC：`SerialContainer` 入口+插入；`syncKeepoutCollisionObjects` |
| `src/motion_interface.cpp` | MoveGroup / 拍照位姿 / 预览服务 |
| `include/.../protected_zones.hpp` | 保护区 AABB 纯核（参数 stride-6） |
| `include/.../view_planner.hpp`、`quality_gate.hpp`、`reconfirm_policy.hpp` | 视点、质量、再确认策略 |
| `config/approach_grasp.yaml` | 运行参数；与 `approach_grasp_node_parameters.yaml` 对齐 |

读单周期：先 XML 看阶段顺序，再 `bt_nodes.cpp` 里同名节点，接触段进 `GraspTask`。工具 IO 只在 BT `ActuateTool`，不 attach 果体。

## 谁调谁

| 方向 | 内容 |
|------|------|
| 被执行器调 | `SurveyScene`、`ExecuteTarget`（PREVIEW / OBSERVE_ONLY / FULL） |
| 订阅 | 感知 `target_observations`；重建 `diagnostics` / `refined_*` / `grasp_decision`；`HarvestState` 间接经观测 selected |
| 发布 | `~/status`、`~/planned_views`、`/peach/manipulation/grasp_hypothesis` |
| 不调用 | 重建 `reset`/`finalize` Trigger；账本在执行器 |

`ExecuteTarget` 以 **goal.target_id** 为准。锁定集里有有效锚点即受理。

## 流程

1. `SurveyScene` → `goToPhotoPose`
2. 批次对每个目标：并行 `BuildTargetModel` + `OBSERVE_ONLY`，再 `FULL`（`skip_observation`）
3. 主树：观察（可跳过）→ 等精化 → 再确认 → MTC → 工具 → 同轴撤退 → 卸果（未标定则跳过）

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 为关。

接触失败置 recovery，须 `acknowledge_recovery`（或执行器 ControlTask ACK）才继续；未确认时执行器不派下一颗。

## 启动

```bash
ros2 launch peach_manipulation_skills approach_grasp.launch.py
```

整栈里 `autostart:=false`，由 lifecycle manager 转换。

## 动作 / 服务

| 动作 | 名 | 行为 |
|------|----|------|
| `SurveyScene` | `~/survey_scene` | 去拍照位姿，带回快照 id / 是否降级 |
| `ExecuteTarget` | `~/execute_target` | 单目标周期 |

服务：`start_cycle`、`cancel_cycle`、`query_state`、`go_to_photo_pose`、`preview_approach_insert`、`preview_full_contact`、`set_execution_armed`、`acknowledge_recovery`。长规划走独立互斥回调组。

真运动须执行器 `execution_enabled` 与本包使能同时开，并经人工授权。关节名必须是权威六轴顺序。
