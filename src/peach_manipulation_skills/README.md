# peach_manipulation_skills

能力端：拍照观察、主动视点、质量门、再确认、MTC 接近/插入、工具、同轴撤退。

**Lifecycle**：仅 **Active** 才允许运动类入口。Unconfigured / Inactive 拒绝 `ExecuteTarget`、`SurveyScene`、`go_to_photo_pose`、预览与 arm。

## 流程

批次侧：

1. 执行器调 `SurveyScene` → 本节点 `goToPhotoPose`（SRDF 命名位姿，默认 `global_photo_pose`）
2. 执行器选 `target_id` 后发 `ExecuteTarget`（FULL）
3. 主树 `config/harvest_tree.xml` 的 `PeachHarvest`：观察扫描 → finalize/质量门 → 再确认 → MTC → 工具 → 同轴撤退

`ExecuteTarget` 以 **goal.target_id** 为准：锁定集里有有效锚点即受理，不再要求感知 `selected_target_id` 一致。

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 为关：只规划、不接触、不 SetIO。

## 启动

```bash
ros2 launch peach_manipulation_skills approach_grasp.launch.py
```

整栈里由 `harvest_system.launch.py` 拉起，并 configure→activate。参数：`config/approach_grasp.yaml`（与 `generate_parameter_library` 对齐）。

## 动作

| 动作 | 名 | 行为 |
|------|----|------|
| `SurveyScene` | `~/survey_scene` | 去拍照位姿，带回当前快照 id / 是否降级 |
| `ExecuteTarget` | `~/execute_target` | 单目标周期；PREVIEW / OBSERVE_ONLY / FULL |

## 服务（节点相对名）

`start_cycle`、`cancel_cycle`、`query_state`、`go_to_photo_pose`、`preview_approach_insert`、`preview_full_contact`、`set_execution_armed`、`acknowledge_recovery`。

长规划服务走独立互斥回调组，不堵订阅。

## 订阅 / 发布

订阅感知 `target_observations`、重建诊断/精化/抓取许可、`/aubo_io_controller/robot_status`。

发布 `~/status`（JSON，闩锁）、`~/planned_views`（视点 Marker）。

## 要点

- 关节顺序与 MoveIt 组必须是权威六轴名。
- 真运动须执行器 `execution_enabled` 与本包 `execution.enabled` 同时开，并经人工授权。
- 接触后若失败，置 recovery，须 `acknowledge_recovery` 才继续。
- 不再调用已删除的 `complete_selected_target`；账本在执行器。
- 观察段仍可能调用重建 `reset` / `finalize` Trigger（能力内部）。
