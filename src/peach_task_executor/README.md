# peach_task_executor

批次**唯一所有者**：显式 `RunHarvest`、选择、账本、`ControlTask`。**launch 绝不自动开批。**

## 流程（现行源码）

```
显式 RunHarvest（须 Lifecycle Active）
  → BeginScene（感知：清身份、scene_epoch）
  → SurveyScene（技能：拍照位姿）
  → 等待 survey_wait_s 或 target_set_locked
  → 循环未入账目标（goal.target_ids 优先，否则已确认观测）
       → execution_enabled=false：记 SKIPPED 后结束（默认）
       → true：ExecuteTarget FULL，按动作结果写 TargetOutcome
  → 结算 HarvestSummary
```

PAUSE 时循环卡住；`ControlTask` CANCEL_NOW 置取消。状态话题带当前 `target_id`，重建优先跟这个 ID 绑定。

不在批次里先调 `BuildTargetModel`（空会话 finalize 会误失败）。观察扫描在 `ExecuteTarget` 行为树内。

## 启动

```bash
# 只起执行器（需先有能力节点）
ros2 launch peach_task_executor executor.launch.py

# 整栈（手臂 + 感知 + 重建 + 技能 + Web + 本节点）
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=sim camera_enabled:=false
```

整栈默认 `hardware_mode:=sim`、`camera_enabled:=false`；真机须显式 `real`。

## 接口

| | 名 |
|--|--|
| 动作 | `~/run_harvest` |
| 服务 | `~/control`（`ControlTask`，`expected_state_seq`） |
| 发布 | `~/state`（`HarvestState`，TRANSIENT_LOCAL）、`~/events`（`CanonicalEvent`，TRANSIENT_LOCAL） |

客户端（可配）：

- `/peach_scene_perception_node/begin_scene`
- `/peach_manipulation_skills_node/survey_scene`
- `/peach_manipulation_skills_node/execute_target`

## 参数

- `execution_enabled` 默认 false
- `survey_wait_s` 默认 8
- `service_timeout_s` / `action_timeout_s`

## 红线

禁止自动 `RunHarvest`。真机运动另开技能端使能并授权。禁止 `/aubo_dashboard/startup`。
