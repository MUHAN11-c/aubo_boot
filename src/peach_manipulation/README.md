# peach_manipulation

四个能力包之一：**机械臂执行**。一节点。拍照、主动视点、质量/安全门、预抓取验证、沿轴套入、刀具 GPIO、原路撤退。不写账本、不调重建 Trigger、不 `BeginScene`、不 `RunHarvest`。详细作用见 [docs/architecture.md](../../docs/architecture.md) §3 `peach_manipulation`。

**Lifecycle**：仅 **Active** 才允许运动类入口。总览：[docs/architecture.md](../../docs/architecture.md)。契约：[docs/io.md](../../docs/io.md)。

```
peach_manipulation/
  include/peach_manipulation/
  src/*.cpp
  config/{peach_manipulation.yaml,manipulation_parameters.yaml,tool_profiles/}
  launch/peach_manipulation.launch.py
```

## 从哪读

| 文件 | 职责 |
|------|------|
| `src/manipulation_skills_node.cpp` + `include/.../manipulation_skills_node_impl.hpp` | 节点外壳：Lifecycle、订阅/服务/动作 |
| `src/cycle.cpp` | `ExecuteTarget` / `SurveyScene` 受理与取消；`authorizeStage` 授权矩阵 |
| `src/stages.cpp` | 阶段执行器：`executeCycle(ctx)` 显式模式 switch，序列与旧主树严格同构 |
| `src/quality_gate.cpp` / `view_planner.cpp` / `safety_gate.cpp` / `target_cache.cpp` | `_core`：质量门、视点、安全门、目标缓存 |
| `include/.../grasp_task.hpp` + `src/grasp_task.cpp` | MTC：预抓取先拍照位再 LIN/CIRC/PTP + 沿轴插入；`syncKeepoutCollisionObjects` |
| `src/motion.cpp` | MoveGroup / 拍照位姿 / 预览服务 |
| `include/.../protected_zones.hpp` | 保护区 AABB 纯核（参数 stride-6） |
| `include/.../view_planner.hpp`、`quality_gate.hpp`、`reconfirm_policy.hpp` | 视点、质量、再确认策略 |
| `config/peach_manipulation.yaml` | 运行参数；与 `config/manipulation_parameters.yaml` 对齐 |

读单周期：先 `stages.cpp` 的 `executeCycle(ctx)` 看阶段顺序，接触段进 `GraspTask`。工具 IO 只在 `ActuateCutter` 阶段（`ToolActuator`），ACK 不等于切断确认。

## 谁调谁

| 方向 | 内容 |
|------|------|
| 被执行器调 | `SurveyScene`、`ExecuteTarget`（PREVIEW / OBSERVE_ONLY / FULL / PREGRASP_ONLY） |
| 订阅 | 感知 `target_observations`；重建 `diagnostics` / `refined_*` / `grasp_decision`。`pregrasp_verification` 由重建发布作观测，技能 `VerifyPregrasp` 用工具 TF 残差（非该话题） |
| 发布 | `~/status`、`~/planned_views`、`/peach/manipulation/grasp_hypothesis` |
| 不调用 | 重建 `reset`/`finalize` Trigger；账本在执行器 |

`ExecuteTarget` 以 **goal.target_id** 为准。锁定集里有有效锚点即受理。

## 流程

1. `SurveyScene` → `goToPhotoPose`（`transit_max_*` 护栏；超限拒绝）
2. 批次对每个目标：并行 `BuildTargetModel` + `OBSERVE_ONLY`，再 `PREGRASP_ONLY` 或 `FULL`（`skip_observation`；调度默认 PREGRASP_ONLY）
3. 阶段序列（`executeCycle`）：观察（可跳过）→ 等精化 → 再确认 → 预抓取验证 →（PREGRASP_ONLY 停住）或套入/工具/原路撤退

默认 `execution.enabled` / `grasp.enabled` / `tool.enabled` 为关。

接触失败撤离未确认，或 `PREGRASP_ONLY` 到位（终局 `SUCCEEDED` + `recovery_required`），须 `acknowledge_recovery`（或执行器 ControlTask ACK）才 Survey / 派下一颗。

## 启动

```bash
ros2 launch peach_manipulation peach_manipulation.launch.py
```

整栈里 `autostart:=false`，由 lifecycle manager 转换。

## 动作 / 服务

| 动作 | 名 | 行为 |
|------|----|------|
| `SurveyScene` | `~/survey_scene` | 去拍照位姿，带回快照 id / 是否降级 |
| `ExecuteTarget` | `~/execute_target` | 单目标周期 |

服务：`start_cycle`、`cancel_cycle`、`query_state`、`go_to_photo_pose`、`preview_approach_insert`、`preview_full_contact`、`set_execution_armed`、`acknowledge_recovery`。长规划走独立互斥回调组。

真运动须执行器 `execution_enabled` 与本包使能同时开，并经人工授权。关节名必须是权威六轴顺序。
