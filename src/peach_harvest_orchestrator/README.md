# peach_harvest_orchestrator

## 简介

本包是桃子采摘批次的**唯一所有者**（C++ 生命周期节点）。它不做运动规划、不做感知，
而是把四路就绪门、批次状态机、三级操作策略和类型化命令串成一条闭环：

```
WAITING_READY →（四路就绪）→ DISCOVERY → 拍照前置（go_to_photo_pose →
reset_global_targets）→ 感知收齐锁定 → RUNNING（按固定优先级逐目标派发
RunTargetCycle）→ 终态分级记账、自动推进 → 本轮耗尽复扫（递减集，max_rounds 上限）
→ COMPLETED → RunHarvest 携带 HarvestSummary 返回
```

单目标运动能力委托 `peach_approach_grasp` 的 RunTargetCycle action；目标身份与收齐
锁定归 `peach_pose_ros2`；Web（`peach_perception_web`）只代理本包的类型化命令。
业务栈不包含 AUBO 上电、松刹车或安全恢复。完整操作手册见
[docs/peach_harvest_operations.md](../../docs/peach_harvest_operations.md)。

## 使用方法

构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-up-to peach_harvest_orchestrator
source install/setup.bash
```

启动（整栈入口自带感知/重建/抓取/Web；也可只起编排器单节点）：

```bash
ros2 launch peach_harvest_orchestrator harvest_system.launch.py   # 采摘整栈
ros2 launch peach_harvest_orchestrator orchestrator.launch.py     # 仅编排器
```

默认 `auto_start_enabled=true` 但 `execution_enabled/grasp_enabled/tool_enabled=false`：
四路就绪后自动进 DISCOVERY 并走拍照/收齐流程，但 `begin_target` 被策略拒绝，**不会
派发任何运动**。开启运动须经 `~/set_operation_policy` 原子下发三级使能（依赖固定
`execution → grasp → tool`，目标周期运行中拒绝修改；两阶段提交，先下发到能力端参数
服务、复查 revision 后再提交状态机）。

常用操作：

```bash
# 长时批次入口（单 goal 守卫；完成/恢复/取消三路径均带 HarvestSummary）
ros2 action send_goal /peach_harvest_orchestrator/run_harvest \
  peach_harvest_msgs/action/RunHarvest "{request_id: run-1, target_source: 0}"

# 批次控制（command：0 暂停 / 1 恢复 / 2 进维护 / 3 出维护 / 4 立即取消 /
# 6 跳过当前目标 / 7 确认恢复；5 重试为预留恒拒）
ros2 service call /peach_harvest_orchestrator/control \
  peach_harvest_msgs/srv/ControlHarvest \
  "{request_id: op-1, expected_revision: <N>, command: 6, reason: ''}"

# 三级策略（示例：仅开执行，保持抓取/工具关闭）
ros2 service call /peach_harvest_orchestrator/set_operation_policy \
  peach_harvest_msgs/srv/SetOperationPolicy \
  "{request_id: pol-1, expected_revision: <N>, auto_start_enabled: true,
    execution_enabled: true, grasp_enabled: false, tool_enabled: false}"
```

`SKIP_TARGET` 仅在目标周期运行中接受：登记跳过意图并真取消活动 goal，取消落地后
记 `CANCELED`（操作员跳过）并推进感知计划；`CANCEL_NOW` 同样真取消 goal，批次进
INTERRUPTED 转暂停。接触段 `recovery_required` 不进账，批次停等现场人工撤离后
`ACKNOWLEDGE_RECOVERY`（保持暂停）再 `RESUME`。

## 接口

发布（生命周期激活后）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/state` | `peach_harvest_msgs/HarvestState` | 批次/阶段/阻塞/使能/revision/run_id/cycle_id/progress（transient_local） |
| `~/events` | `peach_harvest_msgs/HarvestEvent` | 过程与审计事件（severity + run_id/cycle_id/target_id） |

服务与 action：

| 名称 | 类型 | 说明 |
|---|---|---|
| `~/run_harvest` | `RunHarvest` Action | 长时批次入口、200ms 反馈、取消与 HarvestSummary |
| `~/control` | `ControlHarvest` | 暂停/恢复/维护/立即取消/跳过/恢复确认（幂等 request_id + revision 乐观锁） |
| `~/set_operation_policy` | `SetOperationPolicy` | 原子三级使能（两阶段提交） |

订阅（3 个话题名硬编码，暂不可参数重配）：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/peach/perception/target_observations` | `PeachTargetObservationArray` | 锁定沿/选中目标/目标数（发现数累计） |
| `/peach/reconstruction/diagnostics` | `std_msgs/String` | 重建就绪路新鲜度 |
| `/aubo_io_controller/robot_status` | `aubo_msgs/RobotStatus` | motion 就绪路（急停/故障/上电/可运动） |

下游依赖（5 个接口名由参数重配，默认值见参数表）：

| 参数 | 类型 | 用途 |
|---|---|---|
| `target_cycle_action_name` | `RunTargetCycle` Action client | 单目标派发与取消传播 |
| `photo_pose_service_name` | `Trigger` client | 拍照前置：移动到 `global_photo_pose` |
| `reset_targets_service_name` | `Trigger` client | 拍照前置：重置感知收齐窗口 |
| `complete_target_service_name` | `Trigger` client | 目标终态后推进感知固定优先级计划 |
| `approach_node_name` | 参数服务 client | 策略三级使能的原子下发 |

## 参数

`config/orchestrator.yaml` 为权威源（代码 `declare_parameter` 默认值逐一对齐）：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `auto_start_enabled` | `true` | 四路全就绪后自动进入 DISCOVERY |
| `execution_enabled` / `grasp_enabled` / `tool_enabled` | `false` | 三级操作策略初始值（交付默认全关） |
| `readiness.web` | `true` | Web 就绪门（静态开关） |
| `readiness.timeout_s` | `2.0` | 四路输入新鲜度超时（s） |
| `readiness.require_robot_status` | `true` | motion 路是否要求 RobotStatus 正常 |
| `global_photo_joints` | 见 YAML | 拍照位姿关节角存档（仅对照；实际移动走 SRDF 命名状态 `global_photo_pose`） |
| `photo_pose.enabled` | `true` | 拍照前置开关；execution 关时自动跳过移动 |
| `photo_pose.max_retries` | `3` | 拍照前置连续失败此次数后进入 RECOVERY_REQUIRED |
| `photo_pose.retry_cooldown_s` | `5.0` | 拍照前置失败重试冷却（s） |
| `photo_pose.service_timeout_s` | `90.0` | 单次 go_to_photo_pose / reset_global_targets 调用超时（s） |
| `photo_pose.return_on_complete` | `true` | 批次完成后 best-effort 再回一次拍照位姿（仅记事件） |
| `harvest.rescan_until_empty` | `true` | 复扫递减集循环开关 |
| `harvest.max_rounds` | `3` | 总轮次上限（round 从 1 起计，含首轮） |
| `dispatch.retry_delay_s` | `2.0` | goal 被能力端拒绝后的冷却重试间隔（s），须 ≥ 感知帧周期 |
| `dispatch.max_retries` | `4` | 同一目标拒绝重试上限，耗尽按 SKIPPED_UNREACHABLE 记账跳过 |
| `dispatch.max_consecutive_rejections` | `6` | 跨目标连续拒绝熔断上限（接受即清零），达到即进 RECOVERY_REQUIRED |
| `target_cycle_action_name` | `/peach_approach_grasp_node/run_target_cycle` | 单目标 action 名 |
| `approach_node_name` | `/peach_approach_grasp_node` | 能力端节点名（策略下发） |
| `complete_target_service_name` | `/peach_pose_node/complete_selected_target` | 感知计划推进服务名 |
| `reset_targets_service_name` | `/peach_pose_node/reset_global_targets` | 感知重置服务名 |
| `photo_pose_service_name` | `/peach_approach_grasp_node/go_to_photo_pose` | 拍照位姿服务名 |

## 与批次流程的关系

- **就绪门**：perception 只看观测流新鲜（不要求已锁定，收齐窗口期不卡回
  WAITING_READY）；reconstruction 看诊断流新鲜；motion = action server 在线 +
  RobotStatus 正常；web 为静态开关。推算在纯核 `ReadinessTracker` 完成。
- **拍照前置**（节点侧 PhotoStep：`NOT_STARTED → MOVE_PENDING → MOVING →
  RESET_PENDING → RESETTING → DONE`）：两步均为异步 Trigger，future 由 500ms 定时器
  轮询，锁内绝不阻塞等 RPC；失败冷却重试，连续超限进 RECOVERY_REQUIRED。首轮且无需
  移动时直接完成（不打扰感知）；复扫轮即使不移动也必须重置感知开启新一轮收齐。
- **派发**：`allow_dispatch` 纯函数五项前置（拍照前置完成、selected 非空、非重复
  派发、无活动目标、action server 就绪）；`begin_target` 另要求 execution 使能。
- **终态路由**：action 回调锁内按 outcome 记账（`record_target_outcome`），锁外调
  `complete_selected_target` 推进感知；只有 `recovery_required` 中断批次等人工。
- **复扫**：本轮锁定集合耗尽后 `decide_round` 纯函数判定——摘到目标且未达
  `max_rounds` 回拍照位姿新一轮；锁定空集/达上限/复扫关闭则 `complete_batch`。
- **结账**：`RunHarvest` 在 COMPLETED succeed、recovery abort、客户端取消级联取消
  活动目标 goal 后 canceled，三路径均携带 `HarvestSummary`（计数 + 逐目标 outcomes）。
- **RunHarvest 取消 ≠ 批次停止**：取消 action 只停其执行线程（goal 以 canceled
  终局并携带 summary），批次状态机不受影响、仍按当前策略继续运行；要停批次用
  `~/control` 的 `PAUSE`（安全检查点后暂停，可 RESUME）或 `CANCEL_NOW`（真取消
  活动目标 goal，批次进 INTERRUPTED 转暂停）。

## 软件框架

| 文件 | 职责 |
|---|---|
| `include/.../state_machine.hpp` + `src/state_machine.cpp` | 零 ROS 纯核：状态机、`ReadinessTracker`、`allow_dispatch`、`decide_round`、幂等命令与策略终审 |
| `src/harvest_orchestrator_node.cpp` | ROS 接线：生命周期、订阅/客户端、PhotoStep 轮询、RunHarvest 执行线程、事件与状态发布 |
| `config/orchestrator.yaml` | 全部参数的权威默认值 |
| `launch/harvest_system.launch.py` / `orchestrator.launch.py` | 整栈入口 / 单节点入口 |
| `test/test_state_machine.cpp`、`test/test_policy_service.cpp` | 纯核状态机单测与策略服务测试 |

并发约定：互斥锁内只改状态、绝不阻塞等 RPC；action 结果回调锁内路由、锁外推进
感知；策略两阶段提交不持锁跨 RPC；RunHarvest 执行线程随 deactivate/析构 join。
