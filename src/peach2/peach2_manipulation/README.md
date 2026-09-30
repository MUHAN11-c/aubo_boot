# peach2_manipulation

Peach v2 运动能力：所有轨迹与刀具 SetIO 的**单一命令门**、单目标采摘周期（staging → pregrasp → 套入 → 剪切 → 倒放出冠 →
放果）、MoveTo 赶路、plan-only 可达性检查。默认 `execution/grasp/tool` 全 false ⇒ 只规划，不下发任何轨迹或 SetIO；
启动（configure / activate）不动臂、不写 IO。

命令门是应用护栏，不是急停：急停 / 保护停止在柜体与示教器（硬件，不经 ROS）。关门时的“停”= 取消
`execute_trajectory` + `MoveGroupInterface::stop()`（驱动侧透传取消 → `RobotMoveStop`）。

## 公有 API

安装头 `include/peach2_manipulation/`（纯核零 ROS，库 `peach2_manipulation_core`）：

| 头文件 | 内容 |
|--------|------|
| `command_gate.hpp` | `CommandGate`：`Active ∧ robot_status 持续条件（drives ∧ ¬e_stop ∧ ¬in_error ∧ 年龄<0.3 s）∧ 新轨迹前 motion_possible ∧ enables 链 tool⇒grasp⇒execution ∧ 心跳未超时 ∧ ¬cancel ∧ 阶段许可`；`GateStage`（TRANSIT/APPROACH/CONTACT/TOOL/RETREAT/RELEASE/TOOL_SAFE）；`ClosedEdge` |
| `trajectory_reverse.hpp` | `reverse_trajectory`（位置倒序、速度取反、**加速度不变号**）、`reverse_path`（多段逆序拼接去重）、`joint_path_length`、`max_joint_deviation` |
| `staging_geometry.hpp` | `staging_pose`（袋底沿 −axis 0.15–0.25 m，冠外）、`tcp_for_blade`、`pose_with_roll`、`pregrasp_residual`（横向/轴向/倾角，roll 不计）、`insert_geometry` |
| `harvest_cycle.hpp` | `HarvestCycle::run(CycleRequest)` → `CycleResult`（分阶段耗时、reached、failure_code、recovery_required、plan_only）；`ScenePhase`（APPROACH / CONTACT）与 `CycleDeps::scene` 钩子 |
| `bag_obstacles.hpp` | 邻袋碰撞对象纯核：`bag_capsule` / `desired_bag_capsules`（id `peach_bag_<target_id>`，底→颈圆柱，R = d95/2 + `bag_margin_m`）、`BagObstacleSet`（与场景现状求原子 diff：ADD/替换/REMOVE，`adopt` 接管遗留对象，`clear_all`） |
| `move_to.hpp` | `run_move_to`（OMPL 全臂受查；执行关时 plan-only；`MoveToDeps::scene` 规划前写全部邻袋） |
| `motion_backend.hpp` | `MotionBackend` 缝（plan / execute / validate / current_joints / current_tcp / stop） |
| `decision_client.hpp` | `DecisionClient`（同步 GetDecision，**不缓存**）、`TargetSource`、`DecisionView` |
| `conversions.hpp` | IDL ↔ 纯核转换（库 `peach2_manipulation_conversions`；含 FailureCode / ToolState / Outcome 数值 static_assert） |

节点私有（`src/`，不安装）：`ManipulationNode`（LifecycleNode）、`MoveItMotionBackend`、`RosDecisionClient`、`ModelCache`。

## 采摘周期（HarvestTarget）

`PREPARE_TOOL → TRANSIT_STAGING → APPROACH_PREGRASP → VERIFY_PREGRASP → [PREGRASP_ONLY：撤出后结束] → INSERT → CUT → CONFIRM →
RETREAT → TRANSIT_RELEASE → RELEASE → DONE`

- 受理：工具不符 → SKIPPED `TOOL_NOT_FEASIBLE`；FULL 且执行开但 grasp/tool 未开 → SKIPPED `SAFETY_GATE_CLOSED`；
  无模型 / 无许可 / 许可过期 → `MODEL_EXPIRED`；`approach_allowed=false` → 许可的 failure_code；插件不可行 → 插件码。
- plan-only（执行关或 CheckReachability）：走同一规划链（滚转采样 + staging OMPL + 接近 LIN [+ 套入 LIN]），不执行、不碰刀，
  每个结果 `plan_only=true`。全链规划成功 = `SKIPPED / NONE / reason=planned`（`reached=NONE`，没有任何东西动过；
  主审决定不用 SUCCEEDED，未读 `plan_only` 的消费者也不会误计为已采摘）；规划失败 = `SKIPPED` + 非 NONE 规划码
  （后端若返回失败但码为 0，按 `PLAN_FAILED` 报）。可达判定 = `plan_only && failure_code == NONE`。
- **邻袋碰撞（主审跨包决定）：** peach2_scene 挖空所有目标袋，本包在规划前把 `/peach/target_model/models` 里的袋写成
  `peach_bag_*` 圆柱（底→颈，平端，保证袋底下方的 pregrasp 开口在外）。周期内两个场景相位：
  APPROACH（受理后、任何规划前）= 全部袋含当前目标，覆盖 staging 赶路与 staging→pregrasp；CONTACT（INSERT 起，含 plan-only
  的套入规划）= 移除当前目标，一直保持到撤出与放果赶路。周期结束（成功 / 失败 / 取消 / 异常）节点恢复为“全部袋”；MoveTo
  规划前同样写全部袋；CheckReachability 每个目标走 APPROACH→CONTACT，全部结束后恢复。每次变更是一次
  `ApplyPlanningScene`（`is_diff=true`）原子 diff，只动 `peach_bag_` 前缀对象，不改 ACM。首次同步用 `GetPlanningScene`
  接管前一进程遗留的 `peach_bag_*`。场景写入失败 = `DEPENDENCY_UNAVAILABLE`：受理阶段 SKIPPED 不动；INSERT 前失败则按冠内失败撤出。
  deactivate / cleanup / shutdown / error 时删除本节点写入的全部 `peach_bag_*`。
- PREPARE_TOOL 过 TOOL 门并确认刀已张开（独立反馈）；PREGRASP_ONLY 不碰刀。
- TRANSIT_STAGING：插件滚转区间内采样 `staging.roll_samples` 个滚转，逐个规划 staging（OMPL）+ 接近（Pilz LIN），取关节路程最小者。
  `TargetModel.branch_direction_known` 为 true 时 bite / shear 的滚转区间按枝方向收窄（见 peach2_end_effector README），否则全周。
- VERIFY：残差超 3 mm / 5 mm / 2° 做至多 `pregrasp.max_corrections` 次修正 LIN，仍超 → `PREGRASP_RESIDUAL`。
- **INSERT 前、CUT 前各查一次 GetDecision**（`min_model_revision` = 上次 revision；新 revision 位移 > `model_shift_tol_m` → `MODEL_STALE`）；
  `sleeve_allowed` / `cut_allowed` 为 false → 撤出、SKIPPED 对应码。
- 套入：Pilz LIN 到 `tcp_for_blade(neck)`（刃面落在袋颈），偏离 pregrasp 轴 > `insert_lateral_tol_m` 拒绝；需要力传感而无、
  且插件不允许 LIN 回退 → `TOOL_NOT_FEASIBLE`。
- `CUT_NOT_CONFIRMED` → abort_safe 开刀 → 倒放套入段退回 pregrasp → 重套重剪（`cut_retry_max` 次）；其它刀具失败 → recovery。
- RETREAT：已执行的冠内段全部记录，`reverse_path` 倒放 + 逐点碰撞校验；校验失败回退为沿轴 LIN 回 staging。
  撤退 / 开刀 / 放果只要求 `Active ∧ robotReady ∧ ¬e_stop`（+ 执行链或本周期自己关过刀），**不**依赖感知许可。
- 每段执行前：过门（新轨迹检查 motion_possible）、起点偏差 > `start_tolerance_rad`（0.02 rad）重规划一次；执行中每 20 ms 查门与取消，
  关门 / 取消 / 超时即停。
- 冠内失败且撤不出 → `RETREAT_FAILED` + `recovery_required`；关门 / 机器人未就绪时不再尝试运动（只开刀）。

## 图接口（固定名，不是参数）

| 方向 | 名字 | 类型 | QoS |
|------|------|------|-----|
| action server | `/peach/manipulation/harvest_target` | `peach2_interfaces/action/HarvestTarget` | 默认 |
| action server | `/peach/manipulation/move_to` | `peach2_interfaces/action/MoveTo` | 默认 |
| service | `/peach/manipulation/check_reachability` | `peach2_interfaces/srv/CheckReachability` | 默认（`reachable[i]` = `plan_only && failure_code == NONE`；`reasons[i]`：成功为 `planned`，否则同 HarvestResult.reason；忙 `busy`、非 Active `node_not_active`） |
| service | `/peach/manipulation/acknowledge_recovery` | `std_srvs/srv/Trigger` | 默认；**幂等**：无待确认（未锁 recovery 且刀非 FAULT）直接 success `nothing_to_acknowledge`、无副作用；有待确认时忙 / 机器人未就绪返回 false |
| client | `/peach/target_model/get_decision` | `peach2_interfaces/srv/GetDecision` | 默认 |
| client | `/apply_planning_scene`、`/get_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene`、`GetPlanningScene` | 默认（等待 `timeouts.scene_s`） |
| pub | `/peach/manipulation/recovery_required` | `std_msgs/Bool` | reliable, transient_local, 1；activate 时发当前值，之后每次变化（锁定 / ACK 解除）即发 |
| sub | `/peach/enables` | `Enables` | reliable, transient_local, 1 |
| sub | `/peach/target_model/models` | `TargetModelArray` | reliable, transient_local, 1 |
| sub | `/aubo_io_controller/robot_status` | `aubo_msgs/RobotStatus` | best_effort, 5（年龄按接收时刻） |
| sub | `/joint_states` | `sensor_msgs/JointState` | SensorDataQoS（新轨迹前要求 < 0.5 s） |
| pub | `/peach/end_effector/tool_state` | `ToolState` | reliable, transient_local, 1（`tool_state_period_s`） |
| pub | `/diagnostics` | `DiagnosticArray` | gate / tool / cycle / inputs / scene |
| IO（经 `GatedIoBackend`） | `/aubo_io_controller/set_io`、`/aubo_io_controller/io_states` | aubo_msgs | `io_backend:=aubo` |
| MoveIt（伴随节点） | `move_action`（规划）、`execute_trajectory`、`check_state_validity` | moveit_msgs | — |
| bond | `/bond` | bond | heartbeat 0.1 s，超时 `bond_heartbeat_timeout_s` |

回调组：状态订阅一组、GetDecision/SetIO/PlanningScene 客户端一组、服务一组、action 一组、定时器一组（均 MutuallyExclusive）；
动作执行在独立 worker 线程；`MultiThreadedExecutor(6)` 同时自旋 MoveIt 伴随节点。单忙仲裁：harvest / move_to / check_reachability 互斥。

## 参数

声明与校验：`params/peach2_manipulation_parameters.yaml`（generate_parameter_library）；部署值：`config/manipulation.yaml`。
要点：`tool_id`、`plugin_tool_ids`/`plugin_classes`（tool_id → pluginlib 类名）、`io_backend`（aubo|mock）、`io.cmd_pin`/`io.feedback_pin`
（必须不同）、`require_robot_status`（仅 mock 为 false）、`robot_status_max_age_s` 0.3、`enables_timeout_s` 3.0、`moveit.*`、`speed.*`
（transit 0.1、接近 0.05 m/s、撤出 0.03 m/s）、`timeouts.*`、`staging.distance_m` 0.20（0.15–0.25）、`pregrasp.*`、`start_tolerance_rad` 0.02、
`release_named_target` harvest_stow、`cut_retry_max` 1、`bond_heartbeat_timeout_s` 4.0、`bag_margin_m` 0.02（0–0.10）、
`timeouts.scene_s` 2.0。执行 / 抓取 / 刀具许可**不是参数**，只来自 `/peach/enables`。

## 运行（本包不自行启动任何东西）

```bash
ros2 launch peach2_manipulation manipulation.launch.py hardware_mode:=mock tool_profile:=adaptive_shear_v1
# autostart 默认 false：由整栈 lifecycle manager configure/activate；需要 move_group 已在运行。
```

`hardware_mode:=mock` ⇒ `io_backend=mock`、`require_robot_status=false`；`real` ⇒ `aubo`、`true`。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
B="--base-paths src/peach2 --packages-up-to peach2_manipulation --packages-skip peach2_interfaces peach2_core \
   --build-base build/v2/peach2_manipulation --install-base build/v2/peach2_manipulation_install"
colcon build $B --cmake-args -DCMAKE_BUILD_TYPE=Release && colcon test $B
colcon test-result --test-result-base build/v2/peach2_manipulation --verbose
```

gtest（零 ROS 运行时）：`test_bag_obstacles`（胶囊几何、排除当前目标、diff 增/改/删、APPROACH→CONTACT→恢复序列、未提交重试、
遗留接管）、`test_command_gate`、`test_trajectory_reverse`、`test_staging_geometry`、`test_harvest_cycle`（假 MotionBackend /
DecisionClient + 真 AdaptiveShearV1 on MockIoBackend，覆盖 plan-only 标志、场景相位与每段规划的对应、场景失败、受理、全流程、残差、
两次重查、刀具故障、关门/取消、撤退回退、起点重规划）、`test_move_to`、`test_conversions`（反馈三态、plan_only、枝方向）。lint 跳过 cpplint / copyright（spec）。节点与 MoveIt 后端的集成测试（launch_testing + mock_components）是 M1 缺口。

## 接口需求（不改 IDL，记录给接口 owner）

变更 01 已解决：目标碰撞分工（scene 挖空目标袋、本包写 `peach_bag_*`）、枝方向、`CheckReachability.reasons`、
`HarvestResult.plan_only`、`ToolState.feedback` 三态与 `suspected_loopback`、`recovery_required` 话题。仍待定：

1. 相机侧障碍方向（shear 的 `avoid_direction`）无来源。
2. 刀具电流无来源（`actuator_current_a` 发布 NaN）。
3. `MoveTo.Result` 无 plan-only 标志：执行关时仍以 `success=false + SAFETY_GATE_CLOSED + message=plan_only_ok` 表达。
4. 无有效底 / 颈的模型不进 `ModelCache`，因此也不会成为 `peach_bag_*`；若 scene 也挖空了它，该袋在规划中不可见。
   需要约定：scene 只挖空有有效模型的袋，或模型给出保守包络。
5. `TargetModel.header.frame_id` 按 IDL 约定为 base_link，本包直接以 `moveit.base_frame` 写碰撞对象，不做 TF 变换。

## 已知限制与 TODO

- TODO(M0)：刃面约定、L_insert 含义、刀反馈独立性（P0-3）台架核对；`execute_trajectory` 取消后驱动是否确实 `RobotMoveStop` 真机确认；
  `motion_possible` 在执行中为 0 的语义按 spec，真机复核。
- TODO(M0)：Pilz `cartesian_limits.max_trans_vel`（0.25）若在 moveit_config 改动，同步 `moveit.cartesian_max_trans_vel_mps`。
- TODO(M1)：launch_testing（isolated domain + mock_components）：lifecycle Active、bond、QoS、PREGRASP 链、取消路径、关门即停；
  `peach_bag_*` 经 move_group 的实际增删（本轮只有纯核与周期钩子的 gtest）、`recovery_required` 锁定 / ACK 边沿、ACK 幂等。
- TODO(M4)：腕部力传感 → adaptive 导纳套入与接触中止（`has_force_sensing()` 现为 false，走 LIN 回退）；bite 喉部接触判据。
- 急停 / 保护停止沿（`e_stopped` / `in_error` 0→1）：取消在途 goal、刀标 UNKNOWN、锁 recovery；须人工复位示教器后调
  `acknowledge_recovery`，下一周期 PREPARE_TOOL 重新确认张开。本包从不自动复位、不 resume 原轨迹。
- 网络层（只有本节点可调 set_io、只有 task 可调 HarvestTarget）需 SROS2 权限，未在本包实现。
