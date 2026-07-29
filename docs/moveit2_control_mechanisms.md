# MoveIt2 控制机制 × AUBO SDK 实测限制：平滑 / 实时运动控制综述

> **历史参考（2026-07-29 起）**：本文写于 2026-07-24 的旧流式 JTC 架构
> （`aubo_driver::AuboRos2System`、路径门走廊、SEND_SUMMARY、旧 launch 参数
> `use_mock_hardware`/`server_host` 等），所述实现已整体归档 `src_legacy/`。
> 文中 MoveIt2/ros2_control 官方机制综述仍有参考价值；涉及本项目实现的章节
> 以 passthrough 架构现行文档（`docs/usage.md`、`docs/passthrough_migration.md`）
> 为准。

- 整理日期：2026-07-24
- 版本锚点：ROS 2 Jazzy；MoveIt **2.12.4**；ros2_controllers **4.40.1**；ros2_control（hardware_interface）**4.45.2**；realtime_tools **3.11.0**（均为本机 dpkg 实测）
- 硬件侧：AUBO E5，控制柜固件 `V4.5.111.456d8e2-Alpha`，SDK `libaubo_sdk.so.2.5.3`
- 依据：MoveIt2 / ros2_control 官方文档与对应版本源码（附录 9.1/9.2）；本机 SDK 实测（`docs/aubo_sdk_2_5_3_ros2_control_test_report.md`）；本地教程笔记（附录 9.4，经验数字标注为教学参考值）

## 1. 目的与结论速览

目的：

1. 梳理 MoveIt2 控制机械臂的**全部**机制（规划、执行、流式、直通）及其数据通路；
2. 与本机 SDK 实测限制逐条对照，判断每个机制"采用 / 可选 / 受限 / 不可行"；
3. 面向"平滑 + 实时"目标给出分场景推荐架构，并给出以 **home ↔ camera_pose 两个关节位姿**为载体的测试方案。

结论速览：

| 需求 | 推荐机制 | 状态 |
| --- | --- | --- |
| 两点间平滑点位运动（当前需求） | 直接 JTC action（现行 `named_pose_controller.py`）；需要完整碰撞规划时 move_group(OMPL+TOTG)；要确定性轨迹用 Pilz PTP | M1 已实机验证（v7） |
| 平滑性进一步提升 | TOTG 已够用；Ruckig 平滑（jerk 限制）+ TOTG 加密采样 + Pilz 确定性管线 | **已实施并 mock 验证**（2026-07-24，见 §7.2） |
| 实时流式 / 点动 | MoveIt Servo（100 Hz 流式输出） | 未安装；受 10 Hz 反馈限制，见 §3.9 |
| 整段轨迹下发、机器人侧插值 | UR 式 passthrough controller | ros2_controllers 无此件，长期方向 §3.12 |
| 200 Hz 控制回路 / 状态推送反馈 | — | **不可行**：超 SDK 写产能 ~125 pts/s；推送与 TCP2CAN 并存会断连（§5） |

## 2. 控制通路总览

### 2.1 命令通路（自上而下）

```text
任务层    RViz(MotionPlanning) │ named_pose_controller.py │ 用户节点 │ 点动输入设备
             │ moveit_msgs/action/MoveGroup        │ control_msgs/action/FollowJointTrajectory
             │ moveit_msgs/action/ExecuteTrajectory │ trajectory_msgs/JointTrajectory (topic)
             ▼                                     │ control_msgs/JointJog · TwistStamped (Servo)
规划层    move_group ─ 规划器(OMPL/Pilz/STOMP/CHOMP) ─ 时间参数化(TOTG [+Ruckig])
             │ moveit_msgs/RobotTrajectory
执行层    TrajectoryExecutionManager → MoveItControllerManager 插件
             │ （SimpleControllerManager=静态表；Ros2ControlManager=动态发现/切换）
             ▼
控制器层  joint_trajectory_controller @100 Hz（样条插值→每周期写 position 命令接口）
          forward_command_controller / JointGroupPositionController（Float64MultiArray 直通）
             │ 设定点频率 = controller_manager update_rate（100 Hz → 100 pts/s）
             ▼
硬件插件  AuboRos2System：write() 无锁入队(SPSC,1024) → sender 流控
          （批量 ≤8；RIB 水位带 [60,120]；失败批次 pending 重发不丢点）
             │ TCP2CAN robotServiceSetRobotPosData2Canbus（批 ≤16，产能 ~125 pts/s）
             ▼
控制柜    RIB 1200 槽（≈200 个六轴点）→ ~4.7 ms/点 CAN 插补 → 关节伺服
```

### 2.2 状态反馈通路（实测频率，勿与发布频率混淆）

```text
控制柜 → SDK 10 Hz 轮询（status 线程）→ 原子缓存 → read() @100 Hz
       → joint_state_broadcaster → /joint_states @100 Hz（缓存重发，真实新遥测 ~10 Hz）
```

- 33 Hz 实时推送**已关闭**：与 TCP2CAN 并存时服务器 12–17 s 主动断连（2/2 复现，§5-C5）。
- 反馈滞后是设计 JTC 容差的直接依据：流式期间反馈滞后 0.5–1 s 级（§5-C9）。

### 2.3 三个贯穿全文的关键换算

1. **设定点频率 = controller_manager update_rate**。JTC 每个 update 周期从样条取下一个设定点写入命令接口（§4.1）。100 Hz → 100 pts/s < 125 pts/s 产能；200 Hz → 200 pts/s 必积压（实测失败，§5-C1）。
2. **JTC 插值阶数由路点字段决定**：仅位置 → 线性（路点处速度不连续，官方明确 discouraged）；pos+vel → 三次样条；pos+vel+acc → 五次样条（§4.2）。
3. **RIB 槽 ≈ 关节分量**：容量 1200 槽 ≈ 200 个六轴点，消费 ~4.7 ms/点；水位带 [60,120] 槽 ≈ 10–20 点 ≈ 取消残余 47–94 ms（`aubo_ros2_system.hpp` 注释口径，§5-C7）。

## 3. MoveIt2 控制机制逐项

每项按：通路 / 接口 / 平滑性 / 实时性 / 对本机的结论。引用集中列于附录。

### 3.1 move_group：MoveGroup action（规划+执行一体）

- **通路**：客户端 → `move_action` → PlanExecution（规划管线）→ TEM → controller manager 插件 → JTC。
- **接口**：action `move_action`（`moveit_msgs/action/MoveGroup`）。Goal = `MotionPlanRequest`（目标约束、`max_velocity_scaling_factor`/`max_acceleration_scaling_factor`、`pipeline_id`/`planner_id`）+ `PlanningOptions`（`plan_only`、`replan`、`replan_attempts`、`replan_delay`、`look_around`）。**新 goal 会抢占正在执行的 goal**。
- **平滑性**：取决于规划器（§3.5）与时间参数化（§3.6），与执行层无关。
- **实时性**：非实时。OMPL 采样规划耗时 0.1 s~数秒且每次不同；执行本身是"整段轨迹一次性下发"的 goal 语义。
- **对本机**：已配置可用——`moveit.launch.py` 挂 OMPL 管线 + `MoveItSimpleControllerManager`（静态表指向 `aubo_e5_arm_controller/follow_joint_trajectory`）+ TEM 参数 `allowed_execution_duration_scaling: 1.2` / `allowed_goal_duration_margin: 0.5` / `allowed_start_tolerance: 0.01`。注意硬件插件路径门只放行 home↔camera 走廊，**off-path 目标会在 write() 被闩锁**（README 已警告不要在 RViz 执行任意目标）。

### 3.2 ExecuteTrajectory action 与规划服务（规划/执行分离）

- **接口**：
  - 规划：`plan_kinematic_path`（`moveit_msgs/srv/GetMotionPlan`，只规划不执行）；
  - 执行：`execute_trajectory`（`moveit_msgs/action/ExecuteTrajectory`，可指定 `controller_names`）；
  - 其余常用服务：`compute_cartesian_path`（GetCartesianPath，内部同样走 TOTG）、`check_state_validity`（GetStateValidity）、`compute_ik`/`compute_fk`、`get_planning_scene`/`apply_planning_scene`、`query_planner_interface`、`clear_octomap`。
  - 注意：`ExecuteKnownTrajectory` 服务是 **ROS 1 遗物，ROS 2 不存在**（moveit_msgs ros2 分支无此定义）。
- **平滑/实时**：同 3.1；分离模式多了"先审后行"的人工/程序审核点。
- **对本机**：`named_pose_controller.py` 已经在用 `check_state_validity` 对起点→目标做 51 点采样预审（`scripts/named_pose_controller.py:102`），但未用 `plan_kinematic_path`（轨迹是手工 3 点，见 §3.7）。

### 3.3 MoveGroupInterface（C++/Python 客户端封装）

- **本质**：3.1/3.2 的客户端库，不引入新机制。`setNamedTarget("home")` 可直接命中 SRDF `group_state`；`move()` = plan+execute 阻塞调用；`plan()` / `execute()` 可分开。
- **对本机**：SRDF 已定义 `home` 与 `camera_pose` 两个 named states（`config/aubo_e5.srdf:16-31`），MGI/RViz 下拉框一行即可命中，无需手填约束。

### 3.4 MoveItCpp / PlanningComponent（进程内 API）

- **通路**：与 move_group 相同的规划管线和**同一个 TEM**，但在调用方进程内加载，省掉 action/service 的 IPC 往返；官方定位为"需要更多实时控制或工业应用的高级用户"。
- **差异点**：`plan()` 支持 `MultiPipelinePlanRequestParameters`（多管线并行取最优）；2.12 中 `execute()` 的 `blocking` 参数已移除。
- **对本机**：链路不变（仍 TEM→JTC action），省的是毫秒级 IPC；需要自研 C++ 节点时的首选，否则收益有限。

### 3.5 规划器对比（OMPL / Pilz / STOMP / CHOMP）

| 规划器 | 插件 | 输出特性 | 确定性 | 本机状态 |
| --- | --- | --- | --- | --- |
| OMPL（默认） | `ompl_interface/OMPLPlanner` | 几何路径，需时间参数化；RRTConnect 默认 | 随机采样，每次路径不同 | **已配置**（`ompl_planning.yaml`，`longest_valid_segment_fraction: 0.005`） |
| Pilz 工业规划器 | `pilz_industrial_motion_planner/CommandPlanner` | PTP/LIN/CIRC，梯形速度曲线，**自带时间参数化**，关节全同步 | **完全确定**：同一请求必出同一轨迹、时长可预测 | 已安装 2.12.4，**已配置**（双管线之一，PTP 默认） |
| STOMP | `stomp_moveit/StompPlanner` | 优化平滑轨迹 | 随机优化 | 未安装（apt 候选 2.12.4） |
| CHOMP | `chomp_interface/CHOMPPlanner` | 优化轨迹，近障碍处可能抖动 | 确定性优化 | 未安装（apt 候选 2.12.4） |

- Pilz 要点：`planner_id ∈ {PTP, LIN, CIRC}`；LIN/CIRC 要求起始速度为零；极限取 `joint_limits.yaml`（另有 `max_deceleration`）+ `robot_description_planning` 下的 `cartesian_limits`（`max_trans_vel/acc/dec`、`max_rot_vel`）；支持 Sequence（`moveit_msgs/srv/GetMotionSequence` 服务 `/plan_sequence_path`、action `/sequence_move_group`）与 `blend_radius` 混合。Pilz **不做避障**——碰撞即规划失败，不会绕。
- 对本机两点直达场景：Pilz PTP 是最对口的选择（确定、时长可预测、梯形速度天然平滑）；OMPL 已够用但路径有随机性；STOMP/CHOMP 对两点直线没有意义。

### 3.6 时间参数化：TOTG 与 Ruckig 平滑

- **2.12 只剩 TOTG**（`AddTimeOptimalParameterization`，Kunz & Stilman 迭代实现，`moveit_core/trajectory_processing/time_optimal_trajectory_generation.cpp`）；IPTP（`AddTimeParameterization`）与 Iterative Spline 已删除。
- TOTG 特性：速度+加速度限制内近似时间最优；**不限制 jerk**（段边界加速度可能跳变）；假设起止静止；会在 `path_tolerance`（默认 0.1）内重采样路径。参数：`default_response_adapter_parameters.totg.{path_tolerance, resample_dt(0.1), min_angle_change(0.001)}`；请求级 `max_velocity/acceleration_scaling_factor` 在此基础上再缩放。
- **Ruckig 平滑**（`default_planning_response_adapters/AddRuckigTrajectorySmoothing`）：jerk 限制的后处理，需在 `joint_limits.yaml` 为每关节补 `max_jerk`（缺省用内部默认 1000 rad/s³ 并告警）。**必须排在 `AddTimeOptimalParameterization` 之后**——response_adapters 按列表顺序执行（`planning_pipeline.cpp` 顺序循环，默认 `ompl_planning.yaml` 注释明示"列表顺序即处理顺序"），而 Ruckig 源码要求输入已是时间参数化轨迹（"requires a fully configured trajectory as initial guess"）。
- 教学参考（`M10_时间参数化.md`，经验数字为教学参考值）：jerk 限制对谐波减速器寿命影响显著（τ̇ 从 40000→2000 Nm/s 量级），代价是执行时间短距 +33% / 长距 +6%；"运动看起来对但有嗡嗡声/抖动，先查时间参数化而非规划器"。
- **对本机**：当前 TOTG → JTC 五次样条 @100 Hz → SDK 4.7 ms/点插补，链路已有三层平滑，实测运行平稳（v7）。Ruckig 是可选提升项（§7.2），非必需。

### 3.7 绕过 MoveIt 规划：直接 JTC FollowJointTrajectory action（现行方案）

- **通路**：`named_pose_controller.py` → `check_state_validity` 预审 → 手工构造 3 点轨迹 → JTC action。
- **轨迹设计**（`scripts/named_pose_controller.py:126-146`）：起点 @0.1 s → **保持段** @6 s（原地不动，吸收 SDK 冷连接首个查询/写入的数秒阻塞，实测 ~3.5 s）→ 目标 @12 s，全程零速起止。
- **双重安全门**：客户端侧 `_path_progress()` 校验当前位姿在走廊内 + 碰撞采样；硬件侧 `commandOnAllowedPath()` 逐周期把关（§5-C8）。
- **平滑性**：3 个路点经 JTC 样条（pos+vel+acc → 五次）展开为 100 Hz 设定点，实测平滑。
- **实时性**：无规划延迟；goal 发出即被 JTC 执行（机械臂按保持段设计在 ~6 s 后才开始位移）。
- **对本机**：**现行基线，已实机验证**（v7：home→camera SEND_SUMMARY ok=520 fail=1，写延迟 avg 4.2 ms / max 9.9 ms，RIB 峰值 30，到位偏差 ~1e-5 rad）。代价：无全局规划能力，目标被硬编码在两个 named pose。

### 3.8 JTC topic 接口（fire-and-forget 流式）

- **接口**：`~/joint_trajectory`（`trajectory_msgs/msg/JointTrajectory`）——4.40.1 源码确认该订阅仍存在（`joint_trajectory_controller.cpp:1085-1087`，`topic_callback` 在同文件 :1475）。
- **语义**：fire-and-forget——不做容差监控、不发任何通知；违反容差时中止轨迹并保持当前位置；**空轨迹消息会覆盖当前 action goal（但不 abort 它）**。官方明确：需要执行监控就用 action 通道。
- **对本机**：这是 MoveIt Servo 默认输出的落点（§3.9）。单独裸用没有监控与结果反馈，不推荐作为独立机制；作为流式源（Servo/自研节点）的落点是合理用法。

### 3.9 MoveIt Servo（实时点动/流式，未安装）

- **形态**：`moveit_servo` 包，`servo_node` 组件节点 + `moveit_servo::Servo` C++ 类（零 IPC，`getNextJointState()` 直接取下一个关节状态）。apt 有 `ros-jazzy-moveit-servo` 2.12.4 候选，**本机未安装**。
- **接口**：输入 `~/delta_twist_cmds`（TwistStamped）、`~/delta_joint_cmds`（JointJog）、`~/pose_target_cmds`（PoseStamped）；服务 `~/switch_command_type`、`~/pause_servo`；状态 `~/status`（ServoStatus）。
- **实时性**：主循环 `publish_period` 默认 0.01 s（100 Hz，可至数百 Hz），线程尝试 SCHED_FIFO 优先级 40（刻意低于 ros2_control 的 50）；`incoming_command_timeout: 0.1` 超时自动停。相对"规划 100 ms–2 s"的 move_group，Servo 是 <10 ms 级响应（`M14_MoveIt2_MTC工业集成.md` M14.6，教学参考值）。
- **平滑性**：2.12 改为平滑插件制——`online_signal_smoothing::ButterworthFilterPlugin`（默认，系数 1.5）/ `AccelerationLimitedPlugin` / `RuckigFilterPlugin`（jerk+加速度限制，工业臂推荐）。
- **安全**：`check_collisions` 默认开，`collision_check_rate` 仅 10 Hz——碰撞检查是"尽力而为"的安全网，远低于命令频率；另有奇异性阈值与关节限位边距。
- **输出**：`command_out_type = trajectory_msgs/JointTrajectory`（默认，≥3 点滚动窗、近未来时间戳）→ JTC topic；或 `std_msgs/Float64MultiArray` → forward 控制器（UR 真机常见 `forward_position_controller`）。
- **对本机**：输出 100 pts/s 在 SDK 产能内；但**状态反馈只有 ~10 Hz**——Servo 闭环依赖关节状态，10 Hz 反馈决定了闭环带宽上限，适合低速点动/遥操作，不适合"精确到点"（那是 JTC/MoveGroup 的设计目标）。碰撞检查 10 Hz 与本机反馈同量级，不能替代路径门。列为后续路线（§7.3）。

### 3.10 forward_command_controller / JointGroupPositionController（直通）

- **接口**：`~/commands`（`std_msgs/msg/Float64MultiArray`），每个 CM 周期把"最近一次收到的数组"原样写进命令接口。
- **特性**：**纯直通——无插值、无超时、无看门狗、无幅值校验**；消息断流时硬件持续收到最后一次的值。平滑度完全由上游发布速率和内容决定。
- **对本机**：三重不利——无看门狗（上游崩了机械臂保持最后指令）、100/125 pts/s 产能裕量小、反馈 10 Hz。当前**不推荐**；仅在 Servo 生态中作为备选输出通道。

### 3.11 Hybrid Planning（实验性）

- **形态**：`moveit_hybrid_planning`（Jazzy 已发布二进制，官方仍标实验性）：HybridPlanningManager（action `run_hybrid_planning`）+ 全局规划组件（MoveItCpp 管线）+ 局部规划组件（`local_planning_frequency` 频率运行，局部解发布为 JointTrajectory 或 Float64MultiArray）。
- **定位**：在线重规划 + 流式执行的官方探索，本质是"move_group + Servo 式局部环"的组合。
- **对本机**：依赖 Servo 同款流式通路，反馈限制相同；实验性组件不值得在本机安全门槛下引入。**观察，不采用**。

### 3.12 UR 式 passthrough trajectory controller（整段下发，长期方向）

- **事实**：`passthrough_trajectory_controller` **不在 ros2_controllers 任何分支**（4.40.1 已核实）；它是 UR 驱动包 `ur_controllers` 的实现（PR #944，2024-11 合入）。
- **机制**：控制器通过 action 接收整条轨迹，逐点经 GPIO 风格命令接口（`<tf_prefix>trajectory_passthrough/setpoint_positions_{0..5}`、`setpoint_velocities_*`、`setpoint_accelerations_*`、`transfer_state`、`time_from_start`、`abort`、`trajectory_size`）交给硬件组件；**插值由机器人控制器完成**，PC 侧实时性要求大降（UR 官方称无 RT 内核也能稳定运行）。
- **对本机**：AUBO 的 TCP2CAN+RIB 模型与 UR 的 stream 模型**同构**——本机插件的 sender 流控（批量灌点、RIB 水位门控、4.7 ms/点插补）本质上已经是一个"应用层 passthrough"。若将来要把这条链路标准化（Action 语义、abort 回传、speed scaling），可仿 UR 实现自定义控制器 + 硬件 GPIO 接口；那时 PC 侧 100 Hz 硬循环的压力也会显著下降。

## 4. ros2_control 执行侧关键行为（JTC 4.40.1 精读）

### 4.1 设定点生成：频率 = update_rate

- JTC 每个 update 周期执行 `traj_time_ += period`（可乘 speed scaling），并**采样两次**：当前时刻 → `state_desired_`（用于反馈/容差判定），`traj_time_ + update_period` → `command_next_` 写入命令接口（`joint_trajectory_controller.cpp:262-410`）。
- 结论：硬件收到的设定点频率恒等于 CM `update_rate`（本机 100 Hz → 100 pts/s），与轨迹路点密度无关——样条负责在稀疏路点间插值。
- 实现是实时安全的：内存预分配；topic 消息经 `RealtimeBuffer` 进入 RT 线程；`~/controller_state` 经 `RealtimePublisher` 发布。

### 4.2 插值阶数（平滑性的执行侧来源）

| 路点字段 | 插值 | 平滑性 |
| --- | --- | --- |
| 仅 position | 线性 | 位置连续、路点处**速度不连续**（官方明确 discouraged） |
| position+velocity | 三次样条 | 速度连续 |
| position+velocity+acceleration | 五次样条 | 加速度连续（TOTG/手工轨迹均输出此形态） |

- `interpolation_method: none`：不做任何插值，直接取下一个给定路点——上游必须自己保证密度与平滑（本机不用）。
- 本机现行链路全部带 vel/acc → 五次样条，执行侧平滑性已到位。

### 4.3 hold / 取消 / 抢占 / 结束语义（对硬件插件的硬性要求）

- **成功后**：无限重复轨迹末点（默认 `allow_nonzero_velocity_at_trajectory_end: false`，末点非零速的 goal 直接拒绝）。
- **取消（cancel）**：默认 `set_hold_position()`——保持**当前测量位置**、零速度；若配置 `constraints.decelerate_on_cancel: true` 且每关节 `max_deceleration_on_cancel > 0`（需 velocity 状态接口），则按匀减速平滑停止。
- **被新 goal 抢占**：旧 goal 以 `INVALID_GOAL` 取消，新轨迹从当前状态重新拼接样条，无跳变（"遗忘旧轨迹"）。
- **容差违反**：abort + hold（或减速停）。
- **关键推论**：hold 期间 JTC 每周期持续输出**恒定位置指令**。硬件 `write()` 必须幂等——本机已实现"设定点变化 >1e-6 才入队"（`aubo_ros2_system.cpp:447-456`），hold 期不向 RIB 灌点，避免占满缓冲。
- `cmd_timeout`（默认 0=关闭）：从**轨迹末端时刻**起算超时，必须 > `constraints.goal_time` 才生效。本机未配置。

### 4.4 本机 controllers.yaml 逐项核对（4.40.1 parameters.yaml 对照）

| 配置项 | 4.40.1 状态 | 说明 |
| --- | --- | --- |
| `joints` / `command_interfaces: [position]` / `state_interfaces: [position, velocity]` | 有效 | position 命令接口下 **PID 增益不生效**（硬件内部闭环），无需配 `gains` |
| `interpolation_method: splines` | 有效 | 默认即 splines |
| `open_loop_control: false` | **已废弃** | 4.40.1 标记 deprecated（替代品：`interpolate_from_desired_state` + 零增益）；false 是默认值，配置无害，建议后续清理 |
| `state_publish_rate: 50.0` | **无此参数（遗留）** | 4.40.1 的 `~/controller_state` **每个 update 周期都发布**（100 Hz），该行配置不被读取 |
| `action_monitor_rate: 20.0` | 有效 | 默认值 |
| `constraints.goal_time: 8.0` | 有效 | 2026-07-24 下午起 1.5→8.0：吸收 SDK 写停滞后的延迟完成（§5-C3/C9） |
| `constraints.stopped_velocity_tolerance: 0.03` | 有效 | 默认 0.01 |
| `constraints.<joint>.trajectory: 0.75` / `.goal: 0.01` | 有效 | 2026-07-24 下午起 0.15→0.75：吸收写停滞造成的流式滞后；真实脱轨防护由硬件路径门（±0.02 rad）承担（见测试报告当日下午章节） |

### 4.5 controller_manager 实时调度（本机现状）

- `update_rate: 100`、`thread_priority: 50`、`lock_memory: true` 已配置（`src/aubo_e5_bringup/config/controllers.yaml`）；宿主机 realtime 组 + rtprio/memlock 已授权，lowlatency 内核已装（`docs/realtime_setup.md`）；实测 `ros2_control_node` 以 SCHED_FIFO 50 运行（`docs/real_hardware_integration_notes.md`）。
- 权限缺失时节点只告警并回退普通调度，不会启动失败（`ros2_control_node.cpp`）。
- 控制器 update 返回 ERROR → CM 停用该控制器（可配 `fallback_controllers`，本机未配）——本机插件的故障闩锁（traj_fault_/safety_fault_/command_fault_/queue_overflow_ → read() 返回 ERROR）正是走这条标准路径让 JTC 干净 abort。
- 4.45.2 上游有 `cpu_affinity` 参数；本机二进制未验证（realtime_setup.md 记录 strings 未见，未深查）。

## 5. 本机 SDK 实测约束清单（数字摘自测试报告 2026-07-23/24）

| # | 约束 | 数值 / 事实 | 对机制选型的影响 |
| --- | --- | --- | --- |
| C1 | 写路径产能 | **~125 pts/s**（瓶颈是写调用延迟 avg 38–60 ms，与批量 8/16 无关） | CM 上限 100 Hz；200 Hz 必积压（实测失败） |
| C2 | TCP2CAN 批次 | ≤16 可靠；**32 被拒**（1518/1639 失败） | sender 批量 ≤8 正确 |
| C3 | SDK 同步阻塞 | 任意调用偶发 ~5 s（0.2–0.5%） | 保持段、容差放宽、故障闩锁的依据 |
| C4 | 状态轮询 | 10 Hz 稳定；5 ms 目标 → avg 15.5 ms（≈64 Hz）且失败率升 | 反馈带宽 ~10 Hz；闭环类机制受限 |
| C5 | 实时推送 | ~33 Hz 可用，但**与 TCP2CAN 并存 12–17 s 断连**（2/2 复现） | 推送保持关闭，走轮询 |
| C6 | 服务器全局串行 | 高频诊断/保温查询会把另一连接拖到 11–13 s 失新 | sender 诊断 25 ms 间隔、空闲不查 |
| C7 | RIB 缓冲 | 1200 槽 ≈ 200 点；消费 ~4.7 ms/点；取消**非瞬清** | 水位带 [60,120] → 取消残余 47–94 ms |
| C8 | 硬件路径门 | 走廊 ±0.02 rad、端点 ±0.03、逐周期步长 ≤0.0065 rad、方向反转 >0.002 闩锁 | 只有 home↔camera 目标合法 |
| C9 | 反馈滞后 | 流式期间 0.5–1 s 级 | JTC trajectory 容差与 goal_time 放宽的依据（当前 0.75 / 8.0 s） |
| C10 | 残余风险 | 运动中撞 >5 s 阻塞（单次运动概率约 10–20%）→ traj_fault 闩锁，需重启 bringup | 自动重连/再激活是后续加固项 |

## 6. 机制 × 约束匹配矩阵

判定口径：平滑性（轨迹连续性/抖动风险）；实时性（指令响应延迟）；≤125 pts/s（设定点/写请求是否超产能）；反馈要求（对状态新鲜度的依赖 vs C4 的 10 Hz）。

| 机制 | 平滑性 | 实时性 | ≤125 pts/s | 反馈要求 | 结论 |
| --- | --- | --- | --- | --- | --- |
| 直接 JTC action（现行 §3.7） | ✓ 已实测 | 无规划延迟 | ✓ 100 pts/s | 10 Hz 够用（容差已放宽） | **采用（基线）** |
| move_group OMPL+TOTG → JTC（§3.1） | ✓ TOTG+五次样条 | 规划 0.1–2 s，非实时 | ✓ | 同左 | **推荐**：需要碰撞规划/任意 named target 时 |
| ExecuteTrajectory 分离（§3.2） | 同上 | 同上 | ✓ | 同上 | 推荐：先审后行流程 |
| MoveGroupInterface（§3.3） | = 3.1 | = 3.1 | ✓ | = 3.1 | 客户端选型，随 3.1 |
| MoveItCpp（§3.4） | = 3.1 | 省 ms 级 IPC | ✓ | = 3.1 | 可选：自研 C++ 节点时 |
| Pilz PTP/LIN（§3.5） | ✓ 梯形速度、确定性 | 规划快（解析式） | ✓ | = 3.1 | **推荐（两点直达）**，需配管线（§7.2） |
| STOMP/CHOMP（§3.5） | 平滑优 | 秒级优化 | ✓ | = 3.1 | 不必要（两点直线无优化空间） |
| TOTG + Ruckig 平滑（§3.6） | ✓+ jerk 限制 | — | ✓ | — | 可选提升（§7.2） |
| JTC topic 裸用（§3.8） | 取决于上游 | 流式 | ✓ | 无监控 | 不推荐单独用；作流式落点合理 |
| MoveIt Servo（§3.9） | 滤波插件可达 ✓ | **✓ 100 Hz，<10 ms 级** | ✓ 100 pts/s | **10 Hz 闭环带宽受限**（C4） | **受限**：低速点动/遥操作可用；精确到点不用；未安装 |
| forward position 直通（§3.10） | 无插值 | 直通 | 裕量小 | 无看门狗（断流保持最后值） | 不推荐（当前）；Servo 备选输出 |
| Hybrid Planning（§3.11） | — | 局部环流式 | ✓ | 同 Servo | 观察（实验性） |
| UR 式 passthrough（§3.12） | 机器人侧插值最优 | PC 实时要求最低 | ✓ 整段下发 | 机器人侧闭环 | 长期方向；本机插件已是事实 passthrough |

## 7. 分场景推荐架构

### 7.1 点位运动（当前需求，已验证）

```text
named_pose_controller.py（check_state_validity 预审 + 3 点轨迹含保持段）
  → JTC action @100 Hz（五次样条，容差 trajectory 0.75 / goal 0.01 / goal_time 8.0）
  → AuboRos2System（路径门 + SPSC + sender 水位带 [60,120]）
  → TCP2CAN（批 ≤8）→ RIB → 4.7 ms/点插补
```

配置要点（均为实测得出的必需项，勿随意回退）：CM 100 Hz（C1）；轨迹起始保持段（当前配置：起点 @0.1 s → 保持 @6 s → 目标 @12 s，用于吸收 C3 冷启动 ~3.5 s 阻塞）；sender 诊断 25 ms 间隔、空闲不查（C6）；JTC 容差放宽（C9）。需要"任意目标 + 碰撞规划"时换成 move_group（§3.1），执行侧链路不变。

### 7.2 平滑性提升（已于 2026-07-24 实施并 mock 验证）

1. **Ruckig 平滑（已实施）**：`joint_limits.yaml` 每关节已补 `max_jerk`（臂 1500 / 腕 1250 rad/s³，按 j/a≈500 教学参考比例取保守值，基本不改变运动时长）；`moveit.launch.py` 的 `response_adapters` 中 `AddRuckigTrajectorySmoothing` 已加在 `AddTimeOptimalParameterization` **之后**。
2. **更密的时间参数化采样（已实施）**：`ompl.totg.resample_dt: 0.01`（注意参数名是 `<管线命名空间>.totg.*`——generate_parameter_library 剥掉 `default_response_adapter_parameters` 根键，写 `ompl.default_response_adapter_parameters.totg.*` 不会生效）。
3. **Pilz 管线（已实施）**：move_group 配置为双管线 `planning_pipelines: [ompl, pilz_industrial_motion_planner]`（**节点顶层参数**，不要嵌套进 `move_group` 键，否则 move_group 读不到列表会回退 legacy 单管线命名空间）；新增 `config/pilz_industrial_motion_planner_planning.yaml`（PTP 默认）；`joint_limits.yaml` 补 `cartesian_limits`；RViz/请求侧 `pipeline_id := pilz_industrial_motion_planner`、`planner_id := PTP`。
4. 手工轨迹（§3.7 路线）已带 vel/acc → 五次样条，无需改动。

实施验证记录（2026-07-24，mock 链路，`/plan_kinematic_path`，home→camera_pose）：OMPL 管线（TOTG+Ruckig）规划成功，输出 108 点 / 1.07 s（≈10 ms 间隔，证明 `ompl.totg.resample_dt: 0.01` 生效；改前默认 0.1 时为 12 点）；Pilz PTP 规划成功，14 点 / 1.24 s。注意发规划请求时 `max_velocity_scaling_factor`/`max_acceleration_scaling_factor` 必须显式非零——TOTG 会把 0 兜底成 1.0，但 Ruckig 与 Pilz 直接用 0 计算会失败（前者报 Ruckig error -100，后者报 "Velocity scaling not in range"）。

### 7.3 实时流式（后续路线）

- 安装 `ros-jazzy-moveit-servo`，`command_out_type: trajectory_msgs/JointTrajectory`、`command_out_topic: /aubo_e5_arm_controller/joint_trajectory`（走 JTC topic，保留样条插值与基本防护），`publish_period: 0.01`，平滑插件选 `RuckigFilterPlugin` 或默认 Butterworth。
- 预期边界：闭环带宽被 10 Hz 反馈限制（C4）——只做低速点动/遥操作；碰撞检查 10 Hz 不能替代路径门（C8 门在硬件侧仍然生效，点动范围天然被走廊限制）；到点任务仍走 7.1。
- 不建议先走 forward position 直通（§3.10 的三重不利）。

### 7.4 长期：passthrough 标准化

仿 UR `ur_controllers/PassthroughTrajectoryController` 自研控制器 + 硬件侧 `trajectory_passthrough/*` GPIO 接口：整条轨迹带时间戳一次下发，机器人侧（RIB+插补器）执行，PC 实时性要求大降，abort/speed scaling 有标准语义。本机插件 sender 流控已是事实 passthrough，此方向是"把应用层协议升级成 ros2_control 标准接口"。

## 8. 两关节位姿测试方案

> 本节为**方案文档**：本次任务未执行任何真机运动。执行前必须完成 §8.5 安全前提并由现场人员确认。

### 8.1 测试位姿（均在硬件路径门内，勿用其他点）

| 位姿 | shoulder | upperArm | foreArm | wrist1 | wrist2 | wrist3 |
| --- | --- | --- | --- | --- | --- | --- |
| home | 0.0 | -0.0334 | 1.236 | -0.3675 | 1.5701 | 0.0 |
| camera_pose | -0.2741 | 0.4964 | 1.7701 | -0.2979 | 1.5716 | -0.2750 |

三处定义一致：SRDF named states（`aubo_e5.srdf:16-31`）、硬件插件常量（`aubo_ros2_system.hpp:130-134`）、`named_pose_controller.py:22-26`。关节顺序以 `JOINTS` 列表为准。

### 8.2 测试矩阵

| 用例 | 机制 | 内容 | 状态 |
| --- | --- | --- | --- |
| M1 | 直接 JTC action（§3.7） | `/aubo/move_camera_pose`、`/aubo/move_home` 各一次往返 | **基线，v7 已验证**；2026-07-24 下午 home→camera 复测通过（camera→home 遇 SDK 降级中止，见测试报告） |
| M2 | move_group OMPL+TOTG(+Ruckig)（§3.1） | RViz MotionPlanning：Select Goal State 选 `camera_pose` → Plan & Execute；再选 `home` 往返 | 管线已配置；2026-07-24 下午 ExecuteTrajectory 实测中段→home 通过（0.0045 rad）；home→camera 遇 SDK 降级中止 |
| M3 | Pilz PTP（§3.5） | 同 M2，pipeline 切 `pilz_industrial_motion_planner` / planner `PTP` | 管线已配置（2026-07-24），mock 已验证，真机待执行 |

### 8.3 指标与日志来源

- 插件 sender 日志（ros2_control_node 输出）：`SEND_SUMMARY`（ok/fail/rib/peak/ema/batch/need）、`RIB_IDLE`/`RIB_WARN`/`RIB_HIGH`、`TIMING_SPIKE`/`TIMING_HIGH`、`SEND_FAIL`、`FREEZE_STOP`、`FAULT_STOP`、`FEEDBACK_STALE`。
- JTC：action result `error_code`（0=成功）；`/aubo_e5_arm_controller/controller_state`（100 Hz，desired vs actual 误差曲线）。
- 到位误差：运动结束后 `ros2 topic echo --once /joint_states` 与 §8.1 目标值比对。
- 耗时：service/goal 发出到 result 返回的墙钟时间（M1 含 6 s 保持段，总 ~12 s 为预期）。

### 8.4 判定标准

- action/服务成功返回，JTC `error_code = 0`；
- 到位误差 < 0.01 rad（v7 实测 ~1e-5 rad）；
- `SEND_SUMMARY` fail 占比 ≤ 基线（520:1 量级），RIB 峰值 < 300（远低于 1200 溢出）；
- 全程无 `FREEZE_STOP` / `FAULT_STOP` / traj_fault 闩锁；`FEEDBACK_STALE` 不出现 >5 s（C10 已知残余风险除外，出现则记录并重启 bringup）。

### 8.5 安全前提（执行前逐项确认）

1. 完成 `README.md` 与 `docs/real_hardware_integration_notes.md` 的首次真机测试清单：急停、限位、碰撞等级、工装/负载、工作空间清空、低速模式。
2. 启动形态固定为：`hardware_mode:=real robot_ip:=169.254.10.98`（已取消 RT 预检，普通内核直接启动；开机先做网卡 offload/governor 设置）。
3. RViz 只 Plan & Execute 两个 named target；**绝不执行拖动交互标记得到的任意目标**（路径门会闩锁拒绝，C8/C10）。
4. 现场人员把守急停；首次执行任一用例先单向往返一次，确认日志干净后再继续。

### 8.6 命令序列

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch aubo_e5_bringup bringup.launch.py \
  use_mock_hardware:=false enable_real_hardware:=true \
  allow_motion_commands:=true start_moveit:=true server_host:=169.254.10.98

# M1（基线回归）
ros2 service call /aubo/move_camera_pose std_srvs/srv/Trigger
ros2 service call /aubo/move_home std_srvs/srv/Trigger
ros2 topic echo --once /joint_states   # 与 §8.1 目标比对

# M2：RViz MotionPlanning 面板 → Select Goal State = camera_pose → Plan & Execute；随后 home 往返
# 观察：ros2_control_node 日志关键字 SEND_SUMMARY / RIB_IDLE / traj_fault
```

## 9. 附录

### 9.1 官方文档

- MoveIt2 Concepts / move_group：<https://moveit.picknik.ai/main/doc/concepts/move_group/move_group.html>（注：官方文档站按 main 构建，本文结论均已对照 2.12.4 源码核实）
- Trajectory Execution / Low Level Controllers：<https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html>
- Time Parameterization（"2.12 只有 TOTG"）：<https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html>
- OMPL 接口：<https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html>
- Pilz 规划器 how-to：<https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html>
- CHOMP：<https://moveit.picknik.ai/main/doc/how_to_guides/chomp_planner/chomp_planner_tutorial.html>；Planning Adapters：<https://moveit.picknik.ai/main/doc/examples/planning_adapters/planning_adapters_tutorial.html>
- MoveItCpp：<https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html>
- MoveIt Servo：<https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html>
- Hybrid Planning：<https://moveit.picknik.ai/main/doc/examples/hybrid_planning/hybrid_planning_tutorial.html>
- JTC 文档（userdoc / trajectory 插值 / 参数 / decelerate_on_cancel / speed_scaling）：<https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- forward / position / velocity controllers：<https://control.ros.org/jazzy/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>
- controller_manager userdoc（Determinism）与 Controller Chaining：<https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html>
- realtime_tools：<https://control.ros.org/jazzy/doc/realtime_tools/doc/index.html>

### 9.2 源码引用（均按版本标签）

- moveit2 @2.12.4：`moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp`（抢占）；`moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp`（push/execute，无 auto_replace）；`moveit_ros/planning/planning_response_adapter_plugins/src/add_ruckig_traj_smoothing.cpp`；`moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp`；`moveit_ros/moveit_servo`（`servo_node.cpp`、`include/moveit_servo/utils/common.hpp:58` MIN_POINTS_FOR_TRAJ_MSG=3）；`moveit_configs_utils/default_configs/ompl_planning.yaml`、`pilz_industrial_motion_planner_planning.yaml`
- moveit_msgs @ros2：`action/MoveGroup.action`、`action/ExecuteTrajectory.action`（无 ExecuteKnownTrajectory）
- ros2_controllers @4.40.1：`joint_trajectory_controller/src/joint_trajectory_controller.cpp`（:1085-1087 topic 订阅、:1475 topic_callback、:262-410 采样、:1513-1545 cancel、:1911-1923 抢占、:1925 hold、:2025 成功保持）、`src/joint_trajectory_controller_parameters.yaml`（§4.4 对照依据）；`forward_command_controller/include/forward_command_controller/forward_controllers_base.hpp`
- ros2_control @4.45.2：`controller_manager/src/ros2_control_node.cpp`（SCHED_FIFO/lock_memory 告警回退）
- UR passthrough：github.com/UniversalRobots/Universal_Robots_ROS2_Driver PR #944（`ur_controllers/src/passthrough_trajectory_controller.cpp:203-211` GPIO 接口清单）

### 9.3 本工作区文件对照

| 内容 | 路径 |
| --- | --- |
| 硬件插件（路径门/SPSC/sender/status） | `src/aubo_e5_hardware/src/aubo_ros2_system.cpp`、`include/aubo_driver_ros2/aubo_ros2_system.hpp` |
| 控制器与 CM 配置 | `src/aubo_e5_bringup/config/controllers.yaml` |
| 两点位测试节点（现行方案） | `src/aubo_e5_bringup/scripts/named_pose_controller.py` |
| MoveIt 管线/控制器/极限 | `src/aubo_e5_moveit_config/launch/moveit.launch.py`、`config/{controllers,ompl_planning,joint_limits,kinematics}.yaml`、`config/aubo_e5.srdf` |
| SDK 实测报告 | `docs/aubo_sdk_2_5_3_ros2_control_test_report.md` |
| 实时内核/权限配置记录 | `docs/realtime_setup.md` |
| 真机接入核对 | `docs/real_hardware_integration_notes.md` |

### 9.4 本地教程笔记（/home/mu/Videos/Robotics_Tutorial，经验数字为教学参考值）

- `05_运动控制/20_机械臂/M10_时间参数化.md`——TOTG/TOPP-RA/Ruckig 谱系、jerk 与谐波减速器寿命定量（§3.6 支撑）；"抖动先查时间参数化"。
- `05_运动控制/20_机械臂/M11_实时CPP工程.md`——PREEMPT_RT、cyclictest 延迟数字、RT 安全 C++ 禁区（§4.5 背景）。
- `05_运动控制/20_机械臂/M12_ros2_control与硬件驱动.md`——架构与 RT 主循环延迟预算、JTC 机制（position 模式 PID 不生效）、libfranka 1 kHz vs UR 500 Hz 对比（§4 背景；项目 docs 既有引用）。
- `05_运动控制/20_机械臂/M14_MoveIt2_MTC工业集成.md`——MoveIt2 架构、规划器选型表、Servo 管线与调参（§3.5/3.9 支撑）。
- 背景：`M07_OMPL采样规划.md`、`M08_轨迹优化规划器.md`、`调研报告二_全景调研.md`。
- 边界说明：该笔记仓库无 AUBO/遨博内容，真机范例均为 Franka/UR/KUKA；凡 AUBO 数字一律以 `docs/aubo_sdk_2_5_3_ros2_control_test_report.md` 实测为准。

### 9.5 版本核实命令

```bash
dpkg -l | grep -E "ros-jazzy-(moveit|pilz|ros2-controllers|joint-trajectory|realtime-tools)"
apt-cache policy ros-jazzy-moveit-servo ros-jazzy-moveit-hybrid-planning \
  ros-jazzy-moveit-planners-stomp ros-jazzy-moveit-planners-chomp
```
