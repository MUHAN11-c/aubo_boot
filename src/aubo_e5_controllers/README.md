# aubo_e5_controllers — passthrough 轨迹控制器 + IO 控制器插件

## 简介

ros2_control 控制器插件包（ament_cmake，无独立节点/launch），装出两个
`controller_interface` 插件：

- `aubo_e5_controllers/AuboPassthroughTrajectoryController`：FollowJointTrajectory
  action server，把整条轨迹按名重排、可选前置 C1 融合段后，经
  `trajectory_passthrough` GPIO 逐点透传给硬件插件，goal_hold 确认后才回 result。
- `aubo_e5_controllers/AuboIOController`：从 `aubo_io` GPIO 发布 IO/RobotStatus/
  RIB/JointStatus/诊断/事件，并提供 `~/set_io` 服务。

写法参考 UR `ur_controllers`（PassthroughTrajectoryController / GPIOController，
Jazzy 分支），行为语义对齐实测蓝本 aubo_boot；消息用 `aubo_msgs`/`std_msgs`
替代 `ur_msgs`。输入是 MoveIt（经 `aubo_e5_moveit_config/config/controllers.yaml`
映射）或测试脚本的 FJT goal；输出落到 `aubo_e5_hardware`（real/sim 插件）导出的
GPIO 命令接口。由 `aubo_e5_bringup` 经 spawner 拉起；mock 模式不使用本包
（走标准 joint_trajectory_controller）。

## 使用方法

构建与 lint（本包已接入 ament_lint_auto，无业务单测，验证靠 sim 闭环）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_e5_controllers
colcon test --packages-select aubo_e5_controllers && colcon test-result --verbose
source install/setup.bash
```

控制器不单独启动，随 bringup 拉起（sim/real 模式自动 spawn
`aubo_io_controller` + `aubo_passthrough_trajectory_controller`，另加
`joint_state_broadcaster`）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 control list_controllers                # 确认三控制器 active
```

sim 闭环冒烟（改本包代码后必做）：

```bash
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder 3
aubo_py3.12/bin/python tools/passthrough_traj_client.py sine_shoulder   # 压测重采样/流控
```

IO 与状态观察：

```bash
ros2 service call /aubo_io_controller/set_io aubo_msgs/srv/SetIO "{fun: 1, pin: 3, state: 1.0}"
ros2 topic echo --once /aubo_io_controller/io_states      # 全部 IO（fun: 1=板载DO 2=板载AO 3=工具DO 4=工具AO）
ros2 topic echo --once /aubo_io_controller/robot_status   # 上电/急停/运动中/错误
ros2 topic echo --once /aubo_io_controller/rib_status     # [RIB水位, 发送队列点数, 瞬时吞吐pts/s]
ros2 topic echo --once /aubo_io_controller/joint_status   # 各关节电流/温度/跟随误差/错误码
```

注：sim 插件不模拟板载 IO 写回，`set_io` 返回 `success=false` 属预期。

**真机安全约定**：真机运行任何运动（action goal / MoveIt / 测试脚本），速度/
加速度缩放必须先压到 0.1，确认行为符合预期后才逐步放宽；`auto_power_on=false`，
上电需显式调 `/aubo_dashboard/startup`。

控制器参数（权威源 `src/aubo_e5_bringup/config/controllers.yaml`，与
`src/*_parameters.yaml` 声明的默认值一致）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `joints` | （空，bringup 配六关节） | 权威关节顺序，goal 按名重排；read_only |
| `state_interfaces` | [position, velocity] | 硬件提供的关节状态接口；read_only |
| `goal_tolerance_rad` / `goal_vel_tolerance` | 0.02 / 0.01 | goal_hold 位置/速度容差（goal 自带容差可覆盖） |
| `goal_hold_frames` / `goal_check_ms` | 5 / 50 | 回报成功前连续通过次数 / 检查周期 [ms] |
| `goal_time` | 0.0 | 轨迹时长外的额外超时 [s]，0 禁用；goal 的 goal_time_tolerance>0 时覆盖 |
| `blend_threshold_rad` / `blend_steps` | 0.01 / 30 | 首点偏差超阈值时前置融合段；融合点数（5ms/点，30=150ms） |
| `speed_scaling_interface_name` | speed_scaling/speed_scaling_factor | 速度缩放接口全名（本驱动恒 1.0，不做执行期缩放） |
| `tf_prefix` | "" | URDF 无 prefix 机制，非空会导致接口找不到（on_configure 仅 WARN） |
| `check_io_successfull_retries`（IO 控制器） | 10 | set_io 应答重试次数（每次 50ms） |

改参数必须改 `src/*_parameters.yaml`（generate_parameter_library 生成解析代码），
不是手写参数解析。

## 执行逻辑

### AuboPassthroughTrajectoryController

- **接 goal（非实时）**：`goal_received_callback` 校验关节集合（必须恰好是 6 个
  权威关节名）、点数、速度/加速度数组一致性、容差，非法直接 REJECT
  （`src/aubo_passthrough_trajectory_controller.cpp:620`）。接受即抢占：正在执行
  的旧 goal 被写 `abort=1.0` 并 setAborted("Preempted by a new goal")
  （`:818`）。`goal_accepted_callback` 里 `remapJointNames` 按名重排到权威顺序
  （缺省速度/加速度补 0，`:988`），首点偏差超 `blend_threshold_rad` 时
  `blendToFirstPoint` 前置 `blend_steps` 个 5ms smoothstep（3t²-2t³，C1）融合点
  并把原有点 `time_from_start` 整体后移（`:1031`）；轨迹与容差写入
  RealtimeThreadSafeBox，action 的反馈/结果经 50ms 墙钟定时器在非实时上下文执行。
- **传点（实时，随 controller_manager 200Hz）**：与硬件的交替握手状态机
  IDLE(0)/NEW_TRAJECTORY(6)/WAITING_FOR_POINT(1)/TRANSFERRING(2)/TRANSFER_DONE(3)/
  IN_MOTION(4)/DONE(5)，常量与读写方约定见
  `include/aubo_e5_controllers/aubo_passthrough_trajectory_controller.hpp:106`。
  每周期最多传 1 个 setpoint：硬件回 1 后控制器写入该点
  `time_from_start` 与 6 关节位置/速度/加速度（命令接口下标 `i*3+{0,1,2}`），
  置 2；全部传完置 3（`:395-469`）。`transfer_requested_` 守卫修复抢占死锁：
  硬件在应答本次传输时绝不锁存 abort（`:417-428`，背景见头文件注释）。
- **回 result**：硬件上报 DONE(5) 后不立即成功——每 `goal_check_ms` 做一次
  `withinGoalHold`（逐关节 |实际-终点| < 位置容差且 |速度| < 速度容差，
  `:937`），连续 `goal_hold_frames` 次通过才 setSucceeded；`goal_time`
  超时则 abort + GOAL_TOLERANCE_VIOLATED（`:484-537`）。反馈 actual 取关节
  状态接口，desired 优先取机器人侧 `aubo_io/tag_pos_*`/`tag_vel_*`（RIB 板载
  缓冲最深约 2s，透传点相位超前会失真；取不到回退透传点，`:546-599`）。
- **停止路径**：取消=写 abort=1.0 + setCanceled，update() 持续锁存 abort 直到
  硬件回 IDLE（`:774-807`、`:600-605`）；on_deactivate 写 abort=1.0 并 abort
  当前 goal（`:320`）；硬件侧中止（如安全事件）在 `current_index_>0` 后检出
  并 setAborted("Trajectory aborted by hardware")。

### AuboIOController

- `update()` 每周期顺序发 6 路（全部 RealtimePublisher try_publish，非阻塞）：
  `~/io_states`（DO/AO 无状态接口，回显命令值，未命令过报 flag=false）、
  `~/robot_status`（mode 简化为 -1 未上电/1 上电但急停/2 自动）、
  `~/rib_status`（data=[rib_level, send_queue_points, send_rate_pps]）、
  `~/joint_status`（following_error=|tag_pos-实际位置|）、`/diagnostics`
  （1Hz 节流 + health 变化立即发，5 个 status：hardware_health/safety_io/
  rib_stream/joint_errors/last_event）、`~/events`（event_type 变化发一条
  "type=\<n\> code=\<n\>"，transient_local；event_type<0 是"无事件"哨兵不发）。
  事件完整文本以 controller_manager 日志 `[AuboEvent]` 行为准，话题只传数值。
- `~/set_io`：mutex 串行化（async-success 标志共享）；fun 映射到命令接口
  （DO 0..15 / AO 0..3 / 工具 DO、AO 各 0..1；FUN_SET_TOOL_POWER_TYPE 有意不
  支持）。先把 `set_io_async_success` 清成 NaN（等待态）再写命令值，硬件 IO
  异步线程应答 1.0/-1.0；`waitForAsyncCommand` 每 50ms 轮询、最多
  `check_io_successfull_retries` 次（`src/aubo_io_controller.cpp:641-735`）。
- 接口认领顺序由 `CommandInterfaces`/`StateInterfaces` 枚举手工对齐
  （`include/aubo_e5_controllers/aubo_io_controller.hpp:70-116`）——新增接口
  只允许追加尾部，插入中间会让后面的偏移整体错位。

## 软件框架

```text
src/aubo_e5_controllers/
├── aubo_e5_controllers_plugin.xml                    # 两个 controller_interface 插件声明
├── include/aubo_e5_controllers/
│   ├── aubo_passthrough_trajectory_controller.hpp    # 状态机常量契约 + 类声明
│   └── aubo_io_controller.hpp                        # 命令/状态接口偏移枚举（手工对齐）
└── src/
    ├── aubo_passthrough_trajectory_controller.cpp    # FJT action server 实现（1081 行）
    ├── aubo_io_controller.cpp                        # IO/状态/诊断实现（742 行）
    ├── aubo_passthrough_trajectory_controller_parameters.yaml  # generate_parameter_library 声明
    └── aubo_io_controller_parameters.yaml
```

对外接口契约（`~` = 控制器节点命名空间，实际为 `/aubo_<...>_controller/`）：

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| action | `~/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 整条轨迹一次性透传；新 goal 抢占旧 goal |
| srv | `~/set_io` | `aubo_msgs/SetIO` | fun 1-4（板载 DO/AO、工具 DO/AO），异步应答确认 |
| pub | `~/io_states` | `aubo_msgs/IOState` | 板载/工具/安全 IO（stamp 为纳秒字符串，ROS1 遗留） |
| pub | `~/robot_status` | `aubo_msgs/RobotStatus` | 简化 industrial RobotStatus 语义 |
| pub | `~/rib_status` | `std_msgs/Int32MultiArray` | [rib_level, 发送队列点数, 瞬时吞吐 pts/s] |
| pub | `~/joint_status` | `aubo_msgs/JointStatus` | 电流/温度/tag 目标/跟随误差/错误码（6 关节） |
| pub | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1Hz 节流 + health 变化立即发 |
| pub | `~/events` | `std_msgs/String` | SDK 事件（transient_local，数值型） |

GPIO 契约（由 `aubo_e5_hardware` real/sim 插件导出，权威定义见
`aubo_description/urdf/aubo_e5.ros2_control.xacro`）：

- `trajectory_passthrough`（命令）：`setpoint_positions/velocities/accelerations_0..5`、
  `transfer_state`、`trajectory_size`、`time_from_start`、`abort`。
- `aubo_io`（命令）：`do_0..15`、`ao_0..3`、`tool_do_0..1`、`tool_ao_0..1`、
  `set_io_async_success`；（状态）：`di/ai/tool_di/tool_ai`、`estop`、
  `protective_stop`、`power_on`、`collision`、`in_motion`、`rib_level`、
  `joint_error_0..5`、`send_queue_points`、`send_rate_pps`、`tag_pos/tag_vel_0..5`、
  `joint_current/joint_temp_0..5`、`event_type`、`event_code`、`health`。
- `speed_scaling/speed_scaling_factor`（状态）：恒定 1.0，控制器只认领不使用。

包间引用：spawner 配置 `src/aubo_e5_bringup/config/controllers.yaml`（类型注册
+ 参数）；MoveIt 映射 `src/aubo_e5_moveit_config/config/controllers.yaml`
（action_ns=follow_joint_trajectory）；`aubo_hand_eye_calibration` Web 界面与
`diagnostics/live_monitor.py` 订阅上述状态话题。依赖 `aubo_msgs` 与
`generate_parameter_library`，其余为 ros2_control/control_msgs 标准栈。
