# AUBO E5 ROS 2 Jazzy 工作区

面向 AUBO E5 六轴机械臂（老控制器固件，8899 端口旧 SDK v1.3.1）的 ROS 2 Jazzy 驱动。
核心逻辑完全遵循 Humble 实测驱动（aubo_boot）的**一次性下发**模式：整条轨迹一次接收，
硬件侧五次重采样（5ms 点距）→ RIB 水位流控 → TCP2CAN 透传至接口板（5ms/点消费）。
控制器插件为本地自写（参考 UR `PassthroughTrajectoryController` 的写法），经
`trajectory_passthrough` GPIO 契约接入 ros2_control。2026-07-27 起替换原流式 JTC 架构
（旧实现归档于 `src_legacy/`，含 COLCON_IGNORE，不参与构建）。

## 包结构

```
src/
├── aubo_msgs/                  # 自定义消息/服务/action（IOState、RobotStatus、SetIO、
│                               #   GetFK/IK、SetPayload、手眼标定 action/srv）
├── aubo_description/           # E5 工作单元 URDF（table/camera/quick_changer）+
│                               #   ros2_control xacro（接口契约与硬件参数表）
├── aubo_e5_hardware/           # 核心：SystemInterface 真机插件 + 板级模拟器插件 + vendor SDK
├── aubo_e5_controllers/        # AuboPassthroughTrajectoryController + AuboIOController
├── aubo_dashboard/             # 独立服务节点（上电/断电/停止/FK/IK/负载，非运动类）
├── aubo_e5_moveit_config/      # MoveIt 配置（ompl + pilz 双管线）
├── aubo_e5_bringup/            # launch + controllers.yaml + 运维脚本
├── aubo_hand_eye_calibration/  # 手眼标定（未改动）
└── percipio_camera/            # 相机驱动（未改动）
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 三种运行模式

通过 `hardware_mode` 切换（xacro 参数 + launch 参数）：

| 模式 | 硬件插件 | 控制器 | 用途 |
|---|---|---|---|
| `mock` | `mock_components/GenericSystem` | `joint_trajectory_controller` | 标准 ros2_control 回归（MoveIt 同样映射到 JTC） |
| `sim` | `aubo_e5_hardware/AuboE5SimHardware` | passthrough + IO | **passthrough 全链路闭环模拟**（无真机） |
| `real` | `aubo_e5_hardware/AuboE5Hardware` | passthrough + IO + dashboard | 真机 |

```bash
# mock：标准 JTC 链路
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock

# sim：板级模拟器，passthrough 闭环
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim

# real：真机（robot_ip 默认 169.254.10.98）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<控制器IP>

# MoveIt(move_group)与 rviz2 默认随 bringup 一起启动；底层调试时可关闭：
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim moveit_enabled:=false

# 叠加相机 / 手眼标定 / named-pose 快捷服务
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim \
  camera_enabled:=true hand_eye_enabled:=true named_pose_enabled:=true
```

sim 插件行为与真机契约一致：传输状态机（0/1/2/3/4/5/6）、五次重采样、虚拟接口板每周期
消费 1 点（200Hz = 5ms/点）、`rib_level`、abort 丢弃队列。执行耗时与真实轨迹时长一致，
可提前暴露 MoveIt 超时/判定问题。

`real` 模式启动时会先跑 `scripts/realtime_preflight.sh` 做 RT 权限/调度预检，失败即中止。

## 轨迹执行链路（real/sim）

```
MoveIt → FollowJointTrajectory goal
  → AuboPassthroughTrajectoryController（remapJointNames 按名重排、blendToFirstPoint
    首点融合、每周期经 GPIO 传 1 个设定点：transfer_state 状态机 6→1→2→…→3）
  → AuboE5Hardware::write()（设定点入 SPSC 队列，回写状态机）
  → 发送线程（非 RT，4ms）：逐段五次重采样为 5ms 点 → RIB 水位流控
    → SetRobotPosData2Canbus 批量透传 → 接口板 5ms/点消费
  → 队列排空 → DONE(5) → 控制器 goal_hold → action succeed
```

停止原语（三分）：正常完成=自然排空；取消/抢占=清双队列 + `RobotMoveStop` 主动丢弃
RIB；急停/防护停=仅停发清队（本体安全回路主导）。

## MoveIt 参数传递

1. **action 连接**（`aubo_e5_moveit_config/config/controllers.yaml`）：
   `controller_names: [aubo_passthrough_trajectory_controller]` +
   `action_ns: follow_joint_trajectory`——MoveIt 不感知背后是 JTC 还是自研控制器。
2. **goal 内动态参数**：`path_tolerance/goal_tolerance/goal_time_tolerance` 可覆盖控制器
   默认容差；关节顺序以 goal 的 joint_names 为准，控制器 remap 到权威顺序。
3. **静态配置**：move_group 侧 `trajectory_execution.*`（蓝本值 5.0/10.0/0.15）；
   控制器侧参数在 `aubo_e5_bringup/config/controllers.yaml`。
4. **速度缩放**：由 MoveIt 时间参数化完成，控制器不做执行期缩放。

## 关键参数

hardware 参数（URDF `<param>`，见 `aubo_description/urdf/aubo_e5.ros2_control.xacro`）：
`send_period_ms=4`、`rib_target=400`、`rib_slowdown_1/2=300/350`、`batch_min/max=2/8`、
`ema_alpha=0.1`、`stop_retry_ms=20`、`auto_power_on=false` 等，默认值与蓝本一致。

控制器参数（`controllers.yaml`）：`goal_tolerance_rad=0.02`、`goal_vel_tolerance=0.01`、
`goal_hold_frames=5`、`goal_check_ms=50`、`blend_threshold_rad=0.01`、`blend_steps=30`。

## 真机分阶段测试（务必按序）

详细命令手册（控制器管理、action/服务示例、分析工具、排障表）见
[docs/usage.md](docs/usage.md)。要点：

1. 现场确认急停/限位/碰撞等级/低速模式；`hardware_mode:=real` 只核对 6 关节名称、方向、
   位置与示教器一致（joint_state_broadcaster）；
2. 断线/急停注入：验证 read() 报错、控制器停用、不再发旧目标；
3. 上电：示教器手动或 `ros2 service call /aubo_dashboard/startup`（默认
   `auto_power_on=false`，不自动上电）；
4. 速度因子 0.1 的单关节小轨迹；执行中取消（验证 RIB 被丢弃、余点不继续）；
5. MoveIt 整机轨迹。

## 部署注意事项

- **libprotobuf.so.9**：旧 SDK（libauborobotcontroller.so.1.3.1）强依赖 protobuf 2.6.1，
  已 vendor 在 `aubo_e5_hardware/vendor/lib64/`，RPATH（DT_RPATH）自动解析。
- **SDK 运行时配置**：SDK 按进程 CWD 读 `./config/auborobot.conf` 与
  `tracelog.properties`；launch 已把 `ros2_control_node` 与 `aubo_dashboard_node`
  的 cwd 设为各自 share 目录。
- **TCP2CAN 独占**：激活后 SDK 运动 API 被插件独占，示教器运动暂停；deactivate 后恢复。

制作可迁移的源码归档（排除 `build/`、`install/`、`log/`）：

```bash
./scripts/package_workspace.sh
```

## 参考

- 架构蓝本：`/home/mu/Music/e`（含 `docs/implementation_plan.md`、`docs/aubo_sdk_research.md`）
- 写法参考：UR `ur_controllers::PassthroughTrajectoryController`
- 旧架构（流式 JTC）测试报告见 `docs/`（针对已归档实现，仅供历史参考）
