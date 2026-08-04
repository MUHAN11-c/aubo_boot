# aubo_e5_bringup — AUBO E5 工作区唯一启动入口

## 简介

纯 launch/config 集成包（ament_cmake，自身没有任何节点代码），是整个工作区
的**唯一启动入口**：一个 `bringup.launch.py` 按 `hardware_mode`
（mock/sim/real）组装出三种运行形态。输入是 launch 参数（模式、机器人 IP、
各功能开关），输出是按模式拉起的一组进程——robot_state_publisher、
ros2_control_node（硬件插件 + 控制器）、spawner、dashboard（仅 real），
以及 include 进来的 MoveIt（move_group + rviz2）、Percipio 相机、手眼外参
静态 TF、手眼标定流程。本包只做"组装与接线"：URDF 模板来自
aubo_description，硬件插件来自 aubo_e5_hardware，控制器插件来自
aubo_e5_controllers，运行时行为契约见各包 README。

## 使用方法

构建（本包只把 `launch/`、`config/`、`LICENSE` 装进 share，无编译产物）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_e5_bringup
source install/setup.bash
```

启动三种模式：

```bash
# sim：passthrough 全链路闭环模拟，无真机开发/验证首选（无相机时显式关闭）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false

# real：真机（hardware_mode 默认即 real）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98

# mock：mock_components/GenericSystem + 标准 JTC，ros2_control 官方回归链路
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock
```

启动前必做（任何模式）：查旧进程，有残留先按 PID 清掉再启动——旧
component_container/相机/SDK 通道都是独占资源，且旧进程跑的是构建前的旧
二进制，会让"改动已生效"的判断失真（AGENTS.md 第 6 节第 0 条）：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

launch 参数（默认值以 `launch/bringup.launch.py` 的 DeclareLaunchArgument
为权威源，`ros2 launch ... --show-args` 可查全部中文说明）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `hardware_mode` | real | mock \| sim \| real，透传进 xacro 选硬件插件 |
| `robot_ip` | 169.254.10.98 | 真机控制器 IP；mock/sim 不用 |
| `moveit_enabled` | true | include moveit.launch.py（move_group + rviz2） |
| `camera_enabled` | true | include percipio_camera.launch.py |
| `extrinsics_enabled` | true | 手眼外参静态 TF（wrist3_Link→camera_link） |
| `hand_eye_enabled` | false | 手眼标定流程（17 位姿采集 + 求解） |
| `hand_eye_web_enabled` | false | 标定 Web 界面；仅 `hand_eye_enabled:=true` 时生效 |

**真机安全约定**：real 模式下任何运动测试（RViz 拖动、MoveIt 规划执行、
测试脚本），速度/加速度缩放必须先压到 **0.1**，确认行为符合预期后才逐步
放宽；默认 `auto_power_on=false`，启动不会自动上电，上电需显式调用
`/aubo_dashboard/startup` 或示教器操作。完整分阶段真机流程见
`docs/usage.md` 第 7 节。

常用运行期操作（控制器由 controller_manager 管理，无需重启 launch）：

```bash
ros2 control list_controllers          # 查看控制器状态（active/inactive）
ros2 control list_hardware_interfaces  # 查看硬件接口认领情况
```

## 执行逻辑

launch 文件分两段（缘由见 `launch/bringup.launch.py:39-46` 头注释）：launch
参数在声明阶段只有"替换对象"没有具体值，只有进 `OpaqueFunction` 拿到
context 后才能 `.perform(context)` 求值——所以**所有按 mode 分支的逻辑
必须放进回调**，而各自独立开关的功能块可以留在声明式主流程里用
`IfCondition` 惰性求值。

启动时的组装顺序（`launch_nodes()`，`bringup.launch.py:61`）：

1. **mode 合法性检查**（:69-71）：拼错 mode 直接抛 `RuntimeError` 早失败，
   避免带错误配置起一半节点。RT 内核/SCHED_FIFO 预检已取消（:72），普通
   内核直接跑 real。
2. **xacro 现场展开 URDF**（:77-83）：把 `hardware_mode`/`robot_ip` 透传进
   `aubo_description/urdf/aubo_e5.urdf.xacro`，xacro 内部据此选硬件插件
   （`aubo_e5.ros2_control.xacro:38-45`）——"一份 URDF 模板、三种硬件后端"。
3. **核心三节点**（:98-111）：robot_state_publisher（robot_description +
   TF）、ros2_control_node（加载硬件插件与全部控制器，参数来自本包
   `config/controllers.yaml`）、joint_state_broadcaster spawner。
   注意 ros2_control_node 的 **cwd 被设到 aubo_e5_hardware 的 share 目录**
   （:89-93）：旧 SDK 按进程 CWD 读 `./config/auborobot.conf` 与
   `tracelog.properties`，config 随硬件包装在 share 下；mock/sim 不连
   SDK，设了也无害。
4. **按 mode 分支的控制器**（:113-136）：mock 只 spawner 标准
   `joint_trajectory_controller`（对应 MoveIt 的 controllers_mock.yaml）；
   sim/real spawner 自研的 `aubo_io_controller` +
   `aubo_passthrough_trajectory_controller`（passthrough 架构两控制器）。
5. **real 追加 dashboard**（:137-144）：`aubo_dashboard_node`（上电/断电/
   停止/FK/IK/负载等非运动类服务），走 SDK 第二条连接，cwd 同样指到
   aubo_dashboard 自己的 share 目录（也装了 SDK config）。
6. **MoveIt 集成**（:150-162）：`moveit_enabled:=true` 时 include
   `aubo_e5_moveit_config/launch/moveit.launch.py`，按 mode 选控制器映射
   （mock→`controllers_mock.yaml`，sim/real→`controllers.yaml`），并传
   `standalone_state_publishers:=false`——rsp 已在第 3 步起过，不能重复。
7. **三个独立开关的可选块**（`generate_launch_description()`，:175-198）：
   相机 include（`camera_enabled`）、extrinsics_publisher 节点
   （`extrinsics_enabled`，与相机开关独立）、手眼标定 include
   （`hand_eye_enabled`；内部固定 `extrinsics_enabled:=false`，外参 TF 由
   本文件统一管，避免双发）。

错误路径：mode 非法 → 启动即抛错；spawner 带
`--controller-manager-timeout 10`，控制器加载失败 10s 后 spawner 退出并在
日志暴露；launch 自身不做重试，节点级故障直接上抛给操作者。

## 软件框架

文件清单（全部内容即下述四项，无 src/）：

- `launch/bringup.launch.py` — 唯一启动入口，两段式结构（声明 + OpaqueFunction）
- `config/controllers.yaml` — 控制器与 controller_manager 参数的权威源
- `package.xml` / `CMakeLists.txt` — 依赖声明与 share 安装规则
- `LICENSE`（BSD-3-Clause）

`config/controllers.yaml` 注册的四个控制器（类型即插件名）：

| 控制器 | 插件类型 | 模式 |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 三种模式 |
| `aubo_passthrough_trajectory_controller` | `aubo_e5_controllers/AuboPassthroughTrajectoryController` | sim/real |
| `aubo_io_controller` | `aubo_e5_controllers/AuboIOController` | sim/real |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | mock |

关键参数（controllers.yaml 为权威源，蓝本 aubo_boot 默认值，yaml 注释注明
"真机确认后再调"）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `controller_manager.update_rate` | 200 | ros2_control 控制循环频率 (Hz) |
| `aubo_passthrough_trajectory_controller.joints` | 六关节权威顺序 | shoulder/upperArm/foreArm/wrist1/wrist2/wrist3 |
| `goal_tolerance_rad` / `goal_vel_tolerance` | 0.02 / 0.01 | goal_hold 位置/速度容差 |
| `goal_hold_frames` / `goal_check_ms` | 5 / 50 | 连续 N 帧在容差内判成功 / 检查周期 |
| `goal_time` | 0.0 | 轨迹时长外的额外到达时限 (s)，0 禁用超时 |
| `blend_threshold_rad` / `blend_steps` | 0.01 / 30 | 首点 smoothstep（C1）融合阈值与步数 |
| `speed_scaling_interface_name` | speed_scaling/speed_scaling_factor | 速度缩放接口限定名（恒定 1.0） |
| `aubo_io_controller.check_io_successfull_retries` | 10 | set_io 成功确认重试次数（每次 50ms） |
| `joint_trajectory_controller.command_interfaces` | [position] | mock 链路标准 JTC 配置 |

对外接口契约：本包自身不发布任何话题/服务，对外接口就是上面 7 个 launch
参数 + 它拉起的实体。运行期话题/服务/action/GPIO 由被拉起的包提供——
passthrough 状态机与三组 GPIO 契约（`trajectory_passthrough`/`aubo_io`/
`speed_scaling`）见 aubo_e5_controllers 与
`aubo_description/urdf/aubo_e5.ros2_control.xacro`，dashboard 服务见
aubo_dashboard。反向引用：aubo_hand_eye_calibration、peach_pose_ros2 的
README 均以本 launch 为前置启动步骤。

测试：本包只有 lint（ament_lint_auto：copyright/cpplint/lint_cmake/
uncrustify/xmllint），无业务测试；行为验证走 sim 闭环冒烟（AGENTS.md 第 6
节，先 `hardware_mode:=sim` 起本 launch，再跑
`tools/passthrough_traj_client.py` / `tools/motion_analyzer.py`）。
