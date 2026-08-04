# aubo_description — AUBO E5 工作单元 URDF 与 ros2_control 接口契约

## 简介

纯资产包（ament_cmake，**不含任何节点/可执行目标**）：AUBO E5 六轴本体 URDF
（厂商模型）+ 工作单元组件（table / camera / quick_changer）+
`aubo_e5.ros2_control.xacro`（硬件插件选择 + 接口契约 + 硬件参数表，全工作区
唯一权威来源）。输出物是 `robot_description` 文本——由下游 launch 现场展开
xacro 得到，供 `robot_state_publisher`、`ros2_control_node`（controller_manager）、
`move_group`、rviz2 消费。交互方：`aubo_e5_bringup`（展开并透传
`hardware_mode`/`robot_ip`）、`aubo_e5_moveit_config`（展开取几何/运动学）、
`aubo_e5_hardware`（此处声明的插件名与参数由其解析）、`aubo_e5_controllers`
（按此处声明的接口名 claim）。

## 使用方法

本包不单独启动，经 bringup 使用（三种模式对照见 AGENTS.md 第 5 节）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_description && source install/setup.bash

ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98
```

真机（real）启动后任何运动测试，速度/加速度缩放先压 0.1（安全约定，见
AGENTS.md 第 10 节）；默认 `auto_power_on=false`，不会自动上电。

手动展开 / 校验 URDF（调试 xacro 时）：

```bash
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/aubo_e5.urdf.xacro \
  hardware_mode:=sim > /tmp/aubo_e5.urdf
check_urdf /tmp/aubo_e5.urdf
```

顶层 xacro 参数（`urdf/aubo_e5.urdf.xacro:10-15`）：

| xacro arg | 默认 | 说明 |
|---|---|---|
| `hardware_mode` | mock | mock / sim / real，选硬件插件；bringup launch 侧默认 real |
| `robot_ip` | 169.254.10.98 | 真机控制器 IP，仅 real 分支写入 `<param>` |
| `include_table` | true | 工作台（world → table_link） |
| `include_camera` | true | 腕部相机（wrist3_Link → camera_body_link） |
| `include_quick_changer` | true | 快换盘（挂在 camera_body_link 下，依赖 include_camera） |

注意：顶层 xacro 只把 `hardware_mode`/`robot_ip` 两个 arg 透传进 ros2_control 宏
（`aubo_e5.urdf.xacro:17-20`）；`server_port`、`rib_target` 等其余硬件参数
只能用宏默认值，要改就改 `aubo_e5.ros2_control.xacro:9-34` 的默认表。

## 执行逻辑

本包无运行时进程，"执行"指 launch 期的 xacro 展开与产物分发：

1. **装配**（`aubo_e5.urdf.xacro`）：include 本体 `aubo_e5.urdf`（保持厂商定义
   与工作单元隔离）、三个组件 xacro 与 ros2_control xacro；按 `include_*`
   条件实例化组件宏，调用 `aubo_e5_ros2_control` 宏生成 `<ros2_control>` 块。
2. **插件选择**（`aubo_e5.ros2_control.xacro:38-69`）：按 `hardware_mode`
   三分支——mock→`mock_components/GenericSystem`，sim→
   `aubo_e5_hardware/AuboE5SimHardware`，real→`aubo_e5_hardware/AuboE5Hardware`。
   **仅 real 分支**把硬件参数表（robot_ip、RIB 流控、速度/加速度上限等）
   写成 `<param>`；mock/sim 不带参数。
3. **消费路径**：bringup 用 `xacro` 命令现场展开并透传两个 arg
   （`bringup.launch.py:77-83`），同一文本喂给 robot_state_publisher 与
   ros2_control_node；`moveit.launch.py` 用 `xacro.process_file` 展开且**不传
   arg**（硬件块为 mock 默认值）——move_group 只读运动学/几何，不加载硬件
   插件，故无害。
4. **接口契约要点**：`transfer_state` 在 URDF 中声明为 command_interface，
   硬件靠 Jazzy 已 deprecated 的旧式 export 把 passthrough 状态机值回写进该
   接口供控制器读取（刻意写法，勿"升级"，见 AGENTS.md 第 8 节）；`aubo_io`
   命令接口以 NaN 表示"无请求"；`event_type` 以 -1 表示"尚无事件"（xacro
   内注释，`:200-234`）。GPIO 状态接口的名称/顺序必须与两个硬件插件的
   `export_state_interfaces()` 逐一对应，改动需三处同步。
5. **挂载链**：world→table_link（固定）；wrist3_Link→camera_body_link
   （z+0.020）；camera_body_link→quick_changer_link（z+0.0215，yaw π）。
   **`camera_link` 刻意不在 URDF 内**（`camera.xacro:4` 注释）——保留给手眼
   标定外参静态 TF，由 `aubo_hand_eye_calibration` 的 extrinsics_publisher
   发布 wrist3_Link→camera_link。

## 软件框架

```text
urdf/aubo_e5.urdf                  # 本体厂商模型：world + 7 link + 6 转动关节，
                                   #   全关节限位 ±3.05 rad / velocity 3.14 rad/s
urdf/aubo_e5.urdf.xacro            # 顶层装配（arg 声明 + 组件实例化 + 宏调用）
urdf/aubo_e5.ros2_control.xacro    # aubo_e5_ros2_control 宏：插件分支 + 接口契约
urdf/components/{table,camera,quick_changer}.xacro   # 工作单元组件宏
meshes/visual|collision/           # link0-6 + workcell/sensors/tools（URDF 实际引用）
meshes/aubo_e5/                    # 厂商原始网格（未被任何 URDF 引用，保留备查）
```

权威关节顺序（goal 会被控制器 remap 到此序）：`shoulder_joint, upperArm_joint,
foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。每关节接口：position
命令 + position/velocity 状态（`aubo_e5.ros2_control.xacro:72-101`）。
xacro **无 tf_prefix 机制**，接口名固定无前缀（控制器侧参数描述已注明）。

三组 GPIO 接口（`:104-235`）：

| GPIO | 内容 |
|---|---|
| `trajectory_passthrough` | setpoint_positions/velocities/accelerations_0..5 + transfer_state + time_from_start + abort + trajectory_size（全 command_interface） |
| `speed_scaling` | speed_scaling_factor（state，初值 1.0；本驱动不做执行期缩放） |
| `aubo_io` | 命令 do_0..15 / ao_0..3 / tool_do_0..1 / tool_ao_0..1 / set_io_async_success；状态 di_0..15 / ai_0..3 / tool_di/ai、estop、protective_stop、power_on、collision、in_motion、rib_level、joint_error_0..5、tag_pos/tag_vel_0..5、joint_current/joint_temp_0..5、send_queue_points、send_rate_pps、event_type/event_code、health |

硬件参数表（宏默认，仅 real 分支生效，`:9-34`）：`server_port=8899`、
`sdk_username/password`（占位值，勿提交真实凭据）、`send_period_ms=4`、
`rib_target=400`、`rib_slowdown_1/2=300/350`、`batch_min/max=2/8`、
`ema_alpha=0.1`、`ema_boost_ms='10,14,20'`、`stop_retry_ms=20`、
`prefill_points=0`、`force_start_delay_ms=0`、`speed_guard_enabled=false`、
`max_joint_velocity='2.596177×3, 3.110177×3'`、`max_joint_acceleration=
'17.30878×3, 20.73676×3'`、`point_spacing_s=0.005`、`same_point_eps=0.00015`、
`dedup_threshold=0.000001`、`state_timeout_ms=200`、`auto_power_on=false`。
（MoveIt 规划侧限位另由 `aubo_e5_moveit_config/config/joint_limits.yaml`
作为 `robot_description_planning` 提供，与本包 URDF 限位是两份配置。）
