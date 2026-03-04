# aubo_ros2_driver 功能包集合

AUBO 机械臂在 ROS2 Humble 下的驱动、MoveIt 配置、轨迹执行与仿真等，支持「移植前（ROS1 驱动 + 桥接）」与「移植后（全 ROS2 驱动）」两种方式。

---

## 功能包与作用

| 功能包 | 作用 |
|--------|------|
| **aubo_driver_ros2** | ROS2 机械臂驱动：连接控制器，订阅 `moveItController_cmd`，发布 joint_states / aubo/feedback_states，向真实机下发轨迹。多线程：取点 200Hz 批量灌点、送机 RIB 流控（含单条轨迹内停顿修复）。参数 `server_host` 为控制器 IP。 |
| **aubo_ros2_trajectory_action** | FollowJointTrajectory 的 action server：接收 MoveIt 轨迹，发布到 `joint_path_command`，供插值或桥接使用。 |
| **aubo_robot_simulator_ros2** | 轨迹插值节点：订阅 `joint_path_command`，200Hz 五次多项式插值，发布 `moveItController_cmd`。launch 中 `minimum_buffer_size=600`、`motion_update_rate=200.0`。 |
| **aubo_moveit_config** | MoveIt 配置与统一 launch：规划、move_group、RViz、驱动/插值/桥接。**aubo_moveit_bridge_ros1.launch.py** 可选 ROS2 驱动或 ROS1+桥接；**aubo_moveit_pure_ros2.launch.py** 纯 ROS2（无桥接）。使用 `aubo_ros2.xacro`、ros2_control。 |
| **aubo2_moveit_config** | 另一套 MoveIt 配置（可选），含 aubo2_moveit_pure_ros2.launch.py 等。 |
| **feedback_bridge** | 将 aubo_msgs 的 JointTrajectoryFeedback 转为 control_msgs action 所需格式，用于「ROS1 驱动 + 桥接」时反馈回 ROS2。节点名 `feedback_bridge_node`。 |
| **demo_driver** | 应用层 C++ 节点：move_to_pose_server_node、plan_trajectory_server_node、execute_trajectory_server_node、get_current_state_server_node、set_speed_factor_server_node、set_robot_pose_server_node；可选 robot_status_publisher_node。 |
| **demo_interface** | demo_driver 使用的消息/服务接口定义。 |
| **aubo_msgs** | AUBO 消息与服务（JointTrajectoryFeedback、SetIO、GetIK 等）。 |
| **aubo_dashboard_msgs** | Dashboard 相关 msg/srv/action。 |
| **aubo_description** | 机器人模型（URDF/XACRO、mesh），含 `aubo_ros2.xacro`。 |
| **aubo_demo** | 调用 demo_driver 服务/话题的示例（C++）。 |

---

## 编译

在工作空间根目录（如 `aubo_ros2_ws`）：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

仅编译本目录部分包：

```bash
colcon build --packages-select aubo_driver_ros2 aubo_moveit_config aubo_ros2_trajectory_action aubo_robot_simulator_ros2 demo_driver demo_interface aubo_msgs aubo_description feedback_bridge
```

---

## 运行方式

### 移植后（全 ROS2，接真实机）

**方式一：纯 ROS2 launch（推荐）**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
# 指定控制器 IP（默认 169.254.10.98）
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98
```

启动内容：robot_state_publisher、move_group、rviz2、aubo_driver_ros2、aubo_robot_simulator_node、aubo_ros2_trajectory_action、move_to_pose_server_node。

**方式二：桥接 launch 中启用 ROS2 驱动**

```bash
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py \
  use_aubo_driver_ros2:=true \
  aubo_driver_server_host:=169.254.10.98
```

此时不启动 feedback_bridge；插值默认在 ROS2（aubo_robot_simulator_ros2）。

### 移植前（ROS1 驱动 + ros1_bridge）

先启动 ROS1 与 ros1_bridge，再：

```bash
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py simulator_in_ros2:=false
```

`simulator_in_ros2:=false` 时不启动 aubo_robot_simulator_ros2，轨迹经 bridge 到 ROS1 端插值与驱动。

### aubo_moveit_bridge_ros1.launch.py 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| use_aubo_driver_ros2 | false | true：用 aubo_driver_ros2；false：用 ROS1 驱动 + feedback_bridge |
| simulator_in_ros2 | true | true：ROS2 插值（aubo_robot_simulator_ros2）；false：ROS1 插值（需 bridge） |
| aubo_driver_server_host | 169.254.10.98 | use_aubo_driver_ros2=true 时控制器 IP |
| aubo_type | aubo_e5_10 | 机械臂型号 |
| use_fake_hardware | true | 是否使用仿真硬件 |
| use_ros2_control | true | 是否启用 ros2_control |

### aubo_moveit_pure_ros2.launch.py 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| aubo_driver_server_host | 169.254.10.98 | 机械臂控制器 IP |

### 单独运行节点（调试）

```bash
# 驱动
ros2 run aubo_driver_ros2 aubo_driver_ros2 --ros-args -p server_host:=169.254.10.98

# 插值（一般由 launch 启动）
ros2 run aubo_robot_simulator_ros2 aubo_robot_simulator_node
```

### 仅启动 demo_driver 服务（不含机械臂）

```bash
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

会按序启动 move_to_pose_server_node、plan_trajectory_server_node、execute_trajectory_server_node、get_current_state_server_node、set_speed_factor_server_node、set_robot_pose_server_node（需 move_group 等已运行）。

---

## 文档

| 文档 | 说明 |
|------|------|
| **aubo_driver_ros2/doc/PORTING_MOTION_FIX.md** | 移植与运动修复（卡顿、单条轨迹内停顿、插值节点要点） |
| **aubo_moveit_config/doc/TIMEOUT_ROOT_CAUSE.md** | 超时根因分析 |
| **aubo_moveit_config/TROUBLESHOOTING.md** | 故障排除 |
| **aubo_moveit_config/scripts/QUICK_START.md** | 快速上手 |
| **demo_driver/docs/MOVE_TO_POSE_AND_ERROR_MINUS4_ANALYSIS.md** | MoveToPose 与错误 -4 分析 |
| **demo_driver/README.md** | demo_driver 节点与接口说明 |
