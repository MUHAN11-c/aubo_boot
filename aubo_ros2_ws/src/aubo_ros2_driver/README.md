# aubo_ros2_driver 功能包集合

本目录包含 AUBO 机械臂在 ROS2 下的驱动、MoveIt 配置、轨迹执行与仿真等相关功能包，支持「移植前（ROS1 驱动 + 桥接）」与「移植后（全 ROS2 驱动）」两种运行方式。

---

## 功能包与作用

| 功能包 | 作用 |
|--------|------|
| **aubo_driver_ros2** | ROS2 机械臂驱动：连接控制器，订阅 `moveItController_cmd`，发布 joint_states / aubo/feedback_states，向真实机下发轨迹点。单节点、多线程 executor 与回调组，取点线程 200Hz 批量灌点、送机线程实时 RIB 流控（含单条轨迹内停顿修复）。 |
| **aubo_ros2_trajectory_action** | FollowJointTrajectory 的 action server：接收 MoveIt 轨迹目标，发布到 `joint_path_command`，供插值节点或桥接使用。 |
| **aubo_robot_simulator_ros2** | 轨迹插值节点：订阅 `joint_path_command`，200Hz 五次多项式插值，发布 `moveItController_cmd`。launch 中 `minimum_buffer_size` 设为 600。移植后与 driver 同侧；移植前可关闭，由 ROS1 端插值。 |
| **aubo_moveit_config** | MoveIt 配置与统一 launch：规划、MoveIt、RViz、驱动/插值/桥接节点。含 `aubo_moveit_bridge_ros1.launch.py`（可选桥接）与 `aubo_moveit_pure_ros2.launch.py`（纯 ROS2）。 |
| **aubo2_moveit_config** | 另一套 MoveIt 配置（可选）。 |
| **feedback_bridge** | 反馈桥接：将 control_msgs 的 FollowJointTrajectory 反馈转为 action 所需格式，用于「ROS1 驱动 + 桥接」时反馈回 ROS2。 |
| **demo_driver** | 应用层：move_to_pose_server_node、robot_status_publisher 等节点与服务。 |
| **demo_interface** | demo_driver 使用的消息/服务接口。 |
| **aubo_msgs** | AUBO 消息与服务（JointTrajectoryFeedback、SetIO、GetIK 等）。 |
| **aubo_description** | 机器人模型（URDF/XACRO、mesh）。 |
| **aubo_demo** | 调用 demo_driver 的示例。 |

---

## 编译

在 workspace 根目录（如 `aubo_ros2_ws`）下：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

仅编译本目录部分包示例：

```bash
colcon build --packages-select aubo_driver_ros2 aubo_moveit_config aubo_ros2_trajectory_action aubo_robot_simulator_ros2 demo_driver
```

---

## 运行方式

### 移植后（全 ROS2，接真实机）

**方式一：纯 ROS2 launch（推荐）**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
# 指定控制器 IP（默认 169.254.10.98）：
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98
```

**方式二：桥接 launch 中启用 ROS2 驱动**

```bash
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py \
  use_aubo_driver_ros2:=true \
  aubo_driver_server_host:=169.254.10.98
```

- 插值默认在 ROS2（aubo_robot_simulator_ros2，`minimum_buffer_size=600`），无需改。

### 移植前（ROS1 驱动 + ros1_bridge）

需先在外层脚本中启动 ROS1 与 ros1_bridge，再执行：

```bash
ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py simulator_in_ros2:=false
```

- `simulator_in_ros2:=false`：不启动 aubo_robot_simulator_ros2，轨迹经 bridge 到 ROS1 端插值与驱动。
- 项目内脚本：`aubo_ros2_ws/start_IVG_before_migration.sh` 可用来启动移植前环境（若存在）。

### 常用 launch 参数（aubo_moveit_bridge_ros1.launch.py）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_aubo_driver_ros2` | false | true：用 aubo_driver_ros2；false：用 ROS1 驱动 + feedback_bridge |
| `simulator_in_ros2` | true | true：ROS2 插值（aubo_robot_simulator_ros2）；false：ROS1 插值（需 bridge） |
| `aubo_driver_server_host` | 169.254.10.98 | use_aubo_driver_ros2=true 时控制器 IP |

### 单独运行节点（连通性/调试）

**驱动：**

```bash
ros2 run aubo_driver_ros2 aubo_driver_ros2 --ros-args -p server_host:=169.254.10.98
```

**插值（一般由 launch 启动）：**

```bash
ros2 run aubo_robot_simulator_ros2 aubo_robot_simulator_node
```

---

## 文档

- **移植与运动修复总结**（卡顿/单条轨迹内停顿、调试约定、插值节点要点）：`aubo_driver_ros2/doc/PORTING_MOTION_FIX.md`
