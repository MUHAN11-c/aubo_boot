# aubo_ros2_driver 功能包集合

AUBO 机械臂在 ROS2 Humble 下的驱动、MoveIt 配置、轨迹执行与仿真等（全 ROS2 栈）。

---

## 工作空间功能包总览

| 功能包 | 职责 |
|--------|------|
| **aubo_ros2_driver** | 机械臂驱动与运动控制 |
| **aubo_ros2_web_dashboard** | 前端演示页面 |
| **robotwebtools** | 前端依赖库 |
| **camport_ros2** | 相机驱动 |
| **graspnet_ros2** | VCoT-Grasp 大模型 |
| **hand_eye_calibration** | 自动手眼标定 |
| **visual_pose_estimation** | 工件识别与抓取 |
| **vision_perception** | MediaPipe + YOLO OBB 感知 |
| **tool_changer** | 快换与手爪管理 |
| **coffee_latte_demo** | 咖啡拉花演示 |

---

## 功能包与作用

| 功能包 | 作用 |
|--------|------|
| **aubo_driver_ros2** | ROS2 机械臂驱动：连接控制器，订阅 `moveItController_cmd`，发布 joint_states / aubo/feedback_states，向真实机下发轨迹。多线程：取点 200Hz 批量灌点、送机 RIB 流控（含单条轨迹内停顿修复）。参数 `server_host` 为控制器 IP。 |
| **aubo_ros2_trajectory_action** | FollowJointTrajectory 的 action server：接收 MoveIt 轨迹，发布到 `joint_path_command`，供插值节点使用。 |
| **aubo_robot_simulator_ros2** | 轨迹插值节点：订阅 `joint_path_command`，200Hz 五次多项式插值，发布 `moveItController_cmd`。launch 中 `minimum_buffer_size=600`、`motion_update_rate=200.0`。 |
| **aubo_moveit_config** | MoveIt 配置与统一 launch：规划、move_group、RViz、驱动、插值。**aubo_moveit_pure_ros2.launch.py** 为推荐入口；**demo_driver_services.launch.py** 单独起 demo 服务。使用 `aubo_ros2.xacro`、ros2_control。 |
| **aubo2_moveit_config** | 另一套 MoveIt 配置（可选），含 aubo2_moveit_pure_ros2.launch.py 等。 |
| **demo_driver** | 应用层 C++ 节点：move_to_pose_server_node、plan_trajectory_server_node、execute_trajectory_server_node、get_current_state_server_node、set_speed_factor_server_node、set_robot_pose_server_node 及 GraspNet 抓取执行节点。快换功能已迁移至 `tool_changer` 包。 |
| **demo_interface** | demo_driver 使用的消息/服务接口定义。 |
| **aubo_msgs** | AUBO 消息与服务（JointTrajectoryFeedback、SetIO、GetIK 等）。 |
| **aubo_dashboard_msgs** | Dashboard 相关 msg/srv/action。 |
| **aubo_description** | 机器人模型（URDF/XACRO、mesh），含 `aubo_ros2.xacro`；另含遗留 **`world_map/`**（历史 catkin），一般不参与 colcon。 |
| **aubo_demo** | 调用 demo_driver 服务/话题的示例（C++）。 |

---

## 末端工具（Gripper）

| 工具 ID | 类型 | 所属包 | 用途 |
|---------|------|--------|------|
| **gripper0** | 气动夹爪 40 | `tool_changer` | 工件抓取（`visual_pose_estimation`），通过快换对接 |
| **gripper1** | 电动夹爪 A | `coffee_latte_demo` | 咖啡拉花（`coffee_cup` / `milk_cup` 载体），不参与快换 |
| **gripper2** | 电动夹爪 60 | `tool_changer` | AI 抓取（`graspnet_ros2`），通过快换对接 |

- **gripper0 ↔ gripper2 快换**：由 `tool_changer` 包的 `gripper_swap_worker_node`（`/run_gripper_swap`）实现
- **gripper1**：咖啡拉花工具，由 `coffee_latte_demo` 包管理

### IO 引脚分配

所有 IO 通过 `/aubo_driver/set_io`（`io_type=digital_output, io_index + 32 → 硬件引脚`）：

| 逻辑引脚 | 硬件引脚 | 使用节点 | 用途 |
|---------|---------|---------|------|
| **6** | 38 | ExecuteGraspPose / PublishGraspsClient / PublishGraspsAB | 夹爪开/关 |
| **7** | 39 | GripperSwapWorker | 快换盘锁紧/释放 |
| **2** | 34 | LatteNode (DO2) | 打花开关 |
| **4** | 36 | LatteNode (DO4) | 咖啡开关 |

> **注意**：ExecuteGraspPoseWorker 的 IO 语义与 PublishGraspsClientWorker **相反**（`true=打开` vs `true=闭合`），源于气动夹爪不同工位的电气接线差异。添加新 Worker 时需确认物理接线。

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
colcon build --packages-select aubo_driver_ros2 aubo_moveit_config aubo_ros2_trajectory_action aubo_robot_simulator_ros2 demo_driver demo_interface aubo_msgs aubo_description
```

### AUBO SDK 共享库安装

`aubo_driver_ros2` 依赖预编译的 AUBO SDK 共享库 (`libauborobotcontroller.so.1` 等)。CMakeLists.txt 在 `colcon build` 时自动将其安装到 `install/aubo_driver_ros2/lib/`。运行时 `source install/setup.bash` 将路径加入 `LD_LIBRARY_PATH` 即可找到。

**注意**: 首次编译或 SDK 库有更新后必须执行 `colcon build --packages-select aubo_driver_ros2`，否则驱动节点会报 `exit code 127` (library not found) 喵~

---

## 运行方式

### 全 ROS2（接真实机，推荐）

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
# 指定控制器 IP（默认 169.254.10.98）
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98
```

启动内容：robot_state_publisher、move_group、rviz2、aubo_driver_ros2、aubo_robot_simulator_node、aubo_ros2_trajectory_action、move_to_pose_server_node。

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
| **aubo_moveit_config/README.md** | MoveIt 配置、故障排除、ROS2 Control、RViz 映射、工作空间限制脚本 |
| **demo_driver/docs/MOVE_TO_POSE_AND_ERROR_MINUS4_ANALYSIS.md** | MoveToPose 与错误 -4 分析 |
| **demo_driver/README.md** | demo_driver 节点与接口说明 |
| **demo_interface/接口说明.md** | 接口定义说明 |
| **tool_changer/README.md** | 快换与手爪管理说明 |
| **coffee_latte_demo/README.md** | 咖啡拉花演示说明 |
