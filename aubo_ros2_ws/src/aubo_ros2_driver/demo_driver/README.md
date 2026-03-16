# demo_driver

机器人驱动功能包（ROS 2），提供机器人状态发布、运动控制、位姿设置、夹爪快换与 GraspNet 循环抓取等功能。

## 快速开始

```bash
source install/setup.bash

# GraspNet 抓取放置 Worker（订阅 grasp_poses_base，循环抓取→放置→回安全位）
ros2 run demo_driver publish_grasps_client_worker_node

# 夹爪快换 Worker
ros2 run demo_driver gripper_swap_worker_node
```

**调用逻辑**：详见 [docs/GRASP_CALL_FLOW.md](docs/GRASP_CALL_FLOW.md)

## 包结构

```
demo_driver/
├── include/demo_driver/          # 头文件
│   ├── moveit_gripper_io_base.h  # 基类：MoveIt + 夹爪 IO
│   ├── publish_grasps_client_worker.h
│   ├── gripper_swap_worker.h
│   ├── set_robot_pose_server.h
│   └── ...
├── src/                          # 源文件
├── launch/                       # Launch 文件（部分为 ROS1 格式，建议用 demo_driver_services.launch.py）
├── scripts/                      # Python 脚本
└── docs/                         # 文档
```

## 依赖

- `demo_interface`: 消息和服务定义包
- `aubo_msgs`: Aubo 机器人消息包
- `moveit_core`, `moveit_ros_planning_interface`: MoveIt 2
- `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`, `std_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

## 节点说明

### 已启用节点

| 节点 | 可执行文件 | 说明 |
|------|------------|------|
| robot_status_publisher | robot_status_publisher_node | 发布机器人完整状态（在线、使能、运动、关节、笛卡尔位姿） |
| move_to_pose_server | move_to_pose_server_node | 移动到目标位姿服务（关节/笛卡尔空间） |
| plan_trajectory_server | plan_trajectory_server_node | 轨迹规划服务（不执行） |
| execute_trajectory_server | execute_trajectory_server_node | 执行轨迹服务 |
| get_current_state_server | get_current_state_server_node | 获取当前状态服务 |
| set_speed_factor_server | set_speed_factor_server_node | 设置速度因子服务 |
| set_robot_pose_server | set_robot_pose_server_node | 设置机器人位姿服务（欧拉角/关节空间） |
| gripper_swap_worker | gripper_swap_worker_node | 夹爪快换（gripper0 ↔ gripper2） |
| publish_grasps_client_worker | publish_grasps_client_worker_node | GraspNet 循环抓取放置 |

### 已禁用节点（CMakeLists 中注释）

- `robot_io_status_publisher_node`
- `set_robot_enable_server_node`
- `set_robot_io_server_node`
- `read_robot_io_server_node`
- `movel_server_node`

---

### 1. robot_status_publisher_node

**功能**：发布机器人完整状态信息。

#### 发布的话题

- `/robot_status` (demo_interface/RobotStatus)
  - **is_online**, **enable**, **in_motion**, **planning_status**
  - **joint_position_rad/deg**, **cartesian_position**

#### 参数

- `publish_rate` (double, default: 10.0)
- `base_frame` (string, default: "base_link")
- `planning_group_name` (string, default: "manipulator_e5")

---

### 2. move_to_pose_server_node

**功能**：移动到目标位姿服务，支持关节空间和笛卡尔空间规划。

#### 服务

- `/move_to_pose` (demo_interface/MoveToPose)
  - **Request**: target_pose, use_joints, velocity_factor, acceleration_factor
  - **Response**: success, error_code, message

---

### 3. plan_trajectory_server_node

**功能**：规划到目标位姿的轨迹但不执行。

#### 服务

- `/plan_trajectory` (demo_interface/PlanTrajectory)
  - **Request**: target_pose, use_joints
  - **Response**: success, trajectory, planning_time, message

---

### 4. execute_trajectory_server_node

**功能**：执行给定的关节轨迹。

#### 服务

- `/execute_trajectory` (demo_interface/ExecuteTrajectory)
  - **Request**: trajectory
  - **Response**: success, error_code, message

---

### 5. get_current_state_server_node

**功能**：获取当前关节位置、笛卡尔位姿和关节速度。

#### 服务

- `/get_current_state` (demo_interface/GetCurrentState)
  - **Response**: joint_position_rad, cartesian_position, velocity

---

### 6. set_speed_factor_server_node

**功能**：设置机器人整体速度缩放因子。

#### 服务

- `/set_speed_factor` (demo_interface/SetSpeedFactor)
  - **Request**: velocity_factor (0.0–1.0)

---

### 7. set_robot_pose_server_node

**功能**：通过欧拉角（roll-pitch-yaw）或关节空间设置目标位姿。

#### 服务

- `/set_robot_pose` (demo_interface/SetRobotPose)
  - **Request**: target_pose[6], use_joints, is_radian, velocity
  - **Response**: success, error_code, message

#### 调用示例

```bash
# 欧拉角模式（弧度）
ros2 service call /set_robot_pose demo_interface/srv/SetRobotPose "{target_pose: [0.4, 0.0, 0.3, 0.0, 1.57, 0.0], use_joints: false, is_radian: true, velocity: 0.5}"

# 关节空间模式
ros2 service call /set_robot_pose demo_interface/srv/SetRobotPose "{target_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], use_joints: true, is_radian: true, velocity: 0.3}"
```

---

### 8. gripper_swap_worker_node

**功能**：夹爪快换，提供 `/run_gripper_swap` 服务。

#### 服务

- `/run_gripper_swap` (demo_interface/srv/RunGripperSwap)
  - **Request**: direction — `"gripper2"` / `"gripper0"` / `"gripper0_to_gripper2"` / `"gripper2_to_gripper0"`

#### 调用示例

```bash
# 切换到 gripper2
ros2 service call /run_gripper_swap demo_interface/srv/RunGripperSwap "{direction: 'gripper2'}"

# 原有方向仍可使用
ros2 service call /run_gripper_swap demo_interface/srv/RunGripperSwap "{direction: 'gripper0_to_gripper2'}"
ros2 service call /run_gripper_swap demo_interface/srv/RunGripperSwap "{direction: 'gripper2_to_gripper0'}"
```

---

### 9. publish_grasps_client_worker_node

**功能**：GraspNet 抓取放置 Worker。订阅 `grasp_poses_base`（PoseArray），循环执行：清理窗口 → 等待新数据 → 选优 → gripper_tip 补偿 → 抓取接近 → 闭夹爪 → 抬起 → 移动到放置位 → 开夹爪 → 回安全位。

#### 订阅

- `grasp_poses_base` (geometry_msgs/PoseArray): base_link 下抓取位姿

#### 发布

- `grasp_place_status` (std_msgs/String): JSON 格式周期状态（cycle_count, success_count, fail_count 等）

#### 主要参数（可通过 `--ros-args -p` 覆盖）

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| prefer_vertical | bool | true | 选优策略：true 取垂直度最高 |
| grasp_z_offset | double | 0.2 | gripper_tip→end_effector 沿 z 轴补偿 (m) |
| height_above | double | 0.05 | 抓取点上方安全高度 (m) |
| joint_velocity_scaling | float | 0.5 | 速度缩放 [0~1] |
| joint_acceleration_scaling | float | 0.1 | 加速度缩放 [0~1] |
| grasp_poses_topic | string | grasp_poses_base | 抓取位姿话题 |
| wait_poses_timeout_sec | double | 30.0 | 等待窗口就绪超时 (s) |
| grasp_window_size | int | 5 | 滑动窗口大小 |
| min_groups_before_pick | int | 3 | 至少 M 组后再选优 |
| gripper_io_index | int | 7 | Aubo 夹爪 IO pin 号 |
| lift_offset | double | 0.2 | 抓取后沿 Z 轴抬起高度 (m) |
| place_mode | string | home_offset | "pose"/"joints"/"home_offset" |
| place_offset_y, place_offset_z | double | -0.2, -0.20 | 安全位偏移 (m) |
| cycle_delay_sec | double | 1.0 | 每周期结束后等待 (s) |
| max_cycles | int | -1 | 最大周期数，-1 无限循环 |

---

## 笛卡尔路径速度缩放

`MoveitGripperIoBase` 中的笛卡尔运动函数（`runArcPath`、`runArcPathSequence`）支持通过 `velocity_factor` 参数控制执行速度。

### 背景：为何 setMaxVelocityScalingFactor 无效

MoveIt2 的 `computeCartesianPath()` 生成的轨迹已包含固定的 `time_from_start`，执行时不会重新计算时间。在 `execute(plan)` 前调用 `setMaxVelocityScalingFactor()` 对笛卡尔路径**无效**，因为轨迹的时间戳在规划阶段就已确定。

### 实现方式：手动缩放轨迹时间

在 `execute` 前对轨迹做时间缩放，通过 `scaleTrajectoryTime()` 实现：

1. **时间缩放**：`new_time = old_time × (1 / velocity_factor)`
   - `velocity_factor = 0.5` → 轨迹时长变为 2 倍，速度约为 50%
   - `velocity_factor = 0.15` → 轨迹时长约为 6.67 倍，速度约为 15%

2. **速度缩放**：`v_new = v_old / scale`（`scale = 1/velocity_factor`）

3. **加速度缩放**：`a_new = a_old / scale²`

### API 与默认值

| 函数 | 参数 | 默认值 |
|------|------|--------|
| `runArcPath(double z_offset, float velocity_factor)` | velocity_factor | 0.5 |
| `runArcPath(char axis, double offset, float velocity_factor)` | velocity_factor | 0.5 |
| `runArcPathSequence(segments, float velocity_factor)` | velocity_factor | 0.5 |

### 数值示例

| velocity_factor | scale | 原 1 s 轨迹 | 新时长 | 等效速度 |
|-----------------|-------|-------------|--------|----------|
| 1.0 | 1.0 | 1 s | 1 s | 100% |
| 0.5 | 2.0 | 1 s | 2 s | 50% |
| 0.2 | 5.0 | 1 s | 5 s | 20% |
| 0.15 | 6.67 | 1 s | 6.67 s | 15% |

`moveToJoints` 和 `moveToPose` 使用 MoveIt 规划流程，`setMaxVelocityScalingFactor` 在**规划前**设置即可生效，无需手动缩放轨迹。

---

## 构建与运行

### 构建

```bash
cd aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select demo_driver
source install/setup.bash
```

### 运行方式

#### 方式一：通过 aubo_moveit_config 统一启动（推荐）

先启动 MoveIt2，再启动 demo_driver 服务：

```bash
# 终端 1：MoveIt2 + 机械臂
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py

# 终端 2：demo_driver 服务（延迟 15s 启动，等待 MoveIt2 就绪）
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

`demo_driver_services.launch.py` 会启动：move_to_pose、plan_trajectory、execute_trajectory、get_current_state、set_speed_factor、set_robot_pose 等节点。**注意**：若包含 `movel_server_node` 且该节点未编译，启动时会报错，可从 launch 中移除。

#### 方式二：单独运行节点

```bash
# 机器人状态
ros2 run demo_driver robot_status_publisher_node
ros2 topic echo /robot_status

# 移动到目标位姿
ros2 run demo_driver move_to_pose_server_node
ros2 service call /move_to_pose demo_interface/srv/MoveToPose "{target_pose: {position: {x: 0.4, y: 0.3, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, use_joints: false, velocity_factor: 0.5, acceleration_factor: 0.5}"

# 规划轨迹
ros2 run demo_driver plan_trajectory_server_node

# 执行轨迹
ros2 run demo_driver execute_trajectory_server_node

# 获取当前状态
ros2 run demo_driver get_current_state_server_node
ros2 service call /get_current_state demo_interface/srv/GetCurrentState

# 设置速度因子
ros2 run demo_driver set_speed_factor_server_node
ros2 service call /set_speed_factor demo_interface/srv/SetSpeedFactor "{velocity_factor: 0.5}"

# 设置机器人位姿
ros2 run demo_driver set_robot_pose_server_node

# 夹爪快换
ros2 run demo_driver gripper_swap_worker_node

# GraspNet 抓取放置
ros2 run demo_driver publish_grasps_client_worker_node
```

**典型工作流程**：调用 `/plan_trajectory` 规划 → 获取轨迹 → 调用 `/execute_trajectory` 执行。

---

## 脚本

| 脚本 | 说明 |
|------|------|
| `moveit2_tcp_pose_publisher.py` | 发布 TCP 位姿 |
| `verify_kinematics.py` | 运动学验证 |
| `test_movel_service.py` | Movel 服务测试 |

---

## 相关文档

- [docs/GRASP_CALL_FLOW.md](docs/GRASP_CALL_FLOW.md) — GraspNet 抓取放置调用逻辑
- [docs/GRASP_MOTION_CPP_PORT_FINAL_PLAN.md](docs/GRASP_MOTION_CPP_PORT_FINAL_PLAN.md) — 抓取运动 C++ 移植规划
- [docs/MOVE_TO_POSE_AND_ERROR_MINUS4_ANALYSIS.md](docs/MOVE_TO_POSE_AND_ERROR_MINUS4_ANALYSIS.md) — move_to_pose 错误分析
