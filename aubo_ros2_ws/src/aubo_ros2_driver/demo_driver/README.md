# demo_driver

机器人驱动功能包（ROS 2），提供机器人状态发布、运动控制、位姿设置与 GraspNet 循环抓取等功能。

> **注意**：夹爪快换功能已迁移至 `tool_changer` 包（`tool_changer_interface` 接口定义）。

## 第一节：启动命令与服务测试命令

### 1) 启动命令

```bash
# 统一环境（每个终端都先执行）
cd ~/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```bash
# 终端1：MoveIt2 + 机械臂（先启动）
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py
```

```bash
# 终端2：demo_driver 基础服务（/move_to_pose /plan_trajectory /execute_trajectory /get_current_state /set_speed_factor /set_robot_pose）
ros2 launch aubo_moveit_config demo_driver_services.launch.py
```

```bash
# 终端3（可选）：单次抓取 Worker（/execute_single_grasp /loop_grasp_control）
ros2 launch demo_driver execute_grasp_pose_worker.launch.py
```

```bash
# 终端4（可选）：GraspNet 循环抓取 Worker（默认服务 /publish_grasps_worker_loop_control）
ros2 run demo_driver publish_grasps_client_worker_node --ros-args \
  -p grasp_capture_service_name:=/graspnet_capture_control \
  -p loop_control_service_name:=/publish_grasps_worker_loop_control \
  -p auto_start_loop:=false \
  -p min_groups_before_pick:=3
```

```bash
# 终端5（可选）：夹爪快换 Worker（/run_gripper_swap），已迁移至 tool_changer 包
ros2 launch tool_changer gripper_swap_worker.launch.py
```

### 2) 各服务测试命令（按当前 src 实现）

```bash
# 先查看服务是否就绪
ros2 service list | grep -E "/move_to_pose|/plan_trajectory|/execute_trajectory|/get_current_state|/set_speed_factor|/set_robot_pose|/execute_single_grasp|/loop_grasp_control|/publish_grasps_worker_loop_control"
```

```bash
# /move_to_pose (demo_interface/srv/MoveToPose)
ros2 service call /move_to_pose demo_interface/srv/MoveToPose \
"{target_pose: {position: {x: 0.40, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}, target_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], use_joints: false, velocity_factor: 0.3, acceleration_factor: 0.2}"
```

```bash
# /plan_trajectory (demo_interface/srv/PlanTrajectory)
ros2 service call /plan_trajectory demo_interface/srv/PlanTrajectory \
"{target_pose: {position: {x: 0.40, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}, use_joints: false}"
```

```bash
# /execute_trajectory (demo_interface/srv/ExecuteTrajectory)
# 建议：将 /plan_trajectory 返回的 trajectory 原样填入后再执行
ros2 service call /execute_trajectory demo_interface/srv/ExecuteTrajectory \
"{trajectory: {joint_names: [], points: []}}"
```

```bash
# /get_current_state (demo_interface/srv/GetCurrentState)
ros2 service call /get_current_state demo_interface/srv/GetCurrentState "{}"
```

```bash
# /set_speed_factor (demo_interface/srv/SetSpeedFactor)
ros2 service call /set_speed_factor demo_interface/srv/SetSpeedFactor "{velocity_factor: 0.5}"
```

```bash
# /set_robot_pose (demo_interface/srv/SetRobotPose)
# target_pose 含义: [x, y, z, roll, pitch, yaw]
ros2 service call /set_robot_pose demo_interface/srv/SetRobotPose \
"{target_pose: [0.40, 0.00, 0.30, 3.14159, 0.0, 1.5708], use_joints: false, is_radian: true, velocity: 0.3}"
```

```bash
# /run_gripper_swap（已迁移至 tool_changer_interface，需先启动 tool_changer 节点）
ros2 service call /run_gripper_swap tool_changer_interface/srv/RunGripperSwap "{direction: 'gripper2'}"
```

```bash
# /execute_single_grasp (demo_interface/srv/ExecuteGraspPose)
# use_visual_estimation=true 时会调用 /estimate_pose（interface/srv/EstimatePose）
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose \
"{object_id: '3211242785', use_visual_estimation: true}"
```

```bash
# /loop_grasp_control (std_srvs/srv/SetBool) - execute_grasp_pose_worker 循环控制
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}"   # 开始
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"  # 停止（当前周期后停）
```

```bash
# /publish_grasps_worker_loop_control (std_srvs/srv/SetBool) - publish_grasps_client_worker 循环控制
ros2 service call /publish_grasps_worker_loop_control std_srvs/srv/SetBool "{data: true}"   # 开始
ros2 service call /publish_grasps_worker_loop_control std_srvs/srv/SetBool "{data: false}"  # 停止（当前周期后停）
```

**调用逻辑**：详见 [docs/GRASP_CALL_FLOW.md](docs/GRASP_CALL_FLOW.md)

## 包结构

```
demo_driver/
├── include/demo_driver/          # 头文件
│   ├── moveit_gripper_io_base.h  # 基类：MoveIt + 夹爪 IO
│   ├── publish_grasps_client_worker.h
│   ├── publish_grasps_AB.h
│   ├── set_robot_pose_server.h
│   └── ...
├── src/                          # 源文件
├── launch/                       # Launch 与辅助资源（现场推荐用 aubo_moveit_config 的 demo_driver_services.launch.py）
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
| move_to_pose_server | move_to_pose_server_node | 移动到目标位姿服务（关节/笛卡尔空间） |
| plan_trajectory_server | plan_trajectory_server_node | 轨迹规划服务（不执行） |
| execute_trajectory_server | execute_trajectory_server_node | 执行轨迹服务 |
| get_current_state_server | get_current_state_server_node | 获取当前状态服务 |
| set_speed_factor_server | set_speed_factor_server_node | 设置速度因子服务 |
| set_robot_pose_server | set_robot_pose_server_node | 设置机器人位姿服务（欧拉角/关节空间） |
| publish_grasps_client_worker | publish_grasps_client_worker_node | GraspNet 循环抓取放置 |
| publish_grasps_AB | publish_grasps_AB | A/B 工位交替抓取；与上节点独立（默认循环服务 `/publish_grasps_AB_loop_control`，状态 `grasp_place_status_ab`，最终位姿 `/ivg_worker_final_grasp_poses`） |

### 已禁用节点（CMakeLists 中注释）

- `robot_io_status_publisher_node`
- `set_robot_enable_server_node`
- `set_robot_io_server_node`
- `read_robot_io_server_node`
- `movel_server_node`

---

### 1. move_to_pose_server_node

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

### 8. publish_grasps_client_worker_node

**功能**：GraspNet 抓取放置 Worker。订阅 `grasp_poses_base`（PoseArray），循环执行：触发采集 → 等待窗口就绪 → 选优 → gripper_tip 补偿 → 抓取接近 → 抬起 → 放置 → 回安全位。

#### 本次改造总结（服务化循环控制）

本次围绕“降低长期运行占用 + 可远程控制启停”做了两层改造：

- **感知层（graspnet）**：`graspnet_demo_points_node` 从常开推理改为触发式采集。
  - 通过 `/graspnet_capture_control` (`std_srvs/SetBool`) 控制采集开关。
  - 开启后累计 `capture_groups_target` 组有效 PoseArray 自动停采。
- **执行层（worker）**：`publish_grasps_client_worker_node` 从“启动即无限循环”改为“服务控制循环”。
  - 新增 `/publish_grasps_worker_loop_control` (`std_srvs/SetBool`)。
  - `true`：开启循环抓取。
  - `false`：发出关闭请求，**当前周期完成后退出**（若当前空闲则立即退出）。

设计上的关键约束与收益：

1. **数据新鲜性**：每周期开始前清空窗口，只使用本周期新数据。
2. **性能可控**：仅在周期窗口内推理，非采集阶段不做无效计算。
3. **停机可预期**：stop 请求不会粗暴中断机械臂，保证“当前周期收尾后退出”。
4. **线程安全与稳定性**：服务调用使用 `async_send_request + wait_for`，避免已在 executor 中再次 spin 的崩溃。

#### 触发式采集设计思路（重点）

为降低持续推理带来的资源占用，系统由“常开采集”改为“按周期触发采集”：

- `publish_grasps_client_worker_node` 在每个周期开始时，调用 `SetBool(true)` 触发感知采集服务（默认 `/graspnet_capture_control`）。
- `graspnet_demo_points_node` 仅在采集使能状态下运行定时推理并发布抓取位姿。
- 当感知端发布达到目标组数 `capture_groups_target` 后自动停采（`collect_enabled=false`）。
- Worker 在周期结束（成功或失败）时再次调用 `SetBool(false)` 兜底停采，防止异常路径导致采集遗留。

这套设计的核心目标：

1. **降低空闲资源占用**：非采集窗口内不做推理发布。
2. **保证周期数据新鲜性**：每周期先清窗口，再等待新采集数据。
3. **提高故障可恢复性**：失败路径统一停采，避免感知端“跑飞”。

#### 两节点职责边界

- Worker（控制端）负责：
  - 周期编排、运动执行、触发采集/停止采集。
  - 对抓取位姿窗口进行“清理-等待-选优”。
- GraspNet（感知端）负责：
  - 点云推理、姿态发布。
  - 采集会话计数（`collected_groups/target_groups`）与自动停采。

#### 周期时序（简化）

1. Worker 周期开始，`clearGraspWindow()` 清空历史数据。
2. Worker 调用 `/graspnet_capture_control`，`SetBool(true)`。
3. GraspNet 进入采集态，按 `compute_interval_sec` 推理并发布 `grasp_poses_base`。
4. Worker 等待窗口达到 `min_groups_before_pick` 组后选优并执行抓取运动。
5. GraspNet 达到 `capture_groups_target` 自动停采。
6. Worker 在周期尾部再调用 `SetBool(false)` 做兜底。

#### 常见实现陷阱与修复

- **问题**：节点已在 `MultiThreadedExecutor` 中 `spin()`，再调用 `spin_until_future_complete(...)` 等待服务返回，会触发：
  - `Node '...' has already been added to an executor.`
- **原因**：同一个 node 被二次 spin。
- **修复策略**：保持 `async_send_request`，使用 `future.wait_for(timeout)` + `future.get()` 等待结果，不再对该 node 二次 spin。

#### 订阅

- `grasp_poses_base` (geometry_msgs/PoseArray): base_link 下抓取位姿（来自 GraspNet 感知节点）

#### 依赖服务（新增）

- `/graspnet_capture_control` (std_srvs/SetBool):
  - `data=true`：开始采集
  - `data=false`：停止采集

#### 发布

- `grasp_place_status` (std_msgs/String): JSON 格式周期状态（cycle_count, success_count, fail_count 等）

#### 主要参数（可通过 `--ros-args -p` 覆盖）

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| prefer_vertical | bool | true | 选优策略：true 取垂直度最高 |
| grasp_z_offset | double | 0.15 | gripper_tip→end_effector 沿 z 轴补偿 (m) |
| height_above | double | 0.1 | 抓取点上方安全高度 (m) |
| joint_velocity_scaling | float | 1.0 | 速度缩放 [0~1] |
| joint_acceleration_scaling | float | 0.1 | 加速度缩放 [0~1] |
| home_velocity_scaling | float | 0.7 | 回安全位速度缩放 [0~1] |
| home_acceleration_scaling | float | 0.45 | 回安全位加速度缩放 [0~1] |
| grasp_poses_topic | string | grasp_poses_base | 抓取位姿话题 |
| grasp_capture_service_name | string | /graspnet_capture_control | 感知采集控制服务名 |
| grasp_capture_service_timeout_sec | double | 2.0 | 采集控制服务超时 (s) |
| wait_poses_timeout_sec | double | 5.0 | 等待窗口就绪超时 (s) |
| grasp_window_size | int | 5 | 滑动窗口大小 |
| min_groups_before_pick | int | 3 | 至少 M 组后再选优 |
| gripper_io_index | int | 6 | Aubo 夹爪 IO pin 号 |
| lift_offset | double | 0.2 | 抓取后沿 Z 轴抬起高度 (m) |
| place_mode | string | home_offset | "pose"/"joints"/"home_offset" |
| place_offset_y, place_offset_z | double | -0.2, -0.15 | 安全位偏移 (m) |
| cycle_delay_sec | double | 1.0 | 每周期结束后等待 (s) |
| fail_retry_delay_sec | double | 1.0 | 失败后额外等待 (s) |
| max_cycles | int | -1 | 最大周期数，-1 无限循环 |
| cartesian_max_points | int | 50 | 抓取接近笛卡尔轨迹点数上限 |

#### 与 GraspNet 侧参数对齐建议

建议将以下参数保持一致，避免“窗口永远不就绪”或“采集组数不足”：

- Worker：`min_groups_before_pick`
- GraspNet：`capture_groups_target`

推荐初始值：两者都设为 `3`。

#### 推荐启动方式（触发式采集）

```bash
# 终端1：MoveIt/机械臂
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py

# 终端2：GraspNet 点云推理 + TF（触发式采集参数）
ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py \
  capture_groups_target:=3 \
  capture_control_service:=/graspnet_capture_control

# 终端3：抓取 Worker（触发服务名与组数对齐）
ros2 run demo_driver publish_grasps_client_worker_node --ros-args \
  -p grasp_capture_service_name:=/graspnet_capture_control \
  -p min_groups_before_pick:=3

# 终端4：开启循环抓取（服务控制）
ros2 run graspnet_ros2 publish_grasps_worker_loop_control_client --start
```

#### 执行命令（最常用）

```bash
# 1) 启动 MoveIt
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py

# 2) 启动 GraspNet 点云推理（触发式采集）
ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py \
  capture_groups_target:=3 \
  capture_control_service:=/graspnet_capture_control

# 3) 启动 Worker（初始待机，等待 start 命令）
ros2 run demo_driver publish_grasps_client_worker_node --ros-args \
  -p grasp_capture_service_name:=/graspnet_capture_control \
  -p loop_control_service_name:=/publish_grasps_worker_loop_control \
  -p auto_start_loop:=false \
  -p min_groups_before_pick:=3

# 4) 开始循环抓取
ros2 run graspnet_ros2 publish_grasps_worker_loop_control_client --start

# 5) 关闭循环抓取（等待当前周期结束后退出）
ros2 run graspnet_ros2 publish_grasps_worker_loop_control_client --stop
```

#### 联调排障清单

1. 服务存在性：
   - `ros2 service list | rg graspnet_capture_control`
2. 触发功能自检：
   - `ros2 service call /graspnet_capture_control std_srvs/srv/SetBool "{data: true}"`
3. 话题流量检查：
   - `ros2 topic hz /grasp_poses_base`
4. 参数一致性：
   - `min_groups_before_pick == capture_groups_target`
5. 若出现 executor 报错：
   - 检查是否在已 spin 的 node 上调用了 `spin_until_future_complete`（应改为 `future.wait_for`）。
6. 循环控制（新增）：
   - 开启：`ros2 run graspnet_ros2 publish_grasps_worker_loop_control_client --start`
   - 关闭（当前周期结束后退出）：`ros2 run graspnet_ros2 publish_grasps_worker_loop_control_client --stop`

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

#### 方式一：触发式抓取放置（推荐）

按以下顺序启动（与你当前实测流程一致）：

```bash
# 终端 1：MoveIt2 + 机械臂
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py

# 终端 2：GraspNet 点云推理 + TF（触发式采集）
ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py \
  capture_groups_target:=3 \
  capture_control_service:=/graspnet_capture_control

# 终端 3：抓取 Worker
ros2 run demo_driver publish_grasps_client_worker_node --ros-args \
  -p grasp_capture_service_name:=/graspnet_capture_control \
  -p min_groups_before_pick:=3
```

说明：

- `capture_groups_target` 与 `min_groups_before_pick` 建议保持一致（推荐都为 `3`）。
- 若服务名改动，`capture_control_service` 与 `grasp_capture_service_name` 必须同步。

#### 方式二：单独运行节点

```bash
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

# 夹爪快换（已迁移至 tool_changer 包）
ros2 launch tool_changer gripper_swap_worker.launch.py

# GraspNet 抓取放置
ros2 run demo_driver publish_grasps_client_worker_node
```

**典型工作流程**：调用 `/plan_trajectory` 规划 → 获取轨迹 → 调用 `/execute_trajectory` 执行。

---

## ExecuteGraspPoseWorker 服务测试

以下内容原独立文件 `EXECUTE_GRASP_SERVICE_TEST.md`，已并入本文。

### 构建

```bash
cd <你的_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select demo_interface demo_driver
source install/setup.bash
```

### 启动节点

```bash
ros2 launch demo_driver execute_grasp_pose_worker.launch.py
```

### 测试服务

**1. 单次抓取（使用参数常量）**

```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'test_object', use_visual_estimation: false}"
```

**2. 单次抓取（使用视觉估计）**

确保 `/estimate_pose` 服务正在运行，然后：

```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'default', use_visual_estimation: true}"
```

**3. 启动 / 停止循环抓取**

```bash
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}"
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"
```

### 参数说明

节点参数统一使用 **`egp_` 前缀**（与 `automatically_declare_parameters_from_overrides` 配合）。可通过 launch 或 `ros2 run ... --ros-args -p ...` 覆盖，例如：

- `egp_object_id`：循环抓取默认工件 ID，默认 `default`
- `egp_grasp_position` / `egp_grasp_orientation`：抓取位姿
- 其余见 `execute_grasp_pose_worker.cpp` 顶部参数表

### Web「自动抓取」流程

视觉 Web UI 中「自动抓取」会走 **`/api/execute_single_grasp`**，对应 `ExecuteGraspPose`（`use_visual_estimation: true` 时 worker 内会再请求 `/estimate_pose`）。需已启动 `execute_grasp_pose_worker`；桥接请求体可带 **`timeout_sec`**（默认 300，最大 900）。

### 前端集成示例（roslibjs）

```javascript
const executeSingleGraspService = new ROSLIB.Service({
  ros: ros,
  name: '/execute_single_grasp',
  serviceType: 'demo_interface/srv/ExecuteGraspPose'
});
executeSingleGraspService.callService(
  new ROSLIB.ServiceRequest({ object_id: 'default', use_visual_estimation: true }),
  function (result) { /* ... */ }
);
const loopGraspControlService = new ROSLIB.Service({
  ros: ros,
  name: '/loop_grasp_control',
  serviceType: 'std_srvs/srv/SetBool'
});
```

### 日志与排障

```bash
ros2 run rqt_console rqt_console
```

关键日志标签：`[estimatePoseFromVision]`、`[handleExecuteSingleGrasp]`、`[handleLoopGraspControl]`、`[loopGraspThread]`。

- 服务不可用：检查 `ros2 node list` / `ros2 service list | grep grasp`
- 视觉失败：确认 `/estimate_pose` 已注册
- 循环无法启动：查看是否已有循环在运行（日志会提示）

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
