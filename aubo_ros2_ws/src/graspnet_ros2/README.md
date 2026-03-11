# GraspNet ROS2 功能包

GraspNet ROS2 功能包，提供 6 自由度抓取位姿预测功能。

## 功能

- **graspnet_demo_node**: 从文件读取数据（color.png, depth.png, workspace_mask.png, meta.mat），预测抓取并发布 MarkerArray；提供 `trigger_grasp` 服务，可触发拍照、保存到 data_dir、执行预测并发布点云与 MarkerArray
- **graspnet_node**: 订阅 PointCloud2 话题，实时预测抓取（需要 ros_numpy 支持）
- **image_saver**: 订阅相机话题并保存图像

## 安装

1. 确保已安装 ROS2 和相关依赖：
```bash
sudo apt-get install ros-<distro>-rclpy ros-<distro>-sensor-msgs ros-<distro>-visualization-msgs
```

2. 安装 Python 依赖：
```bash
pip install torch open3d scipy Pillow numpy
```

3. 编译包：
```bash
cd ~/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

## 使用方法

### 1. 使用 Demo 节点（从文件读取数据）

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py
```

或指定数据目录：

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py data_dir:=/path/to/data
```

### 1.1 触发拍照服务（trigger_grasp）

当相机节点（如 Percipio）和 graspnet_demo_node 同时运行时，可调用 `trigger_grasp` 服务实现：

1. 调用 SoftwareTrigger 触发相机拍照  
2. 保存 color.png、depth.png 到 data_dir  
3. 生成 workspace_mask.png、meta.mat  
4. 执行模型预测并发布点云与 MarkerArray  

```bash
ros2 service call /graspnet_demo_node/trigger_grasp std_srvs/srv/Trigger
```

需确保 data_dir 已设置，且相机话题（color_image_topic、depth_image_topic）已发布。相机内参优先从 hand_eye_yaml_path 手眼标定文件的 camera_matrix 读取，若无则从 camera_info_topic 获取。

### 2. 使用实时节点（订阅点云话题）

```bash
ros2 run graspnet_ros2 graspnet_node
```

### 3. 保存图像

```bash
ros2 run graspnet_ros2 image_saver
```

## 参数说明

### graspnet_demo_node 参数

- `model_path`: 模型权重文件路径（默认：graspnet-baseline/logs/log_kn/checkpoint-rs.tar）
- `data_dir`: 数据目录路径（默认：graspnet-baseline/doc/pose_1）
- `num_point`: 输入网络的点数（默认：20000）
- `num_view`: GraspNet 视角数量（默认：300）
- `collision_thresh`: 碰撞检测阈值（默认：0.01）
- `voxel_size`: 体素下采样大小（默认：0.01）
- `max_grasps_num`: 发布的最大抓取数量（默认：20）
- `gpu`: GPU 设备编号（默认：0）
- `marker_topic`: MarkerArray 话题名称（默认：grasp_markers）
- `frame_id`: 坐标系名称（默认：camera_frame）
- `use_open3d`: 是否启用 Open3D 可视化（默认：false）
- `trigger_service`: 相机软触发服务名（默认：/software_trigger）
- `camera_id`: 相机 ID（默认：207000152740）
- `color_image_topic`: 彩色图话题（默认：/camera/color/image_raw）
- `depth_image_topic`: 深度图话题（默认：/camera/depth/image_raw）
- `camera_info_topic`: 相机内参话题（默认：/camera/color/camera_info）
- `factor_depth`: 深度缩放因子，Percipio 0.25mm 用 4000，毫米用 1000（默认：4000.0）

## 话题

### 订阅话题

- `/pointcloud` (sensor_msgs/PointCloud2): 点云数据（graspnet_node）

### 发布话题

- `/grasp_markers` (visualization_msgs/MarkerArray): 抓取可视化标记

## 数据格式要求

### 文件数据格式（graspnet_demo_node）

数据目录应包含以下文件：

- `color.png`: RGB 彩色图像
- `depth.png`: 深度图（16 位 PNG）
- `workspace_mask.png`: 工作空间二值掩码
- `meta.mat`: MATLAB 文件，包含：
  - `intrinsic_matrix`: 3x3 相机内参矩阵
  - `factor_depth`: 深度缩放因子

## 依赖

- ROS2 (rclpy, sensor_msgs, visualization_msgs, geometry_msgs)
- PyTorch
- Open3D
- NumPy
- SciPy
- Pillow (PIL)
- graspnetAPI (包含在 graspnet-baseline 中)

## 目录结构

```
graspnet_ros2/
├── graspnet_ros2/          # Python 模块
│   ├── __init__.py
│   ├── image_saver.py      # 图像保存节点
│   ├── graspnet_node.py    # 实时抓取预测节点
│   └── graspnet_demo_node.py  # Demo 节点（从文件读取）
├── graspnet-baseline/      # GraspNet 基线代码
│   ├── models/             # 模型定义
│   ├── utils/              # 工具函数
│   ├── dataset/            # 数据集处理
│   └── ...
├── launch/                 # Launch 文件
│   └── graspnet_demo.launch.py
├── package.xml             # ROS2 包配置
└── setup.py                # Python 包配置
```

## 许可证

MIT License

## 近期改动记录（2026-03）

本节记录本仓当前抓取链路的关键变更，便于现场排障与参数调优。

### 1) `graspnet_demo_points_node.py`：由“服务触发”改为“定时循环”

- 旧流程：通过 `/publish_grasps`（Trigger）手动触发一次推理与发布。
- 新流程：使用定时器循环执行 `点云输入 -> 推理 -> 碰撞检测 -> NMS -> 发布`，无需服务调用。
- 新增参数：
  - `compute_interval_sec`：循环间隔（秒），默认 `1.0`
  - `base_frame`：抓取位姿转换目标坐标系，默认 `base_link`
  - `grasp_poses_topic`：发布抓取位姿话题，默认 `grasp_poses_base`

### 2) TF 发布改为动态 TF

- 旧方式：静态 TF 广播器。
- 新方式：动态 TF 广播器 `TransformBroadcaster`，循环发布：
  - `camera_frame -> grasp_pose_i`
- 作用：每轮推理结果都会更新时间戳，TF 与当前点云更一致。

### 3) 新增抓取位姿话题（客户端不再从 TF 查抓取）

为降低动态 TF 查询偶发失败带来的抖动，`graspnet_demo_points_node.py` 在发布 Marker/TF 后，额外发布：

- 话题：`grasp_poses_base`
- 消息类型：`geometry_msgs/PoseArray`
- 约定：
  - `header.frame_id = base_link`（可由 `base_frame` 参数修改）
  - `poses[]` 为各抓取位姿（与 `grasp_pose_i` 等价的 base 系表达）

节点内部做法：
- 先查 `base_frame -> camera_frame`（最新可用变换）
- 再将每个 grasp 的相机系位姿变换到 `base_frame`
- 最终发布 `PoseArray`

### 4) `publish_grasps_client.py`：抓取位姿全部来自话题

- 旧流程：客户端等待并查询 TF（`base_link -> grasp_pose_i`）。
- 新流程：客户端订阅 `PoseArray`，不再查询抓取 TF。
- 保留 TF 的唯一用途：`run_grasp_approach(...)` 内部获取当前末端位姿（如 `base_link -> tool_tcp`）。

新增参数：
- `grasp_poses_topic`：抓取位姿输入话题，默认 `grasp_poses_base`
- `wait_poses_timeout_sec`：等待位姿话题超时，默认 `30.0`

### 5) 客户端新增“最近多组窗口选优”

问题背景：点云实时变化，最新一组抓取可能不是最优。

新增策略：缓存最近 N 组抓取位姿，在窗口内全量评分，选择垂直度最高的抓取再执行运动。

新增参数：
- `grasp_window_size`：窗口组数，默认 `5`
- `min_groups_before_pick`：至少累计多少组后再选择，默认 `3`

实现要点：
- 回调中把非空 `PoseArray` 追加到 `deque(maxlen=grasp_window_size)`
- `run()` 中先等待窗口满足 `min_groups_before_pick`
- 在最近窗口内按 `_verticality_score` 选择最优抓取

### 6) `grasp_motion_controller.py`：极简运动接口（对标 C++）

运动控制模块已重构为三个公开函数，调用方只需传最少业务输入：

1. `move_to_pose(node, pose)`  
   - 关节空间到位姿（等价 C++ `moveToPose`）。
2. `run_arc_path_sequence(node, segments)`  
   - 多段笛卡尔路径一次规划一次执行（等价 C++ `runArcPathSequence`）。
3. `run_grasp_approach(node, pose_ee, height_above=0.05)`  
   - 抓取业务封装：`XY -> 姿态 -> Z`。

其余配置（group/base_frame/ee_link/容差/速度/阈值/重试）全部收敛到模块默认常量，不再作为函数参数对外暴露。

### 7) MoveIt2 action/service 流程与参数含义

当前实现统一采用 action/service 调用链：
- 关节空间：`MoveGroup` action（`/move_action`）
- 笛卡尔路径：`GetCartesianPath` service（`/compute_cartesian_path`）+ `ExecuteTrajectory` action（`/execute_trajectory`）

#### 7.1 关节空间到位姿（`move_to_pose`）

- 当前调用链（action）：
  1. 构造 `MotionPlanRequest`（`start_state`、`group_name`、`goal_constraints` 等）
  2. 构造 `PlanningOptions`（`plan_only=False`）
  3. 发送 `MoveGroup.Goal` 到 `/move_action`
  4. 等待结果并按 `error_code` 判断成功/失败

- 关键参数说明（`MotionPlanRequest`）：
  - `start_state`：规划起点；当前传空 `RobotState()`，由 `move_group` 使用其当前状态
  - `group_name`：规划组（默认 `manipulator`）
  - `goal_constraints`：位姿约束（位置球体 + 姿态容差）
  - `num_planning_attempts`：规划尝试次数
  - `allowed_planning_time`：规划时间预算（秒）
  - `max_velocity_scaling_factor` / `max_acceleration_scaling_factor`：速度/加速度缩放

#### 7.2 笛卡尔路径（`run_arc_path_sequence` / `run_grasp_approach`）

- 当前调用链（服务 + action）：
  1. `GetCartesianPath` 服务：`/compute_cartesian_path`
  2. `ExecuteTrajectory` action：`/execute_trajectory`

- 关键参数说明（`GetCartesianPath.Request`）：
  - `header.frame_id`：waypoints 所在参考系（当前默认 `base_link`）
  - `start_state`：起始机器人状态（当前使用空 `RobotState()`，由 MoveIt 当前状态解析）
  - `group_name`：规划组（默认 `manipulator`）
  - `link_name`：笛卡尔插值末端 link（默认 `tool_tcp`）
  - `waypoints`：笛卡尔路径关键点序列
  - `max_step`：笛卡尔插值步长（米），越小点越密
  - `jump_threshold` / `prismatic_jump_threshold` / `revolute_jump_threshold`：关节跳变约束
  - `avoid_collisions`：是否开启碰撞检测

- 关键参数说明（`ExecuteTrajectory.Goal`）：
  - `trajectory`：待执行的 `RobotTrajectory`

### 8) 回退策略与姿态一致性

- 为提升执行稳定性，笛卡尔路径在以下情况自动回退 `move_to_pose`：
  1. `fraction < 1.0`
  2. 轨迹点数过多（`CARTESIAN_MAX_POINTS_FOR_EXECUTION = 60`）
- 抓取流程默认启用 `DEFAULT_FLIP_GRASP_Z_180`，回退到关节空间时会做对应姿态处理，避免末端多转 180°。

### 9) 建议启动顺序

1. 启动 `graspnet_demo_points.launch.py`（含相机、GraspNet 节点、move_group）
2. 等待 `grasp_poses_base` 开始稳定发布
3. 启动：
   - `ros2 run graspnet_ros2 publish_grasps_client`
4. 按现场效果调整：
   - `compute_interval_sec`
   - `grasp_window_size`
   - `min_groups_before_pick`
   - `height_above` / `grasp_z_offset`
