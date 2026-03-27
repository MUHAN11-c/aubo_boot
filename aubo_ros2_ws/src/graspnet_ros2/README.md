# GraspNet ROS2 功能包

在 ROS2 中运行 GraspNet 基线推理：6-DOF 抓取预测、Marker/TF/`PoseArray` 发布，以及与 MoveIt2、手眼标定、现场 IVG 流水线的集成说明。

## 目录

- [功能一览](#功能一览)
- [编译与安装](#编译与安装)
- [Demo：文件 RGB-D（graspnet_demo_node）](#demo文件-rgb-dgraspnet_demo_node)
- [点云推理（graspnet_demo_points_node）与 TF Launch](#点云推理graspnet_demo_points_node与-tf-launch)
- [实时节点 graspnet_node](#实时节点-graspnet_node)
- [图像保存 image_saver](#图像保存-image_saver)
- [发布服务 `/publish_grasps`（Demo 手动/自动模式）](#发布服务-publish_graspsdemo-手动自动模式)
- [GraspNet → ROS2 坐标系转换](#graspnet--ros2-坐标系转换)
- [坐标转换与 TF 验证](#坐标转换与-tf-验证)
- [包内测试（test.launch.py）](#包内测试testlaunchpy)
- [IDE / Pyright 导入路径](#ide--pyright-导入路径)
- [话题与数据格式摘要](#话题与数据格式摘要)
- [目录结构](#目录结构)
- [其他文档（未并入本文）](#其他文档未并入本文)
- [许可证](#许可证)
- [近期改动记录（2026-03）](#近期改动记录2026-03)

---

## 功能一览

| 能力 | 入口 |
|------|------|
| 从目录读取 color/depth/mask/meta，推理并发布 | `ros2 launch graspnet_ros2 graspnet_demo.launch.py` |
| 订阅 `PointCloud2`，循环/采集推理，发布 `grasp_poses_base` 等 | `ros2 launch graspnet_ros2 graspnet_demo_points.launch.py`、`graspnet_demo_points_with_tf.launch.py` |
| 订阅点云（简化实时节点） | `ros2 run graspnet_ros2 graspnet_node` |
| 订阅相机话题存图 | `ros2 run graspnet_ros2 image_saver` |
| 手眼静态 TF、相机链路 | `hand_eye_static_tf_node`，见各 points launch |
| 连通性自检 | `ros2 launch graspnet_ros2 test.launch.py` |

现场 IVG 全栈常配合：`graspnet_demo_points_with_tf.launch.py`（`launch_camera:=false` 时点云由外部提供）。

---

## 编译与安装

### 依赖

```bash
# ROS2（将 <distro> 换为 foxy/humble 等）
sudo apt-get install ros-<distro>-rclpy ros-<distro>-sensor-msgs \
  ros-<distro>-visualization-msgs ros-<distro>-geometry-msgs

# Python（建议在所用 conda/venv 中安装）
pip install torch open3d scipy Pillow numpy
```

### 编译与 source

```bash
cd <你的_ws>   # 例如 ~/IVG2.0/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

### 路径与安装说明

- `setup.py` 会将 `graspnet-baseline/` 下文件安装到 `install/graspnet_ros2/share/graspnet_ros2/graspnet-baseline/`（排除 `.git`、`__pycache__`、`.so` 等；扩展模块需在环境中单独编译/安装）。
- 节点优先使用源码旁 `graspnet-baseline`，否则使用上述 share 路径（见各节点内 `_get_graspnet_baseline_root` 一类逻辑）。

### 验证安装

```bash
ls install/graspnet_ros2/share/graspnet_ros2/graspnet-baseline/
ros2 run graspnet_ros2 graspnet_demo_node --ros-args -p data_dir:=/path/to/data
```

### 常见问题

- **找不到 baseline**：重新 `colcon build`，确认 `source install/setup.bash`；必要时 `export GRASPNET_BASELINE_DIR=/path/to/graspnet-baseline`。
- **找不到权重**：默认 `graspnet-baseline/logs/log_kn/checkpoint-rs.tar`；可用 `-p model_path:=...` 指定。

---

## Demo：文件 RGB-D（graspnet_demo_node）

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py
# 或
ros2 launch graspnet_ros2 graspnet_demo.launch.py data_dir:=/path/to/data
```

### trigger_grasp 服务

在相机与节点同时运行时，可触发软触发、保存图、生成 mask/meta、推理并发布：

```bash
ros2 service call /graspnet_demo_node/trigger_grasp std_srvs/srv/Trigger
```

内参优先从手眼 YAML 的 `camera_matrix` 读取，否则从 `camera_info_topic`。

### graspnet_demo_node 主要参数

- `model_path`：权重（默认 `graspnet-baseline/logs/log_kn/checkpoint-rs.tar`）
- `data_dir`：数据目录
- `num_point` / `num_view` / `collision_thresh` / `voxel_size` / `max_grasps_num` / `gpu`
- `marker_topic` / `frame_id` / `use_open3d`
- `trigger_service`、`camera_id`、彩色/深度/相机信息话题、`factor_depth`（如 Percipio 0.25mm 常用 `4000`）

### 数据目录文件

- `color.png`、`depth.png`（16 位）、`workspace_mask.png`
- `meta.mat`：`intrinsic_matrix`、`factor_depth`

---

## 点云推理（graspnet_demo_points_node）与 TF Launch

```bash
# 点云版 + 可选相机与手眼 TF
ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py

# IVG 常用：不启包内相机，点云由外部提供；发布手眼 TF
ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py \
  launch_camera:=false launch_hand_eye_tf:=true
```

默认输入点云话题：`/camera/depth_registered/points`；默认权重：`baseline_dir/logs/log_kn/checkpoint-rs.tar`。

要点（与下游 `demo_driver` 对齐）：

- 定时循环推理，参数如 `compute_interval_sec`；采集由 `/graspnet_capture_control`（`std_srvs/SetBool`）控制。
- 发布 `grasp_poses_base`（`geometry_msgs/PoseArray`，`frame_id` 默认 `base_link`，可由 `base_frame` 改）。
- 动态 TF：`camera_frame` → `grasp_pose_i`。

---

## 实时节点 graspnet_node

```bash
ros2 run graspnet_ros2 graspnet_node
```

订阅 `PointCloud2`（默认话题见节点实现），实时预测并发布 Marker 等（依赖环境与话题配置）。

---

## 图像保存 image_saver

```bash
ros2 run graspnet_ros2 image_saver
```

订阅相机话题并保存图像（参数见节点内声明）。

---

## 发布服务 `/publish_grasps`（Demo 手动/自动模式）

适用于 **`graspnet_demo_node`** 场景：通过服务触发一次推理结果的发布（与当前 **点云版** 的定时循环 + `PoseArray` 流程不同）。

### 手动模式（推荐，便于 RViz 慢启动）

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false
```

节点加载模型与数据后等待服务，不会自动发布。

RViz2 中建议：`Fixed Frame` 为 `world` 或 `base_link`；添加 `MarkerArray` → `/grasp_markers`，`PointCloud2` → `/graspnet_pointcloud`。

就绪后触发：

```bash
ros2 run graspnet_ros2 publish_grasps_client
# 或
ros2 service call /publish_grasps std_srvs/srv/Trigger
# 或 scripts/publish_grasps.sh
```

### 自动模式

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=true
```

启动约 1 秒后自动计算并发布一次（RViz 未就绪时可能看不到首次可视化）。

### 服务行为

- 类型：`std_srvs/Trigger`
- **首次调用**：计算抓取并发布 MarkerArray、点云等
- **再次调用**：直接发布缓存结果，不重新计算（需重新计算请重启节点）

### Launch 常用参数示例

| 参数 | 默认 | 说明 |
|------|------|------|
| `auto_run` | `false` | 是否启动后自动跑一轮 |
| `use_open3d` | 视 launch | Open3D 窗口 |
| `max_grasps_num` | 视 launch | 发布抓取个数 |
| `data_dir` | 自动检测 | 输入数据目录 |

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py \
  auto_run:=false use_open3d:=false max_grasps_num:=5
```

### 调试

```bash
ros2 service list | grep publish_grasps
ros2 node info /graspnet_demo_node
```

---

## GraspNet → ROS2 坐标系转换

GraspNet 与 ROS 工具坐标约定不同，需在 TF / Marker 中统一为「ROS 习惯」：**Z = approach（接近方向）**。

### 定义对照

- **GraspNet** `rotation_matrix` 列：col0 = approach，col1 = width，col2 = height。
- **ROS 末端常用**：X = width，Y = height，Z = approach。

### 转换（与代码一致）

```python
R_graspnet = grasp.rotation_matrix  # (3, 3)
R_ros = np.column_stack([
    R_graspnet[:, 1],  # ROS X = width
    R_graspnet[:, 2],  # ROS Y = height
    R_graspnet[:, 0],  # ROS Z = approach
])
```

实现位置（文件模式 Demo）：`graspnet_demo_node.py` 中 `publish_grasp_tf()`、`create_grasp_markers()` 等（行号随版本变动，请搜索函数名）。

**注意**：下游控制凡基于已发布 TF / `PoseArray` 的，应与上述一致；若只改一处会导致「RViz 对但臂错」之类问题。

---

## 坐标转换与 TF 验证

1. 启动 Demo（示例）：
   ```bash
   cd <你的_ws> && source /opt/ros/<distro>/setup.bash && source install/setup.bash
   ros2 launch graspnet_ros2 graspnet_demo.launch.py use_open3d:=true
   ```
2. RViz2：显示 TF，检查 `grasp_pose_0` — **蓝色 Z 轴应沿手指伸出（approach）**，与 Marker 夹爪一致。
3. 命令行查看变换：
   ```bash
   ros2 run tf2_ros tf2_echo base_link grasp_pose_0
   ros2 run tf2_tools view_frames
   ```
4. 机械臂联调：调用 `publish_grasps_client`（或当前系统使用的抓取接口），确认规划/执行姿态合理。

若姿态异常：先查手眼 `wrist3_Link`（或你的 `ee_frame_id`）→ `camera_frame`，再沿链查 `grasp_pose_0`。

---

## 包内测试（test.launch.py）

```bash
cd <你的_ws>
colcon build --packages-select graspnet_ros2
source install/setup.bash

# 基础：路径、模型加载、数据读取
ros2 launch graspnet_ros2 test.launch.py

# 含推理（需 GPU）
ros2 launch graspnet_ros2 test.launch.py test_prediction:=true

# 指定路径
ros2 launch graspnet_ros2 test.launch.py \
  model_path:=/path/to/checkpoint-rs.tar \
  data_dir:=/path/to/data

# RViz
ros2 launch graspnet_ros2 test.launch.py use_rviz:=true
```

也可直接：

```bash
ros2 run graspnet_ros2 graspnet_test_node --ros-args \
  -p test_model_load:=true -p test_data_read:=true -p test_prediction:=false
```

**Launch 参数摘要**：`test_model_load`、`test_data_read`、`test_prediction`、`model_path`、`data_dir`、`use_rviz`、`rviz_config`。

测试节点失败时：确认已编译并 source、baseline 与权重路径、`doc/pose_1` 或自定义数据目录下四件套文件齐全。

仓库内另有 `test_quick.sh` 可配合使用。

---

## IDE / Pyright 导入路径

`models.graspnet`、`graspnetAPI` 等位于 `graspnet-baseline/`，运行时由 `sys.path` 注入，静态分析需手动加路径。

**pyrightconfig.json**（路径按工作区调整）：

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages"
  ]
}
```

**.vscode/settings.json** 示例：

```json
{
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "<绝对路径>/src/graspnet_ros2/graspnet-baseline"
  ],
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "<绝对路径>/src/graspnet_ros2/graspnet-baseline"
  ]
}
```

将 `humble` / `python3.10` 换成本机 distro 与 Python 版本。修改后 `Developer: Reload Window`。

---

## 话题与数据格式摘要

### graspnet_node

- 订阅：`/pointcloud`（`sensor_msgs/PointCloud2`，以节点参数为准）
- 发布：`/grasp_markers`（`visualization_msgs/MarkerArray`，默认名可配置）

### 点云版节点

- 订阅：参数 `input_pointcloud_topic`
- 发布：`marker_topic`、`grasp_poses_topic`（默认 `grasp_poses_base`）、动态 TF 等

---

## 目录结构

```
graspnet_ros2/
├── graspnet_ros2/           # Python 模块（节点、运动客户端等）
├── graspnet-baseline/       # 模型与基线代码（随包安装到 share）
├── launch/
├── config/
├── scripts/
├── doc/                     # 独立文档，未并入本 README
├── resource/
├── test/
├── package.xml
├── setup.py
└── README.md
```

---

## 其他文档（未并入本文）

- **本包 `doc/`**：如仿真与 RViz 参考等，保持独立，不合并进本文件。
- **`graspnet-baseline/`** 及其子目录若含 **`doc/`**（如数据集说明、自定义推理等），仍留在原路径，请参阅对应 Markdown。

---

## 许可证

MIT License

---

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

### TF 诊断（文件 Demo Launch 场景）

```text
ros2 run tf2_ros tf2_echo base_link wrist3_Link
ros2 run tf2_ros tf2_echo wrist3_Link camera_frame
ros2 run tf2_ros tf2_echo camera_frame grasp_pose_0
ros2 run tf2_tools view_frames
```

详见上文 [坐标转换与 TF 验证](#坐标转换与-tf-验证)。
