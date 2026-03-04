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
