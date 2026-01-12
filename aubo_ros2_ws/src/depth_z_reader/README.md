# Depth Z Reader

ROS2 功能包，用于从 percipio 相机获取深度 z 值。

## 功能

- 订阅深度图像话题
- 读取指定像素位置的深度 z 值
- 支持读取图像中心点或指定坐标的深度值
- 发布深度值到 ROS2 话题
- 定期输出深度值到日志

## 使用方法

### 1. 编译

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select depth_z_reader
source install/setup.bash
```

### 2. 运行节点

#### 使用 launch 文件（推荐）

```bash
ros2 launch depth_z_reader depth_z_reader.launch.py
```

#### 使用命令行参数

```bash
ros2 run depth_z_reader depth_z_reader_node \
  --ros-args \
  -p depth_image_topic:=/camera/depth/image_raw \
  -p pixel_x:=-1 \
  -p pixel_y:=-1 \
  -p depth_scale:=0.00025 \
  -p publish_center_depth:=true \
  -p publish_rate:=10.0
```

### 3. 参数说明

- `depth_image_topic` (string, 默认: "/camera/depth/image_raw"): 深度图像话题名称
- `pixel_x` (int, 默认: -1): 要读取的像素 x 坐标，-1 表示图像中心
- `pixel_y` (int, 默认: -1): 要读取的像素 y 坐标，-1 表示图像中心
- `depth_scale` (double, 默认: 0.00025): 深度缩放因子，将原始深度值转换为米。**重要**：此值取决于相机的 `scale_unit`。查看相机启动日志中的 `Depth stream scale unit` 来确定正确的值：
  - 如果 `scale_unit = 0.25`（如 PS800-E1），使用 `0.00025`（默认值）
  - 如果 `scale_unit = 1.0`，使用 `0.001`
  - 如果 `scale_unit = 0.001`，使用 `0.001`
  - 公式：`depth_scale = scale_unit * 0.001`（转换为米）
- `publish_center_depth` (bool, 默认: true): 是否发布中心点深度值到话题
- `publish_rate` (double, 默认: 10.0): 发布深度值的频率（Hz）

### 4. 话题

#### 订阅
- `depth/image_raw` (sensor_msgs/Image): 深度图像话题

#### 发布
- `depth_z/center` (std_msgs/Float32): 中心点深度 z 值（米）

### 5. 示例

#### 读取图像中心点的深度值（默认）

```bash
ros2 launch depth_z_reader depth_z_reader.launch.py
```

#### 读取指定像素位置的深度值

```bash
ros2 launch depth_z_reader depth_z_reader.launch.py \
  pixel_x:=320 \
  pixel_y:=240
```

#### 订阅不同的深度图像话题

如果您的相机使用不同的话题名称，可以通过参数指定：

```bash
ros2 launch depth_z_reader depth_z_reader.launch.py \
  depth_image_topic:=/your_camera/depth/image_raw
```

#### 查看可用的深度话题

在运行节点前，可以先查看系统中可用的深度话题：

```bash
ros2 topic list | grep depth
```

## 依赖

- rclcpp
- sensor_msgs
- cv_bridge
- image_transport
- std_msgs
- geometry_msgs
- OpenCV

