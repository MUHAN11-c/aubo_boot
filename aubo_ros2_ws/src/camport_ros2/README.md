# camport_ros2

知微（Percipio）深度相机在 **ROS 2** 下的驱动与上层接口，本目录为**多包工作区子目录**，实际可 colcon 的包位于 `src/` 下。

---

## 目录结构

```
camport_ros2/
└── src/
    ├── percipio_camera/           # 相机节点（图像、点云、动态参数、设备事件等）
    ├── percipio_camera_interface/ # 状态发布、参数设置、软触发等接口节点
    └── image_data_bridge/         # 彩色图 → ImageData，供手眼等模块使用
```

各包详细话题/服务说明见对应包内 **`README.md`**。

---

## 依赖（系统包示例）

```text
ros-${ROS_DISTRO}-image-transport
ros-${ROS_DISTRO}-image-publisher
ros-${ROS_DISTRO}-camera-info-manager
ros-${ROS_DISTRO}-diagnostic-updater
ros-${ROS_DISTRO}-diagnostic-msgs
```

---

## 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select percipio_camera percipio_camera_interface image_data_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 启动相机节点

```bash
ros2 launch percipio_camera percipio_camera.launch.py
```

---

## 常用话题（默认命名空间 `camera`，以实际 launch 为准）

| 话题 | 类型 | 含义 |
|------|------|------|
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色内参与分辨率 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度内参 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 深度坐标系点云 |
| `/camera/depth_registered/points` | `sensor_msgs/PointCloud2` | 配准到彩色系的点云（需启用相应选项） |
| `/camera/left_ir/image_raw` 等 | `sensor_msgs/Image` | 红外等辅助流 |
| `/camera/device_event` | `std_msgs/String` | 设备连接/离线/超时等事件 |

调试时可使用 `ros2 topic list`、`ros2 service list`、`rviz2` 订阅上述话题。

---

*Percipio SDK 与 CMake 中 OpenCV 组件（如 `photo`、`highgui`）需与 `percipio_camera` 的 `CMakeLists.txt` 一致，否则可能出现链接或运行时符号错误。*
原因说明
日志里的 CompressedPublisher 和 compressed_depth_image_transport 来自 image_transport::create_publisher()：会在同一节点上为每个 */image_raw 自动挂 彩色压缩、深度压缩 等插件。Percipio 同时发 彩色 rgb8 和 深度 16UC1 时，ROS2 里经常出现 插件串流/格式错配（深度压缩收到 rgb8、彩色压缩又去处理 16UC1），于是刷屏报错。这与 camport_ros2 / percipio_camera 使用 image_transport 发布多路图像是同一类问题。

修改说明（本地工作区）
在 percipio_camera 中已改为使用普通 rclcpp::Publisher<sensor_msgs::msg::Image> 发布 color/depth/.../image_raw，不再经过 image_transport 插件链，因此不会再拉起上述 CompressedPublisher / compressed_depth。

percipio_camera_node.h：image_transport::Publisher → rclcpp::Publisher<Image>::SharedPtr，去掉无用 include。
percipio_camera_node.cpp：create_publisher<Image>(...)；get_subscription_count()；publish(*image_msg)（与当前 cv_bridge 返回的 shared_ptr 一致）。
CMakeLists.txt：去掉对 image_transport、image_publisher 的依赖（本包内已无引用）。
代价：节点不再自动提供 .../image_raw/compressed、.../compressedDepth 等派生话题；rosbridge / RWT / 订阅 image_raw 的节点不受影响。若你以后必须用网络压缩图，可单独起 image_transport republish 节点，只接在需要的话题上。

已在本机执行 colcon build --packages-select percipio_camera 通过。