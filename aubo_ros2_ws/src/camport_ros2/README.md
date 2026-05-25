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
cd ~/aubo_boot/aubo_ros2_ws
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

> 注意: `percipio_camera` 已改为直接用 `rclcpp::Publisher<Image>` 发布图像（非 image_transport），不再自动生成 compressed/compressedDepth 等派生话题。如需压缩图可单独起 `image_transport republish` 节点喵~。

## 参考

- 各子包详细文档见 `src/` 下对应的 `README.md`
- 相机型号与参数: `README.md` §0.5