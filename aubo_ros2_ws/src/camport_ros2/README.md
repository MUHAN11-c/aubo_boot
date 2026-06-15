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

---

## 故障排查

### `Open device fail : -1005` (TY_STATUS_DEVICE_ERROR) 无限重试

**现象**：

```
[ERROR] [percipio_device]: Open device fail : -1005
(每 3 秒重复，永不成功)
```

SDK 内部日志可能伴随：
```
Invalid device info!
WriteReg 0x00000a00 0x00000000 failed
Exception occurred while closing device: Destination address required, Error code: 89
```

**根因**：

多网卡主机上，Linux 内核为所有 UP 状态的接口自动创建 `169.254.0.0/16` link-local 路由（APIPA/Zeroconf）。当 WiFi 和以太网同时启用时，SDK 的 `TYOpenDevice()` 基于设备 ID 的内部查找机制会被此路由干扰，导致 GVCP 控制通道握手失败（`GevHeartbeatTimeout` 寄存器 `0x0A00` 写入失败）。

注意以下各项均**正常**，不要被误导：
- `ping 169.254.10.110` ✅（ICMP 可达）
- `TYUpdateInterfaceList()` / `TYGetDeviceList()` ✅（UDP 广播发现正常）
- 手动 GVCP 单播发包 ✅（Python UDP socket 能收到 ACK）

**修复（已实施，2026-06-09）**：

在 `percipio_device.cpp:device_open()` 中增加回退逻辑：

```cpp
// 1. 优先尝试 TYOpenDevice (设备 ID 方式)
status = TYOpenDevice(hIface, deviceId, &handle);

// 2. 失败时回退到 TYOpenDeviceWithIP (IP 直连)
if (status != TY_STATUS_OK && deviceIP && deviceIP[0] != '\0') {
    RCLCPP_WARN_STREAM(..., "falling back to TYOpenDeviceWithIP(" << deviceIP << ")");
    status = TYOpenDeviceWithIP(hIface, deviceIP, &handle);
}
```

改动涉及三个文件：
| 文件 | 改动 |
|------|------|
| `include/percipio_device.h` | `PercipioDevice` 构造函数和 `device_open()` 增加 `deviceIP` 参数；新增 `strDeviceIP` 成员 |
| `src/percipio_device.cpp` | `device_open()` 增加 `TYOpenDeviceWithIP` 回退逻辑；`Reconnect()` 传递 IP 参数 |
| `src/percipio_camera_node_driver.cpp` | `initializeDevice()` 从 `TY_DEVICE_BASE_INFO.netInfo.ip` 传入相机 IP |

**依据**：Percipio SDK 文档 (`TYApi.h:542`) 明确说明：
> "If there is a routing connection between your host and the camera, you can try to open the camera with TYOpenDeviceWithIP."

**临时手动排查命令**：

```bash
# 检查是否有 WiFi 上的 169.254 路由
ip route show | grep "169.254"

# 直接测试 GVCP 单播连通性（若 ACK 可达说明网络层正常）
python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.bind(('169.254.10.11', 0))
s.sendto(b'\x42\x01\x00\x02\x00\x00\x00\x08', ('169.254.10.110', 3956))
try:
    data, addr = s.recvfrom(1024)
    print(f'ACK: {len(data)} bytes from {addr}')
except socket.timeout:
    print('TIMEOUT: GVCP 不可达')
"

# 临时绕过（删除 WiFi link-local 路由，重启后恢复）
sudo ip route del 169.254.0.0/16 dev wlo1
```