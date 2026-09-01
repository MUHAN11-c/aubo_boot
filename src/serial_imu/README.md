# serial_imu

USB 串口 IMU（QinHeng CH340 `1a86:7523`，0xA4 寄存器协议）。**不是**采摘五包，不进 `harvest_system`，lifecycle 不管。

现行行为以源码为准。栈内摘要：[architecture.md](../../docs/architecture.md) §3 `serial_imu`、[io.md](../../docs/io.md)、[testing.md](../../docs/testing.md)。本文件是本包现场手册。

```
serial_imu/
  serial_imu/imu_node.py      # 节点：串口、话题、TF
  serial_imu/protocol.py      # 无 ROS：切帧、校验、缩放
  config/serial_imu.yaml
  launch/serial_imu.launch.py
  rviz/serial_imu.rviz
  udev/99-imu-usb-serial.rules
```

---

## 1. 查设备

本机 USB 转串口是 CH340，不是主板 `/dev/ttyS*` 占位节点。

```bash
lsusb | grep -i 1a86
# 期望：QinHeng Electronics CH340 serial converter（id 1a86:7523）

ls -l /dev/ttyUSB0 /dev/serial/by-id/ /dev/serial/by-path/
dmesg -T | grep -iE 'ttyUSB|ch341|1a86'
# 期望：ch341-uart converter now attached to ttyUSB0
```

稳定路径（插拔后不要写死 `ttyUSB0`）：

`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`

该芯片没有唯一序列号。节点按 `port` → `port_fallbacks` 找口：`/dev/imu`，再 by-id，再 `/dev/ttyUSB0`。

模组是**主动上报**，不通询也发流。只监听、不写读指令。旧例里的 `A4 03 08 23 D2` 是轮询读寄存器，现行固件不用。

---

## 2. 固定串口名（udev）

```bash
# 安装规则（需先 source install，或从源码拷）
sudo cp $(ros2 pkg prefix serial_imu)/share/serial_imu/udev/99-imu-usb-serial.rules \
  /etc/udev/rules.d/
# 源码副本：src/serial_imu/udev/99-imu-usb-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger /dev/ttyUSB0
ls -l /dev/imu
# 期望：/dev/imu -> ttyUSB0
```

规则按 `idVendor=1a86`、`idProduct=7523` 匹配，`SYMLINK+=imu`，组 `dialout`，模式 `0660`。机器上若有第二块同样 CH340，两条都会变成 `/dev/imu`。

权限：`usermod -aG dialout` **对已经打开的终端无效**。

```bash
sudo usermod -aG dialout $USER
newgrp dialout          # 当前终端立刻带上 dialout，不必注销
groups                  # 须含 dialout
```

`Errno 13 Permission denied` 时节点打不开口。旧版会直接退出，RViz 报 `Fixed Frame [world] does not exist`（`world` 从未发布）。现行：先发静态 `world→imu_link`，口失败则每 2 s 重试，日志提示 `newgrp dialout`。

---

## 3. 协议与解析

实现：`serial_imu/protocol.py`。在字节流里找 `A4 03`，按第 4 字节长度切帧，校验和 = 除末字节外所有字节之和的低 8 位。

典型主动包：起始寄存器 `0x08`，载荷 35 字节（`0x23`），总长 40。

| 字节 | 含义 |
|------|------|
| `A4` | 帧头 |
| `03` | 读功能码（上报沿用） |
| `08` | 起始寄存器 |
| `23` | 载荷 35 字节 |
| 35B | 小端 `<hhhhhhhhhBhhhhhhhh`：acc×3、gyro×3、rpy×3、`uint8` 磁场等级、temp、mag×3、Q0..Q3 |
| 末字节 | 校验 |

缩放（与现场节点一致）：

| 字段 | 公式 | 单位 |
|------|------|------|
| acc | int16 / 2048 × 9.8 | m/s² |
| gyro | int16 / 16.4 × π/180 | rad/s |
| roll/pitch/yaw | int16 / 100 | 度（解包保留，不进 Imu 消息） |
| temp | int16 / 100 | °C |
| mag | int16 / 1000，再 ×1e-4 | 协议高斯 → `MagneticField` 特斯拉 |
| 四元数 | int16 / 10000 | ROS xyzw，**`w=Q0`，`x=Q1`** |

`sensor_msgs/Imu` 发布：`linear_acceleration`、`angular_velocity`（rad/s）、`orientation`（xyzw）。协方差：`data` 用对角小量；`data_raw` 的 `orientation_covariance[0]=-1`（表示无融合姿态）。

---

## 4. ROS 2 集成（imu_tools 惯例）

对齐 [imu_tools](https://index.ros.org/p/imu_tools/) / 常见驱动：raw 给滤波器，融合姿态另发 `imu/data`。本机已有 `imu_filter_madgwick`，**默认不起**（姿态已在模组里）。

QoS：`SensorDataQoS`（best_effort, volatile）。RViz 订 Imu 必须 Reliability = **Best Effort**，默认 Reliable 会收不到。

| 名字 | 类型 | 含义 |
|------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | 模组四元数 + acc/gyro；RViz Imu 插件订这个 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 仅 acc/gyro，`orientation_covariance[0]=-1` |
| `/imu/mag` | `sensor_msgs/MagneticField` | 特斯拉 |
| `/imu/temp` | `sensor_msgs/Temperature` | °C |
| TF `parent→imu_link` | 静态 | 安装位，单位姿态。默认 parent=`world` |
| TF `parent→imu_attitude` | 动态 | 模组四元数，拧模块时这个轴在转 |

`header.frame_id` = `imu_link`。

**不要把姿态写进 `imu_link`。** 姿态只在 `/imu/data.orientation` 和可选的 `imu_attitude`。RViz Imu 插件会按 TF 把姿态变到 Fixed Frame；若 `imu_link` 已经转过一次，看起来像轴拧反、转两次。接手臂：`tf_parent_frame:=base_link`，否则 `world` 和臂的 `base_link` 是两棵树。

口未开时仍发静态 `parent→imu_link`，避免 RViz `Fixed Frame [world] does not exist`。

---

## 5. 启动

缺 Imu 插件时 launch 会提示（不阻止启动）：

```bash
sudo apt install ros-jazzy-imu-tools    # 含 rviz_imu_plugin；madgwick 常已随发行版装上
```

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select serial_imu
source install/setup.bash
sudo usermod -aG dialout $USER
newgrp dialout
ros2 launch serial_imu serial_imu.launch.py
```

日志须有 `已打开 /dev/imu`（或 by-id）。`use_rviz:=false` 只起驱动。

```bash
ros2 topic echo /imu/data --qos-reliability best_effort
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo world imu_attitude
```

和采摘 RViz 叠在一起：

```bash
ros2 launch serial_imu serial_imu.launch.py use_rviz:=false tf_parent_frame:=base_link
```

Fixed Frame 改成 `base_link`。

---

## 6. RViz 里看到的东西

配置：`rviz/serial_imu.rviz`。Fixed Frame = `world`。TF 白名单默认只画 `world` / `imu_link` / `imu_attitude`。

### 6.1 为什么会冒出很多坐标系

RViz **TF** 显示默认会画出当前图里**所有** `/tf`。采摘栈若同时在跑，那些是 URDF **真坐标系**，不是 IMU 垃圾帧：

| 坐标系 | 干什么 |
|--------|--------|
| `base_link` | 臂基座，规划/感知原点 |
| `shoulder_Link` … `wrist3_Link` | 六轴连杆 |
| `camera_link` / `camera_*_optical_frame` | 手眼与点云 |
| `tcp` / `tool_axis` / `sleeve_mouth` | 套袋工具 |
| `world` | IMU 可视化固定参考 |
| `imu_link` | 模组安装位 |
| `imu_attitude` | 模组姿态 |

看 IMU 只需后三个。Displays → TF → Frames → All Enabled 可再勾臂链。只看 IMU 时不要开 `harvest_system`。

### 6.2 `imu_link Axes` 和 `imu_attitude Axes`

都是 RViz **Axes**：在某个 TF 原点画 RGB=XYZ。

| 显示 | 含义 | 拧模块时 |
|------|------|----------|
| `imu_link Axes` | 安装坐标系（静态单位姿态） | **不该转** |
| `imu_attitude Axes` | 把模组四元数画成会动的轴 | **跟着转** |

`imu_attitude` 不进规划、不是 URDF 连杆。TF 显示若已勾这两个 frame，再开两路 Axes 会重叠，可关其一。

### 6.3 Displays → Imu（插件）

`rviz_imu_plugin` 画 `/imu/data`，不是 URDF。

| 项 | 默认 | 含义 |
|----|------|------|
| Topic / Status | `/imu/data`，Best Effort | `N messages received` 才算订上 |
| `fixed_frame_orientation` | 开 | 用 Fixed Frame（`world`）摆姿态；关掉容易像转两次 |
| Box properties | 关 | 小长方体跟 `orientation` 转，表示壳体朝向；scale 单位米 |
| Axes properties | 开 | 插件自己的三色轴，与 `imu_attitude Axes` 同类，会叠 |
| Acceleration | 关 | 画 `linear_acceleration`。静止应接近「天」（约 9.8） |
| Derotate acceleration | 开 | 先用姿态转到世界再画，静止接近世界 +Z；关则画传感器三轴 |

建议：姿态用 **插件 Axes 或 Box 二选一**；核对重力再开 Acceleration。陀螺仪没有单独箭头。

---

## 7. 参数

`config/serial_imu.yaml`，根键 = 节点名 `serial_imu`。launch 可用 `tf_parent_frame:=base_link` 覆盖。

| 参数 | 默认 | 作用 |
|------|------|------|
| `port` | `/dev/imu` | 首选口 |
| `port_fallbacks` | by-id、`ttyUSB0` | 依次试 |
| `baudrate` | 115200 | |
| `frame_id` | `imu_link` | Imu header |
| `read_period_s` | 0.005 | 读串口定时器 |
| `publish_tf` | true | 静态 parent→imu_link |
| `tf_parent_frame` | `world` | 接手臂改 `base_link` |
| `publish_attitude_tf` | true | 动态 parent→imu_attitude |
| `attitude_frame_id` | `imu_attitude` | |

---

## 8. 不负责

采摘调度、MoveIt、底盘 `/scan`、生命周期名单。不替代预留的底盘 IMU。
