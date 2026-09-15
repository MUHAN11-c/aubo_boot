# serial_imu

USB 串口 IMU（QinHeng USB 转串适配器：CH340 `1a86:7523`（旧）或 CH343 `1a86:55d3`（现行），模组走 0xA4 寄存器协议）。**不是**采摘五包，lifecycle 不管。随 `harvest_system` 起（`imu_enabled` 默认 true），不进只读 bringup。

现行行为以源码为准。栈内摘要：[architecture.md](../../docs/architecture.md) §3 `serial_imu`、[io.md](../../docs/io.md)、[testing.md](../../docs/testing.md)（怎么跑）。本文件是本包现场手册。

```
serial_imu/
  serial_imu/imu_node.py      # 节点：串口、原始/修正两路、TF、/diagnostics
  serial_imu/protocol.py      # 无 ROS：切帧、校验、缩放、协方差对角
  serial_imu/frame.py         # 无 ROS：坐标系修正（倒装 Rx）+ parent 对齐
  config/serial_imu.yaml
  launch/serial_imu.launch.py
  rviz/serial_imu.rviz
  test/test_protocol.py
  test/test_frame.py
  udev/99-imu-usb-serial.rules
```

---

## 1. 查设备

本机 USB 转串口是 QinHeng 适配器，不是主板 `/dev/ttyS*` 占位节点。现场出现过两种芯片：

```bash
lsusb | grep -i 1a86
# 期望（二选一）：
# QinHeng Electronics CH340 serial converter（id 1a86:7523，旧线）
# QinHeng Electronics USB Single Serial（id 1a86:55d3，现行）

ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ /dev/serial/by-path/
dmesg -T | grep -iE 'ttyUSB|ttyACM|ch34|cdc_acm|1a86'
# 期望：CH340 → ch341-uart attached to ttyUSB0；CH343 → cdc_acm attached to ttyACM0
```

稳定路径（插拔后不要写死 `ttyUSB0`）：

- CH343（带唯一序列号）：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6060520-if00`
- CH340（无序列号）：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`

节点按 `port` → `port_fallbacks` 找口：`/dev/imu`，再两条 by-id，再 `/dev/ttyUSB0`。

模组是**主动上报**，不通询也发流。只监听、不写读指令。旧例里的 `A4 03 08 23 D2` 是轮询读寄存器，现行固件不用。

---

## 2. 固定串口名（udev）

```bash
sudo cp $(ros2 pkg prefix serial_imu)/share/serial_imu/udev/99-imu-usb-serial.rules \
  /etc/udev/rules.d/
# 源码副本：src/serial_imu/udev/99-imu-usb-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger   # 或 sudo udevadm trigger /dev/ttyACM0 /dev/ttyUSB0
ls -l /dev/imu
# 期望：/dev/imu -> ttyUSB0（CH340）或 /dev/imu -> ttyACM0（CH343）
```

规则按 `idVendor=1a86` 加 `idProduct`（`7523` CH340 或 `55d3` CH343）匹配，`SYMLINK+=imu`，组 `dialout`，模式 `0660`。两种芯片各一条规则，同一时刻只有一条生效。机器上若插第二块同型适配器（或新旧各一块），两条规则会抢 `/dev/imu`，后插的赢。

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

`sensor_msgs/Imu` 协方差：该字段未提供 → `covariance[0]=-1`；未知 → 全 0。默认 `gyro_available: false`（09-09 实测陀螺恒 0），故两路 `angular_velocity_covariance[0]=-1`。禁止用对角小数假装有标定。

`/diagnostics`（`diagnostic_updater`）：串口开闭 + `imu/data` 帧率（期望 `expected_rate_hz` 75，容差窗 0.6×–1.4×）。不进采摘 observability。

数据口径（2026-09-09 实测）：

- **陀螺三轴输出恒为 0**。姿态唯一来源是融合四元数。
- 模组原始静止比力沿 **−Z**（倒装芯片）；`/imu/data` 经 `frame.py` 标准 **Rx(180°)** 后静止比力沿 **+Z**。贴装残差不要写进 `frame_rpy_deg`。
- `mag_level=0`（无磁融合），yaw 靠模组内部陀螺积分：51 s 静止实测总漂移 0.29°。
- 帧率 ≈74.7 Hz；静止 acc 噪声 std ≈0.003 m/s²；|acc|≈9.6。

---

## 4. ROS 2 集成

话题名沿用 [imu_tools](https://index.ros.org/p/imu_tools/)。本机已有 `imu_filter_madgwick`，**默认不起**（姿态已在模组里）。

两路由 `frame.py` 分清，节点只转发：

| 名字 | 类型 | 含义 |
|------|------|------|
| `/imu/data_raw` | `sensor_msgs/Imu` | **原始**：模组体轴，协议原样（含姿态） |
| `/imu/data` | `sensor_msgs/Imu` | **修正**：`frame_rpy_deg`（默认 Rx(180°)）+ 可选 `align_to_parent`。RViz Imu 插件订这个 |
| `/imu/mag` | `sensor_msgs/MagneticField` | 特斯拉，已随修正转到 `imu_link` |
| `/imu/temp` | `sensor_msgs/Temperature` | °C |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 串口 + 帧率 |
| TF `parent→imu_link` | 静态 | 安装位，单位姿态。默认 parent=`world` |
| 服务 `imu/align_to_parent` | `std_srvs/Trigger` | 把当前 IMU↔parent 差当误差清掉；叠 TCP 时用 |

QoS：`Reliable` + `Volatile` + KeepLast 10。`header.frame_id` = `imu_link`。

**不要把姿态写进 `imu_link`。** 姿态只在 `/imu/data.orientation`。RViz Imu 插件会按 TF 把姿态变到 Fixed Frame；若 `imu_link` 已经转过一次，看起来像轴拧反、转两次。接手臂：`tf_parent_frame:=base_link` 或 `tcp`，否则 `world` 和臂的 `base_link` 是两棵树。

口未开时仍发静态 `parent→imu_link`，避免 RViz `Fixed Frame [world] does not exist`。

无磁融合，上电 yaw 任意。**不要**把这个数写进 `frame_rpy_deg`。`align_to_parent` 把当前 IMU↔TCP 差当误差清掉；每次上电或接触前调一次服务（`align_on_start` 默认会自动采）。

---

## 5. 启动

日常跟采摘整栈（mock / real 相同，默认已开 IMU）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
sudo usermod -aG dialout $USER
newgrp dialout
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
# 真机：hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 关掉 IMU：imu_enabled:=false
```

整栈里 IMU 挂在 `tcp` 上并对齐，不起自己的 RViz。画面在 MoveIt RViz 的 **Peach → Imu**（订 `/imu/data`，盒子在 TCP）。不要找 TF 轴里的静止 `imu_link`。再对齐一次：

```bash
ros2 service call /imu/align_to_parent std_srvs/srv/Trigger
```

不要另起 `serial_imu.launch.py` 与整栈并行（预检会拒启）。

只看 IMU、不起采摘。缺 Imu 插件时 launch 会提示（不阻止启动）：

```bash
sudo apt install ros-jazzy-imu-tools    # 含 rviz_imu_plugin
```

```bash
colcon build --packages-select serial_imu
source install/setup.bash
ros2 launch serial_imu serial_imu.launch.py
```

日志须有 `已打开 /dev/imu`（或 by-id）。`use_rviz:=false` 只起驱动。

```bash
ros2 topic echo /imu/data
ros2 topic echo /imu/data_raw
ros2 topic hz /imu/data
ros2 topic echo /diagnostics
```

叠到已有 URDF（不起采摘）：

```bash
ros2 launch serial_imu serial_imu.launch.py use_rviz:=false \
  tf_parent_frame:=tcp align_to_parent:=true
```

Fixed Frame 用 `base_link`。RViz Imu 插件 `fixed_frame_orientation=true`：盒子在 TCP 位置，姿态用对齐后的 `/imu/data`。

纯核（零 ROS）：

```bash
PYTHONPATH=src/serial_imu pytest src/serial_imu/test/test_protocol.py src/serial_imu/test/test_frame.py
```

---

## 6. RViz 里看到的东西

配置：`rviz/serial_imu.rviz`。Fixed Frame = `world`。显示对齐 [imu_tools `rviz_imu_plugin`](https://github.com/CCNYRoboticsLab/imu_tools) 源码视觉默认。

RViz **TF** 显示默认会画出当前图里**所有** `/tf`。本配置白名单只留 `world` / `imu_link`，且 **不画 TF 轴**（避免和插件 Axes 叠）。只看 IMU 时不要开 `harvest_system`。`imu_link` 是正装安装位（静态，姿态不写进此帧）。

### Displays → Imu（`rviz_imu_plugin`）

官方插件订 `/imu/data`，在 **Fixed Frame** 里画姿态（`fixed_frame_orientation=true`）：

| 项 | 本配置 |
|----|--------|
| Topic | `/imu/data`，Reliable |
| `fixed_frame_orientation` | 开 |
| Axes | 开，scale **0.15 m** |
| Box | 开，**0.07×0.10×0.03 m** 灰 |
| Acceleration | 开，scale **0.05**，黄，derotate |

拧模块：RGB 轴和灰盒子跟着转。黄箭头是 **比力**（accelerometer specific force），不是重力向下：静止时桌子往上托，读数指向天空。`Derotate acceleration` 用融合四元数把传感器系读数转到 `world`，姿态与加速度自洽时箭头应接近世界 **+Z**。关掉 Derotate 则沿 `imu_link` 画：正装发布后应沿盒子 **+Z**。

---

## 7. 参数

`config/serial_imu.yaml`，根键 = 节点名 `serial_imu`。launch 可用 `tf_parent_frame:=base_link` 覆盖。

| 参数 | 默认 | 作用 |
|------|------|------|
| `port` | `/dev/imu` | 首选口 |
| `port_fallbacks` | 两条 by-id、`ttyUSB0` | 依次试 |
| `baudrate` | 115200 | |
| `frame_id` | `imu_link` | Imu header |
| `read_period_s` | 0.005 | 读串口定时器 |
| `publish_tf` | true | 静态 parent→imu_link |
| `tf_parent_frame` | `world` | 接手臂改 `base_link` 或 `tcp` |
| `frame_rpy_deg` | `[180, 0, 0]` | 模组体轴→`imu_link`：标准倒装 Rx(180°)。物理正装改 `[0,0,0]`。贴歪/无磁 yaw 不写这里 |
| `align_to_parent` | false | true=把 IMU 相对 `tf_parent_frame` 的当前差当误差清掉（叠 TCP） |
| `align_on_start` | true | `align_to_parent` 时启动自动采一次 |
| `align_reference_frame` | `base_link` | 对齐查 TF：reference→parent |
| `gyro_available` | false | false → 陀螺 `covariance[0]=-1` |
| `angular_velocity_variance` | 0 | 仅 `gyro_available` 时用；0=未知 |
| `linear_acceleration_variance` | 0 | 加速度对角方差；0=未知 |
| `orientation_variance` | 0 | 融合姿态对角方差；0=未知 |
| `magnetic_field_variance` | 0 | 磁场对角方差；0=未知 |
| `expected_rate_hz` | 75 | `/diagnostics` 帧率期望 |

---

## 8. 不负责

采摘调度、MoveIt、底盘 `/scan`、生命周期名单。不替代预留的底盘 IMU。不做自适应工具偏移（`tcp_actual` 臂侧缝仍预留、未实现；2026-09-15 起自适应圆柱 `adaptive_cylinder_v1` 的柔性偏斜由 imu_follow 姿态跟随消化，`tcp_actual` 仍是未来刚性偏移量的缝）。
