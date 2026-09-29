# serial_imu

USB 串口 IMU（QinHeng USB 转串适配器：CH340 `1a86:7523`（旧）或 CH343 `1a86:55d3`（现行），模组走 0xA4 寄存器协议）。**不是**采摘五包，lifecycle 不管。随 `harvest_system` 起（`imu_enabled` 默认 true），不进只读 bringup。

同包还有咬合末端 5 点力传感器（0xA5 帧，100Hz，见第 8 节）：**与 IMU 帧共用 `/dev/imu` 同一条串口**（2026-09-29 实测），本节点内建分流发 `/force/points`；独立节点 `serial_force_node` 是"只看力"的用法，不进 `harvest_system`。

现行行为以源码为准。栈内摘要：[architecture.md](../../docs/architecture.md) §3 `serial_imu`、[io.md](../../docs/io.md)、[testing.md](../../docs/testing.md)（怎么跑）。本文件是本包现场手册。

```
serial_imu/
  serial_imu/imu_node.py       # 节点：串口、原始/修正两路、TF、/diagnostics
  serial_imu/protocol.py       # 无 ROS：切帧、校验、缩放、协方差对角
  serial_imu/frame.py          # 无 ROS：坐标系修正（倒装 Rx）+ parent 对齐
  serial_imu/force_node.py     # 节点：5 点力串口、逐帧打印 kgf、牛顿话题
  serial_imu/force_protocol.py # 无 ROS：0xA5 力帧切帧、校验、kgf 缩放
  config/serial_imu.yaml
  config/serial_force.yaml
  launch/serial_imu.launch.py
  launch/serial_force.launch.py
  rviz/serial_imu.rviz
  test/test_protocol.py
  test/test_frame.py
  test/test_force_protocol.py
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
| `/force/points` | `std_msgs/Float64MultiArray` | 5 点力，**牛顿**（REP-103），顺序通道 1/2/3/5/7；同口 A5 帧分流（第 8 节） |
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
ros2 launch peach_harvester harvest_system.launch.py \
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
PYTHONPATH=src/serial_imu pytest src/serial_imu/test/test_protocol.py src/serial_imu/test/test_frame.py src/serial_imu/test/test_force_protocol.py
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
| `force_print` | false | 同口 A5 力帧逐帧打印 kgf（≈100Hz；整栈默认关防刷屏） |
| `force_print_decimate` | 1 | 力打印抽稀：每 N 帧一条 |
| `force_channel_labels` | 见 yaml | 力通道 1/2/3/5/7 点位标签（第 8 节） |

---

## 8. 5 点力传感器（咬合末端，`serial_force_node`）

咬合式末端盘面均布 5 点力传感器，采集板（定制固件）100Hz 主动上报。**2026-09-29 实测：A5 力帧与 A4 IMU 帧共用同一条 CH343 串口（`/dev/imu`→ttyACM0）交错到达**（力 ≈100Hz + IMU ≈75Hz），没有独立力口。因此一个口只有一个拥有者，两种用法**不许同时**：

1. **随 IMU 一起（推荐）**：`serial_imu_node` 内建同口分流（`feed_mux`），解析两类帧、发布 `/imu/*` 与 `/force/points`，`force_print:=true` 时另打 kgf 流。随 `harvest_system` 也是这条路径（打印默认关）。
2. **只看力**：独立节点 `serial_force_node` 开同一条口只解 A5 帧（A4 帧当噪声跳过）。`imu_enabled:=false` 的整栈旁可用。

传感器型号 IMS-C04A（小量程）：感应区直径 4mm，灵敏度范围 50g–2kg。

### 协议（17 字节定长帧）

| 偏移 | 字节 | 含义 |
|------|------|------|
| 0 | `A5` | 帧头，整帧只出现一次 |
| 1+3i | `01 02 03 05 07` | 5 组通道号（采集板输入 0/1/2/4/7 → 输出重编号） |
| 2+3i | int16 小端 ×100 | 该通道力值，kgf（0.01 kgf 分辨率，有符号） |
| 16 | uint8 | 校验 = 前 16 字节累加和低 8 位（与 IMU 帧同规则） |

文档回归样例（`test/test_force_protocol.py` 固化）：

- 全零：`A5 01 00 00 02 00 00 03 00 00 05 00 00 07 00 00 B7`
- 五点 0.20/1.00/2.10/0.55/1.99 kgf：`A5 01 14 00 02 64 00 03 D2 00 05 37 00 07 C7 00 FF`

解析在 `serial_imu/force_protocol.py`（零 ROS）：找 `A5` → 17 字节窗口校验通道号+累加和 → 解码；假帧头（载荷里也会出现 `A5`）跳一字节重扫。

### 通道 → 咬合末端点位

从 CAD 截图读出（图分辨率有限，**2/5 与 3/7 两组有误读可能**）。现场逐点按压打印流核对，错了改 `channel_labels` 参数，不动代码：

| 通道 | 圆盘点位（面对安装板） |
|------|------------------------|
| 1 | 左中（≈9 点钟） |
| 2 | 左下（≈7 点钟） |
| 3 | 右下（≈5 点钟） |
| 5 | 右上（≈1–2 点钟） |
| 7 | 左上（≈11–12 点钟） |

### 启动与输出

```bash
colcon build --packages-select serial_imu
source install/setup.bash

# 用法一：随 IMU 节点分流（force_print_decimate 抽稀打印）：
ros2 launch serial_imu serial_imu.launch.py use_rviz:=false force_print:=true
# 整栈里等价于 imu_enabled:=true + force_print（launch 参数默认 false）

# 用法二：只看力（勿与整栈/serial_imu 并行，抢同一口）：
ros2 launch serial_imu serial_force.launch.py
```

打印每帧一行（100Hz；`print_decimate` / `force_print_decimate` 每 N 帧打一条）。用法一前缀 `#F`，用法二前缀 `#`：

```
#F123 F1[左中(9点)]=+0.00 F2[左下(7点)]=+0.20 F3[右下(5点)]=+2.10 F5[右上(1-2点)]=+0.55 F7[左上(11-12点)]=+1.99 kgf
```

两种用法都发话题 `/force/points`（`std_msgs/Float64MultiArray`，Reliable+Volatile）——按 **REP-103 SI 牛顿**发 5 值，顺序同通道 1/2/3/5/7；`ros2 topic hz /force/points` 验 100Hz。打印用协议原生 **kgf**（与量程 50g–2kg 同单位），换算 N ×9.80665。`/diagnostics`：串口开闭 + 帧率（用法二期望 100Hz，窗 0.6×–1.4×）。

无输出时依次核对：波特率（默认 115200，厂家不同就改参数）、口是否已被另一个节点占用（`/dev/imu` 只能开一份）、采集板供电与 Tx/Rx。2026-09-29 实测空载全零（956 帧/10s 全过校验）。

### 参数（`config/serial_force.yaml`，用法二）

| 参数 | 默认 | 作用 |
|------|------|------|
| `port` | `/dev/imu` | 与 IMU 同口（实测共用）；独立力口出现后再改 |
| `port_fallbacks` | `ttyUSB1`、`ttyACM1` | 依次试 |
| `baudrate` | 115200 | |
| `read_period_s` | 0.005 | 读串口定时器 |
| `expected_rate_hz` | 100 | `/diagnostics` 帧率期望 |
| `print_data` | true | 逐帧打印（kgf） |
| `print_decimate` | 1 | 每 N 帧打一条；1=全部 ≈100Hz |
| `channel_labels` | 见 yaml | 通道→点位标签，现场校对后改这里 |

用法一的对应参数在 `config/serial_imu.yaml`：`force_print`（默认 false）、`force_print_decimate`、`force_channel_labels`（同表语义）。

---

## 9. 不负责

采摘调度、MoveIt、底盘 `/scan`、生命周期名单。不替代预留的底盘 IMU。不做自适应工具偏移（`tcp_actual` 臂侧缝仍预留、未实现；自适应末端的柔性偏斜由 imu_follow 姿态跟随消化（2026-09-15 起接 `adaptive_cylinder_v1`，2026-09-28 起随工具换代为 `adaptive_shear_v1`），`tcp_actual` 仍是未来刚性偏移量的缝）。

5 点力节点另不负责：力控/接触检测判据（只打印与发布原始值）、进 `harvest_system`、写 PlanningScene。点位标签以现场按压校对为准。
