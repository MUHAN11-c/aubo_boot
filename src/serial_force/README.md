# serial_force

咬合末端 5 点力传感器驱动：QinHeng USB 转串适配器（CH340 `1a86:7523` 旧线 /
CH343 `1a86:55d3` 现行），采集板 100Hz 主动上报 0xA5 定长帧。发布
`/force/points`（`std_msgs/Float64MultiArray`，SI 牛顿），可选逐帧打印 kgf。

**不是**采摘五包，lifecycle 不管。随 `harvest_system` 起（`force_enabled` 默认
true），不进只读 bringup。

> 2026-10-19 用户裁定：IMU / `imu_follow` 自适应末端方法暂时废弃，本包收敛为
> 纯力传感器；原 `serial_imu` 包名与 `/dev/imu` 设备名同步改为 `serial_force`
> / `/dev/force`。历史在 git（`git log -- src/serial_force`）。

现行行为以源码为准。栈内摘要：[系统架构](../peach/peach_bringup/doc/architecture.md)
§3、[系统 io](../peach/peach_bringup/doc/io.md)、
[testing.md](../peach/peach_system_tests/doc/testing.md)（怎么跑）。

```
serial_force/
  serial_force/force_node.py     # 节点：串口、逐帧打印 kgf、/force/points、/diagnostics
  serial_force/force_protocol.py # 无 ROS：0xA5 力帧切帧、校验、kgf 缩放
  scripts/force_web.py           # 调试 Web：订 /force/points，SSE 推浏览器按 CAD 实测点位显示
  scripts/force_web.html         # 上面的页面（整机 220mm/盘面 72mm + 五点 + tcp 三轴 + 侧视）
  scripts/face_fruit.png         # 盘面渲染（与 URDF 同一 visual STL，72 mm 视场）
  scripts/face_full.png          # 整机渲染（同一 STL，220 mm 视场）
  scripts/render_force_pads.py   # 生成 doc/force_pads_on_tool.png（五点核对图）
  scripts/make_web_face.py       # 生成 face_full.png / face_fruit.png
  doc/force_pad_positions.md     # 5 点膜片：CAD→URDF 坐标推导、Web/图实现、复现
  config/serial_force.yaml
  launch/serial_force.launch.py
  test/test_force_protocol.py
  udev/99-force-usb-serial.rules
```

---

## 1. 查设备与固定串口名

现场出现过两种 QinHeng 芯片：

```bash
lsusb | grep -i 1a86
# CH340 1a86:7523（旧线）→ ttyUSB*
# CH343 1a86:55d3（现行）→ ttyACM*
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
dmesg -T | grep -iE 'ttyUSB|ttyACM|ch34|cdc_acm|1a86'
```

udev 规则把适配器固定为 `/dev/force`（插拔后不要写死 `ttyUSB0`）：

```bash
sudo cp $(ros2 pkg prefix serial_force)/share/serial_force/udev/99-force-usb-serial.rules \
  /etc/udev/rules.d/
# 源码副本：src/serial_force/udev/99-force-usb-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/force
# 期望：/dev/force -> ttyUSB0（CH340）或 /dev/force -> ttyACM0（CH343）
```

规则按 `idVendor=1a86` + `idProduct`（`7523` / `55d3`）匹配，`SYMLINK+=force`，
组 `dialout`，模式 `0660`。插第二块同型适配器时两条规则会抢 `/dev/force`，
后插的赢。节点按 `port` → `port_fallbacks` 找口：`/dev/force`，再
`/dev/ttyUSB0`、`/dev/ttyACM0`。

权限：`usermod -aG dialout` **对已经打开的终端无效**。

```bash
sudo usermod -aG dialout $USER
newgrp dialout          # 当前终端立刻带上 dialout
groups                  # 须含 dialout
```

`Errno 13 Permission denied` 时节点打不开口；节点每 2 s 重试，日志提示
`newgrp dialout`。

---

## 2. 协议（17 字节定长帧）

采集板 100Hz 主动上报，校验和 = 除末字节外所有字节之和的低 8 位。

| 偏移 | 字节 | 含义 |
|------|------|------|
| 0 | `A5` | 帧头 |
| 1+3i | `01 02 03 05 07` | 5 组通道号（采集板输入 0/1/2/4/7 → 输出重编号） |
| 2+3i | int16 小端 ×100 | 该通道力值，kgf（0.01 kgf 分辨率，有符号） |
| 16 | uint8 | 校验 = 前 16 字节累加和低 8 位 |

文档回归样例（`test/test_force_protocol.py` 固化）：

- 全零：`A5 01 00 00 02 00 00 03 00 00 05 00 00 07 00 00 B7`
- 五点 0.20/1.00/2.10/0.55/1.99 kgf：`A5 01 14 00 02 64 00 03 D2 00 05 37 00 07 C7 00 FF`

解析在 `serial_force/force_protocol.py`（零 ROS）：找 `A5` → 17 字节窗口校验
通道号 + 累加和 → 解码；假帧头（载荷里也会出现 `A5`）跳一字节重扫。

传感器型号 IMS-C04A（小量程）：感应区直径 4 mm，量程 50 g–2 kg。

### 通道 → 咬合末端点位

2026-10-16 用户裁定：**五点在正五边形顶点（等角等距），5 在正上方顶点**。
2026-10-19 用 CAD 总装 `_tools/shear_pipeline/bite2/extract_report.json` 的
`per_part_bbox_m` 复核：半径 16.31–16.45 mm，膜片平面在 `tcp z = −72.29 mm`。
核对/修改走 `channel_labels` 参数，不动代码：

| 通道 | 圆盘位置 | CAD 零件 | 自 `+Y_tcp` 顺时针 | 半径 |
|------|----------|----------|--------------------|------|
| 5 | 上（顶点） | `力敏电阻000` | 0.0° | 16.45 mm |
| 7 | 右上 | `力敏电阻001` | 74.7° | 16.33 mm |
| 3 | 右下 | `力敏电阻002` | 144.6° | 16.32 mm |
| 2 | 左下 | `力敏电阻004` | 216.3° | 16.31 mm |
| 1 | 左上 | `力敏电阻003` | 286.2° | 16.35 mm |

Web（`force_web.html`）按同一组角度/半径把五点叠到末端渲染上：默认**整机
220 mm**，切**盘面 72 mm** 读数；页内可切**果实侧/臂侧**（臂侧是镜像，
1↔7、2↔3 左右互换，通道值不变）。图心画 **tcp 三轴**，侧视条标出
`wrist3` 原点与 `+Z_tcp` 方向。

坐标推导、数据链、渲染/Web 工具、复现命令与已知限制见
**[doc/force_pad_positions.md](doc/force_pad_positions.md)**（含
`doc/force_pads_on_tool.png` 四面板核对图）。

---

## 3. 启动与输出

```bash
colcon build --packages-select serial_force
source install/setup.bash

# 随采摘整栈（mock / real 相同；force_enabled 默认 true）：
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
# 真机：hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98

# 只起力传感器（勿与整栈并行，抢同一口）：
ros2 launch serial_force serial_force.launch.py

# Web 实时页（只订话题不碰串口，可与驱动并行）：
python3 src/serial_force/scripts/force_web.py   # http://127.0.0.1:8099
```

打印每帧一行（100Hz；`print_decimate` 每 N 帧打一条）：

```
#123 F1[左上]=+0.00 F2[左下]=+0.20 F3[右下]=+2.10 F5[上]=+0.55 F7[右上]=+1.99 kgf
```

话题 `/force/points`（Reliable + Volatile）按 **REP-103 SI 牛顿**发 5 值，
顺序同通道 1/2/3/5/7；`ros2 topic hz /force/points` 验 100Hz。打印用协议原生
**kgf**，换算 N ×9.80665。`/diagnostics`：串口开闭 + 帧率（期望 100Hz，
窗 0.6×–1.4×）。

无输出时依次核对：波特率（默认 115200）、口是否已被另一个节点占用
（`/dev/force` 只能开一份）、采集板供电与 Tx/Rx。

---

## 4. 参数（`config/serial_force.yaml`）

根键 = 节点名 `serial_force`。

| 参数 | 默认 | 作用 |
|------|------|------|
| `port` | `/dev/force` | 首选口 |
| `port_fallbacks` | `ttyUSB0`、`ttyACM0` | 依次试 |
| `baudrate` | 115200 | |
| `read_period_s` | 0.005 | 读串口定时器 |
| `expected_rate_hz` | 100 | `/diagnostics` 帧率期望 |
| `print_data` | true | 逐帧打印（kgf） |
| `print_decimate` | 1 | 每 N 帧打一条；1=全部 ≈100Hz |
| `channel_labels` | 见 yaml | 通道→点位标签，现场校对后改这里 |

---

## 5. 测试

```bash
PYTHONPATH=src/serial_force pytest src/serial_force/test/test_force_protocol.py
```

纯核零 ROS：文档回归帧、通道校验、假帧头重扫、半帧等待。`colcon test
--packages-select serial_force` 另跑 flake8 / pep257。

---

## 6. 不负责

采摘调度、MoveIt、底盘 `/scan`、生命周期名单。不做力控/接触检测判据（只打印
与发布原始值）、不写 PlanningScene。点位标签以现场按压校对为准。
