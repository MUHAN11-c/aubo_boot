# serial_force

咬合末端 5 点力传感器驱动：QinHeng USB 转串适配器（CH340 `1a86:7523` 旧线 /
CH343 `1a86:55d3` 现行），采集板 100Hz 主动上报 0xA5 定长帧。发布
`/force/points`（`std_msgs/Float64MultiArray`，SI 牛顿）与
`/force/travel`（毫米），可选逐帧打印 N/行程。

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
  serial_force/force_node.py     # 节点：串口、逐帧打印 N/行程/AD、/force/points、/force/travel、/force/ad、/diagnostics
  serial_force/force_protocol.py # 无 ROS：0xA5 32 字节帧切帧、校验、克→牛、行程/AD/哨兵
  scripts/force_web.py           # 调试 Web：5 点力 + CoP，SSE 按 CAD 点位叠加显示
  scripts/force_web.html         # 上面的页面（整机 220mm/盘面 72mm + 五点 + tcp 三轴 + 侧视）
  scripts/face_fruit.png         # 盘面渲染（与 URDF 同一 visual STL，72 mm 视场）
  scripts/face_full.png          # 整机渲染（同一 STL，220 mm 视场）
  scripts/render_force_pads.py   # 生成 doc/force_pads_on_tool.png（五点核对图）
  scripts/make_web_face.py       # 生成 face_full.png / face_fruit.png
  doc/force_pad_positions.md     # 5 点膜片：CAD→URDF 坐标推导、Web/图实现、复现
  doc/force_frame_v3.md          # 32 字节 v3 帧格式（2026-10-21，克+行程+原始 AD）
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

## 2. 协议（32 字节定长帧，2026-10-21 v3）

采集板 100Hz 主动上报。2026-10-21 起为 **32 字节 v3**（v2 的 22 字节基础上
每通道增加 2 字节原始 AD），完整说明与现场实测样例见
**[doc/force_frame_v3.md](doc/force_frame_v3.md)**。

```
A5 | ch | gL | gH | mm | aL | aH | ... ×5 ... | sum
```

| 字段 | 长度 | 含义 |
|------|------|------|
| `A5` | 1 | 帧头 |
| `ch` | 1 | 输出通道号：`01 / 02 / 03 / 05 / 07` |
| `gL gH` | 2 | 力，**uint16 小端，单位克**（1 LSB = 1 g） |
| `mm` | 1 | 行程 **×10**（1 LSB = 0.1 mm）；`0x3C` = 6.0 mm 满量程/哨兵 |
| `aL aH` | 2 | **原始测量 AD**，uint16 小端；标定/诊断，不直接换算力 |
| `sum` | 1 | 前 **31** 字节累加和的低 8 位 |

- 换算：`N = g × 9.80665 / 1000`；`travel_mm = raw × 0.1`；AD 原样透传。
- 单点 50–400 g（2026-10-20 用户确认：0–6 mm ↔ 400 g，约 66.7 g/mm）；
  五点合计 2 kg；**5 mm ≈ 333 g/点**（合计 ≈ 1.67 kg / 16.35 N）。
- 行程有效 0–5.9 mm，`0x3C`（6.0 mm）是满量程/哨兵。
- 校验：`sum(frame[0:31]) & 0xFF == frame[31]`。

回归样例（`test/test_force_protocol.py` 固化；2026-10-21 现场帧）：

- 现场帧：`A5 01 F3 00 25 B0 06 02 18 01 2A D2 05 03 BE 00 1D 19 05
  05 36 01 2F DC 05 07 09 01 28 2C 06 43`
  （243/280/190/310/265 g，3.7/4.2/2.9/4.7/4.0 mm，AD 1712/1490/1305/1500/1580）
- 全零（32 字节）：`A5 01 00 00 00 00 00 02 00 00 00 00 00 03 00 00 00 00 00
  05 00 00 00 00 00 07 00 00 00 00 00 B7`
- 通道 2 行程哨兵 `3C`：`A5 01 F3 00 25 B0 06 02 18 01 3C D2 05 03 BE 00 1D 19 05
  05 36 01 2F DC 05 07 09 01 28 2C 06 55`

解析在 `serial_force/force_protocol.py`（零 ROS）：找 `A5` → 32 字节窗口校验
通道号 + 累加和 → 解码；假帧头（载荷里也会出现 `A5`）跳一字节重扫。v1/v2 的
旧帧不会被误认，升级固件后必须同时升级本包。

传感器型号 IMS-C04A（小量程）：感应区直径 4 mm；单点 50–400 g
（0–6 mm 行程），五点合计 2 kg。

### 通道 → 咬合末端点位

2026-10-16 用户裁定：**五点在正五边形顶点（等角等距），5 在正上方顶点**。
2026-10-19 用 CAD 总装 `_tools/shear_pipeline/bite2/extract_report.json` 的
`per_part_bbox_m` 复核：半径 16.31–16.45 mm，膜片平面在 `tcp z = −72.29 mm`。
核对/修改走 `channel_labels` 参数，不动代码：

| 通道 | 圆盘位置（果实侧，2026-10-21 确认） | CAD 零件 | 自 `+Y_tcp` 顺时针 | 半径 |
|------|----------|----------|--------------------|------|
| 5 | 上（顶点） | `力敏电阻000` | 0.0° | 16.45 mm |
| 1 | 右上 | `力敏电阻001` | 74.7° | 16.33 mm |
| 2 | 右下 | `力敏电阻002` | 144.6° | 16.32 mm |
| 3 | 左下 | `力敏电阻004` | 216.3° | 16.31 mm |
| 7 | 左上 | `力敏电阻003` | 286.2° | 16.35 mm |

2026-10-21 用户确认果实侧映射；臂侧是镜像（1↔7、2↔3）。



Web（`force_web.html`）按同一组角度/半径把五点叠到末端渲染上：默认**整机
220 mm**，切**盘面 72 mm** 读数；页内可切**果实侧/臂侧**（臂侧是镜像，
1↔7、2↔3 左右互换，通道值不变）。图心画 **tcp 三轴**，侧视条标出
`wrist3` 原点、`+Z_tcp` 果实侧与 `−Z_tcp` 臂侧方向。

页面还实时显示 **5 点力 CoP**：`CoP = Σ fᵢ·pᵢ / Σ fᵢ`（tcp 系，mm），红点叠加在盘面/整机视图，
面板给出 `(x, y)` 与 `|CoP|`；断线/无效通道不参与，力大的点会把 CoP 拉过去。

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
# 真机整栈连同此页：ros2 launch peach_bringup real_web.launch.py
```

网页除手动「20 s 记录」外，会自动把接触段写进**当前采摘会话**：
`runs/session_*/force/episode_NNN.csv`（旁路同名 json，`index.jsonl` 汇总）。
手动记录也在同一目录，文件名 `force_20s_<时间>.csv`。页面可回放已写入的
`episode_*.csv`（选择接触段后播放或拖动），不必守着实时曲线。会话袋还在写才算当前这一轮；
栈还没开会话时先留在内存，会话出现后再写入。不另开目录。抓取阶段的墙钟由调度写在同一会话的 `cycles/`，和这里的 `force/` 放在一起。
只在任一路力达到约 0.15 N 时开始，回到约 0.08 N 并稳住 0.35 s 后封口；
`contact_s` 是从出现变化到仍高于回到门的时长，`duration_s` 含归零尾段。
空载不写。180 s 仍未归零会以 `max_duration` 封口。

打印每帧一行（100Hz；`print_decimate` 每 N 帧打一条）：

```
#123 F1[左上]=+2.38N/3.7mm/AD1712 F2[左下]=+2.75N/4.2mm/AD1490 F3[右下]=+1.86N/2.9mm/AD1305 F5[上]=+3.04N/4.7mm/AD1500 F7[右上]=+2.60N/4.0mm/AD1580
```

行程哨兵通道打印 `!`（该通道 `/force/points`、`/force/travel`、`/force/ad` 都发 NaN）。

话题：

- `/force/points`（Reliable + Volatile）：5 值，**REP-103 SI 牛顿**，顺序 1/2/3/5/7；
- `/force/travel`（同 QoS）：5 值，**毫米**（0.1 mm/LSB），哨兵通道为 `NaN`；
- `/force/ad`（同 QoS）：5 值，**原始 AD**，哨兵通道为 `NaN`。

`ros2 topic hz /force/points` 验 100Hz。`/diagnostics`：串口开闭 + 帧率
（期望 100Hz，窗 0.6×–1.4×）。Web 同时订阅三个话题，条下显示行程与 AD，
并实时计算 5 点 CoP。

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
| `print_data` | true | 逐帧打印（N / mm） |
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

## 6. 仿真与真实分开

5 点力 CoP 闭环仿真在独立包 **`serial_force_sim`**（mock 臂、假力源），不在本包：
模型、控制律、20 组统计与限界见
[../serial_force_sim/doc/force_align_sim.md](../serial_force_sim/doc/force_align_sim.md)。

本包只保留真机驱动与调试页；两者只通过 `/force/points` 契约衔接，本包不依赖
`serial_force_sim`，仿真也不读写串口。

## 7. 不负责

采摘调度、MoveIt、底盘 `/scan`、生命周期名单。真机驱动不做力控/接触检测判据
（只打印与发布原始值）、不写 PlanningScene；§6 的闭环只在 mock 仿真里验证，
不改驱动行为。点位标签以现场按压校对为准。
