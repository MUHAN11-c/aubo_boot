# serial_force

咬合末端五点力传感器驱动：QinHeng USB 转串（CH340 `1a86:7523` /
CH343 `1a86:55d3`）→ 采集板 100 Hz 主动上报 0xA5 32 字节帧 →
`/force/sample`（原子帧）+ `/force/points|travel|ad|status`（兼容 array）。
五点力实时页已合并到 8090 `peach_web /force.html`；历史数据走会话 bag/分析。

- 不是 lifecycle 节点，不进 `peach_lifecycle_manager` 名单；
- 不是控制/许可节点：不发运动、不写 PlanningScene、不参与抓取授权；
- 力值只用于显示、记录与离线分析；闭环/CoP 工具在本包可选 executable，仿真源用 `source:=sim`；
- 一个串口只能有一个拥有者，勿与整栈并行再起第二个 `serial_force_node`。

## 构建与启动

```bash
source /opt/ros/jazzy/setup.bash
# 工作区若用 aubo_py3.12，先 activate 或确认 python3 指向它
colcon build --packages-select serial_force
source install/setup.bash

# 只起驱动（勿与整栈并行）
ros2 launch serial_force serial_force.launch.py

# 仿真源（不需要串口，使用 mock 臂 + TF）
ros2 launch serial_force serial_force.launch.py source:=sim

```

真机整栈用 `peach_bringup/launch/harvest_system.launch.py`（`force_enabled`
默认 true）；8090 页面包含五点力页。

## 设备与权限

```bash
lsusb | grep -i 1a86
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/
```

udev 规则把适配器固定为 `/dev/force`：

```bash
sudo cp $(ros2 pkg prefix serial_force)/share/serial_force/udev/99-force-usb-serial.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/force
```

规则按 `idVendor=1a86` + `idProduct`（`7523` / `55d3`）匹配，`SYMLINK+=force`，
组 `dialout`，模式 `0660`。插两个同型适配器会抢 `/dev/force`，后插的赢。
权限：`sudo usermod -aG dialout $USER`，当前终端 `newgrp dialout`；节点每 2 s
重试，`Errno 13` 时日志提示。节点按 `port` → `port_fallbacks` 找口。

## 接口

| 名字 | 类型 | 含义 |
|---|---|---|
| `/force/sample` | `peach_interfaces/ForceSample` | 原子 5 通道帧：N / 力等效 mm / raw AD / status / UUID / sequence；Reliable depth 100 |
| `/force/points` | `Float64MultiArray` | 5 值，SI 牛顿，顺序 1/2/3/5/7 |
| `/force/travel` | `Float64MultiArray` | 5 值，力等效 mm（`力/66`，0.1 mm/LSB），不是独立位移 |
| `/force/ad` | `Float64MultiArray` | 5 值，原始 AD |
| `/force/status` | `UInt8MultiArray` | 0 OK、1 SATURATED、2 OPEN、3 INVALID |

6.0 mm 既是 370 g 满量程也是断线哨兵：`/force/travel` 保留 6.0、`/force/ad`
保留原始值，不置 NaN；`AD>=3000` 判 OPEN，否则 `travel>=5.95` 判 SAT。
帧布局、校验、样帧、QoS 与记录列见 [`doc/io.md`](doc/io.md)。

## 参数

`config/serial_force.yaml`（根键 `serial_force`）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `port` | `/dev/force` | 首选串口 |
| `port_fallbacks` | ttyUSB0、ttyACM0 | 依次试 |
| `baudrate` | 115200 | 8N1 |
| `read_period_s` | 0.005 | 读串口定时器（100 Hz） |
| `expected_rate_hz` | 100.0 | `/diagnostics` 频率窗 0.6–1.4× |
| `open_ad_threshold` | 3000.0 | AD ≥ 此值判断线/悬空 |
| `full_travel_mm` | 5.95 | 行程 ≥ 此值判 SAT |
| `print_data` | true | 逐帧打印 N/mm/AD/status |
| `print_decimate` | 1 | 每 N 帧打印一条 |
| `channel_labels` | 右上/右下/左下/上/左上 | 通道 1/2/3/5/7 果实侧标签 |

## 力页面

8090 `peach_web /force.html` 显示实时 5 点/CoP；历史力帧在会话 bag，接触段分析由 observability 统一完成。本包不再提供 `--port`/`--runs-root` 或第二套 HTTP 服务。

## 测试

```bash
colcon test --packages-select serial_force
```

只跑官方 `flake8` / `pep257`。帧解析是否正确，看节点日志的 N/mm/AD 对照
[`doc/io.md`](doc/io.md) 的现场样帧；`ros2 topic hz /force/points` 验 100 Hz；
`/diagnostics` 看串口与帧率。

## 文档

| 文档 | 内容 |
|---|---|
| [`doc/architecture.md`](doc/architecture.md) | 模块、数据流、会话记录、并发、边界 |
| [`doc/io.md`](doc/io.md) | 话题/QoS、0xA5 帧、参数、诊断、记录列 |
| [`doc/geometry.md`](doc/geometry.md) | 5 点 CAD 位置、坐标变换、CoP、工具 |
| [`doc/validation.md`](doc/validation.md) | 台架/真机数据、已证明/未证明、下一步 |
| [`doc/hardware.md`](doc/hardware.md) | RP-C5/MY2901/STM32、raw AD 与 mm 语义 |

## 不负责

采摘调度、MoveIt、底盘 `/scan`、IMU、力闭环、功能安全。8090 力页是调试页，
不是第二控制面；没有 SetIO/使能/运动入口。真机硬件急停在柜/示教器，不经 ROS。
