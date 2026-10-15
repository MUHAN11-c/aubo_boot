# peach_vegetation

GPU 枝 / 叶二维分割。订阅彩色图，发布 `sensor_msgs/Image` 掩膜（叶绿 / 枝红叠加）。
**不写 PlanningScene、不删 octomap、不发运动、不进 `harvest_system`。**
话题/参数契约权威见 [doc/io.md](doc/io.md)（status JSON 实键、QoS、参数表）与 [doc/architecture.md](doc/architecture.md)。

首版直接构造 Frangi（torch Hessian，有 CUDA 用 `cuda:0`）+ Excess Green / HSV 叶。
同一进程内序列化推理。与 YOLO / MobileSAM 分进程，同卡时不要和采摘感知对打。
**生命周期**：节点 `main()` `ensure_active()` 自激活是唯一转换源（2026-09-29 修复后 launch 与 `ros2 run` 均可；launch `autostart` 参数为无操作兼容位）——勿用 `ros2 lifecycle set` 手动操作。`device: 显式 cuda` 无卡则 configure 失败；相机重启后订阅会楔死，须重启本节点（详见 doc/io.md）。

## 公有入口

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch peach_vegetation vegetation.launch.py
# 无相机时可 remap 到 bag 里的彩色图：
# ros2 launch peach_vegetation vegetation.launch.py image:=/camera/color/image_raw
```

| 话题 | 类型 | 含义 |
|------|------|------|
| `image`（launch remap → `/camera/color/image_raw`） | `sensor_msgs/Image` | 输入 bgr8 |
| `/peach/vegetation/leaf_mask` | `sensor_msgs/Image` mono8 | 叶 |
| `/peach/vegetation/branch_mask` | `sensor_msgs/Image` mono8 | 枝 / 细木质脊 |
| `/peach/vegetation/overlay` | `sensor_msgs/Image` bgr8 | 绿叶红枝叠加 |
| `/peach/vegetation/status` | `std_msgs/String` JSON | 耗时 / 覆盖率 / 丢帧 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 健康 |

## 构建 / 测试 / 许可

`colcon build --packages-select peach_vegetation`。
`colcon test --packages-select peach_vegetation`。
torch 走工作区 `aubo_py3.12`（`requirements.txt`，numpy == 1.26.4）。无 CUDA 回退 CPU。
BSD-3-Clause。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。
