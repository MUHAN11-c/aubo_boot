# peach_vegetation

GPU 枝 / 叶二维分割。订阅彩色图，发布 `sensor_msgs/Image` 掩膜（叶绿 / 枝红叠加）。
**不写 PlanningScene、不删 octomap、不发运动、不进 `harvest_system`。**

首版直接构造 Frangi（torch Hessian，有 CUDA 用 `cuda:0`）+ Excess Green / HSV 叶。
同一进程内序列化推理。与 YOLO / MobileSAM 分进程，同卡时不要和采摘感知对打。

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
