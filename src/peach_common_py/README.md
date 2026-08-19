# peach_common_py

被多个 Python 包共用的**零 ROS 纯核**（`peach_common_py.ros` 除外）。帧率 EMA 已下沉到 `peach_scene_perception`，本包不再提供 timing。

## 职责

跨包复用、且至少两个消费者的小工具：深度单位、TF 矩阵、有界队列、时钟、注册表、过程数据目录。

## 模块

| 模块 | 作用 |
|------|------|
| `depth_geometry` | 深度图归一成 uint16 毫米（Percipio `depth_scale_unit` 或 32FC1 米） |
| `tf_utils` | `Transform` → 4×4、点/方向变换、重力在相机系的投影 |
| `bounded_worker` | 有界队列单写者线程，满则拒新任务，保积分顺序 |
| `clock` | `Clock` / `ManualClock`，纯核计时不绑 rclpy |
| `registry` | 按名装配检测器/分割器/管线等实现 |
| `harvest_data` | 过程数据根目录与事件追加 |
| `ros.clock_adapter` | 把 `rclpy` 时钟适配成纯核 `Clock`（仅此子包可 import rclpy） |

## 使用

```python
from peach_common_py.depth_geometry import normalize_depth_to_uint16_mm
from peach_common_py.ros.clock_adapter import RclpyClockAdapter  # 节点侧显式 import
```

纯核文件禁止 `import rclpy`。Python 解释器用工作区 `aubo_py3.12`，numpy **1.26.4**。
