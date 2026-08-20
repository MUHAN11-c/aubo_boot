# peach_common_py

被多个 Python 包共用的**零 ROS 纯核**（`peach_common_py.ros` 除外）。帧率 EMA 在 `peach_scene_perception.peach_pose.timing_metrics`，本包不提供 timing。

总览：[docs/flow.md](../../docs/flow.md)。

## 从哪读

| 模块 | 作用 | 谁用 |
|------|------|------|
| `fitting.py` | 法线、球/柱 RANSAC | 感知 `pipeline`（经 `peach_pose/fitting.py` 再导出）、重建 `geometry_refiner` |
| `depth_geometry.py` | 深度图 → uint16 毫米 | 感知、重建 |
| `tf_utils.py` | `Transform` → 4×4、点/方向 | 感知、重建 |
| `bounded_worker.py` | 有界队列单写者 | 重建积分顺序 |
| `clock.py` | 纯核时钟 | 节点经 `ros.clock_adapter` |
| `registry.py` | 按名装配检测器/分割器 | 感知 `impls` |
| `harvest_data.py` | 过程数据根目录 | 观测落盘辅助 |
| `ros/clock_adapter.py` | **唯一**允许 `import rclpy` 的子模块 | 节点侧 |

```python
from peach_common_py.depth_geometry import normalize_depth_to_uint16_mm
from peach_common_py.fitting import ...  # 几何拟合
from peach_common_py.ros.clock_adapter import RclpyClockAdapter
```

纯核文件禁止 `import rclpy`。Python：`aubo_py3.12`，numpy **1.26.4**。
