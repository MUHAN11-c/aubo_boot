# ivg_utils — IVG 共享工具包

项目级共享工具函数和常量，zero ROS dependency（不 import rclpy）。

## 模块

### `ivg_utils.math`
四元数/旋转矩阵/欧拉角转换工具（全局不可修改）

| 函数 | 用途 |
|------|------|
| `quaternion_to_rotation_matrix(q)` | Hamilton 四元数 (xyzw) → 3×3 旋转矩阵 |
| `rotation_matrix_to_quaternion(R)` | 旋转矩阵 → Hamilton 四元数 |
| `filter_components_by_params(...)` | 按参数过滤组件（VPE 姿态后处理用） |

### `ivg_utils.io`
AUBO 机器人 IO 引脚常量（全局不可修改）

```python
IO_GRIPPER = 6       # 夹爪数字输出引脚
IO_QUICK_SWAP = 7    # 快换数字输出引脚
```

### `ivg_utils.robot`
AUBO E5 机器人物理常量（全局不可修改）

```python
ROBOT_WORKING_RADIUS = 0.8865  # AUBO E5 工作半径 (m)
JOINT_COUNT = 6                # 关节数量
```
