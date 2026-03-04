# coordinate_transforms_py

Python 库与 ROS2 节点：与世界/相机/像素坐标系、2D↔3D、重投影误差、四元数/旋转矩阵/欧拉角、点与坐标系变换相关的接口与 C++ 包 **coordinate_transforms** 约定一致。**core** 仅依赖 numpy/scipy（无 rclpy），**ros2** 封装 geometry_msgs、tf2_ros、sensor_msgs。

---

## 文件说明

```
coordinate_transforms_py/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── coordinate_transforms_py
├── config/
│   └── camera_example.yaml                    # 示例相机内参
├── config/rviz/
│   └── coord_tf_visualization.rviz            # RViz2 配置
├── launch/
│   ├── coord_tf_demo.launch.py                 # 仅启动 demo 节点
│   └── coord_tf_visualization.launch.py       # 启动 demo 节点 + RViz2
├── test/
│   └── test_core.py                            # core 单元测试（pytest）
└── coordinate_transforms_py/
    ├── __init__.py
    ├── core/                                   # 非 ROS2 模块（NumPy/scipy 计算，注释含公式）
    │   ├── __init__.py
    │   ├── coordinate_systems.py              # 内参、2D↔3D、重投影误差
    │   ├── rotation_conversion.py              # 四元数/旋转矩阵/欧拉角（scipy）
    │   ├── rotation_conversion_numpy.py       # 可选：同 API，仅 NumPy 实现（无 scipy）
    │   └── point_transform.py                  # 手系、单点/轴、多点、位姿、直线
    ├── ros2/                                   # ROS2 模块（依赖 core）
    │   ├── __init__.py
    │   ├── coordinate_systems_ros2.py
    │   ├── rotation_conversion_ros2.py
    │   ├── point_transform_ros2.py
    │   └── coord_tf_demo_node.py               # 发布 TF + Marker 的 demo 节点
    ├── run_core_demo.py                        # 非 ROS2 示例（仅 core）
    ├── run_visualization_demo.py               # 非 ROS2：matplotlib 2D 可视化
    └── run_opencv_process_demo.py              # 可选：OpenCV 绘制并保存 PNG/YAML
```

- **输出目录**：上述 demo/可视化脚本生成的文件写入 `COORD_TF_OUTPUT_DIR`（默认 `./coord_tf_output`），见下方使用方法。

---

## 依赖

```bash
pip install numpy scipy
# 可选：可视化
pip install matplotlib
# 可选：OpenCV 过程数据
pip install opencv-python
```

---

## 构建

```bash
cd aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select coordinate_transforms_py
source install/setup.bash
```

---

## 使用方法

### ROS2 节点与 Launch

| 命令 | 说明 |
|------|------|
| `ros2 run coordinate_transforms_py coord_tf_demo_node` | 发布 world → camera_optical_frame 的 TF 及坐标轴 Marker |
| `ros2 launch coordinate_transforms_py coord_tf_demo.launch.py` | 仅启动上述 demo 节点 |
| `ros2 launch coordinate_transforms_py coord_tf_visualization.launch.py` | 启动 demo 节点 + RViz2（可加 `use_rviz:=false` 关闭 RViz） |

### 非 ROS2 示例与可视化

| 命令 | 说明 | 输出文件 |
|------|------|----------|
| `ros2 run coordinate_transforms_py run_core_demo` | 仅调用 core：投影/反投影/重投影误差/旋转/变换示例 | 无文件，仅终端输出 |
| `ros2 run coordinate_transforms_py run_visualization_demo` | 2D 重投影示意（需 matplotlib） | `coord_tf_output/coord_tf_visualization_2d.png` |
| `ros2 run coordinate_transforms_py run_opencv_process_demo` | OpenCV 绘制 2D 点与重投影误差（需 opencv-python） | `coord_tf_output/coord_tf_reprojection.png`、`coord_tf_process_data.yaml` |

指定输出目录示例：

```bash
export COORD_TF_OUTPUT_DIR=/path/to/output
ros2 run coordinate_transforms_py run_visualization_demo
```

### 单元测试

```bash
cd aubo_ros2_ws/src/coordinate_transforms_py
python3 -m pytest test/test_core.py -v --tb=short
```

或直接运行测试文件（需已安装 pytest）：

```bash
python3 test/test_core.py
```

### 作为库使用

- **core**：`from coordinate_transforms_py.core import ...`，默认使用 numpy、scipy（rotation_conversion 用 scipy.spatial.transform），无 rclpy。
- **可选无 scipy 分支**：`from coordinate_transforms_py.core.rotation_conversion_numpy import quaternion_to_rotation_matrix, ...`，与 rotation_conversion 同 API，仅依赖 NumPy，公式见注释。
- **ros2**：`from coordinate_transforms_py.ros2 import ...`，依赖 rclpy、tf2_ros、geometry_msgs、sensor_msgs、visualization_msgs 等。

---

## 与 coordinate_transforms 的关系

C++ 实现与文档在 **coordinate_transforms** 包中；本包提供与之一致的 Python 接口与示例。详细坐标系与变换说明见 `coordinate_transforms` 的 `share/coordinate_transforms/doc/COORDINATE_SYSTEMS_AND_TRANSFORMS.md`。
