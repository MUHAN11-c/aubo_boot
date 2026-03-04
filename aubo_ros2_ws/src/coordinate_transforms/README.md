# coordinate_transforms

C++ 库与 ROS2 节点：世界/相机/像素坐标系、2D↔3D 投影与重投影误差、四元数/旋转矩阵/欧拉角、点与坐标系变换（手系、单点、单轴、复杂点/轴）。**core** 为纯数学无 ROS 依赖，**ros2** 封装 geometry_msgs、tf2、sensor_msgs。

---

## 文件说明

```
coordinate_transforms/
├── CMakeLists.txt
├── package.xml
├── README.md
├── doc/
│   └── COORDINATE_SYSTEMS_AND_TRANSFORMS.md   # 坐标系与变换说明（非 ROS2 / ROS2 分节）
├── include/coordinate_transforms/
│   ├── core/                                  # 非 ROS2 头文件
│   │   ├── coordinate_systems.hpp             # 内参、2D↔3D、重投影误差
│   │   ├── rotation_conversion.hpp            # 四元数/旋转矩阵/欧拉角转换
│   │   └── point_transform.hpp                # 手系、单点/轴、多点、位姿、直线变换
│   └── ros2/                                  # ROS2 头文件（依赖 core）
│       ├── coordinate_systems_ros2.hpp
│       ├── rotation_conversion_ros2.hpp
│       └── point_transform_ros2.hpp
├── src/
│   ├── core/                                  # core 实现
│   │   ├── coordinate_systems.cpp
│   │   ├── rotation_conversion.cpp            # 无 Eigen 时：手写公式
│   │   ├── rotation_conversion_eigen.cpp      # 有 Eigen 时：Eigen 计算（二选一编译）
│   │   └── point_transform.cpp
│   ├── ros2/                                  # ros2 实现与 demo 节点
│   │   ├── coordinate_systems_ros2.cpp
│   │   ├── rotation_conversion_ros2.cpp
│   │   ├── point_transform_ros2.cpp
│   │   └── coord_tf_demo_node.cpp             # 发布 TF + Marker 的 demo 节点
│   └── tools/                                 # 非 ROS2 可执行
│       ├── export_visualization.cpp           # 导出 PCD + CSV（无 OpenCV）
│       └── opencv_process_data.cpp            # 可选：OpenCV 时编译，导出 PNG + YAML
├── config/
│   └── camera_example.yaml                    # 示例相机内参
├── config/rviz/
│   └── coord_tf_visualization.rviz            # RViz2 配置
└── launch/
    ├── coord_tf_demo.launch.py                # 仅启动 demo 节点
    └── coord_tf_visualization.launch.py      # 启动 demo 节点 + RViz2
```

- **输出目录**：`export_visualization`、`opencv_process_data` 生成的文件写入 `COORD_TF_OUTPUT_DIR`（默认 `./coord_tf_output`），详见文档。

---

## 构建

```bash
cd aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select coordinate_transforms
source install/setup.bash
```

可选：若需编译 `opencv_process_data`，需已安装 OpenCV；未安装时仅跳过该可执行。

---

## 使用方法

### ROS2 节点与 Launch

| 命令 | 说明 |
|------|------|
| `ros2 run coordinate_transforms coord_tf_demo_node` | 发布 world → camera_optical_frame 的 TF 及坐标轴 Marker |
| `ros2 launch coordinate_transforms coord_tf_demo.launch.py` | 仅启动上述 demo 节点 |
| `ros2 launch coordinate_transforms coord_tf_visualization.launch.py` | 启动 demo 节点 + RViz2（可加 `use_rviz:=false` 关闭 RViz） |

### 非 ROS2 工具（导出文件）

| 命令 | 说明 | 输出文件 |
|------|------|----------|
| `ros2 run coordinate_transforms export_visualization` | 导出示例 3D 点与 2D 投影 | `coord_tf_output/coord_tf_sample.pcd`、`coord_tf_sample_2d.csv` |
| `ros2 run coordinate_transforms opencv_process_data` | 需编译时启用 OpenCV；绘制 2D 点与重投影误差并保存 | `coord_tf_output/coord_tf_reprojection.png`、`coord_tf_process_data.yaml` |

指定输出目录示例：

```bash
export COORD_TF_OUTPUT_DIR=/path/to/output
ros2 run coordinate_transforms export_visualization
```

### 作为库使用

- **core**：仅链接 `coordinate_transforms_core`，包含 `include/coordinate_transforms/core/` 头文件，无 ROS 依赖。
- **ros2**：链接 `coordinate_transforms_ros2`，依赖 rclcpp、geometry_msgs、tf2、sensor_msgs、visualization_msgs 等。

---

## 文档

详见 `doc/COORDINATE_SYSTEMS_AND_TRANSFORMS.md`（安装后位于 `share/coordinate_transforms/doc/`）。

## 与 coordinate_transforms_py 的关系

Python 实现与示例在 **coordinate_transforms_py** 包中，接口与本文档约定一致；README 可引用该包。
