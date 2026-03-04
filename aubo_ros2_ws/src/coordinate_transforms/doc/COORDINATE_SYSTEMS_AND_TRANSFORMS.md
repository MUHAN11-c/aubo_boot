# 坐标系与变换说明（Coordinate Systems and Transforms）

本文档说明世界/相机/像素坐标系、2D↔3D、重投影误差、四元数/旋转矩阵/欧拉角、以及点与坐标系变换（手系、单点、单轴、复杂点/轴）。**非 ROS2（core）** 与 **ROS2（ros2）** 分开说明。

## 0. 通用约定

- 右手系；单位：米、弧度。
- 欧拉角顺序：RPY（roll, pitch, yaw）。
- 四元数：(x, y, z, w)，与 ROS `geometry_msgs/Quaternion` 一致。
- 相机光学系：X 右、Y 下、Z 前（OpenCV/ROS 约定）。
- 内参 K：fx, fy, cx, cy；像素 (u,v)，深度 Z。

## 1. 坐标系

### 1.1 非 ROS2（core）

- **世界系 / 相机系 / 像素系**：纯数学定义；相机↔世界用 4×4 外参 T_w_c。
- **3D→2D（投影）**：u = fx·X/Z + cx，v = fy·Y/Z + cy；Z≤0 无效。
- **2D→3D（反投影）**：X = (u−cx)·Z/fx，Y = (v−cy)·Z/fy，Z = Z。
- **重投影误差**：e = √[(u_obs−u_proj)² + (v_obs−v_proj)²]（像素）。

### 1.2 ROS2

- 世界系：`map`/`odom`/`base_link`；相机系：`camera_optical_frame`。
- 内参来自 `sensor_msgs/CameraInfo`；相机↔世界用 TF 或 4×4 矩阵。
- ros2 层封装 core 的 project/unproject/reprojection_error。

## 2. 四元数、旋转矩阵、欧拉角

- **2.1 非 ROS2**：core 提供 quat↔R、quat↔RPY、R↔RPY；C++ 用 std::array，无 ROS 类型。
- **2.2 ROS2**：ros2 提供 `geometry_msgs::Quaternion`/Pose 与 core 互转；tf2::Quaternion setRPY/getRPY。

## 3. 同系与跨系变换

- **同系**：p' = R*p + t 或 p' = T*p（4×4）。
- **跨系**：p_B = T_B_A * p_A；ROS2 用 tf2::Buffer::lookupTransform、tf2_geometry_msgs::doTransform。

## 4. 坐标系变换详细（手系、单点、单轴、复杂点/轴）

- **手系**：右手系↔左手系用反射矩阵（如 diag(1,1,-1)）；接口 `handedness_transform`、`right_to_left`、`left_to_right`。
- **单点**：`transform_point(p, T)`。
- **单轴（方向）**：`transform_direction(v, R)` 仅旋转。
- **多点**：`transform_points(P, T)`。
- **完整架/位姿**：`transform_pose`、`transform_frame`。
- **多轴**：`transform_directions(V, R)`。
- **带位置轴（直线）**：`transform_line(point, direction, T)`。

## 5. 可视化

- **ROS2**：运行 `coord_tf_demo_node` 发布 TF；用 `coord_tf_visualization.launch.py` 启动节点+RViz2；在 RViz 中启用 TF 显示。
- **非 ROS2 C++**：运行 `export_visualization` 导出 PCD/CSV；若编译了 `opencv_process_data`，可导出 PNG/YAML。
- **非 ROS2 Python**：见 coordinate_transforms_py 包中 `run_visualization_demo.py`。

**输出目录**：上述工具与脚本生成的 PCD、CSV、PNG、YAML 等文件统一写入**输出目录**，默认为当前工作目录下的 `coord_tf_output`；可通过环境变量 `COORD_TF_OUTPUT_DIR` 指定其他路径（如 `export COORD_TF_OUTPUT_DIR=/path/to/output`）。目录不存在时会自动创建。

## 6. 可选分支：OpenCV 与过程数据保存

- 若编译时找到 OpenCV，会编译 `opencv_process_data`：读图/画 2D 点与重投影误差、保存 PNG 与 YAML。
- Python 包可选脚本 `run_opencv_process_demo.py`（opencv-python 为 extras）。
- 输出文件同样写入 `COORD_TF_OUTPUT_DIR` 或 `coord_tf_output`。
