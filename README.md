# aubo_ros2_ws 工作空间总结

本工作空间为基于 **ROS 2 Humble** 的奥博（Aubo）机械臂视觉抓取与运动规划系统，集成相机驱动、手眼标定、视觉位姿估计、抓取预测与 MoveIt 运动控制。

---

## 1. 目录结构概览

```
aubo_ros2_ws/
├── src/
│   ├── aubo_ros2_driver/          # 奥博机械臂驱动与 MoveIt 配置
│   ├── camport_ros2/              # 知微(Percipio) 相机驱动与接口
│   ├── coordinate_transforms/     # 坐标系与变换（C++：四元数/旋转矩阵/RPY、投影、点变换）
│   ├── coordinate_transforms_py/  # 同上 Python 实现（NumPy/scipy，与 C++ 约定一致）
│   ├── graspnet_ros2/             # GraspNet 6-DoF 抓取位姿预测
│   ├── graspnet-baseline/         # GraspNet 基线算法（Python 包名: graspnet）
│   ├── hand_eye_calibration/      # 手眼标定（Web UI）
│   └── visual_pose_estimation/   # 视觉位姿估计（C++/Python）
├── build/
├── install/
├── log/
└── test_coordinate_transforms.sh  # 坐标变换包：编译 + 安装/可执行/单元测试（可选 launch）
```

---

## 2. 功能包列表与说明

### 2.1 机械臂与运动控制（aubo_ros2_driver）

| 包名 | 类型 | 说明 |
|------|------|------|
| **aubo_description** | ament_cmake | 机械臂描述文件（URDF/XACRO）与网格 |
| **aubo_moveit_config** | ament_cmake | Aubo 与 MoveIt 的配置与启动（aubo_i5 等） |
| **aubo_msgs** | ament_cmake | 与机械臂控制器交互的消息与服务定义 |
| **aubo_dashboard_msgs** | ament_cmake | Dashboard 相关消息 |
| **aubo_ros2_trajectory_action** | ament_cmake | 轨迹 Action 服务（FollowJointTrajectory） |
| **demo_driver** | ament_cmake | 机器人状态与运动接口（MoveIt 规划/执行、位姿控制等） |
| **demo_interface** | ament_cmake | demo_driver 的消息/服务接口（含 ROS1 bridge 映射） |
| **feedback_bridge** | ament_cmake | FollowJointTrajectory Feedback 转 Action Feedback 桥接 |
| **aubo_demo** | ament_cmake | 调用 demo_driver 服务/话题的示例（C++） |

### 2.2 相机与图像（camport_ros2）

| 包名 | 类型 | 说明 |
|------|------|------|
| **percipio_camera** | ament_cmake | 知微相机驱动（深度/彩色、深度修复、MatViewer 等） |
| **percipio_camera_interface** | ament_cmake | 相机状态与控制的消息/服务接口 |
| **image_data_bridge** | ament_cmake | 将相机图像转换为统一 ImageData 消息格式的桥接节点 |

### 2.3 坐标系与变换（coordinate_transforms）

| 包名 | 类型 | 说明 |
|------|------|------|
| **coordinate_transforms** | ament_cmake | 坐标系与变换 C++ 库：四元数/旋转矩阵/RPY、2D↔3D 投影、点/架变换；可选 Eigen 分支 |
| **coordinate_transforms_py** | ament_python | 同上 Python 实现（NumPy/scipy），可选无 scipy 的 NumPy 分支；与 C++ 约定一致 |

### 2.4 视觉与抓取

| 包名 | 类型 | 说明 |
|------|------|------|
| **graspnet** | ament_python | GraspNet 基线算法（graspnet-baseline 的 Python 包） |
| **graspnet_ros2** | ament_python | GraspNet ROS2 封装：6-DoF 抓取位姿预测与相关节点 |
| **hand_eye_calibration** | ament_python | 单目相机手眼标定，基于 Web UI 的交互式标定 |
| **visual_pose_estimation** | ament_cmake | 基于单目模板匹配的工件 3D 抓取姿态估计（C++） |
| **visual_pose_estimation_python** | ament_cmake+python | 视觉位姿估计的 Python 实现 |
| **interface** | ament_cmake | 视觉位姿估计相关服务与消息接口 |

---

## 3. 主要启动与使用

### 3.1 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3.2 坐标变换包测试脚本

在工作空间根目录运行，使用系统 Python 与 ROS2（脚本会退出 conda/venv）：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
./test_coordinate_transforms.sh              # 完整：编译 + 安装检查 + 可执行 + launch + Python 单元测试
./test_coordinate_transforms.sh --no-build   # 跳过编译，仅运行测试
./test_coordinate_transforms.sh --no-launch # 跳过 launch 测试
```

测试项：环境检查、C++/Python 包编译与安装、C++/Python 可执行（export_visualization、run_core_demo、coord_tf_demo_node 等）、输出文件、Python core 单元测试（pytest 或内联运行）。

### 3.3 机械臂 + MoveIt（与 ROS1 桥接）

- **aubo_moveit_bridge_ros1.launch.py**：机械臂描述、MoveIt、控制器、RViz 等一体化启动（支持与 ROS1 桥接）。

### 3.4 相机

- **percipio_camera**：`percipio_camera.launch.py` 启动相机节点。
- **image_data_bridge**：将相机话题转为 ImageData 格式，供上位应用使用。

### 3.5 手眼标定

- **hand_eye_calibration_launch.py** / **hand_eye_calibration_tf.launch.py**：启动标定节点与 TF。

### 3.6 视觉位姿估计

- **visual_pose_estimation**：C++ 实现，对应 launch 在 `visual_pose_estimation/launch/`。
- **visual_pose_estimation_python**：Python 实现，`visual_pose_estimation_python.launch.py`。

### 3.7 GraspNet 抓取

- **graspnet_demo.launch.py** / **graspnet_demo_with_tf.launch.py**：抓取预测与 TF 演示。
- **percipio_camera_calibration.launch.py**：相机标定相关。

### 3.8 demo_driver 服务示例

- **aubo_demo**：示例程序，展示如何调用 demo_driver 的规划、执行、位姿控制等。

---

## 4. 依赖关系简图

```
aubo_demo ──► demo_interface, demo_driver
demo_driver ──► aubo_msgs, demo_interface, moveit_*, feedback_bridge(可选)
aubo_moveit_config ──► aubo_description, moveit_*, rviz2, controller_manager
feedback_bridge ──► control_msgs, aubo_msgs

coordinate_transforms ──► Eigen3(可选)
coordinate_transforms_py ──► numpy, scipy（可选无 scipy 的 NumPy 分支）

graspnet_ros2 ──► percipio_camera_interface, cv_bridge, sensor_msgs, ...
image_data_bridge ──► percipio_camera_interface, cv_bridge
hand_eye_calibration ──► interface, cv_bridge
visual_pose_estimation / visual_pose_estimation_python ──► interface, cv_bridge
```

---

## 5. 外部依赖要点

- **ROS 2**：Humble。
- **MoveIt 2**：运动规划与执行。
- **OpenCV**：图像处理与 GUI（percipio_camera 需链接 opencv_photo、opencv_highgui）。
- **coordinate_transforms**：C++ 可选 Eigen3；无 Eigen 时使用手写实现。Python 依赖 numpy、scipy（可选仅 NumPy 分支）。
- **GraspNet**：graspnet_ros2 依赖 graspnet-baseline（建议在 conda/独立环境中安装 numpy、torch、open3d、scipy、Pillow 等）。
- **手眼标定**：Flask、OpenCV、NumPy 等（见 hand_eye_calibration package.xml）。

---

## 6. 注意事项

1. **percipio_camera**：CMake 中需显式 `find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui photo)` 并链接 `OpenCV_LIBS`，否则 `list_devices` 等会报 `cv::inpaint`、`cv::imshow` 等未定义引用。
2. **visual_pose_estimation_python**：`setup.cfg` 中已使用 `script_dir`、`install_scripts`（下划线形式），避免 setuptools 弃用警告。
3. **ROS1 桥接**：与 ROS1 联合使用时需单独启动 ros1_bridge，并参考 aubo_moveit_config 中 bridge 相关 launch。

---

## 7. 代码推送（Git）

仓库远程地址：**https://github.com/MUHAN11-c/aubo_boot**

### 日常提交与推送

```bash
cd /home/mu/IVG2.0
git add -A
git commit -m "说明你做了哪些修改"
git push
```

### 首次推送或设置上游分支

```bash
cd /home/mu/IVG2.0
git push -u origin main
```

若推送被拒绝（例如因历史重写），可使用强制推送（仅当远程为自有仓库且确认无他人协作时使用）：

```bash
git push -u origin main --force
```

---

*文档根据当前 `src/` 下 package.xml 与 launch 结构整理，如有增删包或节点请同步更新本文档。*
