# aubo_ros2_ws 工作空间总结

本工作空间为基于 **ROS 2 Humble** 的奥博（Aubo）机械臂视觉抓取与运动规划系统，集成相机驱动、手眼标定、视觉位姿估计、抓取预测与 MoveIt 运动控制。

**更完整、与源码同步的说明**（22 个包清单、`demo_driver` 可执行节点表、`start_IVG_graspnet_points_fastapi.sh` 步骤与环境变量等）见：**`aubo_ros2_ws/README.md`**。

---

## 1. 目录结构概览

```
aubo_ros2_ws/
├── src/
│   ├── aubo_ros2_driver/          # 奥博机械臂驱动与 MoveIt 配置、demo_driver 等
│   ├── camport_ros2/src/          # percipio_camera / interface / image_data_bridge
│   ├── coordinate_transforms/     # 坐标系与变换（C++）
│   ├── coordinate_transforms_py/  # 同上 Python（与 C++ 约定一致）
│   ├── graspnet_ros2/             # GraspNet 6-DoF（内含 graspnet-baseline）
│   ├── hand_eye_calibration/      # 手眼标定（Web UI）
│   └── visual_pose_estimation/
│       └── src/                   # interface、visual_pose_estimation(C++)、visual_pose_estimation_python
├── rosbags/                       # 可选录包目录
├── build/  install/  log/
├── start_IVG.sh
├── start_IVG_graspnet_points.sh
├── start_IVG_graspnet_points_fastapi.sh
└── test_coordinate_transforms.sh
```

当前 `aubo_ros2_ws/src` 下 **22** 个 ROS 2 包（以各 `package.xml` 为准，含 `aubo_description/world_map`）。

未纳入本仓库的第三方参考（如 UR5e MoveIt 抓取示例）见 `aubo_ros2_ws/src/graspnet_ros2/doc/` 等文档中的外链。

---

## 2. 功能包列表与说明

### 2.1 机械臂与运动控制（aubo_ros2_driver）

| 包名 | 类型 | 说明 |
|------|------|------|
| **aubo_description** | ament_cmake | 机械臂描述（URDF/XACRO、网格）；子目录 **`world_map/`** 为遗留 catkin 场景包，通常不参与 colcon |
| **aubo_moveit_config** | ament_cmake | 主 MoveIt 配置与 launch（纯 ROS2、`demo_driver_services` 等） |
| **aubo2_moveit_config** | ament_cmake | 另一套 MoveIt 配置（如 aubo2 相关 launch） |
| **aubo_msgs** | ament_cmake | 与机械臂控制器交互的消息与服务定义 |
| **aubo_dashboard_msgs** | ament_cmake | Dashboard 相关消息 |
| **aubo_ros2_trajectory_action** | ament_cmake | FollowJointTrajectory → `joint_path_command` |
| **aubo_driver_ros2** | ament_cmake | ROS2 真机驱动（`moveItController_cmd`、joint_states 等） |
| **aubo_robot_simulator_ros2** | ament_python | 轨迹插值：`joint_path_command` → `moveItController_cmd` |
| **demo_driver** | ament_cmake | 机器人状态与运动接口（MoveIt 规划/执行、位姿控制、抓取 Worker 等）；依赖 **`interface`**（`package.xml`）；可执行列表见 **`aubo_ros2_ws/README.md` 2.1.1** |
| **demo_interface** | ament_cmake | demo_driver 的消息/服务接口 |
| **aubo_demo** | ament_cmake | 调用 demo_driver 的 C++ 示例可执行文件与 `scripts/` 测试脚本 |

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
| **graspnet_ros2** | ament_python | GraspNet ROS2 封装：6-DoF 抓取位姿预测（含 graspnet-baseline，随包安装） |
| **hand_eye_calibration** | ament_python | 单目相机手眼标定，基于 Web UI 的交互式标定 |
| **visual_pose_estimation** | ament_cmake | 基于单目模板匹配的工件 3D 抓取姿态估计（C++） |
| **visual_pose_estimation_python** | ament_python | 视觉位姿估计 Python 实现 + FastAPI/Web（`web_ui/`） |
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

### 3.3 机械臂 + MoveIt

- **aubo_moveit_pure_ros2.launch.py**：机械臂描述、MoveIt、驱动、插值、RViz 等一体化启动（推荐）。
- **`demo_driver_services.launch.py`**：`demo_driver` 侧 MoveIt 服务（`/move_to_pose`、`/plan_trajectory` 等），与抓取 Worker 分终端启动；细节见 `aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/README.md`。

### 3.4 相机

- **percipio_camera**：`percipio_camera.launch.py` 启动相机节点。
- **image_data_bridge**：将相机话题转为 ImageData 格式，供上位应用使用。

### 3.5 手眼标定

- **hand_eye_calibration_launch.py** / **hand_eye_calibration_tf.launch.py**：启动标定节点与 TF。

### 3.6 视觉位姿估计

- **visual_pose_estimation**：C++ 实现，对应 launch 在 `visual_pose_estimation/launch/`。
- **visual_pose_estimation_python**：Python 实现，`visual_pose_estimation_python.launch.py`。

### 3.7 GraspNet 抓取

- **graspnet_ros2**：文件 Demo（`graspnet_demo.launch.py` 等）、点云 Demo（`graspnet_demo_points.launch.py`）；现场 IVG 常用 **`graspnet_demo_points_with_tf.launch.py`**（与 `demo_driver`、FastAPI 脚本联动，常配 `launch_camera:=false`）。详见 `aubo_ros2_ws/src/graspnet_ros2/README.md`。
- **percipio_camera_calibration.launch.py**（graspnet_ros2 包内）：相机标定相关 launch。

### 3.8 demo_driver 服务示例

- **aubo_demo**：示例程序，展示如何调用 demo_driver 的规划、执行、位姿控制等。

---

## 4. 依赖关系简图

```
aubo_demo ──► demo_interface, demo_driver
demo_driver ──► aubo_msgs, demo_interface, interface, moveit_*
aubo_moveit_config ──► aubo_description, moveit_*, rviz2, controller_manager
aubo_driver_ros2 ──► aubo_msgs, demo_interface

coordinate_transforms ──► Eigen3(可选)
coordinate_transforms_py ──► numpy, scipy（可选无 scipy 的 NumPy 分支）

graspnet_ros2 ──► percipio_camera_interface, cv_bridge, sensor_msgs, ...
image_data_bridge ──► percipio_camera_interface, cv_bridge
hand_eye_calibration ──► interface, demo_interface, percipio_camera_interface, cv_bridge
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
- **visual_pose_estimation_python**：FastAPI、uvicorn 等（见该包 `package.xml` 与 `web_ui/`）。

---

## 6. 注意事项

1. **percipio_camera**：CMake 中需显式 `find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui photo)` 并链接 `OpenCV_LIBS`，否则 `list_devices` 等会报 `cv::inpaint`、`cv::imshow` 等未定义引用。
2. **visual_pose_estimation_python**：`setup.cfg` 中已使用 `script_dir`、`install_scripts`（下划线形式），避免 setuptools 弃用警告。
3. **关于“`joint_state_count` 很高 + 最新帧毫秒级，但 `getCurrentPose` 仍失败”**：这通常不是“机器人没有发 `joint_states`”，而是 **MoveIt 的 `CurrentStateMonitor` 回调没被执行到**。  
   在本项目中，`/execute_single_grasp` 服务曾与默认 `MutuallyExclusive` 回调组竞争，长耗时服务回调占住执行槽位后，MoveIt 订阅虽然有新消息进入中间件，`joint_state_count` 也会持续增长，但 `CurrentStateMonitor` 内部状态时间戳不更新，最终触发 `Failed to fetch current robot state` / `getCurrentPose` 失败。  
   已验证的修复是：将抓取服务放入独立 `service_cb_group_`，避免阻塞默认组；同时保留估姿客户端的独立回调组配置，保证服务回调内异步等待期间状态更新可并发处理。

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
