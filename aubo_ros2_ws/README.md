# aubo_ros2_ws 工作空间说明

基于 **ROS 2 Humble**（可用 `ROS_DISTRO_NAME` 覆盖）的奥博（Aubo）机械臂系统：MoveIt 2、真机驱动、知微（Percipio）相机、手眼标定、坐标变换、视觉位姿估计（C++/Python + FastAPI）、GraspNet 点云抓取，以及 `demo_driver` 侧规划/执行与抓取 Worker。

当前 `src/` 下共 **22** 个 ROS 2 包（以各目录内 `package.xml` 为准，含 `aubo_description/world_map` 独立包）。

---

## 1. 目录结构概览

```
aubo_ros2_ws/
├── src/
│   ├── aubo_ros2_driver/                 # 机械臂描述、MoveIt、驱动、demo、消息、仿真、demo_driver
│   ├── camport_ros2/src/                 # percipio_camera / percipio_camera_interface / image_data_bridge
│   ├── coordinate_transforms/            # 坐标变换 C++ 库与示例节点
│   ├── coordinate_transforms_py/         # 与 C++ 约定一致的 Python 实现
│   ├── graspnet_ros2/                    # GraspNet ROS 2 封装（内含 graspnet-baseline，随包安装）
│   ├── hand_eye_calibration/             # 手眼标定 Web UI + ROS 2 节点
│   └── visual_pose_estimation/
│       └── src/
│           ├── interface/                # 视觉相关 msg/srv 等接口定义
│           ├── visual_pose_estimation/   # 位姿估计算法（C++）
│           └── visual_pose_estimation_python/  # Python 节点 + FastAPI/Web（web_ui/）
├── rosbags/                              # 录包输出目录（如 ivg_session，脚本可覆盖）
├── start_IVG.sh                          # 手眼等整栈（terminator 多标签）
├── start_IVG_graspnet_points.sh          # 点云 GraspNet 路线整栈
├── start_IVG_graspnet_points_fastapi.sh  # IVG 全栈：点云 GraspNet + VPE FastAPI + rosbag 等
├── start_IVG_graspnet_points_fastapi_web_legacy.sh   # 在上者基础上第 14 步：aubo_ros2_web_dashboard（8090 + rosbridge）
├── start_IVG_web_dashboard.sh            # （若仓库中保留）仅替换 FastAPI 为 Web 控制台；或直接用上一脚本「全栈 + 控制台」
├── test_coordinate_transforms.sh         # coordinate_transforms(_py) 编译与测试脚本
├── build/  install/  log/                # colcon 产物
```

说明：

- **`aubo_description/world_map/`**：遗留 **catkin** 格式场景包，一般不作为主工作流的一部分参与 colcon。
- **GraspNet 训练用大数据**（历史 `graspnet-baseline/dataset/tolerance/`）已可按需移除；**现场推理**仅需权重 `logs/log_kn/checkpoint-rs.tar` 与模型代码，详见 `src/graspnet_ros2/README.md`。
- 仿真与 RViz 联合调试的补充说明见 `src/graspnet_ros2/doc/RVIZ2_GAZEBO_SIM_REFERENCE.md`（该路径含 `doc/`，未并入 graspnet 主 README 正文）。
- **`demo_driver`** 的 GraspNet/运动排障长文见包内 `src/aubo_ros2_driver/demo_driver/docs/`（含 `docs/` 子目录，与包根 `README.md` 分工）。

---

## 2. 功能包列表（与源码位置一致）

### 2.1 机械臂与运动控制（`src/aubo_ros2_driver/`）

| 包名 | 构建类型 | 说明 |
|------|----------|------|
| **aubo_description** | ament_cmake | URDF/XACRO、网格；子目录 **`world_map/`** 为可选 catkin 场景包 |
| **aubo_moveit_config** | ament_cmake | 主 MoveIt 配置：`aubo_moveit_pure_ros2.launch.py`、`demo_driver_services.launch.py` 等（详见该包 `launch/`） |
| **aubo2_moveit_config** | ament_cmake | 另一套 MoveIt 配置（如 aubo2 相关 launch） |
| **aubo_msgs** | ament_cmake | 与控制器交互的 msg/srv |
| **aubo_dashboard_msgs** | ament_cmake | Dashboard 相关 msg/srv/action |
| **aubo_ros2_trajectory_action** | ament_cmake | `FollowJointTrajectory` → `joint_path_command` |
| **aubo_driver_ros2** | ament_cmake | 真机驱动：`moveItController_cmd`、`joint_states` 等 |
| **aubo_robot_simulator_ros2** | ament_python | 订阅 `joint_path_command`，插值发布 `moveItController_cmd` |
| **demo_driver** | ament_cmake | MoveIt 规划/执行、位姿与速度服务、夹爪快换、抓取 Worker（C++）；**服务列表与 IVG 启动见包内 `demo_driver/README.md`** |
| **demo_interface** | ament_cmake | `demo_driver` 的 msg/srv 定义 |
| **aubo_demo** | ament_cmake | 调用 `demo_driver` 的 C++ 示例与 `scripts/` 工具脚本 |

#### 2.1.1 `demo_driver` 可执行节点（与 `CMakeLists.txt` 当前启用项一致）

| 可执行名 | 作用摘要 |
|----------|----------|
| `robot_status_publisher_node` | 机器人状态发布 |
| `move_to_pose_server_node` | 关节空间到位姿（`/move_to_pose`） |
| `plan_trajectory_server_node` | `/plan_trajectory` |
| `execute_trajectory_server_node` | `/execute_trajectory` |
| `get_current_state_server_node` | `/get_current_state` |
| `set_speed_factor_server_node` | `/set_speed_factor` |
| `set_robot_pose_server_node` | `/set_robot_pose` |
| `gripper_swap_worker_node` | 夹爪快换 `/run_gripper_swap` |
| `publish_grasps_client_worker_node` | GraspNet 循环抓取 Worker（与 `/graspnet_capture_control`、`/publish_grasps_worker_loop_control` 等联调） |
| `execute_grasp_pose_worker_node` | 单次/循环抓取 `/execute_single_grasp`、`/loop_grasp_control` |

**本包内 launch**（`demo_driver/launch/`）：`execute_grasp_pose_worker.launch.py`、`moveit2_tcp_pose_publisher.launch.py`。  
**整机 MoveIt + 多服务聚合启动**一般由 **`aubo_moveit_config`** 的 launch 完成（与 `start_IVG_graspnet_points_fastapi.sh` 步骤 1–2 一致）。

### 2.2 相机与图像（`src/camport_ros2/src/`）

| 包名 | 说明 |
|------|------|
| **percipio_camera** | 相机节点（深度/彩色、点云、参数等） |
| **percipio_camera_interface** | 状态、参数、软触发等接口节点 |
| **image_data_bridge** | 彩色图桥接（launch 中常 `input_image_topic:=/camera/color/image_raw`） |

### 2.3 坐标系与变换

| 包名 | 说明 |
|------|------|
| **coordinate_transforms** | C++：四元数/旋转矩阵/RPY、2D↔3D、点/坐标系变换；可选 Eigen3 |
| **coordinate_transforms_py** | Python（NumPy / 可选 scipy），API 与 C++ 对齐 |

### 2.4 视觉、手眼与抓取

| 包名 | 路径 | 说明 |
|------|------|------|
| **interface** | `visual_pose_estimation/src/interface` | 视觉相关公共接口（`demo_driver`、`hand_eye_calibration`、`visual_pose_estimation*` 等依赖） |
| **visual_pose_estimation** | `visual_pose_estimation/src/visual_pose_estimation` | 位姿估计 C++ 节点 |
| **visual_pose_estimation_python** | `visual_pose_estimation/src/visual_pose_estimation_python` | Python 节点 + **`web_ui/`**（FastAPI、配置、前端）；`package.xml` 声明 `python3-fastapi`、`python3-uvicorn` 等 |
| **hand_eye_calibration** | `hand_eye_calibration` | Web 手眼标定（默认浏览器端口多为 **8080**，以 launch 为准）；依赖 `interface`、`demo_interface`、`percipio_camera_interface` |
| **graspnet_ros2** | `graspnet_ros2` | 点云/文件 Demo、`graspnet_demo_points_with_tf.launch.py` 等；**完整说明见包内 `README.md`** |

---

## 3. 构建与环境

```bash
cd /path/to/aubo_ros2_ws
source /opt/ros/humble/setup.bash
# 若本机有独立 MoveIt 覆盖工作空间（与启动脚本一致）：
# source ~/ws_moveit/install/setup.bash
colcon build
source install/setup.bash
```

`start_IVG_graspnet_points_fastapi.sh` 中默认还会在步骤 **0** 执行一次全工作空间 **`colcon build`**，环境链为：`/opt/ros/<ROS_DISTRO_NAME>` →（若存在）`~/ws_moveit/install/setup.bash` → 本仓库 `install/setup.bash`。可通过 **`AUBO_ROS2_WS`**、**`ROS_DISTRO_NAME`** 覆盖路径与发行版。

---

## 4. 现场启动脚本（工作空间根目录）

| 脚本 | 用途 |
|------|------|
| **start_IVG.sh** | 手眼/相机/MoveIt 等整栈（无 GraspNet FastAPI 扩展） |
| **start_IVG_graspnet_points.sh** | 在上一类基础上按「点云 GraspNet」路线组织多标签启动 |
| **start_IVG_graspnet_points_fastapi.sh** | **IVG 全栈**：机械臂 + `demo_driver` + 相机 + 手眼 + 视觉位姿（Python）+ **GraspNet 点云（`graspnet_demo_points_with_tf`，`launch_camera:=false`）** + 抓取 Worker + **VPE FastAPI Web（8088）** + 可选 rosbag |
| **start_IVG_graspnet_points_fastapi_web_legacy.sh** | 与 **`start_IVG_graspnet_points_fastapi.sh` 相同**，另在 FastAPI 之后增加 **`ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py`**（**rosbridge + tf2_web_republisher + 静态页**；**`WEB_DASH_HOST`** / **`WEB_DASH_PORT`** / **`ROSBRIDGE_PORT`** / **`IVG_STRIP_PROXY_FOR_DASH_LAUNCH`** 等）；**rosbag 为第 15 步**；结束时打印本机与局域网链接 |
| **start_IVG_web_dashboard.sh** | （若仓库中保留）与 **start_IVG_graspnet_points_fastapi.sh** 步骤 **0–12、14** 相同；第 **13** 步为 **`web_dashboard.launch.py`**（默认 HTTP **8090**，WebSocket **9090**） |

**共性要求**：已安装 **terminator**（`sudo apt install terminator`），脚本在独立标签页中启动各节点。

**`start_IVG_graspnet_points_fastapi.sh` 步骤概览（与脚本一致）**：

0. `colcon build`  
1. `aubo_moveit_pure_ros2.launch.py`  
2. `demo_driver_services.launch.py`  
3. `percipio_camera.launch.py`  
4. `camera_control.launch.py`  
5. `image_data_bridge.launch.py`（默认 `input_image_topic:=/camera/color/image_raw`）  
6. `hand_eye_calibration_launch.py`  
7. `visual_pose_estimation_python.launch.py`  
8. `graspnet_demo_points_with_tf.launch.py`（`launch_camera:=false`，`launch_hand_eye_tf:=true`）  
9. `execute_grasp_pose_worker.launch.py`  
10. `gripper_swap_worker_node`  
11. `publish_grasps_client_worker_node`  
12. 关键 service 列表自检（`rg`）  
13. `visual_pose_estimation_web.launch.py`（默认 **8088**，`WEB_HOST` / `WEB_PORT`）  
14. `ros2 bag record` → `rosbags/ivg_session`（**`IVG_ROSBAG_DIR`** / **`IVG_ROSBAG_TOPICS`**）

**`start_IVG_graspnet_points_fastapi_web_legacy.sh`**：步骤 **0–13** 与上表相同；**14**. `ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py`；**15**. `ros2 bag record`（同上）。

**`start_IVG_web_dashboard.sh`**（若仓库中保留该脚本）：步骤 0–12、14 与 **`start_IVG_graspnet_points_fastapi.sh`** 对应步骤相同；步骤 **13** 为 `ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py`，常用环境变量：**`WEB_DASH_HOST`**、**`WEB_DASH_PORT`**（默认 **8090**）、**`ROSBRIDGE_PORT`**（默认 **9090**，仅本机 rosbridge；浏览器经网关 **`/ws/rosbridge` 转发**，不必直连 9090）、**`IVG_STRIP_PROXY_FOR_DASH_LAUNCH`**。系统 deb：**`ros-humble-rosapi`**、**`ros-humble-rosbridge-suite`**、**`ros-humble-tf2-web-republisher`**。

**Web 入口（典型）**：

- 手眼标定界面：**http://localhost:8080**（脚本结束提示）  
- 视觉位姿 FastAPI：**http://127.0.0.1:8088**（`start_IVG_graspnet_points_fastapi.sh` 可用 `WEB_HOST` / `WEB_PORT` 修改）  
- Web 面板（本包 **FastAPI** 网关：`web/public/` + **转发** rosbridge WS 与 MJPEG）：**http://127.0.0.1:8090/**（一般**只需开放此端口**）
- ROS 控制台（话题订阅可视化、服务调用、动作与参数列表、节点关系图、2D 地图/雷达、3D 点云/雷达；**IVG 快捷条**对齐 IVG 默认话题与服务）：**http://127.0.0.1:8090/topics_lab.html**（亦可通过 **`start_IVG_graspnet_points_fastapi_web_legacy.sh`** 一并拉起）

`aubo_ros2_web_dashboard` 包含静态前端（`web/public/`）与 **ROS 2 包元数据 + launch**：HTTP 由 **`fastapi_static_gateway`**（Uvicorn）托管，并默认将浏览器 **WebSocket / 视频流** 转发至本机 **rosbridge_suite** 与 **web_video_server**（详见该包 README）。**不包含** IVG 业务侧独立 FastAPI（如 graspnet 位姿服务）。

---

## 5. 其他常用命令

**坐标变换包测试**（脚本会处理 conda/venv，详见脚本内注释）：

```bash
cd /path/to/aubo_ros2_ws
source /opt/ros/humble/setup.bash
./test_coordinate_transforms.sh
./test_coordinate_transforms.sh --no-build
```

**各子系统细节**：

- 机械臂服务、抓取 Worker、话题约定：`src/aubo_ros2_driver/demo_driver/README.md`  
- GraspNet 参数、坐标系、点云节点：`src/graspnet_ros2/README.md`  
- 仓库级变更与注意事项（OpenCV、MoveIt 回调组、`joint_state_count` 等）：若本工作空间位于 **IVG2.0** 仓库内，见**上一级目录**的 `README.md`；亦可能有 `CHANGELOG_IVg到IVG2.0纯ROS2移植_*.md` 等根级文档。

---

## 6. 依赖关系简图（与当前 `package.xml` 一致，随源码演变）

```
aubo_demo ──► demo_interface, demo_driver
demo_driver ──► aubo_msgs, demo_interface, interface, moveit_*（经 MoveIt 配置与 launch）
aubo_moveit_config ──► aubo_description, moveit_*, rviz2, ros2_control 相关
aubo_driver_ros2 ──► aubo_msgs, demo_interface

coordinate_transforms ──► Eigen3（可选）
coordinate_transforms_py ──► numpy、scipy（可选）

graspnet_ros2 ──► sensor_msgs, geometry_msgs, tf2_* 等（点云版常与 hand_eye / 外部相机 launch 联用）
image_data_bridge ──► percipio_camera_interface, cv_bridge
hand_eye_calibration ──► interface, demo_interface, percipio_camera_interface, cv_bridge
visual_pose_estimation / visual_pose_estimation_python ──► interface, cv_bridge
```

---

## 7. 外部依赖要点

- **ROS 2**（默认 Humble）、**MoveIt 2**、**ros2_control**（按 launch 与 `aubo_moveit_config` 配置）。
- **OpenCV**：`percipio_camera` 建议在 CMake 中链接 `core imgproc highgui photo` 等组件（上一级仓库 `README.md` 中若有说明可对照）。
- **GraspNet**：`torch`、`open3d`、`scipy` 等建议在 conda/venv 中预装；权重路径见 `graspnet_ros2` README。
- **手眼 Web**：Flask、OpenCV、NumPy 等（见 `hand_eye_calibration/package.xml`）。
- **视觉 Python Web**：FastAPI、uvicorn 等（见 `visual_pose_estimation_python/package.xml` 与 `web_ui/`）。

---

## 8. 注意事项（摘录）

1. **`/execute_single_grasp` 与 `getCurrentPose` / 当前状态异常**：若 `joint_state_count` 持续增长但 MoveIt 仍报无法获取机器人状态，可能与**服务回调与默认互斥回调组**有关；长耗时服务使用**独立回调组**的修复说明见**上一级仓库 `README.md`** 中相关小节（若存在）。

---

*修改 `src/` 下包名、launch、`demo_driver` 可执行列表或启动脚本时，请同步更新本文件及上一级仓库的总览文档。*
