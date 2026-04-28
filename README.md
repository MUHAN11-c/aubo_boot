# IVG — 灵视智能视觉抓取系统

基于 **ROS 2 Humble** + **MoveIt 2** 的奥博（Aubo）机械臂视觉抓取与运动规划系统，集成相机驱动、手眼标定、视觉位姿估计、GraspNet 抓取预测与 Web 前端控制。

---

## 1. 工作空间架构

```
aubo_ros2_ws/src/
├── aubo_ros2_driver/               # 机械臂驱动与运动控制（10 个子包）
├── aubo_ros2_web_dashboard/        # 前端演示页面（FastAPI 网关 + ROSLIB.js）
├── robotwebtools/                  # 前端依赖库（roslibjs / ros3djs / ros2djs）
├── camport_ros2/src/               # 知微相机驱动（3 个子包）
├── graspnet_ros2/                  # VCoT-Grasp 大模型（6-DoF 抓取预测）
├── hand_eye_calibration/           # 自动手眼标定（Web UI）
├── visual_pose_estimation/         # 工件识别与抓取（3 个子包）
├── vision_perception/              # MediaPipe + YOLO OBB 感知
├── tool_changer_interface/         # 快换与手爪管理接口定义
├── tool_changer/                   # 快换与手爪管理（gripper0 ↔ gripper2）
└── coffee_latte_demo/              # 咖啡拉花演示（IO 控制 + Web 前端）
```

**11 个顶层功能包，23 个 ROS 2 子包。**

---

## 2. 功能包清单

### 2.1 机械臂驱动与运动控制 — `aubo_ros2_driver/`

| 子包 | 类型 | 职责 |
|------|------|------|
| **aubo_driver_ros2** | ament_cmake | ROS2 真机驱动：订阅 `moveItController_cmd`，发布 joint_states / robot_status / io_states，提供 set_io / get_ik / get_fk 服务 |
| **aubo_ros2_trajectory_action** | ament_cmake | FollowJointTrajectory action server：接收 MoveIt 轨迹 → `joint_path_command` |
| **aubo_robot_simulator_ros2** | ament_python | 轨迹插值：`joint_path_command` → 200Hz 五次多项式 → `moveItController_cmd` |
| **aubo_moveit_config** | ament_cmake | MoveIt 配置与统一 launch：move_group、RViz、驱动、插值 |
| **demo_driver** | ament_cmake | 应用层服务节点：move_to_pose / plan_trajectory / execute_trajectory / get_current_state / set_speed_factor / set_robot_pose / execute_grasp_pose / publish_grasps |
| **demo_interface** | ament_cmake | demo_driver 的消息与服务接口定义 |
| **aubo_description** | ament_cmake | 机器人模型（URDF/XACRO/mesh），含快换法兰与咖啡机配件 |
| **aubo_msgs** | ament_cmake | Aubo 驱动消息与服务（JointTrajectoryFeedback、SetIO、GetIK 等） |
| **aubo_dashboard_msgs** | ament_cmake | Dashboard 相关 msg/srv/action |
| **aubo_demo** | ament_cmake | 调用 demo_driver 服务/话题的 C++ 示例客户端与 Python 测试脚本 |

### 2.2 前端 — `aubo_ros2_web_dashboard/` + `robotwebtools/`

| 包 | 类型 | 职责 |
|----|------|------|
| **aubo_ros2_web_dashboard** | ament_python | FastAPI 网关（静态页 + 同源 rosbridge WS 代理 + MJPEG 流代理）+ ROSLIB.js SPA 前端 |
| **robotwebtools** | JS 库 | roslibjs / ros3djs / ros2djs 浏览器端 ROS 通信库 |

### 2.3 相机与图像 — `camport_ros2/src/`

| 子包 | 类型 | 职责 |
|------|------|------|
| **percipio_camera** | ament_cmake | 知微相机驱动（深度/彩色、点云、深度修复） |
| **percipio_camera_interface** | ament_cmake | 相机状态与控制的消息/服务接口 |
| **image_data_bridge** | ament_cmake | 相机图像 → 统一 ImageData 消息格式的桥接节点 |

### 2.4 视觉与抓取

| 包 | 类型 | 职责 |
|----|------|------|
| **graspnet_ros2** | ament_python | GraspNet 6-DoF 抓取位姿预测（含 graspnet-baseline） |
| **hand_eye_calibration** | ament_python | 单目相机手眼标定（Web UI 交互式） |
| **visual_pose_estimation** | ament_cmake | 基于模板匹配的工件 3D 抓取姿态估计（C++） |
| **visual_pose_estimation_python** | ament_python | 工件姿态估计 Python 实现 + FastAPI Web（`web_ui/`） |
| **interface** (VPE) | ament_cmake | 视觉位姿估计服务与消息接口 |
| **vision_perception** | ament_python | MediaPipe Holistic + YOLO OBB 人体/物体感知 |

### 2.5 快换与手爪 — `tool_changer_interface/` + `tool_changer/`

| 包 | 类型 | 职责 |
|----|------|------|
| **tool_changer_interface** | ament_cmake | 快换接口定义：RunGripperSwap / ChangeTool / GetCurrentTool 服务 + ToolChangerStatus / ToolIOStatus 消息 |
| **tool_changer** | ament_cmake | 夹爪快换 Worker（gripper0 ↔ gripper2 双向自动快换），提供 `/run_gripper_swap` 等服务 |

### 2.6 咖啡拉花 — `coffee_latte_demo/`

| 包 | 类型 | 职责 |
|----|------|------|
| **coffee_latte_demo** | ament_python | 咖啡拉花演示：IO 控制节点（对接 Aubo 驱动硬件 IO）、Web 前端面板、工具网格 |

---

## 3. 末端工具与 IO 引脚

| 工具 ID | 类型 | 所属包 | 用途 |
|---------|------|--------|------|
| **gripper0** | 气动夹爪 φ40 | tool_changer | 工件抓取（visual_pose_estimation），快换对接 |
| **gripper1** | 电动夹爪 A | coffee_latte_demo | 咖啡拉花（coffee_cup / milk_cup 载体），不参与快换 |
| **gripper2** | 电动夹爪 φ60 | tool_changer | AI 抓取（graspnet_ros2），快换对接 |

**IO 引脚映射**（`/aubo_driver/set_io(io_type=digital_output, io_index + 32 → 硬件引脚)`）：

| 逻辑引脚 | 硬件引脚 | 使用节点 | 用途 |
|---------|---------|---------|------|
| 6 | 38 | ExecuteGraspPose / PublishGrasps | 夹爪开/关 |
| 7 | 39 | GripperSwapWorker | 快换盘锁紧/释放 |
| 2 | 34 | LatteNode (DO2) | 打花开关 |
| 4 | 36 | LatteNode (DO4) | 咖啡开关 |

> **注意**：ExecuteGraspPoseWorker 的 IO 语义与 PublishGraspsClientWorker/ABWorker **相反**（`true=打开` vs `true=闭合`），源于气动夹爪不同工位的电气接线差异。

---

## 4. 数据流架构

```
相机 (percipio_camera)
  ├── /camera/color/image_raw ──→ visual_pose_estimation ──→ /estimate_pose (服务)
  ├── /camera/depth_registered/points ──→ graspnet_ros2 ──→ /grasp_poses_base (话题)
  └── /camera_status ──→ image_data_bridge ──→ /image_data

手眼标定 (hand_eye_calibration)
  └── TF: wrist3_Link → camera_frame（静态变换）

抓取执行 (demo_driver)
  ├── /execute_single_grasp ← 前端/API → /estimate_pose → MoveIt 运动 → /aubo_driver/set_io
  ├── /publish_grasps_worker_loop_control ← 前端 → /grasp_poses_base → 选优 → 抓取循环
  └── /loop_grasp_control ← 前端 → 循环抓取控制

快换管理 (tool_changer)
  ├── /run_gripper_swap ← 前端 → 放旧爪 → 取新爪 → 回安全位
  └── /tool_changer_status → 前端状态显示

咖啡拉花 (coffee_latte_demo)
  ├── /set_latte_do2 ← 前端 → /aubo_driver/set_io(io_index=2)
  ├── /set_latte_do4 ← 前端 → /aubo_driver/set_io(io_index=4)
  └── /latte_di_status → 前端 ← /aubo_driver/io_states

运动控制链路
  move_group → follow_joint_trajectory (action)
    → aubo_ros2_trajectory_action → joint_path_command (topic)
    → aubo_robot_simulator (200Hz 插值) → moveItController_cmd (topic)
    → aubo_driver_ros2 → 真实机械臂

前端 (aubo_ros2_web_dashboard)
  浏览器 ← rosbridge WebSocket (:9090) ← ROS 2
  浏览器 ← web_video_server (:8089) ← MJPEG 流
  浏览器 ← FastAPI 网关 (:8090) ← 静态页 + 同源代理
```

---

## 5. 构建与运行

### 5.1 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash   # MoveIt 2 工作空间（如有）
colcon build
source install/setup.bash
```

### 5.2 一键启动（推荐）

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
./start_IVG_graspnet_points_fastapi_web_legacy.sh
```

该脚本按依赖顺序在 terminator 分屏终端中启动 **16 步**完整流程：

| 步骤 | 内容 | 包 |
|------|------|-----|
| 1 | 真实机械臂驱动 + MoveIt + RViz | aubo_moveit_config |
| 2 | demo_driver 基础服务（5 个） | demo_driver |
| 3 | 相机节点 | percipio_camera |
| 4 | 相机控制（软件触发） | percipio_camera_interface |
| 5 | 图像数据桥接 | image_data_bridge |
| 6 | 手眼标定 | hand_eye_calibration |
| 7 | 视觉姿态估计（Python） | visual_pose_estimation_python |
| 8 | GraspNet 点云节点 | graspnet_ros2 |
| 9 | 抓取位姿执行 Worker | demo_driver |
| 10 | 夹爪快换 Worker | tool_changer |
| 11 | 咖啡拉花演示 | coffee_latte_demo |
| 12 | GraspNet 循环抓取 Worker | demo_driver |
| 13 | 关键服务就绪校验 | — |
| 14 | FastAPI Web 服务 | visual_pose_estimation_python |
| 15 | IVG Web 网关（静态站） | aubo_ros2_web_dashboard |
| 16 | rosbag 录制 | — |

### 5.3 单独启动

```bash
# 仅机械臂 + MoveIt
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py

# 仅 demo_driver 服务
ros2 launch aubo_moveit_config demo_driver_services.launch.py

# 仅快换 Worker
ros2 launch tool_changer gripper_swap_worker.launch.py

# 仅咖啡拉花
ros2 launch coffee_latte_demo coffee_latte_demo.launch.py

# 仅 Web 网关
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
```

### 5.4 访问地址

| 面板 | URL |
|------|-----|
| 门户首页 | `http://127.0.0.1:8090/index.html` |
| 视觉抓取 | `http://127.0.0.1:8090/vision_grasp_panel.html` |
| 咖啡拉花 | `http://127.0.0.1:8090/coffee_latte_panel.html` |
| TF 监控 | `http://127.0.0.1:8090/tf_monitor_panel.html` |
| 设置 | `http://127.0.0.1:8090/settings_panel.html` |
| VPE FastAPI | `http://127.0.0.1:8088/` |
| 手眼标定 | `http://127.0.0.1:8080/` |

---

## 6. 依赖关系

```
前端 (JS/HTML)
  └── rosbridge ← ROS 2 话题/服务

aubo_ros2_web_dashboard
  └── rosbridge_suite, tf2_web_republisher, web_video_server

demo_driver
  ├── demo_interface, aubo_msgs
  ├── interface (VPE 服务类型)
  └── moveit_core, moveit_ros_planning_interface

tool_changer
  ├── tool_changer_interface, demo_interface (SetRobotIO)
  └── moveit_core, moveit_ros_planning_interface

coffee_latte_demo
  ├── demo_interface (SetRobotIO, RobotIOStatus)
  └── std_srvs

graspnet_ros2
  └── percipio_camera_interface, cv_bridge, sensor_msgs

hand_eye_calibration
  └── interface (VPE), demo_interface, percipio_camera_interface

visual_pose_estimation / _python
  ├── interface (VPE)
  ├── demo_interface, tool_changer_interface
  └── cv_bridge, FastAPI (Python)
```

---

## 7. 外部依赖

| 依赖 | 用途 |
|------|------|
| ROS 2 Humble | 核心框架 |
| MoveIt 2 | 运动规划与执行 |
| OpenCV | 图像处理（percipio_camera 需链接 photo/highgui） |
| GraspNet-baseline | 6-DoF 抓取预测（需 torch/open3d/scipy/Pillow） |
| FastAPI + uvicorn | VPE Web 服务 + Dashboard 网关 |
| rosbridge_suite | 浏览器 ↔ ROS 2 WebSocket 桥 |
| web_video_server | MJPEG 相机流 HTTP 服务 |
| terminator | 多终端分屏启动 |

---

## 8. 注意事项

1. **percipio_camera**：CMake 需显式 `find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui photo)` 并链接 `OpenCV_LIBS`，否则未定义引用 `cv::inpaint`、`cv::imshow`。

2. **MoveIt CurrentStateMonitor 回调竞争**：`/execute_single_grasp` 服务曾与默认 `MutuallyExclusive` 回调组竞争，长耗时服务回调导致 `CurrentStateMonitor` 状态时间戳不更新 → `Failed to fetch current robot state`。已修复：将抓取服务放入独立 `service_cb_group_`。

3. **IO 引脚语义差异**：ExecuteGraspPoseWorker（`true=打开`）与 PublishGraspsClientWorker/ABWorker（`true=闭合`）的夹爪 IO 语义相反，源于不同工位的电气接线。硬件接线变更时需同步更新代码中的 IO 逻辑。

4. **工具快换**：gripper0 ↔ gripper2 双向快换由 `tool_changer` 包独立管理，`/run_gripper_swap` 服务需在机械臂驱动就绪后单独启动。

5. **GraspNet Z 轴 180° 翻转**：GraspNet 预测位姿的 Z 轴与机器人末端执行器相反，所有 GraspNet 消费者均通过 `applyGraspZFlip180()` 自动修正。

---

## 9. Git 仓库

```bash
cd /home/mu/IVG2.0
git remote -v                                    # https://github.com/MUHAN11-c/aubo_boot

git add -A
git commit -m "描述你的修改"
git push -u origin main
```

---

*文档与 `src/` 下 package.xml 及启动脚本保持同步。新增/删除包或节点时请同步更新本文档。*
