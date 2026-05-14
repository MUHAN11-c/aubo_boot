# IVG — 灵视智能视觉抓取系统

基于 **ROS 2 Humble** + **MoveIt 2** 的奥博（Aubo）机械臂视觉抓取与运动规划系统，集成相机驱动、手眼标定、视觉位姿估计、GraspNet 抓取预测与 Web 前端控制。

> **新机器部署请参阅 [DEPLOYMENT.md](DEPLOYMENT.md)** — 包含完整的环境搭建、依赖安装、编译构建和启动流程喵~

---

## 0. 硬件配置与网络信息

### 0.1 硬件清单

| 设备 | 型号 | 规格 | 连接方式 |
|------|------|------|---------|
| **机械臂** | AUBO E5 | 6-DOF, 负载 5kg, 臂展 886mm, 重复定位精度 ±0.02mm | 以太网直连 (TCP/IP) |
| **控制器** | AUBO 控制箱 | 集成伺服驱动 + 安全模块 + IO 板 | 以太网直连 |
| **深度相机** | Percipio FM830 | 深度 1280×960 + 彩色 1280×960, 帧率 30fps, 工作距离 0.2–2m | USB 3.0 |
| **GPU** | NVIDIA GeForce RTX 3090 | 24GB VRAM, CUDA 12.6, 驱动 595.58.03 | PCIe 4.0 ×16 |
| **工控机** | 组装台式机 | Ubuntu 22.04.5 LTS, Kernel 6.8.0-111, Python 3.10.12 | — |
| **气动夹爪 40** | gripper0 | 行程 40mm, 平行开合, 气压 0.2–0.7MPa | 快换盘 + 气管 |
| **电动夹爪 60** | gripper2 | 行程 60mm, 伺服控制, 24V DC | 快换盘 + 供电/信号 |
| **电动夹爪 A** | gripper1 | 行程可调, 伺服控制, 24V DC | 法兰固定 (不参与快换) |
| **快换盘** | — | 气动锁紧/释放, 定位精度 ±0.01mm | IO index 7 (硬件引脚 39) |

### 0.2 网络与 IP 配置

| 项目 | 地址 | 说明 |
|------|------|------|
| **机械臂控制器 IP** | `169.254.10.98` | 链路本地地址 (Link-Local)，控制器默认静态 IP |
| **机械臂控制端口** | `8899` | TCP 端口，SDK 双连接 (conn_control_ + conn_status_) |
| **工控机机械臂侧 IP** | `169.254.10.x` (自动) | 由 avahi-autoipd 或 DHCP 分配，需与控制器同子网 |
| **工控机物理网口** | `enp4s0` / 板载网口 | 直连机械臂控制器，不经过交换机 |
| **AUBO SDK 版本** | libauborobotcontroller.so.1.3.1 | 64-bit, 预编译，路径 `aubo_driver_ros2/lib/lib64/aubocontroller/` |

**连接验证**：
```bash
ping 169.254.10.98                          # 链路可达
nc -zv 169.254.10.98 8899                   # TCP 端口可达
```
启动脚本 `start_aubo_new_driver.sh` 在步骤 1 前自动执行 TCP 端口探测，不可达时回退到仿真模式 (mock_components/GenericSystem) 喵~

### 0.3 IO 引脚完整分配

机械臂控制箱提供数字量 IO（输入/输出）和模拟量 IO。ROS 2 端通过 `/aubo_driver/set_io` 服务控制输出引脚。

**IO 寻址规则**：`io_type=digital_output`, `io_index + 32 → 硬件引脚编号`（逻辑引脚 6 → 硬件引脚 38）

| 逻辑引脚 | 硬件引脚 | 使用节点 | 用途 | IO 语义 |
|---------|---------|---------|------|---------|
| **2** | 34 | LatteNode (DO2) | 打花器开关 | `true=开, false=关` |
| **4** | 36 | LatteNode (DO4) | 咖啡机开关 | `true=开, false=关` |
| **6** | 38 | ExecuteGraspPoseWorker | 气动夹爪 40 开/关 | `true=打开, false=闭合` |
| **6** | 38 | PublishGraspsClientWorker | 电动夹爪 60 开/关 | `true=闭合, false=打开` ⚠️ 反相 |
| **6** | 38 | PublishGraspsABWorker | AB 抓取夹爪开/关 | `true=闭合, false=打开` ⚠️ 反相 |
| **7** | 39 | GripperSwapWorker | 快换盘锁紧/释放 | `true=锁紧, false=释放` |

> ⚠️ **IO 语义不一致警告**：`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）的夹爪 IO 语义**相反**，源于气动夹爪（gripper0）与电动夹爪（gripper2）的电气接线差异。添加新 Worker 时需确认物理接线方向喵~

**IO 调用示例**：
```bash
# 夹爪闭合 (gripper0 — 气动，true=打开)
ros2 service call /aubo_driver/set_io ivg_interfaces/SetRobotIO \
  "{io_type: 'digital_output', io_index: 6, value: 0.0}"

# 快换盘锁紧
ros2 service call /aubo_driver/set_io ivg_interfaces/SetRobotIO \
  "{io_type: 'digital_output', io_index: 7, value: 1.0}"
```

### 0.4 工具端电源配置

| 逻辑引脚 | 电压 | 使用工具 | 说明 |
|---------|------|---------|------|
| 工具端 DOUT0 (逻辑 16) | 24V | gripper2 (电动 60) | 电动夹爪供电/信号 |
| 工具端 DOUT1 (逻辑 17) | 24V | gripper2 (电动 60) | 电动夹爪辅助信号 |

工具端电压通过 AUBO 控制箱内部继电器切换（0V / 12V / 24V），由 `/aubo_driver/set_io` 的 `FUN_SET_TOOL_VOLTAGE` 功能设置喵~

### 0.5 相机参数

| 参数 | 值 |
|------|-----|
| 型号 | Percipio FM830 |
| 深度分辨率 | 1280 × 960 @ 30fps |
| 彩色分辨率 | 1280 × 960 @ 30fps |
| 工作距离 | 0.2m – 2.0m |
| 深度精度 | ±1mm @ 1m |
| 接口 | USB 3.0 (SuperSpeed) |
| 触发模式 | 连续模式（默认）/ 软件触发 (`/software_trigger`) |
| 相机序列号 | `207000152740` |
| ROS 2 命名空间 | `camera`（话题 `/camera/color/image_raw` 等） |
| 深度缩放因子 | 0.00025 (raw uint16 × 0.25mm → 米) |

**相机话题**：
| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth 5 | 彩色图像 (RGB8) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth 5 | 深度图像 (16UC1) |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | 深度点云 |
| `/camera/depth_registered/points` | `sensor_msgs/PointCloud2` | BEST_EFFORT, depth 5 | 彩色点云 (depth→color 对齐) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE, depth 1 | 彩色相机内参 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE, depth 1 | 深度相机内参 |
| `/camera_status` | `ivg_interfaces/CameraStatus` | RELIABLE, depth 10 | 相机在线状态与参数 |

### 0.6 快换盘结构

```
法兰 (wrist3_Link)
  │
  ├── kuaihuan_Link (快换法兰, Z=+0.033m 安装面)
  │     │
  │     └── 快换锁止机构 (IO 7, 咬合深度 3.8mm)
  │           │
  │           └── 工具安装面 (attach_offset.z = 0.033m)
  │                 │
  │                 ├── gripper0 (气动 40, 绕 Z 旋转 0°)
  │                 ├── gripper1 (电动 A, 绕 Z 旋转 0°)
  │                 └── gripper2 (电动 60, 绕 Z 旋转 90°)
  │
  └── camera_frame (手眼标定 TF: wrist3 → camera, 静态广播)
```

快换对接工位通过 `tools.yaml` 配置，包含 dock 位置、接近/退避距离、IO 配置等喵~

---

## 1. 工作空间架构

```
aubo_ros2_ws/src/
├── ivg_interfaces/                 # 统一自定义接口（17 msg + 34 srv = 51 类型）
├── aubo_ros2_driver/               # 机械臂驱动与运动控制（5 个子包）
├── aubo_ros2_web_dashboard/        # 前端演示页面（FastAPI 网关 + ROSLIB.js）
├── robotwebtools/                  # 前端依赖库（roslibjs / ros3djs / ros2djs）
├── camport_ros2/src/               # 知微相机驱动（2 个子包）
├── graspnet_ros2/                  # GraspNet 大模型（6-DoF 抓取预测）
├── hand_eye_calibration/           # 自动手眼标定（Web UI）
├── visual_pose_estimation/         # 工件识别与抓取（1 个子包）
├── vision_perception/              # MediaPipe + YOLO OBB 感知
├── tool_changer/                   # 夹爪快换（物理运动 + PlanningScene 附着 + URDF 更新）
├── coffee_latte_demo/              # 咖啡拉花演示（IO 控制 + Web 前端）
├── latte_imitation/                # RM65→AUBO E5 拉花轨迹模仿学习 (MoveIt2 标准管线)
└── ros_arm_tutorials/              # 《ROS 机械臂开发与实践》教学代码（6 个子包）
```

**12 个顶层功能包，20 个 ROS 2 子包。**

---

## 2. 功能包清单

### 2.1 机械臂驱动与运动控制 — `aubo_ros2_driver/`

#### 新框架（纯 C++，推荐）

| 节点 | 职责 |
|------|------|
| **JointTrajectoryController** | C++ 预计算插值 + 独立发送线程 + RIB 流控 → `AuboHardwareInterface` |
| **AuboStateBroadcaster** | 回调驱动的状态广播，200Hz 发布 `joint_states` |
| **AuboDashboardNode** | LifecycleNode，提供 20 个 ROS2 服务（IK/FK/IO 等 SDK 全功能） |

核心文件：`aubo_driver_ros2/include/aubo_driver_ros2/joint_trajectory_controller.h`、`aubo_hardware_interface.h`、`aubo_dashboard_node.h`

#### 旧框架（Python 插值）

```
move_group → follow_joint_trajectory (action)
  → aubo_ros2_trajectory_action → joint_path_command (topic)
  → aubo_robot_simulator_ros2 (200Hz Python 五次多项式插值)
  → moveItController_cmd (topic) → aubo_driver_ros2 → 真实机械臂
```

#### 全部子包

| 子包 | 类型 | 职责 |
|------|------|------|
| **ivg_interfaces** | msg/srv | 统一自定义接口包：17 msg + 34 srv = 51 类型（合并自旧 aubo_msgs/demo_interface/interface 等） |
| **aubo_driver_ros2** | C++ | ROS2 真机驱动：JointTrajectoryController + AuboDashboardNode + AuboStateBroadcaster |
| **aubo_moveit_config** | Python/C++ | MoveIt 配置与统一 launch（含 `aubo_new_driver.launch.py`） |
| **demo_driver** | C++ | 应用层服务节点：move_to_pose / plan_trajectory / execute_grasp_pose / publish_grasps 等 |
| **aubo_description** | URDF/STL | 机器人模型（URDF/XACRO/mesh），含快换法兰 kuaihuan_Link |
| **aubo_dashboard_msgs** | msg/srv | Dashboard 相关 msg/srv/action |

### 2.2 前端 — `aubo_ros2_web_dashboard/` + `robotwebtools/`

| 包 | 类型 | 职责 |
|----|------|------|
| **aubo_ros2_web_dashboard** | Python + JS | FastAPI 网关（静态页 + rosbridge WS 代理 + 3D mesh 服务）+ 纯 JS SPA 前端 |
| **robotwebtools** | JS 库 | 本地化 roslibjs / ros3djs / ros2djs（离线，已打补丁） |

### 2.3 相机与图像 — `camport_ros2/src/`

| 子包 | 类型 | 职责 |
|------|------|------|
| **percipio_camera** | C++ | 知微相机驱动（深度/彩色、点云、深度修复）+ camera_control_node |
| **image_data_bridge** | C++ | 相机图像 → 统一 ImageData 消息格式的桥接节点 |

### 2.4 视觉与抓取

| 包 | 类型 | 职责 |
|----|------|------|
| **graspnet_ros2** | Python | GraspNet 6-DoF 抓取位姿预测（含 graspnet-baseline） |
| **hand_eye_calibration** | Python | 单目相机手眼标定（Web UI 交互式） |
| **visual_pose_estimation_python** | Python | 工件姿态估计 + FastAPI Web（`web_ui/`） |
| **vision_perception** | Python | MediaPipe Holistic + YOLO OBB 人体/物体感知 |

### 2.5 快换与手爪 — `tool_changer/`

| 节点 | 可执行文件 | 职责 |
|------|-----------|------|
| **gripper_swap_worker** | `gripper_swap_worker_node` | 物理快换：MoveGroupInterface 运动规划 + IO 控制（四类轨迹原语） |
| **scene_attach_worker** | `scene_attach_worker_node` | 虚拟更新：PlanningScene 附着 + `robot_description` 参数实时更新（三节点同步） |

服务：`/run_gripper_swap`、`/change_tool`、`/get_current_tool`、`/scene_attach`、`/scene_detach`

详见 `tool_changer/README.md` — 含完整排错记录和架构设计文档。

### 2.6 咖啡拉花 — `coffee_latte_demo/` + `latte_imitation/`

| 包 | 类型 | 职责 |
|----|------|------|
| **coffee_latte_demo** | Python | 咖啡拉花演示：IO 控制节点（DO2/DO4）、Web 前端面板、工具网格 |
| **latte_imitation** | Python | RM65 数据集 → MoveIt2 computeCartesianPath + execute 管线（模仿学习） |

---

## 3. 末端工具与 IO 引脚

### 3.1 工具清单

| 工具 ID | 类型 | 所属包 | 用途 | 绕 Z 旋转 | attach_offset.z |
|---------|------|--------|------|----------|----------------|
| **gripper0** | 气动夹爪 φ40 | tool_changer | 工件抓取（visual_pose_estimation） | 0° | 0.033m |
| **gripper1** | 电动夹爪 A | coffee_latte_demo | 咖啡拉花载体 | 0° | 0.033m |
| **gripper2** | 电动夹爪 φ60 | tool_changer | AI 抓取（graspnet_ros2） | 90° | 0.033m |
| **gripper1coffeecup** | 咖啡杯工具 | coffee_latte_demo | 拉花用咖啡杯 | 180° | 0.033m |
| **gripper1milkcup** | 牛奶杯工具 | coffee_latte_demo | 拉花用牛奶杯 | -90° | 0.033m |

> 安装面：`kuaihuan_Link` Z=+0.033m。工具 mesh 原点（安装面）与法兰安装面对齐，快换锁止机构咬合 3.8mm。详见 `tool_changer/README.md` 第 4 节。

### 3.2 IO 引脚映射

（`/aubo_driver/set_io(io_type=digital_output, io_index + 32 → 硬件引脚)`）：

| 逻辑引脚 | 硬件引脚 | 使用节点 | 用途 |
|---------|---------|---------|------|
| 6 | 38 | ExecuteGraspPose / PublishGrasps | 夹爪开/关 |
| 7 | 39 | GripperSwapWorker | 快换盘锁紧/释放 |
| 2 | 34 | LatteNode (DO2) | 打花开关 |
| 4 | 36 | LatteNode (DO4) | 咖啡开关 |

> **注意**：`ExecuteGraspPoseWorker` 的 IO 语义与 `PublishGraspsClientWorker`/`ABWorker` **相反**（`true=打开` vs `true=闭合`），源于气动夹爪不同工位的电气接线差异。

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
  gripper_swap_worker: 物理快换运动 + IO 控制
    ├─ publishToolStatus(false) ← 切换前清除（避免碰撞干扰）
    ├─ 执行切换运动（moveToJoints + CartesianPath + IO）
    └─ publishToolStatus("gripperX") ← 切换后发布
         └→ scene_attach_worker
              ├─ PlanningScene 附着（RViz2 Scene Robot 实时显示）
              └─ URDF 参数更新 → robot_state_publisher/rviz2/move_group → Web 前端

咖啡拉花 (coffee_latte_demo)
  ├── /set_latte_do2 ← 前端 → /aubo_driver/set_io(io_index=2)
  ├── /set_latte_do4 ← 前端 → /aubo_driver/set_io(io_index=4)
  └── /latte_di_status → 前端 ← /aubo_driver/io_states

运动控制链路（新框架）
  move_group → FollowJointTrajectory Action
    → JointTrajectoryController (C++ 预计算插值 + RIB 流控)
    → AuboHardwareInterface (SDK 封装) → 真实机械臂

运动控制链路（旧框架）
  move_group → follow_joint_trajectory (action)
    → aubo_ros2_trajectory_action → joint_path_command (topic)
    → aubo_robot_simulator (200Hz 插值) → moveItController_cmd (topic)
    → aubo_driver_ros2 → 真实机械臂

前端 (aubo_ros2_web_dashboard)
  浏览器 ← rosbridge WebSocket (:9090) ← ROS 2
  浏览器 ← web_video_server (:8089) ← MJPEG 流
  浏览器 ← FastAPI 网关 (:8090) ← 静态页 + 3D mesh 服务 (/api/ivg/robot-mesh/)
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

选择性构建：
```bash
colcon build --packages-select aubo_driver_ros2 ivg_interfaces aubo_moveit_config
colcon build --packages-select tool_changer
```

### 5.2 启动

**新框架**（推荐）：
```bash
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
# 自动 TCP 探测：可达→真实硬件，不可达→仿真模式
```

**快速重启**（跳过编译和录包）：
```bash
SKIP_BUILD=1 SKIP_ROSBAG=1 ./start_aubo_new_driver.sh
```

**工具快换**（需与完整系统同时运行）：
```bash
ros2 launch tool_changer gripper_swap_worker.launch.py
```

**一键启动**（全栈系统，详见 [DEPLOYMENT.md](DEPLOYMENT.md) 第 9 节）：
```bash
cd aubo_ros2_ws && ./start_aubo_new_driver.sh
```

该脚本按依赖顺序启动完整流程（共 16 步）：

| 步骤 | 内容 | 包 |
|------|------|-----|
| 0 | colcon build 全量构建 | — |
| 1 | 机械臂核心 (Controller + Dashboard + StateBroadcaster + MoveIt2 + RViz2) | aubo_moveit_config |
| 2 | Demo Driver 基础服务 | demo_driver |
| 3 | 知微相机驱动 | percipio_camera |
| 4 | 相机控制（软件触发） | percipio_camera |
| 5 | 图像数据桥接 | image_data_bridge |
| 6 | 手眼标定 Web | hand_eye_calibration |
| 7 | 视觉姿态估计 | visual_pose_estimation_python |
| 8 | GraspNet 点云抓取预测 | graspnet_ros2 |
| 9 | 抓取执行 Worker | demo_driver |
| 10 | 夹爪快换 Worker | tool_changer |
| 11 | 咖啡拉花演示 | coffee_latte_demo |
| 12 | GraspNet 循环抓取 Worker | demo_driver |
| 13 | 服务就绪校验 | — |
| 14 | VPE FastAPI Web 服务 | visual_pose_estimation_python |
| 15 | IVG Web 网关 | aubo_ros2_web_dashboard |
| 16 | rosbag 录制 | — |

### 5.3 访问地址

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
  ├── ivg_interfaces (RobotStatus, ExecuteGraspPose, RunGripperSwap, ToolChangerStatus 等)
  └── rosbridge_suite, FastAPI, robot_state_publisher (TF)

demo_driver
  ├── ivg_interfaces (MoveToPose, PlanTrajectory, ExecuteGraspPose, EstimatePose 等)
  └── moveit_core, moveit_ros_planning_interface

tool_changer
  ├── ivg_interfaces (SetRobotIO, RunGripperSwap, ChangeTool, ToolChangerStatus)
  ├── moveit_core, moveit_ros_planning_interface
  ├── ament_index_cpp (xacro 路径解析)
  └── yaml-cpp (tools.yaml 解析)

coffee_latte_demo
  ├── ivg_interfaces (SetRobotIO, RobotIOStatus)
  └── std_srvs

graspnet_ros2
  └── ivg_interfaces, cv_bridge, sensor_msgs

hand_eye_calibration
  └── ivg_interfaces (ImageData, RobotStatus, SoftwareTrigger, SetCameraParameters)

visual_pose_estimation / _python
  ├── ivg_interfaces (EstimatePose, EstimatePose2D, CartesianPosition, ListTemplates)
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

## 8. 重要技术细节

1. **ROS 2 参数隔离**：ROS 2 **没有全局参数服务器**。`launch` 中 `parameters=[robot_description]` 把参数分别写入各节点私有存储。修改 `/robot_state_publisher` 的参数不影响 `/rviz2` 和 `/move_group`。更新 `robot_description` 时需用 `AsyncParametersClient` 逐一设置全部三个节点。

2. **`robot_state_publisher` 不订阅话题**：ROS 2 Humble 的 `robot_state_publisher` 不订阅 `/robot_description` 话题，只通过 `/parameter_events` 监听自身参数变更。必须用 `set_parameters()` 而非 `publish()` 来触发 `setupURDF()` 重建 TF 树。

3. **`shared_from_this()` 陷阱**：不能在 Node 构造函数中调用，会抛出 `std::bad_weak_ptr`。延迟到回调中首次使用时调用。

4. **工具切换碰撞安全**：`gripper_swap_worker.changeToTool()` 入口处调用 `publishToolStatus(false)` 清除附着体，避免切换运动中夹爪与机械臂自碰撞导致轨迹规划失败。

5. **percipio_camera**：CMake 需显式 `find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui photo)` 并链接 `OpenCV_LIBS`。

6. **MoveIt CurrentStateMonitor 回调竞争**：长耗时服务需放入独立的 callback group，否则阻塞状态监听。

7. **IO 引脚语义差异**：`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）逻辑相反。

8. **GraspNet Z 轴 180° 翻转**：所有 GraspNet 消费者通过 `applyGraspZFlip180()` 自动修正。

9. **`tools.yaml` 与 `aubo_e5.urdf.xacro` 必须对齐**：每个工具的 `attach_offset`（YAML）与 `gripper_link` 宏的 `origin`（xacro）姿态必须严格一致。

---

## 9. Git 仓库

```bash
cd /home/mu/IVG2.0
git remote -v                                    # https://github.com/MUHAN11-c/aubo_boot

git add -A
git commit -m "描述你的修改"
git push -u origin dev
```

**默认工作在 `dev` 分支，`main` 为稳定版本。**

---

*文档与 `src/` 下 package.xml 及启动脚本保持同步。新增/删除包或节点时请同步更新本文档。*
