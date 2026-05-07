# IVG2.0 — 奥博机械臂 ROS2 Humble 全栈系统参考手册

基于 **ROS 2 Humble** 的奥博（Aubo）机械臂系统，涵盖：MoveIt 2 运动规划、真机驱动、知微（Percipio）相机、手眼标定、视觉位姿估计（C++/Python + FastAPI Web）、GraspNet 点云抓取、YOLO OBB 感知、工具快换、咖啡拉花演示、ROS2 基础教程。

当前 `src/` 下共 **27** 个 ROS 2 功能包。

---

## 1. 整体系统架构

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         【Web 人机交互层】                                 │
│  aubo_ros2_web_dashboard (FastAPI 网关 :8090 + rosbridge :9090)           │
│  ├─ static/  前端面板 (ROSLIB.js + Chart.js + Bootstrap)                  │
│  ├─ /ws/rosbridge        → rosbridge_suite (话题/服务/动作 WebSocket)     │
│  ├─ /api/ivg/proxy/*     → web_video_server (MJPEG)                      │
│  └─ 子面板: topics_lab / coffee_latte_panel / tool_changer_panel         │
├──────────────────────────────────────────────────────────────────────────┤
│  visual_pose_estimation_python/web_ui/ (FastAPI :8088)                    │
│  ├─ /api/pose/*  姿态估计 REST API                                       │
│  ├─ /api/grasp/* 抓取控制 REST API                                       │
│  └─ /ws          位姿估计 WebSocket                                      │
├──────────────────────────────────────────────────────────────────────────┤
│  hand_eye_calibration (Flask Web :8080)                                   │
│  └─ Web GUI: 手眼标定流程 (采集→标定→验证)                                │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                         【应用业务层】                                     │
│                                                                          │
│  ┌─────────────────────┐  ┌──────────────────────┐                       │
│  │ demo_driver         │  │ tool_changer          │                      │
│  │ (运动规划+抓取执行)   │  │ (夹爪快换 Worker)      │                      │
│  │                     │  │                      │                       │
│  │ Services:           │  │ Services:            │                       │
│  │ /move_to_pose       │  │ /run_gripper_swap    │                       │
│  │ /plan_trajectory    │  │ /change_tool         │                       │
│  │ /execute_trajectory │  │ /get_current_tool    │                       │
│  │ /get_current_state  │  │                      │                       │
│  │ /set_speed_factor   │  │ Topic:               │                       │
│  │ /set_robot_pose     │  │ /tool_changer_status │                       │
│  │ /movel              │  │                      │                       │
│  │ /read_robot_io      │  │ Client:              │                       │
│  │ /set_robot_enable   │  │ /aubo_driver/set_io  │                       │
│  │ /execute_single_    │  │ MoveGroupInterface   │                       │
│  │   grasp             │  │ PlanningScene        │                       │
│  │ /loop_grasp_control │  │ (STL 场景管理)       │                       │
│  │ /publish_grasps_    │  │                      │                       │
│  │   worker_loop_ctrl  │  │                      │                       │
│  │ /graspnet_capture   │  │                      │                       │
│  │   _control          │  │                      │                       │
│  └─────────────────────┘  └──────────────────────┘                       │
│                                                                          │
│  ┌─────────────────────┐  ┌──────────────────────┐                       │
│  │ coffee_latte_demo   │  │ vision_perception     │                      │
│  │ (咖啡拉花 IO 控制)    │  │ (YOLO OBB 感知)       │                      │
│  │                     │  │                      │                       │
│  │ Node: latte_node    │  │ Nodes:               │                       │
│  │ Services:           │  │ yolo_obb_node        │                       │
│  │ /set_latte_do2      │  │ yolo_track_node      │                       │
│  │ /set_latte_do4      │  │ video_publisher_node │                       │
│  │ Topic:              │  │                      │                       │
│  │ /latte_di_status    │  │ Topics:              │                       │
│  │ Client:             │  │ /vision/yolo_obb/*   │                       │
│  │ /aubo_driver/set_io │  │ /vision/yolo_track/* │                       │
│  └─────────────────────┘  └──────────────────────┘                       │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                         【视觉感知层】                                     │
│                                                                          │
│  ┌──────────────────────┐  ┌─────────────────────┐                       │
│  │ visual_pose_estimation│  │ graspnet_ros2        │                      │
│  │ _python               │  │ (GraspNet 点云抓取)   │                      │
│  │ (工件位姿估计)          │  │                     │                       │
│  │                       │  │ Launch:              │                       │
│  │ Services:             │  │ graspnet_demo_points │                       │
│  │ /estimate_pose        │  │ _with_tf.launch.py   │                       │
│  │ /estimate_pose_2d     │  │                     │                       │
│  │ /list_templates       │  │ Subscribes:          │                       │
│  │ /standardize_template │  │ Pub: grasp_markers,  │                      │
│  │ /update_params        │  │      grasp_poses_base│                       │
│  │                       │  │ Sub: /camera/depth_  │                       │
│  │ Topic: /system_status │  │      registered/points│                      │
│  │                       │  │ Srv: /graspnet_      │                       │
│  │ Subscribes:           │  │      capture_control │                       │
│  │ /camera/color/image   │  │ TF:  camera_frame→   │                       │
│  │ /camera/depth/image   │  │      grasp_pose_i    │                       │
│  │ /aubo_driver/         │  └─────────────────────┘                       │
│  │   robot_status        │                                               │
│  │                       │  ┌─────────────────────┐                       │
│  │ Client:               │  │ hand_eye_calibration │                       │
│  │ /software_trigger     │  │ (手眼标定 + Web UI)   │                       │
│  └──────────────────────┘  │                     │                       │
│                            │ Sub: /image_data,    │                       │
│                            │   /camera/depth/*,   │                       │
│                            │   /aubo_driver/      │                       │
│                            │   robot_status       │                       │
│                            │ Client: /software_   │                       │
│                            │   trigger, /set_     │                       │
│                            │   robot_pose,        │                       │
│                            │   /move_to_pose      │                       │
│                            │ Web: Flask :8080     │                       │
│                            └─────────────────────┘                       │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                         【相机驱动层】                                     │
│                                                                          │
│  percipio_camera          percipio_camera_interface    image_data_bridge │
│  ┌────────────────┐       ┌──────────────────┐       ┌──────────────┐   │
│  │ Topics:        │       │ Service:         │       │ Sub:         │   │
│  │ /camera/color/ │       │ /software_trigger│       │ /camera/color│   │
│  │   image_raw    │       │ Topic:           │       │   /image_raw │   │
│  │ /camera/depth/ │       │ /camera_status   │       │ Pub:         │   │
│  │   image_raw    │       │  (CameraStatus)  │       │ /image_data  │   │
│  │ /camera/depth/ │       └──────────────────┘       │ (ImageData)  │   │
│  │   registered/  │                                  └──────────────┘   │
│  │   points       │                                                     │
│  └────────────────┘                                                     │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                     【机械臂运动控制层】                                    │
│                                                                          │
│  ┌─────────────────────┐  ┌──────────────────────┐                       │
│  │ aubo_moveit_config  │  │ aubo_driver_ros2      │                      │
│  │ (MoveIt2 统一入口)    │  │ (真机驱动)             │                      │
│  │                     │  │                      │                       │
│  │ Launch:             │  │ Sub:                 │                       │
│  │ aubo_moveit_pure    │  │ /moveItController_cmd│                       │
│  │   _ros2.launch.py   │  │ Pub:                 │                       │
│  │ demo_driver_        │  │ /joint_states        │                       │
│  │   services.launch.py│  │ /aubo/feedback_states│                       │
│  │                     │  │ /aubo_driver/io_states      │                       │
│  │ 启动:                │  │ /aubo/robot_status   │                       │
│  │ move_group           │  │ Service:             │                       │
│  │ rviz2                │  │ /aubo_driver/set_io  │                       │
│  │ robot_state_publisher│  │ /aubo_driver/get_ik  │                       │
│  │ ros2_control (仿真用 mock)         │  └──────────────────────┘                       │
│  └─────────────────────┘                                                │
│                                                                          │
│  ┌──────────────────────────────┐  ┌─────────────────────────────┐      │
│  │ aubo_ros2_trajectory_action  │  │ aubo_robot_simulator_ros2   │      │
│  │ (FollowJointTrajectory)      │  │ (轨迹插值器)                  │      │
│  │                              │  │                             │      │
│  │ Action:                      │  │ Sub: /joint_path_command    │      │
│  │ joint_trajectory_controller/follow_joint_trajectory     │  │ Pub: /moveItController_cmd  │      │
│  │ Pub: /joint_path_command     │  │ 200Hz 五次多项式插值          │      │
│  └──────────────────────────────┘  └─────────────────────────────┘      │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────┐        │
│  │ aubo_description  (URDF/XACRO 机器人模型)                      │        │
│  │ aubo_msgs         (msg/srv: JointTrajectoryFeedback, SetIO…) │        │
│  │ aubo_dashboard_msgs (Dashboard 相关 msg/srv/action)           │        │
│  │ demo_interface    (demo_driver 接口: SetRobotIO, RobotStatus) │        │
│  │ tool_changer_interface (快换接口: RunGripperSwap, ChangeTool) │        │
│  │ interface         (视觉接口: EstimatePose, CartesianPosition) │        │
│  └──────────────────────────────────────────────────────────────┘        │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┼──────────────────────────────────────┐
│                     【教学演示层】                                         │
│                                                                          │
│  ros_arm_tutorials/                                                      │
│  ├── base_demo/         ROS2 基础: 话题发布/订阅、服务/客户端、参数       │
│  ├── advance_demo/      ROS2 进阶: Action、TF、RViz Marker、rqt          │
│  └── xarm_moveit_demo/  MoveIt2 演示: 7个完整示例                        │
│      ├── hello_moveit               最简 MoveIt2 入门                   │
│      ├── moveit_pose_demo           位姿控制 + 碰撞物体                  │
│      ├── moveit_joint_pose_demo     关节空间 + 夹爪控制                  │
│      ├── moveit_planning_scene_demo 规划场景信息获取                    │
│      ├── moveit_beeline_demo        笛卡尔直线路径 (三角形)             │
│      ├── moveit_arcline_demo        笛卡尔圆弧路径 (圆形)               │
│      └── moveit_pick_place_demo     MTC 完整 Pick & Place              │
└──────────────────────────────────────────────────────────────────────────┘
```

### 数据流总览

```
相机 → percipio_camera → /camera/color/image_raw, /camera/depth/image_raw, /camera/depth_registered/points
                                    │
          ┌─────────────────────────┼──────────────────────────┐
          ▼                         ▼                          ▼
  image_data_bridge          vision_perception       visual_pose_estimation
  Sub: /camera/color/*      Sub: /camera/color/*    Sub: /camera/color/image_raw
  Pub: /image_data           Pub: /vision/yolo_*/*   Sub: /camera/depth/image_raw
  (ImageData 格式)           (obb/track/detect)      Sub: /aubo_driver/robot_status
                             (obb/track/detect)      Sub: /aubo_driver/robot_status
          │                         │                Srv: /estimate_pose
          │                         │                Srv: /estimate_pose_2d
          │                         │                Pub: /system_status
          │                         │                          │
          │              ┌──────────┘                          │
          ▼              ▼                                     │
  hand_eye_calibration   graspnet_ros2 (点云→6DOF抓取)         │
  Sub: /image_data       Sub: /camera/depth_registered/points   │
  Sub: /camera/depth/*   Pub: grasp_markers (MarkerArray)       │
  Srv: /capture          Pub: grasp_poses_base (PoseArray)     │
  Srv: /calibrate         ──────────────────────────────────────┘
          │                         │                          
          │                         ▼                          
          │←── 标定结果 ──→  demo_driver (抓取执行)             
          │    (T_C_E)       /execute_single_grasp              
          │                  /publish_grasps_worker_loop_control
          │                         │
          │              ┌──────────┼──────────┐
          │              ▼          ▼          ▼
          │     aubo_moveit_config  trajectory  simulator
          │     (move_group)        _action     (五次多项式插值)
          │              │          │          │
          │              │  joint_trajectory_controller/
          │              │  follow_joint_trajectory (Action)
          │              │          │
          │              │          ▼ joint_path_command (JointTrajectory)
          │              │    aubo_robot_simulator
          │              │          │
          │              │          ▼ moveItController_cmd (JointTrajectoryPoint @200Hz)
          │              │    aubo_driver_ros2
          │              │          │
          │              │          ├─ Pub: joint_states (JointState, 100Hz)
          │              │          ├─ Pub: aubo/feedback_states (FollowJointTrajectory_Feedback)
          │              │          ├─ Pub: /aubo_driver/robot_status (RobotStatus, 50Hz)
          │              │          ├─ Pub: /aubo_driver/io_states (RobotIOStatus, 50Hz)
          │              │          └─ Srv: /aubo_driver/set_io (SetRobotIO)
          │              │                    │
          │     ┌────────┼────────────────────┼───────────────┐
          │     ▼        ▼                    ▼               ▼
          │  tool_changer    coffee_latte_demo    execute_grasp   PublishGrasps
          │  /run_gripper_   /set_latte_do2/do4  _pose_worker    Client
          │  swap            /latte_di_status     (IO index 6)    (IO index 6)
          │  (IO index 7)
          ▼
    静态 TF 发布: wrist3_Link → camera_frame (手眼标定结果)
```


---

## 2. 完整功能列表及 ROS2 接口详解

### 2.1 机械臂驱动与控制

#### 2.1.1 aubo_driver_ros2 — 真机驱动

**功能**: 连接实体机械臂控制器，接收运动指令并下发轨迹；上报关节状态、IO 状态、机器人状态。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `moveItController_cmd` | `trajectory_msgs/JointTrajectoryPoint` | 接收插值后的轨迹点 (由 simulator 发布，QoS depth 20000) |
| **Subscribe** | `trajectory_execution_event` | `std_msgs/String` | MoveIt 执行状态事件 (planning/executing/stop/error) |
| **Subscribe** | `robot_control` | `std_msgs/String` | 上电控制命令 (powerOn) |
| **Subscribe** | `teach_cmd` | `std_msgs/Float32MultiArray` | 示教模式命令 |
| **Subscribe** | `moveAPI_cmd` | `std_msgs/Float32MultiArray` | 直接 API 运动命令 |
| **Subscribe** | `/aubo_driver/controller_switch` | `std_msgs/Int32` | 切换控制器模式 (AuboAPI ↔ RosMoveIt) |
| **Publish** | `joint_states` | `sensor_msgs/JointState` | 发布 100Hz 实时关节状态 (QoS depth 3000) |
| **Publish** | `aubo/feedback_states` | `control_msgs/action/FollowJointTrajectory_Feedback` | 发布轨迹反馈 (desired/actual/error positions) |
| **Publish** | `/aubo_driver/real_pose` | `std_msgs/Float32MultiArray` | 发布实时关节角度 (QoS depth 500) |
| **Publish** | `/aubo_driver/robot_status` | `demo_interface/RobotStatus` | 发布 50Hz 完整机器人状态 (关节+位姿+使能+运动) |
| **Publish** | `/aubo_driver/rib_status` | `std_msgs/Int32MultiArray` | 环形缓冲区状态 [queue_size, control_mode, connected_flag] |
| **Publish** | `aubo_driver/cancel_trajectory` | `std_msgs/UInt8` | 轨迹取消信号 |
| **Publish** | `/aubo_driver/io_states` | `demo_interface/RobotIOStatus` | 发布 50Hz IO 状态 (控制箱 + 工具端) |
| **Service** | `/aubo_driver/set_io` | `demo_interface/SetRobotIO` | 设置数字/模拟/工具端 IO (io_type: digital_output, io_index: 0-N, value: 0/1) |
| **Service** | `/aubo_driver/get_ik` | `aubo_msgs/GetIK` | 逆运动学求解 (Aubo SDK) |
| **Service** | `/aubo_driver/get_fk` | `aubo_msgs/GetFK` | 正运动学求解 (Aubo SDK) |
| **Param** | `server_host` | string | 机械臂控制器 IP (默认 169.254.10.98) |

#### 2.1.2 aubo_ros2_trajectory_action — 轨迹动作服务

**功能**: 将 MoveIt 规划的轨迹通过 FollowJointTrajectory Action 发布到 `joint_path_command`。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Action** | `joint_trajectory_controllerjoint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Action Server: 接收完整关节轨迹并监督执行 |
| **Publish** | `joint_path_command` | `trajectory_msgs/JointTrajectory` | 发布完整轨迹供 simulator 插值 (QoS depth 100) |
| **Subscribe** | `aubo/feedback_states` | `control_msgs/action/FollowJointTrajectory_Feedback` | 监听实际关节位置，连续 5 帧在 0.02 rad 容差内判定 Goal 完成 |
| **Subscribe** | `trajectory_execution_event` | `std_msgs/String` | 接收 MoveIt 停止/取消事件 |

#### 2.1.3 aubo_robot_simulator_ros2 — 轨迹插值器

**功能**: 订阅 `joint_path_command`，使用五次多项式按 200Hz 插值输出 `moveItController_cmd`。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `joint_path_command` | `trajectory_msgs/JointTrajectory` | 订阅待插值的完整轨迹 |
| **Publish** | `moveItController_cmd` | `trajectory_msgs/JointTrajectoryPoint` | 发布五次多项式插值后的轨迹点 (QoS depth 2000) |
| **Subscribe** | `/aubo_driver/rib_status` | `std_msgs/Int32MultiArray` | 可选: 读取环形缓冲区状态用于流速调节 |
| **Subscribe** | `/aubo_driver/real_pose` | `std_msgs/Float32MultiArray` | 可选: 空闲时更新初始关节位置 |
| **Param** | `minimum_buffer_size` | int | 缓冲区最小值 (600) |
| **Param** | `motion_update_rate` | double | 插值频率 (200.0 Hz) |

#### 2.1.4 aubo_moveit_config — MoveIt2 配置与启动

**功能**: 提供统一的 MoveIt2 启动入口，集成 move_group、RViz、ros2_control (仿真用 mock)、驱动、插值。

**Launch: aubo_moveit_pure_ros2.launch.py** (推荐主入口)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `aubo_driver_server_host` | 169.254.10.98 | 机械臂控制器 IP |

启动内容: robot_state_publisher + move_group + rviz2 + aubo_driver_ros2 + aubo_robot_simulator_node + aubo_ros2_trajectory_action + demo_driver 服务

**Launch: demo_driver_services.launch.py** — 单独启动 demo_driver 全部服务

使用到的 ROS2 组件: `robot_state_publisher`, `move_group`, `rviz2`, `ros2_control (仿真用 mock)`, `joint_state_broadcaster`

#### 2.1.5 aubo_description — 机器人模型

**功能**: 提供 URDF/XACRO 机器人描述文件、STL 网格、ros2_control (仿真用 mock) 配置。

**TF 发布**:
| 帧名 | 类型 | 说明 |
|------|------|------|
| `base_link` → `*_joint` → `*_link` | 静态 TF | 通过 robot_state_publisher 发布机器人运动学树 |

---

### 2.2 应用服务层

#### 2.2.1 demo_driver — 运动规划与抓取执行服务

**功能**: 提供运动规划、轨迹执行、抓取控制的完整应用层服务。

| 可执行节点 | 接口 | 类型 | 说明 |
|-----------|------|------|------|
| **move_to_pose_server** | `/move_to_pose` | `demo_interface/MoveToPose` | 关节空间或笛卡尔空间运动到目标位姿 (需 move_group) |
| **plan_trajectory_server** | `/plan_trajectory` | `demo_interface/PlanTrajectory` | 用 MoveIt OMPL 规划轨迹但不执行 |
| **execute_trajectory_server** | `/execute_trajectory` | `demo_interface/ExecuteTrajectory` | 执行已规划好的 JointTrajectory |
| **get_current_state_server** | `/get_current_state` | `demo_interface/GetCurrentState` | 获取关节角度 (rad) + TCP 位姿 + 关节速度 |
| **set_speed_factor_server** | `/set_speed_factor` | `demo_interface/SetSpeedFactor` | 动态调节最大速度缩放因子 (0~1) |
| **set_robot_pose_server** | `/set_robot_pose` | `demo_interface/SetRobotPose` | 移动到指定位姿 (支持关节角/笛卡尔，支持度/弧度) |
| （可选启用） | `/movel` | `demo_interface/Movel` | 单轴直线运动 (仅 x/y/z，不旋转) |
| （可选启用） | `/read_robot_io` | `demo_interface/ReadRobotIO` | 读取指定 IO 当前值 |
| （可选启用） | `/set_robot_enable` | `demo_interface/SetRobotEnable` | 使能/去使能机器人 |
| **execute_grasp_pose_worker** | `/execute_single_grasp` | `demo_interface/ExecuteGraspPose` | 执行单次抓取放置 (支持视觉估计/参数常量模式) |
| | `/loop_grasp_control` | `std_srvs/SetBool` | 循环抓取控制 (true=开始, false=停止) |
| **publish_grasps_client_worker** | `/publish_grasps_worker_loop_control` | `std_srvs/SetBool` | Worker 循环抓取控制 |
| | `/graspnet_capture_control` | `std_srvs/SetBool` | 感知采集控制 (触发 GraspNet 推理) |

**依赖的其他 ROS2 接口**:
- **Client**: `/aubo_driver/set_io` — IO 控制 (夹爪开/关, IO index 6 → 硬件引脚 38)
- **Client**: `/aubo_driver/get_ik` — IK 求解 (plan_trajectory_server/set_robot_pose_server)
- **Client**: `/aubo_driver/get_fk` — FK 求解 (get_current_state_server)
- **Client**: `/estimate_pose` — 视觉姿态估计 (execute_grasp_pose_worker, 用于视觉模式)
- **Client**: `/run_gripper_swap` — 工具快换
- **Client**: MoveIt2 `joint_trajectory_controllerjoint_trajectory_controller/follow_joint_trajectory` Action
- **Subscribe**: `joint_states` — 关节状态反馈 (get_current_state 备用源)
- **Subscribe**: `/aubo_driver/robot_status` — 机器人状态
- **PublishGraspsClientWorker 额外 Subscribe**: `grasp_poses_topic` (geometry_msgs/PoseArray, 默认来自 GraspNet)

#### 2.2.2 tool_changer — 夹爪快换管理

**功能**: gripper0 ↔ gripper2 双向自动快换，含场景管理 (STL 可视化)。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Service** | `/run_gripper_swap` | `tool_changer_interface/RunGripperSwap` | 方向式快换 (direction: gripper0_to_gripper2 / gripper2_to_gripper0 / gripper2) |
| **Service** | `/change_tool` | `tool_changer_interface/ChangeTool` | 按工具 ID 切换 (tool_id: gripper0 / gripper2), 支持空→取、两工具互切 |
| **Service** | `/get_current_tool` | `tool_changer_interface/GetCurrentTool` | 查询当前装载的工具 (tool_id/name/type/parameters) |
| **Publish** | `/tool_changer_status` | `tool_changer_interface/ToolChangerStatus` | 发布当前工具状态 (tool_id/is_connected/tool_parameters) |
| **Client** | `/aubo_driver/set_io` | `demo_interface/SetRobotIO` | IO type=digital_output, index=7 → 硬件引脚 39 (快换盘锁紧/释放) |
| **Use** | `MoveGroupInterface("manipulator")` | — | setJointValueTarget/setPoseTarget/computeCartesianPath/move |
| **Use** | `PlanningSceneInterface` | — | 管理 gripper0/1/2/coffeecup/milkcup 五种 STL 场景物体 |

**参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joint_velocity_scaling` | 0.7 | 关节速度缩放 |
| `joint_acceleration_scaling` | 0.3 | 关节加速度缩放 |
| `home_velocity_scaling` | 0.7 | 回 Home 速度缩放 |
| `home_acceleration_scaling` | 0.3 | 回 Home 加速度缩放 |
| `gripper_io_index` | 7 | 快换 IO 逻辑引脚 |
| `joint_cartesian_switch_delay_sec` | 0.05 | 关节↔笛卡尔切换延时 |
| `simulation_skip_io` | false | 仿真模式跳过 IO |

**快换流程**: 关节空间移动 → 笛卡尔对接/脱开 → IO 控制锁紧/释放 → 场景物体 attach/detach → 回 Home

#### 2.2.3 coffee_latte_demo — 咖啡拉花 IO 控制

**功能**: 通过 Aubo 驱动控制咖啡机和打花器的开关 IO，配合 Web 面板实现可视化控制。

**Node: latte_node**

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Service** | `/set_latte_do2` | `std_srvs/SetBool` | 打花开关 (true=ON / false=OFF) |
| **Service** | `/set_latte_do4` | `std_srvs/SetBool` | 咖啡开关 (true=ON / false=OFF) |
| **Publish** | `/latte_di_status` | `std_msgs/String` | 每 5s 发布 IO 综合状态 |
| **Subscribe** | `/aubo_driver/io_states` | `demo_interface/RobotIOStatus` | 读取 DI2/DI3/DI4 (digital_inputs[2/3/4]) 反馈 |
| **Client** | `/aubo_driver/set_io` | `demo_interface/SetRobotIO` | DO2 (引脚 34) / DO4 (引脚 36) |

**IO 映射**:
| 逻辑 IO | 硬件引脚 | 用途 |
|---------|---------|------|
| DO2 (io_index=2) | 34 | 打花开关 |
| DO4 (io_index=4) | 36 | 咖啡开关 |
| DI2 | 34 | 咖啡反馈 |
| DI3 | 35 | 打花反馈 |
| DI4 | 36 | 警告反馈 |

---

### 2.3 视觉感知层

#### 2.3.1 vision_perception — YOLO OBB 感知

**功能**: 使用 Ultralytics YOLO26 模型进行旋转框检测 (OBB) 和目标检测 (Track)。

**Node: yolo_detect_node** (标准检测)

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像输入 |
| **Publish** | `/vision/yolo_detect/image` | `sensor_msgs/Image` | 标注后的输出图像 |

参数: `input_topic`, `model_path` (yolo26n.pt), `conf_threshold`, `device`, `classes`

**Node: yolo_obb_node** (旋转框检测)

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像输入 |
| **Publish** | `/vision/yolo_obb/image` | `sensor_msgs/Image` | 标注后的输出图像 |
| **Publish** | `/vision/yolo_obb/detections` | `std_msgs/String` | JSON 检测结果 |
| **Publish** | `/vision/yolo_obb/markers` | `visualization_msgs/MarkerArray` | RViz 可视化标记 |

**参数**: `input_topic`, `output_image_topic`, `detection_topic`, `marker_topic`, `model_path` (yolo26n-obb.pt), `conf_threshold` (0.25), `iou_threshold` (0.7), `device` (cuda:0), `publish_markers`, `display_result`

**Node: yolo_track_node** (目标检测跟踪)

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像输入 |
| **Publish** | `/vision/yolo_track/image` | `sensor_msgs/Image` | 标注后的输出图像 |
| **Publish** | `/vision/yolo_track/detections` | `std_msgs/String` | JSON 检测结果 |

目标类别: cup(41), bowl(45/拉花缸), bottle(39), spoon(44)

**Node: video_publisher_node** (视频文件发布)

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Publish** | `/camera/color/image_raw` | `sensor_msgs/Image` | 按帧率发布视频帧 |

**参数**: `video_path`, `output_topic`, `fps`, `loop`, `resize_width`, `resize_height`

**Launch 文件**:
- `yolo_obb.launch.py` → yolo_obb_node
- `video_track.launch.py` → video_publisher_node + yolo_track_node
- `test_video.launch.py` → video_publisher_node + yolo_obb_node

#### 2.3.2 visual_pose_estimation_python — 工件位姿估计

**功能**: 基于深度图和模板匹配的工件 6D 姿态估计，含 FastAPI Web 界面。

**Node: visual_pose_estimation_python**

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Service** | `/estimate_pose` | `interface/EstimatePose` | 3D 姿态估计: 输入深度+彩色图+工件ID → 抓取/准备/预放置/放置姿态 |
| **Service** | `/estimate_pose_2d` | `interface/EstimatePose2D` | 2D 检测: 输入 RGB 图 → 中心坐标+旋转角度+置信度 |
| **Service** | `/list_templates` | `interface/ListTemplates` | 列出可用模板库 (工件列表) |
| **Service** | `/standardize_template` | `interface/StandardizeTemplate` | 对模板进行标准化处理 |
| **Service** | `/update_params` | `interface/UpdateParams` | 动态更新算法参数 |
| **Publish** | `/system_status` | `std_msgs/String` | 系统状态日志 |
| **Subscribe** | `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图缓存 (BEST_EFFORT QoS) |
| **Subscribe** | `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图缓存 (BEST_EFFORT QoS) |
| **Subscribe** | `/aubo_driver/robot_status` | `demo_interface/RobotStatus` | 机器人当前位姿 (计算 T_B_C) |
| **Client** | `/software_trigger` | `percipio_camera_interface/SoftwareTrigger` | 触发相机拍照 |

**参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `calib_file` | "" | 手眼标定文件路径 |
| `template_root` | "" | 模板库根目录 |
| `capture_camera_id` | "207000152740" | 相机 ID |
| `cache_image_max_age_sec` | 8.0 | 图像缓存最大有效期 (秒) |
| `depth_image_topic` | `/camera/depth/image_raw` | 深度图话题 |
| `color_image_topic` | `/camera/color/image_raw` | 彩色图话题 |
| `depth_scale` | 0.00025 | 深度值缩放因子 |
| `depth_search_radius` | 3 | 深度搜索半径 (像素) |
| `gripper_opening_mm` | 50.0 | 夹爪开口 (毫米) |
| `gripper_length_mm` | 100.0 | 夹爪长度 (毫米) |

**FastAPI Web (端口 8088)**:
| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/pose/estimate` | POST | 姿态估计 |
| `/api/pose/templates` | GET | 模板列表 |
| `/api/pose/standardize` | POST | 模板标准化 |
| `/api/grasp/execute` | POST | 执行抓取 |
| `/api/robot/status` | GET | 机器人状态 |
| `/api/system/params` | GET/PUT | 参数查询/更新 |
| `/api/camera/capture` | POST | 手动采集 |
| `/ws` | WebSocket | 实时通信 |

#### 2.3.3 graspnet_ros2 — GraspNet 点云抓取

**功能**: 基于 VCoT-Grasp 大模型的六自由度抓取姿态生成。

| 接口类型 | 名称 | 默认值/类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `input_pointcloud_topic` | `/camera/depth_registered/points` (`PointCloud2`) | 场景点云输入 |
| **Publish** | `marker_topic` | `grasp_markers` (`MarkerArray`) | 4 段圆柱可视化抓取姿态 (手指+手腕+手掌) |
| **Publish** | `grasp_poses_topic` | `grasp_poses_base` (`PoseArray`) | 转换到 base_link 帧的抓取位姿列表 (经 TF lookup) |
| **Service** | `capture_control_service` | `/graspnet_capture_control` (`std_srvs/SetBool`) | true=开始采集 N 组→自动停止, false=手动停止 |
| **TF** | `camera_frame → grasp_pose_i` | dynamic | 每个抓取姿态的独立 TF 帧 |
| **TF lookup** | `base_frame → camera_frame` | — | 查询 TF 将相机系抓取转换到基座系 |

**参数**: `baseline_dir` (GraspNet 代码根目录), `model_path` (权重 .tar), `compute_interval_sec` (1.0s 循环间隔), `capture_groups_target` (3 组自动停止), `num_view` (300), `collision_thresh` (0.01m), `max_grasps_num` (5)

**publish_grasps_client 节点** (继承 GraspMotionController):

| 接口类型 | 名称 | 默认值/类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `grasp_poses_topic` | `grasp_poses_base` (`PoseArray`) | 接收 GraspNet 发布的抓取位姿 |
| **参数** | `grasp_window_size` | 5 | 滑动窗口大小: 缓存最近 N 组位姿 |
| **参数** | `min_groups_before_pick` | 3 | 至少 M 组入库后才开始挑选最垂直抓取 |
| **参数** | `prefer_vertical` | true | true=选垂直度最高 (approach 沿 -Z), false=取第一组 |
| **参数** | `grasp_z_offset` | 0.15m | gripper_tip → end_effector z 轴补偿 |
| **参数** | `height_above` | 0.05m | 抓取点上方安全距离 (笛卡尔路径) |
| **参数** | `wait_poses_timeout_sec` | 30.0s | 等待窗口就绪超时 |

**数据流**: `/camera/depth_registered/points` → GraspNet 推理 (20000→采样, 300 view, 碰撞检测, NMS→Top5) → `grasp_poses_base` (PoseArray in base_link) → publish_grasps_client (窗口选优+gripper_tip补偿+MoveIt笛卡尔路径执行)

**Launch 文件**:
- `graspnet_demo_points_with_tf.launch.py` — 主启动 (使用点云 + TF)
- `graspnet_demo_points.launch.py` — 点云输入版本
- `graspnet_demo.launch.py` — 深度图输入版本
- `graspnet_demo_with_tf.launch.py` — 带 TF 的深度图版本
- `octomap.launch.py` — 八叉树建图

**参数**: `launch_camera` (是否启动相机), `launch_hand_eye_tf` (是否加载手眼 TF 变换), 模型权重路径、置信度阈值等

#### 2.3.4 hand_eye_calibration — 手眼标定

**功能**: 基于 Web 界面的自动手眼标定流程 (采集→标定→验证)，输出相机到末端的变换矩阵。

**Node: hand_eye_calibration_node**

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `/image_data` | `percipio_camera_interface/ImageData` | 标定板图像 (Percipio 格式) |
| **Subscribe** | `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图 — 标定板 Z 值校验 |
| **Subscribe** | `/aubo_driver/robot_status` | `demo_interface/RobotStatus` | 机器人当前位姿 (用于手眼计算) |
| **Client** | `/software_trigger` | `percipio_camera_interface/SoftwareTrigger` | 触发相机采集单帧 |
| **Client** | `/set_robot_pose` | `demo_interface/SetRobotPose` | 标定流程中自动移动机器人 |
| **Client** | `/move_to_pose` | `demo_interface/MoveToPose` | 标定流程中自动移动机器人 |
| **Publish** | `ee_link → camera_link` | 静态 TF (TransformStamped) | 发布标定结果 (hand_eye_tf_publisher 子节点) |
| **Param** | `web_host` | string | Flask 监听地址 (默认 localhost) |
| **Param** | `web_port` | int | Flask 端口 (默认 8080) |
| **Param** | `depth_scale_unit` | float | 深度缩放因子 = 0.00025 (raw×scale=m) |

**标定方法** (Flask API `/api/hand_eye/calibrate`):
- **custom**: SVD 初始估计 + scipy.optimize.least_squares (Levenberg-Marquardt) 非线性优化 (XY+Z 约束)
- **opencv**: cv2.calibrateHandEye() 标准算法 (TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS)

**Web 界面**: Flask HTTP (默认 `http://localhost:8080`)，提供图像预览→采集位姿→标定计算→结果验证的可视化流程。

---

### 2.4 相机驱动层

#### 2.4.1 percipio_camera — 知微相机驱动

**功能**: 驱动 Percipio 深度相机，发布彩色/深度/IR 图像、点云、相机内参，支持软件触发。相机发布在 `camera_name` 命名空间下（默认 `camera`）。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Publish** | `{ns}/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| **Publish** | `{ns}/depth/image_raw` | `sensor_msgs/Image` | 深度图像 (16UC1, 单位 0.25mm) |
| **Publish** | `{ns}/depth/points` | `sensor_msgs/PointCloud2` | 深度点云 |
| **Publish** | `{ns}/depth_registered/points` | `sensor_msgs/PointCloud2` | 彩色点云 (depth 对齐到 color) |
| **Publish** | `{ns}/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| **Publish** | `{ns}/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |
| **Publish** | `{ns}/device_event` | `std_msgs/String` | 设备事件 (Transient Local QoS) |
| **Param** | `camera_name` | string | 相机命名空间 (默认 `camera`) |
| **Param** | `color_enable/depth_enable` | bool | 启用彩色/深度流 |
| **Param** | `color_point_cloud_enable` | bool | 启用彩色点云 (自动开启 depth_registration) |
| **Param** | `depth_registration_enable` | bool | 开启深度对齐到彩色 |

#### 2.4.2 percipio_camera_interface — 相机控制接口

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Service** | `/software_trigger` | `percipio_camera_interface/SoftwareTrigger` | 软件触发采集单帧 |
| **Service** | `/get_camera_info` | — | 获取相机参数信息 |
| **Service** | `/set_camera_param` | — | 设置相机参数 |
| **Publish** | `/camera_status` | — | 相机状态 |

#### 2.4.3 image_data_bridge — 图像格式桥接

**功能**: 将标准 `sensor_msgs/Image` 转换为 Percipio 专有 `ImageData` 格式 (含 camera_id)，供 hand_eye_calibration 和视觉位姿估计使用。

| 接口类型 | 名称 | 类型 | 说明 |
|---------|------|------|------|
| **Subscribe** | `input_image_topic` | `sensor_msgs/Image` | 原始彩色图 (默认 `color/image_raw`，camera 命名空间下 → `/camera/color/image_raw`) |
| **Publish** | `output_topic` | `percipio_camera_interface/ImageData` | ImageData 格式 (默认 `/image_data`) |
| **Param** | `camera_id` | string | 相机 ID (默认 `207000152740`) |
| **Param** | `use_jpeg_encoding` | bool | 是否启用 JPEG 压缩 (默认 false) |

**等价节点**: `hand_eye_calibration` 包内含 `image_data_converter_node`，功能相同但更轻量。

---

### 2.5 Web 人机交互层

#### 2.5.1 aubo_ros2_web_dashboard — Web 控制面板

**功能**: 启动 4 个子进程组成的 Web 栈: rosbridge (WebSocket→ROS2 桥) + tf2_web_republisher (TF Web 发布) + web_video_server (MJPEG 视频流) + FastAPI 网关 (uvicorn, 统一入口)。

**架构**: 网关本身不创建 ROS2 节点，而是通过 WebSocket 客户端连接到 rosbridge (127.0.0.1:9090)，实现 HTTP/WS 代理转发。

| 子进程 | 包 | 通信方式 |
|--------|-----|---------|
| `rosbridge_server` | `rosbridge_suite` | WebSocket :9090 ← ROS2 消息总线 |
| `tf2_web_republisher` | `tf2_web_republisher` | 将 `/tf` 话题转为 Web 可消费格式 |
| `web_video_server` | `web_video_server` | HTTP MJPEG :8089 (需 `image_transport`) |
| `fastapi_static_gateway` | `aubo_ros2_web_dashboard` | uvicorn :8090, 托管 `web/public/`, 代理 rosbridge WS + MJPEG |

**系统 ROS2 包依赖**: `rosbridge_suite`, `tf2_web_republisher`, `tf2_web_republisher_interfaces`, `web_video_server`, `image_transport`, `image_transport_plugins`

**Web 入口** (只需开放 8090):
| URL | 说明 |
|-----|------|
| `http://IP:8090/` | Web 面板主页 — 拓扑图/关节曲线/末端位姿监控 |
| `http://IP:8090/topics_lab.html` | ROS 控制台 — 话题/服务/动作/参数/节点关系图 |
| `http://IP:8090/ws/rosbridge` | rosbridge WebSocket (网关代理 → localhost:9090) |
| `http://IP:8090/api/ivg/proxy/web-video/*` | MJPEG 视频流代理 (网关 → localhost:8089) |

**其他独立 Web 服务**:
| 服务 | 端口 | 说明 |
|------|------|------|
| `visual_pose_estimation_web` | 8088 | 视觉位姿 FastAPI (独立节点) |
| `hand_eye_calibration` | 8080 | 手眼标定 Flask Web (独立节点) |

**关键 YAML 配置** (`config/defaults.yaml`): gateway.port=8090, rosbridge.port=9090, web_video.port=8089

---

### 2.6 教学演示层

#### 2.6.1 base_demo — ROS2 基础教程

**功能**: 演示 ROS2 核心概念：话题 (Topic)、服务 (Service)、参数 (Parameter)。

| 文件 | 接口 | 说明 |
|------|------|------|
| `topic_pub.cpp/py` | **Publish** `/chatter` (`std_msgs/String`) | 话题发布者示例 |
| `topic_sub.cpp/py` | **Subscribe** `/chatter` (`std_msgs/String`) | 话题订阅者示例 |
| `service_server.cpp/py` | **Service** `/add_two_ints` | 加法服务端示例 |
| `service_client.cpp/py` | **Client** `/add_two_ints` | 加法客户端示例 |
| `param_demo.cpp/py` | **Param** `my_param` | 参数声明/读取/回调示例 |

#### 2.6.2 advance_demo — ROS2 进阶教程

**功能**: 演示 ROS2 高级概念：Action、TF 坐标变换、RViz Marker、launch 文件。

| 文件 | 接口 | 说明 |
|------|------|------|
| `action_server.cpp/py` | **Action** `/fibonacci` | Action 服务端 (计算斐波那契) |
| `action_client.cpp/py` | **Client** `/fibonacci` Action | Action 客户端 |
| `tf_pub.cpp` | **Publish** TF `base_link→child_frame` | TF 动态发布者 (运动中的坐标系) |
| `tf_listen.cpp` | **Subscribe** TF | TF 监听者 (查询坐标变换) |
| `pub_marker.cpp` | **Publish** `/visualization_marker` | RViz Marker 可视化 |

#### 2.6.3 xarm_moveit_demo — MoveIt2 完整演示

7 个演示程序，从入门到 MTC 高级 Pick & Place：

| 可执行文件 | 功能 | 使用的 MoveIt2 接口 | ROS2 接口 |
|-----------|------|---------------------|-----------|
| `hello_moveit` | 最简入门: 设目标→规划→执行 | `MoveGroupInterface("xarm")`, `setPoseTarget()`, `plan()`, `execute()` | `rclcpp::Node` |
| `moveit_pose_demo` | 位姿控制 + 碰撞物体增删 | `MoveGroupInterface("xarm"/"gripper")`, `PlanningSceneInterface`, `CollisionObject` | `rclcpp::Node` |
| `moveit_joint_pose_demo` | 关节空间 + 夹爪控制 | `MoveGroupInterface("xarm"/"gripper")`, `setJointValueTarget()`, `setNamedTarget()` | `rclcpp::Node` |
| `moveit_planning_scene_demo` | 规划场景信息 + 当前状态获取 | `getPlanningFrame()`, `getEndEffectorLink()`, `getCurrentPose()`, `getCurrentJointValues()`, `setGoalPositionTolerance()`, `setGoalOrientationTolerance()` | `rclcpp::Node` |
| `moveit_beeline_demo` | 笛卡尔直线路径 (三角形) | `computeCartesianPath()`, `CartesianPath`, `waypoints` 路径点构造 | `rclcpp::Node` |
| `moveit_arcline_demo` | 笛卡尔圆弧路径 (Y-Z平面画圆) | `computeCartesianPath()`, `jump_threshold`, `eef_step` | `rclcpp::Node` |
| `moveit_pick_place_demo` | MTC 完整 Pick & Place (18 个阶段) | MTC `Task`, `SerialContainer`, `stages::CurrentState/MoveTo/MoveRelative/Connect/GenerateGraspPose/GeneratePlacePose/ComputeIK/ModifyPlanningScene`, solvers: `PipelinePlanner`/`JointInterpolationPlanner`/`CartesianPath` | `rclcpp::Node`, `PlanningSceneInterface`, `MultiThreadedExecutor` |

---

## 3. 功能包完整列表

| # | 包名 | 路径 | 构建 | 说明 |
|---|------|------|------|------|
| 1 | **aubo_description** | aubo_ros2_driver/ | ament_cmake | URDF/XACRO 机器人模型 |
| 2 | **aubo_msgs** | aubo_ros2_driver/ | ament_cmake | 驱动消息与服务定义 |
| 3 | **aubo_dashboard_msgs** | aubo_ros2_driver/ | ament_cmake | Dashboard 接口定义 |
| 4 | **aubo_driver_ros2** | aubo_ros2_driver/ | ament_cmake | 真机驱动节点 |
| 5 | **aubo_ros2_trajectory_action** | aubo_ros2_driver/ | ament_cmake | FollowJointTrajectory Action |
| 6 | **aubo_robot_simulator_ros2** | aubo_ros2_driver/ | ament_python | 轨迹插值仿真器 |
| 7 | **aubo_moveit_config** | aubo_ros2_driver/ | ament_cmake | MoveIt2 配置与 launch |
| 8 | **demo_interface** | aubo_ros2_driver/ | ament_cmake | demo_driver 接口定义 |
| 9 | **demo_driver** | aubo_ros2_driver/ | ament_cmake | 运动规划与抓取服务 |
| 10 | **percipio_camera** | camport_ros2/src/ | ament_cmake | 知微相机驱动 |
| 11 | **percipio_camera_interface** | camport_ros2/src/ | ament_cmake | 相机控制接口 |
| 12 | **image_data_bridge** | camport_ros2/src/ | ament_cmake | 图像桥接 |
| 13 | **interface** | visual_pose_estimation/src/ | ament_cmake | 视觉接口定义 (EstimatePose, CartesianPosition) |
| 14 | **visual_pose_estimation_python** | visual_pose_estimation/src/ | ament_python | Python 位姿估计 + Web (FastAPI) |
| 15 | **myrobot_description** | ros_arm_tutorials/ | ament_cmake | 教程用自定义 3-DOF 移动机器人模型 |
| 16 | **hand_eye_calibration** | hand_eye_calibration/ | ament_python | 手眼标定 Web |
| 17 | **graspnet_ros2** | graspnet_ros2/ | ament_python | GraspNet 点云抓取 |
| 18 | **vision_perception** | vision_perception/ | ament_python | YOLO OBB 旋转框感知 |
| 19 | **tool_changer_interface** | tool_changer_interface/ | ament_cmake | 快换接口定义 |
| 20 | **tool_changer** | tool_changer/ | ament_cmake | 夹爪快换 Worker |
| 21 | **coffee_latte_demo** | coffee_latte_demo/ | ament_python | 咖啡拉花 IO 控制 |
| 22 | **aubo_ros2_web_dashboard** | aubo_ros2_web_dashboard/ | ament_python | Web 控制面板 |
| 23 | **base_demo** | ros_arm_tutorials/ | ament_cmake | ROS2 基础教程 |
| 24 | **advance_demo** | ros_arm_tutorials/ | ament_cmake | ROS2 进阶教程 |
| 25 | **xarm_moveit_demo** | ros_arm_tutorials/ | ament_cmake | MoveIt2 7 个演示 |
| 26 | **xarm_description** | ros_arm_tutorials/ | ament_cmake | 教程用机器人模型 |
| 27 | **xarm_moveit_config** | ros_arm_tutorials/ | ament_cmake | 教程用 MoveIt 配置 |

---

## 4. 启动脚本步骤详解

### start_IVG_graspnet_points_fastapi.sh (全栈启动)

| 步骤 | Launch / 命令 | 功能 | 涉及的 ROS2 接口 |
|------|--------------|------|-----------------|
| 0 | `colcon build` | 编译全工作空间 | — |
| 1 | `aubo_moveit_pure_ros2.launch.py` | MoveIt + 驱动 + 插值 + Action + RViz | move_group, `joint_states`, `joint_trajectory_controllerjoint_trajectory_controller/follow_joint_trajectory` Action, `joint_path_command → moveItController_cmd` 管道 |
| 2 | `demo_driver_services.launch.py` | 全部 demo 服务 | `/move_to_pose`, `/plan_trajectory`, `/execute_trajectory`, etc. |
| 3 | `percipio_camera.launch.py` | 相机驱动 (depth_registration 开启) | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth_registered/points` (GraspNet 用), `/camera/depth/points`, `/camera/*/camera_info` |
| 4 | `camera_control.launch.py` | 相机控制接口 | `/software_trigger`, `/camera_status` |
| 5 | `image_data_bridge.launch.py` | Image→ImageData 格式转换 | Sub: `/camera/color/image_raw`, Pub: `/image_data` (percipio ImageData), 供 hand_eye/位姿估计 使用 |
| 6 | `hand_eye_calibration_launch.py` | 手眼标定 Flask Web :8080 | Sub: `/image_data`+`/camera/depth/*`+`/aubo_driver/robot_status`, Client: `/software_trigger`+`/set_robot_pose`+`/move_to_pose`, HTTP API: `/api/hand_eye/calibrate`(custom/opencv) |
| 7 | `visual_pose_estimation_python.launch.py` | 位姿估计节点 | `/estimate_pose`, `/list_templates`, `/system_status` |
| 8 | `graspnet_demo_points_with_tf.launch.py` | GraspNet 点云→6DOF 抓取生成 | Sub: `/camera/depth_registered/points`, Pub: `grasp_markers` (MarkerArray) + `grasp_poses_base` (PoseArray in base_link), TF: `camera_frame→grasp_pose_i`, Srv: `/graspnet_capture_control` |
| 9 | `execute_grasp_pose_worker.launch.py` | 抓取执行 Worker | `/execute_single_grasp`, `/loop_grasp_control` |
| 10 | `gripper_swap_worker_node` | 工具快换 Worker | `/run_gripper_swap`, `/tool_changer_status`, `/aubo_driver/set_io` |
| 11 | `publish_grasps_client_worker_node` | GraspNet 循环抓取 | `/publish_grasps_worker_loop_control`, `/graspnet_capture_control` |
| 12 | 服务自检 (`rg`) | 验证关键服务就绪 | — |
| 13 | `visual_pose_estimation_web.launch.py` | 位姿 FastAPI :8088 | REST API + WebSocket |
| 14 | `ros2 bag record` | 录包 (可选) | 录制配置的话题集 |

### 关键环境变量

| 变量 | 说明 |
|------|------|
| `AUBO_ROS2_WS` | 工作空间根路径 |
| `ROS_DISTRO_NAME` | ROS 发行版 (默认 humble) |
| `WEB_HOST` / `WEB_PORT` | VPE FastAPI 绑定地址 (默认 8088) |
| `WEB_DASH_HOST` / `WEB_DASH_PORT` | Web 面板地址 (默认 8090) |
| `ROSBRIDGE_PORT` | rosbridge WebSocket 端口 (默认 9090) |
| `IVG_ROSBAG_DIR` | rosbag 保存目录 (默认 rosbags/ivg_session) |
| `IVG_ROSBAG_TOPICS` | 录制的自定义话题列表 |

---

## 5. 构建与环境

```bash
cd /path/to/aubo_ros2_ws
source /opt/ros/humble/setup.bash
# 如有独立 MoveIt 覆盖:
# source ~/ws_moveit/install/setup.bash
colcon build
source install/setup.bash
```

仅编译特定包:
```bash
colcon build --packages-select aubo_driver_ros2 aubo_moveit_config demo_driver
colcon build --packages-select xarm_moveit_demo tool_changer
colcon build --packages-select vision_perception visual_pose_estimation_python
```

---

## 6. 外部依赖

| 依赖 | 用途 |
|------|------|
| **ROS 2 Humble** | 核心通信框架 |
| **MoveIt 2** | 运动规划 (OMPL 规划器) |
| **ros2_control (仿真用 mock)** | 机器人控制框架 |
| **OpenCV** | 图像处理 (`core imgproc highgui photo`) |
| **Ultralytics YOLO** | YOLO26 OBB 旋转框检测 |
| **GraspNet (VCoT-Grasp)** | 六自由度抓取姿态生成 |
| **PyTorch, Open3D, scipy** | GraspNet 依赖 |
| **FastAPI, uvicorn** | Web API 服务 |
| **Flask** | 手眼标定 Web UI |
| **rosbridge_suite** | WebSocket ↔ ROS2 桥接 |
| **tf2_web_republisher** | TF 数据 Web 发布 |
| **web_video_server** | MJPEG 视频流 |
| **Eigen3** | 线性代数 (坐标变换 C++) |
| **NumPy** | 数值计算 (Python) |

---

## 7. IO 引脚完整分配

| 逻辑引脚 | 硬件引脚 | 使用者 | 用途 | 通道 |
|---------|---------|--------|------|------|
| **2** | 34 | LatteNode | 打花开关 (DO2) | `/aubo_driver/set_io(io_index=2)` |
| **4** | 36 | LatteNode | 咖啡开关 (DO4) | `/aubo_driver/set_io(io_index=4)` |
| **6** | 38 | ExecuteGraspPose / PublishGraspsClient / PublishGraspsAB | 夹爪开/关 | `/aubo_driver/set_io(io_index=6)` |
| **7** | 39 | GripperSwapWorker | 快换盘锁紧/释放 | `/aubo_driver/set_io(io_index=7)` |

> IO 规则: `io_type=digital_output`, `io_index + 32 → 硬件引脚`

---

## 8. 末端工具管理

| 工具 ID | 类型 | 管理包 | 用途 | 对接方式 |
|---------|------|--------|------|---------|
| **gripper0** | 气动夹爪 φ40 | tool_changer | 工件抓取 (visual_pose_estimation) | 自动快换对接 |
| **gripper1** | 电动夹爪 A | coffee_latte_demo | 咖啡拉花 (coffee_cup / milk_cup) | 不参与快换 |
| **gripper2** | 电动夹爪 φ60 | tool_changer | AI 抓取 (graspnet_ros2) | 自动快换对接 |

**快换流程**: gripper0 ↔ gripper2 双向自动切换:
1. 关节空间移动到 dock 工位
2. 笛卡尔接近 → IO 释放/锁定
3. 笛卡尔抬离
4. 移动到目标 dock → 对接 → 锁定
5. 场景物体 attach/detach → 回 Home

---

## 9. 依赖关系图

```
                    ┌──────────────────────┐
                    │  aubo_ros2_web_      │
                    │  dashboard            │
                    │  (rosbridge + 前端)    │
                    └──────────┬───────────┘
                               │ rosbridge WS
        ┌──────────────────────┼──────────────────────┐
        ▼                      ▼                      ▼
┌──────────────┐  ┌───────────────────┐  ┌───────────────────┐
│ demo_driver  │  │ tool_changer      │  │ coffee_latte_demo │
│ (运动/抓取)   │  │ (快换)            │  │ (IO 控制)          │
├──────────────┤  ├───────────────────┤  ├───────────────────┤
│ dep:         │  │ dep:              │  │ dep:              │
│ demo_interface│ │ tool_changer_if   │  │ demo_interface    │
│ aubo_msgs    │  │ demo_interface    │  │ aubo_msgs         │
│ interface    │  │ aubo_msgs         │  │                   │
│ moveit_*     │  │ moveit_core       │  │                   │
└──────┬───────┘  └────────┬──────────┘  └────────┬──────────┘
       │                   │                      │
       │                   └──────────┬───────────┘
       │                              │
       │              /aubo_driver/set_io (IO 服务)
       │                              │
       ▼                              ▼
┌────────────────────────────────────────────────┐
│              aubo_driver_ros2                   │
│              (真机驱动)                          │
│                                                │
│  依赖: aubo_msgs, demo_interface                │
│  发布: joint_states, aubo/feedback_states,      │
│        /aubo_driver/robot_status,               │
│        /aubo_driver/io_states                   │
│  服务: /aubo_driver/set_io, get_ik, get_fk      │
│  订阅: moveItController_cmd                     │
└──────────────────────┬─────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
┌────────────┐ ┌────────────┐ ┌──────────────────┐
│ trajectory │ │ simulator  │ │ aubo_moveit_     │
│ _action    │ │ _ros2      │ │ config           │
│ (Action)   │ │ (五次多项式)│ │ (MoveIt 入口)     │
│ Pub: joint │ │ Pub: moveIt│ │ Launch: all      │
│ _path_cmd  │ │ Controller │ │                  │
│ Sub: aubo/ │ │ _cmd       │ │                  │
│ feedback_  │ │ Sub: joint │ │                  │
│ states     │ │ _path_cmd  │ │                  │
└────────────┘ └────────────┘ └──────────────────┘

视觉感知链:
┌────────────┐   ┌───────────────┐   ┌──────────────────┐
│ percipio   │──▶│ vision_       │──▶│ visual_pose_     │
│ _camera    │   │ perception    │   │ estimation_python│
│ (相机驱动)  │   │ (YOLO OBB)    │   │ (工件位姿估计)     │
└──┬──┬──┬──┘   └───────────────┘   └────────┬─────────┘
   │  │  │                                    │
   │  │  │ /camera/depth_registered/points    │
   │  │  └──────────────┐                     │
   │  │ /camera/color/*  │                     │ /estimate_pose
   │  ▼                  ▼                     ▼
   │ image_data_bridge  graspnet_ros2    demo_driver
   │ Pub: /image_data   Pub: grasp_      (抓取执行)
   │ (ImageData)        poses_base
   │       │
   ▼       ▼
┌──────────────────┐
│ hand_eye_        │
│ calibration      │
│ Sub: /image_data │
│ (手眼标定Web)    │
└──────────────────┘
```

---

*最后更新: 2026-04-30*
*维护者: muhan11, wangxiaoyun@iscas.ac.cn*
