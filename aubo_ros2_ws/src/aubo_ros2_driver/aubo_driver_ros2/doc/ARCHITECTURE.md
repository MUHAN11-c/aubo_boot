# aubo_driver_ros2 架构文档

## 设计哲学

**借鉴 ros2_control 架构，但不依赖 ros2_control 框架**。

AUBO SDK 是同步阻塞的 TCP 通信库（单次调用 2-225ms），无法满足 ros2_control 的实时（RT）约束。但 ros2_control 的核心设计思想——硬件抽象层、控制器分离、生命周期管理——完全值得借鉴。

我们采用 **"独立驱动 + ros2_control 模式"** 的混合方案，参考 UR ROS2 Driver 的成熟架构。

### 参考来源

| 参考 | 借鉴的设计 |
|------|-----------|
| UR `URPositionHardwareInterface` | async I/O 线程 + GPIO named interfaces |
| UR `DashboardClientROS` | LifecycleNode + 独立进程 + Trigger service |
| Franka `FrankaExecutor` | 后台 MultiThreadedExecutor 隔离非RT工作 |
| ros2_control `joint_trajectory_controller` | 5次插值 + Action 状态机 |
| `PORTING_MOTION_FIX.md` | RIB 实时查询、数据竞争、start_move_ 误关 |

---

## 整体架构

```
┌──────────────────────────────────────────────────────────────────┐
│                     Launch File                                  │
│  aubo_moveit_pure_ros2.launch.py                                │
│                                                                  │
│  Nodes launched:                                                 │
│    aubo_dashboard_node       (LifecycleNode, 非RT管理)           │
│    aubo_state_broadcaster    (200+50Hz 状态广播)                 │
│    joint_trajectory_controller (200Hz 轨迹插值+Action)           │
│    move_group                (MoveIt2)                           │
│    robot_state_publisher     (标准 ROS2)                         │
│    rviz2                     (可视化)                            │
└──────────────────────────────────────────────────────────────────┘

数据流:

MoveIt2 (move_group)
  │ FollowJointTrajectory Action Goal
  ▼
joint_trajectory_controller
  │ 200Hz 5次多项式插值 (C++)
  │ Action 状态机 (goal/cancel/succeed/abort)
  │ 目标容差检查 (连续5帧 < 0.02rad)
  ▼
AuboHardwareInterface (controller 实例)
  │ conn_control_: writeTrajectoryPoints() → robotServiceSetRobotPosData2Canbus
  │ conn_control_: readDiagnosis() → RIB 流量控制
  │ conn_status_: readJointState() → 关节位置/速度
  ▼
AUBO Robot Controller (TCP port 8899)

并行:

aubo_state_broadcaster (独立 HardwareInterface)
  │ 200Hz → joint_states, aubo/feedback_states
  │ 50Hz  → aubo/robot_status, aubo/rib_status, aubo/io_states
  │ 回调   → RobotEventCallback → 紧急停止/防护停止/碰撞

aubo_dashboard_node (独立 HardwareInterface, LifecycleNode)
  │ 系统管理: /aubo/startup, shutdown, brake_release, stop
  │ 运动控制: /aubo/move_joint, move_line, teach_start/stop
  │ 配置:    /aubo/set_payload, set_collision_class, set_tool
  │ 运动学:  /aubo/get_fk, get_ik
  │ 诊断:    /aubo/get_robot_info, get_joint_status, get_safety_config
  │ IO:     /aubo/set_io
```

---

## 组件详解

### 1. AuboHardwareInterface（硬件抽象层）

**位置**: `include/aubo_driver_ros2/aubo_hardware_interface.h`

**职责**: 封装 AUBO SDK 的所有通信操作，向上层提供统一的 read/write 接口。

```
AuboHardwareInterface
├── conn_control_  [ServiceInterface] — TCP2CAN 轨迹流 + RIB 诊断
├── conn_status_   [ServiceInterface] — 状态查询 + IO + SDK 回调
│
├── 生命周期: init() → enterTcp2CanbusMode() → ... → leaveTcp2CanbusMode() → shutdown()
│
├── 实时路径 (控制循环线程调用):
│   ├── readJointState()     → robotServiceGetCurrentWaypointInfo
│   ├── readDiagnosis()      → robotServiceGetRobotDiagnosisInfo
│   ├── writeTrajectoryPoints()  → robotServiceSetRobotPosData2Canbus (batch)
│   └── writeTrajectoryPoint()   → robotServiceSetRobotPosData2Canbus (single)
│
├── 异步路径 (非RT线程调用):
│   ├── readSafetyIOStatus() → robotServiceGetBoardIOStatus (ControllerDI)
│   ├── readFullIOStatus()   → 多种 IO 类型批量查询
│   ├── writeIOCommand()     → robotServiceSetBoardIOStatus
│   ├── writeToolIOCommand() → robotServiceSetToolDigitalIOType + SetToolDOStatus
│   └── writeToolPowerType() → robotServiceSetToolPowerVoltageType
│
└── SDK 回调 (SDK 内部线程执行):
    ├── onRobotEvent()        → 事件通知 (急停/碰撞/断开等)
    ├── onJointStatus()       → 关节状态实时推送
    ├── onRoadPoint()         → 路点实时推送
    └── onEndSpeed()          → 末端速度实时推送
```

**设计关键**:
- 每个 HardwareInterface 实例拥有独立的 ServiceInterface 连接
- 不同节点（Controller / Broadcaster / Dashboard）使用各自的 HardwareInterface 实例
- 连接隔离避免了 SDK 冲突

### 2. JointTrajectoryController（轨迹控制器）

**位置**: `include/aubo_driver_ros2/joint_trajectory_controller.h`

**职责**: 替代 Python `aubo_robot_simulator_ros2` + `aubo_ros2_trajectory_action`。

```
JointTrajectoryController : rclcpp::Node
│
├── Action 服务器: FollowJointTrajectory
│   ├── handleGoal()     → 校验+接受轨迹目标
│   ├── handleCancel()   → 取消当前目标
│   └── handleAccepted() → 准备执行
│
├── 200Hz 控制循环: update()
│   ├── 计算当前应执行的轨迹段
│   ├── quinticInterpolate() → 5次多项式插值
│   ├── hw_->writeTrajectoryPoints() → 下发到机器人
│   ├── 发布 joint_states + feedback_states
│   └── withinGoalConstraints() → 检查是否到达目标
│
└── 插值算法:
    ├── quinticInterpolate()     → 5次多项式 (从 Python simulator 移植)
    ├── blendToFirstPoint()      → 边界混合过渡 (PORTING 根因A)
    └── remapJointNames()        → 关节名称重映射
```

**5次多项式系数公式**:
```
a1 = v_last
a2 = 0.5 * a_last
h  = p_curr - p_last
a3 = 0.5/T³ * (20h - (8v_curr + 12v_last)T - (3a_last - a_curr)T²)
a4 = 0.5/T⁴ * (-30h + (14v_curr + 16v_last)T + (3a_last - 2a_curr)T²)
a5 = 0.5/T⁵ * (12h - 6(v_curr + v_last)T + (a_curr - a_last)T²)

position(t) = p_last + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
```

### 3. AuboDashboardNode（仪表板节点）

**位置**: `include/aubo_driver_ros2/aubo_dashboard_node.h`

**职责**: 暴露全部非实时 SDK 功能（参考 UR DashboardClientROS）。

**Lifecycle 状态机**:
```
UNCONFIGURED ──on_configure──▶ INACTIVE ──on_activate──▶ ACTIVE
     ▲                            │                          │
     │                            │                          │
     └──on_cleanup────────────────┘          on_deactivate───┘
     │
     └──on_shutdown──▶ FINALIZED
```

**20 个 ROS2 服务**:
```
/aubo/startup            → rootServiceRobotStartup (上电+松刹车+碰撞等级)
/aubo/shutdown           → robotServiceRobotShutdown
/aubo/brake_release      → robotServiceReleaseBrake
/aubo/stop               → rootServiceRobotMoveControl(RobotMoveStop)
/aubo/fast_stop          → robotMoveFastStop
/aubo/collision_recover  → robotServiceCollisionRecover
/aubo/move_joint         → robotServiceJointMove (自动处理 TCP2CAN 切换)
/aubo/move_line          → robotServiceLineMove
/aubo/teach_start        → robotServiceTeachStart
/aubo/teach_stop         → robotServiceTeachStop
/aubo/set_collision_class → robotServiceSetRobotCollisionClass
/aubo/set_payload        → robotServiceSetToolDynamicsParam
/aubo/set_tool_kinematics → robotServiceSetToolKinematicsParam
/aubo/set_tool_voltage   → robotServiceSetToolPowerVoltageType
/aubo/set_io             → robotServiceSetBoardIOStatus
/aubo/get_fk             → robotServiceRobotFk
/aubo/get_ik             → robotServiceRobotIk
/aubo/get_robot_info     → robotServiceGetRobotDevInfoService
/aubo/get_joint_status   → robotServiceGetRobotJointStatus
/aubo/get_safety_config  → robotServiceGetRobotSafetyConfig
```

### 4. AuboStateBroadcaster（状态广播器）

**位置**: `src/aubo_state_broadcaster.cpp`

**职责**: 替代旧的 `publishIOMsg` 和 `publishJointStateAndFeedbackLoop` 线程。

```
AuboStateBroadcaster : rclcpp::Node
│
├── 200Hz fastTick() → joint_states + aubo/feedback_states
│
└── 50Hz timerTick() → aubo/robot_status + aubo/rib_status
```

---

## 与旧架构对比

| 维度 | 旧架构 | 新架构 |
|------|--------|--------|
| 轨迹插值 | Python simulator (GIL) | C++ handleAccepted 预计算, 一次性 |
| 轨迹发送 | 4ms 循环, 自适应批量 2-8 | 独立 sendLoop 线程, ROS1 移植 |
| 插值-发送耦合 | 紧密耦合 (同一 timer) | **完全解耦** (插值≠发送) |
| RIB 流控 | real-time 查询, 手动追踪 | 降频查询(120/250ms), 同连接, RIB≥300 门控 |
| 状态广播 | std::thread 手动管理 | ROS2 wall timer + SDK 回调 |
| 关节状态来源 | 轮询 getCurrentWaypointInfo | RoadPoint+JointStatus 回调 |
| SDK 功能暴露 | 仅 22/163 函数 | Dashboard 20服务 + 全部 163 函数有接口 |
| SDK 连接数 | 3 个 (send/receive/mac_size) | Controller:2 个(control/status) |
| 生命周期 | 普通 Node, 构造函数 login | LifecycleNode, 显式状态转换 |
| 消息类型 | Float32MultiArray 传关节数据 | sensor_msgs::JointState |
| 运动平滑度 | RIB 断供导致间歇卡顿 | 预计算消除断供, 实测流畅 |

### 关键实测数据

- 轨迹 2.69s / 539 预计算点: 约 1s 内全部发完, RIB 稳定, 无卡顿异响
- 回调 JointStatus: 33.5Hz (位置+速度+电流+温度)
- 轮询 joint_states: 100Hz (仅位置)
- SDK 批量发送(8点/批): 平均 5-15ms, EMA 追踪
- RIB 门控 RIB≥300: 有效防溢出, 4ms 重查不卡死

---

## 文件组织

```
aubo_driver_ros2/
├── include/aubo_driver_ros2/
│   ├── aubo_hardware_interface.h          # 硬件抽象层 (新建)
│   ├── joint_trajectory_controller.h      # 轨迹控制器 (新建)
│   ├── aubo_dashboard_node.h              # Dashboard 节点 (新建)
│   ├── aubo_driver.h                      # 原始驱动 (保留兼容)
│   ├── serviceinterface.h                 # AUBO SDK API
│   ├── AuboRobotMetaType.h               # SDK 数据类型
│   └── otg/                               # OTG 运动库
├── src/
│   ├── aubo_hardware_interface.cpp        # 硬件抽象层实现 (新建)
│   ├── joint_trajectory_controller.cpp    # 轨迹控制器实现 (新建)
│   ├── aubo_dashboard_node.cpp            # Dashboard 实现 (新建)
│   ├── dashboard_node_main.cpp            # Dashboard 入口 (新建)
│   ├── aubo_state_broadcaster.cpp         # 状态广播器 (新建)
│   ├── aubo_driver_ros2.cpp               # 原始驱动实现
│   └── driver_node_ros2.cpp               # 原始驱动入口
├── config/
│   ├── auborobot.conf
│   └── tracelog.properties
├── lib/                                   # AUBO SDK 预编译库
├── doc/
│   ├── SDK_CONFLICT_RULES.md              # SDK 冲突规则 (新建)
│   ├── ARCHITECTURE.md                    # 本文档 (新建)
│   ├── API_MAPPING.md                     # SDK→ROS2 API 映射 (新建)
│   ├── PORTING_MOTION_FIX.md              # 移植调试记录
│   ├── MOTION_JITTER_FIX_SUMMARY_2026-03-18.md
│   └── DIRECT_API_TEST_COMMANDS.md
├── CMakeLists.txt
└── package.xml
```

---

## Git 提交记录与操作过程

```bash
# 查看当前分支
git branch                    # 确认在 dev 分支

# 暂存变更
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/CMakeLists.txt
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/package.xml
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/README.md
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/include/aubo_driver_ros2/*.h
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/src/*.cpp
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/doc/*.md
git add aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/launch/aubo_new_driver.launch.py
git add aubo_ros2_ws/src/aubo_ros2_driver/demo_interface/srv/*.srv
git add aubo_ros2_ws/src/aubo_ros2_driver/demo_interface/CMakeLists.txt
git add aubo_ros2_ws/start_aubo_new_driver.sh

# 提交
git commit -m "新框架驱动: JointTrajectoryController + Dashboard + StateBroadcaster
核心改动:
- 预计算200Hz轨迹插值 (handleAccepted)
- 独立发送线程 (ROS1 publishWaypointToRobot移植)
- RIB流量控制 (同连接,降频查,≥300门控,自适应批量2-8,EMA补偿)
- SDK回调替代轮询 (RoadPoint+JointStatus 33Hz推送)
- Dashboard LifecycleNode (20 ROS2服务)"

# 远程有更新,先暂存本地未提交变更
git stash

# 拉取并变基 到最新 dev 分支
git pull --rebase origin dev

# 推送
git push origin dev

# 恢复暂存
git stash pop

# 提交历史
git log --oneline -5
# f854c2e52 新框架驱动: JointTrajectoryController + Dashboard + StateBroadcaster
# 1491ffba4 Delete build directory
# cd1c0ab3f feat: 新旧两套机械臂驱动架构并存
```

## 开发调试命令速查

```bash
# === 构建 ===
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select aubo_driver_ros2 demo_interface aubo_moveit_config

# === 启动 (新框架) ===
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
# 或启动脚本 (含视觉/Web全家桶):
./start_aubo_new_driver.sh

# === 启动 (旧框架) ===
ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py aubo_driver_server_host:=169.254.10.98

# === 单独测试新组件 ===
ros2 run aubo_driver_ros2 aubo_dashboard_node --ros-args -p server_host:=169.254.10.98
ros2 lifecycle set /aubo_dashboard configure
ros2 lifecycle set /aubo_dashboard activate

ros2 run aubo_driver_ros2 aubo_state_broadcaster --ros-args -p server_host:=169.254.10.98
ros2 run aubo_driver_ros2 joint_trajectory_controller --ros-args -p server_host:=169.254.10.98
ros2 run aubo_driver_ros2 aubo_callback_monitor --ros-args -p server_host:=169.254.10.98

# === Dashboard 服务测试 ===
ros2 service call /aubo/startup std_srvs/srv/Trigger {}
ros2 service call /aubo/move_joint demo_interface/srv/MoveJoint "{joints: [0,-0.5,1.2,0,0.8,0], velocity: 0.8}"
ros2 service call /aubo/get_fk aubo_msgs/srv/GetFK "{joint: [0,0,0,0,0,0]}"
ros2 service call /aubo/set_collision_class demo_interface/srv/SetCollisionClass "{grade: 6}"
ros2 service call /aubo/set_payload aubo_msgs/srv/SetPayload "{mass: 1.0, center_of_gravity: {x: 0.0, y: 0.0, z: 0.1}}"

# === Git 操作 ===
git add [files]
git commit -m "message"
git pull --rebase origin dev
git push origin dev
```

## 开发过程关键节点

| 时间 | 事件 | 结论 |
|------|------|------|
| 初版 | 单线程 update() 直接调 SDK | SDK 阻塞拖慢插值 → 运动慢 10 倍 |
| v2 | 插值+发送分两线程 (队列桥接) | 队列被抽空 → RIB 断供 → 卡顿 |
| v3 | 预计算轨迹 (handleAccepted) + sendLoop | **运动平滑** ✅ |
| v4 | RIB 门控 + 自适应批量 | 发太快 RIB 溢出 → 速度突变断电 |
| v5 | RIB 改回 conn_control_ (同连接) | 门控正常 ✅ |
| v6 | 目标检查修复 + 日志 | **Goal reached ✅** |
| v7 | 全流程成功 | Plan and Execute request complete ✅ |

## 核心教训

1. **RIB 必须在同一条连接上读写** — 发送用 conn_control_, 查询用 conn_status_ → RIB 不更新
2. **预计算 + 独立发送线程** — 消除插值与发送的时序耦合
3. **SDK 阻塞无法绕过** — 正确做法是把阻塞隔离在独立线程
4. **5 次插值 + 200Hz 路点频率** — 机器人控制器内部插值足够平滑
5. **回调数据优于轮询** — 33Hz 推送 + 编码器速度 + 电流温度, 30Hz 延迟更小

## 动态 URDF 重载（RViz2 平滑渲染 + 碰撞模型更新）

### 背景问题

工具快换系统切换末端夹爪时，需要 RViz2 和 move_group 同步更新机械臂模型：
- **RViz2** 需要重新渲染夹爪模型（平滑着色）
- **move_group** 需要更新碰撞模型（路径规划避开夹爪）

**原有问题**：

| 组件 | 初始加载方式 | 动态重载机制 |
|------|------------|---------|
| `robot_state_publisher` | 构造函数读 `robot_description` 参数 | `parameterUpdate()` → `onParameterEvent()` → `setupURDF()` ✓ |
| RViz2 `MotionPlanning` 插件 | `PlanningSceneDisplay::loadRobotModel()` 读一次 | **无** ✗ |
| `move_group` | `PlanningSceneMonitor` → `RobotModelLoader` → `RDFLoader` 读一次 | **无** ✗ |

**结果**：工具切换后 RViz2 和 move_group 的 RobotModel 仍为旧版（不含夹爪 link），导致：
1. RViz2 夹爪模型不显示 → 只能用 PlanningScene `AttachedCollisionObject`（平面着色，效果差）
2. move_group 碰撞模型不含夹爪 → 路径规划可能碰撞

### 解决方案

**核心思路**：复用 MoveIt2 初始加载 URDF 的完整渲染路径（Assimp 平滑着色），改为动态触发。

修改 3 个包：

| 包 | 修改内容 |
|------|---------|
| `moveit_ros_visualization` | `PlanningSceneDisplay` 新增 `/robot_description` 话题订阅，URDF 变更时自动重载 RobotModel |
| `moveit_ros_planning_interface` | `common_objects.cpp` 新增 `clearSharedRobotModelLoader()` 清除静态缓存 |
| `tool_changer` | `scene_attach_worker` 新增 `/robot_description` 话题发布者，直接发布 URDF |

### 数据流

```
工具切换 → gripper_swap_worker
  → /tool_changer_status
    → scene_attach_worker.onToolStatus()
      ├─ removeToolFromWorld()              ← PlanningScene diff (dock 显示)
      ├─ addToolToWorldDock()               ← 旧工具放回 dock
      └─ updateRobotDescription()
          ├─ robot_description_pub_->publish(urdf)     ← /robot_description 话题 (NEW)
          │   └─ RViz2 PlanningSceneDisplay.onRobotDescriptionTopic()
          │       ├─ 首次消息记录内容，跳过（初始加载由 onEnable 完成）
          │       ├─ 后续变更: set_parameter() + pending_urdf_ ← 绕过缓存
          │       └─ loadRobotModel()
          │           └─ createPlanningSceneMonitor()
          │               └─ RobotModelLoader(Options(urdf_string, ""))
          │                   └─ RDFLoader 直接用 URDF 字符串 → 重建 RobotModel
          │                       └─ Assimp 平滑渲染 ✓
          └─ param_client_->set_parameters(...)  ← robot_state_publisher
              └─ setupURDF() → 发布新 TF (gripperX_Link) ✓
```

### 关键代码修改

#### 1. `planning_scene_display.h` — 新增成员

```cpp
// /robot_description 话题订阅（动态 URDF 重载）
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
std::string last_loaded_urdf_;   // 去重：跳过初始消息和重复内容
std::string pending_urdf_;       // 非空时 createPlanningSceneMonitor 直接透传 URDF 字符串
void onRobotDescriptionTopic(const std_msgs::msg::String::ConstSharedPtr& msg);
```

#### 2. `planning_scene_display.cpp` — onInitialize() 创建订阅

```cpp
robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::ConstSharedPtr& msg) { onRobotDescriptionTopic(msg); });
```

#### 3. `planning_scene_display.cpp` — 回调逻辑

```cpp
void PlanningSceneDisplay::onRobotDescriptionTopic(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  if (msg->data.empty() || msg->data == last_loaded_urdf_) return;

  bool is_initial = last_loaded_urdf_.empty();
  last_loaded_urdf_ = msg->data;
  if (is_initial) return;  // 首次跳过（初始加载由 onEnable 完成）

  node_->set_parameter(rclcpp::Parameter("robot_description", msg->data));
  pending_urdf_ = msg->data;  // 绕过 getSharedRobotModelLoader 静态缓存

  if (isEnabled())
    addBackgroundJob([this] { loadRobotModel(); }, "loadRobotModel");
}
```

#### 4. `planning_scene_display.cpp` — createPlanningSceneMonitor() 直接构造

```cpp
PlanningSceneMonitorPtr PlanningSceneDisplay::createPlanningSceneMonitor()
{
  RobotModelLoaderPtr rml;
  if (!pending_urdf_.empty())
  {
    // 绕过静态缓存 + 参数读取 → URDF 字符串直达 RDFLoader
    RobotModelLoader::Options opt(pending_urdf_, "" /* srdf */);
    rml = std::make_shared<RobotModelLoader>(node_, opt);
    pending_urdf_.clear();
  }
  else
  {
    rml = getSharedRobotModelLoader(node_, "robot_description");
  }
  return std::make_shared<PlanningSceneMonitor>(node_, rml, getNameStd() + "_psm");
}
```

#### 5. `scene_attach_worker.cpp` — 直接发布 URDF

```cpp
// 构造函数中创建发布者
robot_description_pub_ = create_publisher<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local());

// updateRobotDescription() 中直接发布（不依赖 robot_state_publisher 重发）
void SceneAttachWorker::updateRobotDescription(const std::string& tool_id)
{
  // 1. 直接发布 URDF → RViz2 立即收到并触发重载
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf;
  robot_description_pub_->publish(std::move(msg));

  // 2. 设置参数 → robot_state_publisher 重载 TF
  param_client_->set_parameters({rclcpp::Parameter("robot_description", urdf)}, ...);
}
```

#### 6. `scene_attach_worker.cpp` — onToolStatus() 简化

去掉冗余的 `AttachedCollisionObject`（产生平面着色叠加），只用 world dock CollisionObject + URDF 更新：

```cpp
void SceneAttachWorker::onToolStatus(const ToolChangerStatus& msg)
{
  const std::string new_tool = msg.is_connected ? msg.tool_id : "";
  if (new_tool == current_attached_tool_) return;

  // 旧工具放回 world dock, 新工具从 dock 移除
  if (!current_attached_tool_.empty()) addToolToWorldDock(current_attached_tool_);
  if (!new_tool.empty()) removeToolFromWorld(new_tool);

  // URDF 更新 → 平滑渲染 (Assimp) + 碰撞几何 (URDF <collision>)
  updateRobotDescription(new_tool);
  current_attached_tool_ = new_tool;
}
```

### 调试记录

#### 问题 1: `visualization_msgs/msg/marker.hpp` 编译错误

**现象**：`tool_changer` 编译失败，`scene_attach_worker.h` include 了不存在的 `visualization_msgs/msg/marker.hpp`

**修复**：删除未实现的 `publishToolMarker()` 声明和对应 include

#### 问题 2: `ParameterNotDeclaredException` 导致 RViz2 崩溃

**现象**：`onRobotDescriptionTopic` 中 `node_->set_parameter()` 抛出异常

**修复**：加 `has_parameter()` 检查 + `declare_parameter()` 兜底；增加 `is_initial` 判断避免首次消息触发重载

#### 问题 3: `getSharedRobotModelLoader` 静态缓存返回旧 Loader

**现象**：日志显示 `Loading robot model 'aubo_e5'`（确认重载发生），但 `gripperX_Link not found` 持续报错。新模型不含夹爪 link。

**根因**：`getSharedRobotModelLoader()` 中 `weak_ptr.lock()` 返回旧 `RobotModelLoader`。旧 loader 内部存的是初始 URDF（无夹爪），`set_parameter()` 不会让它重新读取。即使 `clearRobotModel()` 销毁了 PSM，aliasing `RobotModelPtr`（来自 `getSharedRobotModel` 的 `s.models_` 缓存）仍持有 loader 的引用计数。

**尝试过**：
1. 新增 `clearSharedRobotModelLoader()` → overlay/underlay 两套 `SharedStorage` 静态变量独立副本，清除 overlay 缓存无效
2. 直接构造 `RobotModelLoader::Options(urdf_string, srdf_string)` → URDF 字符串直达 `RDFLoader` → **成功绕过所有缓存** ✓

#### 问题 4: overlay/underlay 包含路径冲突

**现象**：编译 `moveit_ros_visualization` 时 `clearSharedRobotModelLoader` 符号找不到

**根因**：`source` 顺序导致 underlay 的 `moveit_ros_planning_interface` include 路径优先于 overlay

**修复**：`source` 顺序改为 overlay 最后（保证 `AMENT_PREFIX_PATH` 中 overlay 在最前）

### 待解决

- [ ] **touch_links 碰撞排除**：去掉 AttachedCollisionObject 后，夹爪与 `camera_Link`/`wrist3_Link` 的碰撞排除需要通过动态 SRDF（`robot_description_semantic`）或 PlanningScene ACM 条目实现
- [ ] **move_group 动态重载**：`move_group` 的 `PlanningSceneMonitor` 同样不监听 `/robot_description`，需要类似修改才能更新其碰撞模型

## 参考

- `doc/SDK_CONFLICT_RULES.md` — SDK 模式冲突规则详解
- `doc/API_MAPPING.md` — 120+ SDK 函数到 ROS2 接口的完整映射
- `doc/PORTING_MOTION_FIX.md` — ROS1→ROS2 移植完整记录 (13 轮调试)
- UR ROS2 Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Franka ROS2: https://github.com/frankaemika/franka_ros2
- AUBO ROS1: https://github.com/AuboRobot/aubo_robot
# AUBO 新框架驱动架构

## 完整控制链路

```
RViz MotionPlanning 面板
  │ Plan & Execute
  ▼
move_group (MoveIt2)
  │ FollowJointTrajectory Action Goal
  ▼
JointTrajectoryController
  ├─ handleAccepted: 预计算 200Hz 完整轨迹插值
  ├─ sendLoop: 独立线程, ROS1 风格自适应批量发送
  └─ update: 安全检查 + 目标容差检查
  ▼
AuboHardwareInterface
  ├─ conn_control_ (TCP2CAN): 轨迹发送 + RIB 诊断
  └─ conn_status_ (普通): 状态查询 + 事件回调
  ▼
AUBO 机器人控制器

并行节点:
  aubo_state_broadcaster
    ├─ RoadPointCallback (33Hz) → joint_states
    └─ JointStatusCallback (33Hz) → aubo/feedback_states

  aubo_dashboard_node (LifecycleNode)
    └─ 20 ROS2 服务
```

## 核心设计决策

### 1. 预计算轨迹 (handleAccepted)

收到 Goal 后一次性完成全部 200Hz 插值,存入 `precomputed_` 向量。
不与 sendLoop 竞争 CPU,没有时序耦合。

### 2. 独立发送线程 (sendLoop)

完全移植 ROS1 `publishWaypointToRobot` 的流量控制算法:
- RIB 诊断降频查询 (120/250ms, RIB≤0 立即查)
- 自适应批量: ceil((400-rib)/6), min 2, max 8
- EMA 发送耗时补偿 (0.9*old + 0.1*new)
- 自适应睡眠 (忙/RIB低→1ms, 闲→4ms)
- RIB≥300 门控防溢出, 每 4ms 重查

### 3. RIB 在同一条连接上读写

ROS1 用 `robot_mac_size_service_` 既发轨迹点又查 RIB。
错误做法: conn_control_ 发,conn_status_ 查 → 不同连接,RIB 值不更新。
正确做法: conn_control_ 既发又查 → 同一条连接,RIB 实时。

### 4. 双线程架构

插值线程(update timer 200Hz) 和发送线程(sendLoop) 完全解耦。
插值线程不做任何 SDK 调用,保证 200Hz 节拍。

## 与 ROS1 官方驱动对齐情况

| 功能 | ROS1 aubo_robot | 新框架 |
|------|:---:|:---:|
| 轨迹插值 | Python simulator | C++ handleAccepted |
| 发送线程 | publishWaypointToRobot | sendLoop |
| RIB 查询 | robot_mac_size_service_ (独立连接) | conn_control_ (TCP2CAN) |
| RIB 查询频率 | 有队列120ms, 空闲250ms | 同 |
| 批量计算 | ceil((400-rib)/6), min2 max8 | 同 |
| EMA 补偿 | 0.9*old + 0.1*new, >10ms 加大批量 | 同 |
| 睡眠策略 | 忙→1ms, RIB低→1ms, 闲→4ms | 同 |
| 门控阈值 | 无明确(基于实时RIB值) | RIB≥300 暂停 |
| 状态反馈 | RobotStatus 50Hz | StateBroadcaster 回调 33Hz |
| SDK 服务 | 部分(IO/启动) | Dashboard 20服务全覆盖 |

## 启动脚本

两套并存,互不冲突:

```bash
# 旧框架
./start_IVG_graspnet_points_fastapi_web_legacy.sh

# 新框架 (纯新架构)
./start_aubo_new_driver.sh

# 新框架 (仅启动机械臂部分, 用于测试)
ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=169.254.10.98
```

## 关键参数

| 参数 | 值 | 说明 |
|------|------|------|
| 插值频率 | 200 Hz | update_period_ = 0.005s |
| 批量最小 | 2 点 | ROS1 kBaseCntPerSend |
| 批量最大 | 8 点 | ROS1 kMaxAdaptiveCntPerSend |
| RIB 目标 | 400 槽 | ROS1 expect_macsz |
| RIB 门控 | 300 槽 | 保守,留 100 槽余量 |
| EMA 权重 | 0.9/0.1 | 平滑发送耗时 |
| 目标容差 | 0.02 rad | ROS2 trajectory_action 一致 |
| 目标保持 | 5 帧 | 连续容差满足次数 |
