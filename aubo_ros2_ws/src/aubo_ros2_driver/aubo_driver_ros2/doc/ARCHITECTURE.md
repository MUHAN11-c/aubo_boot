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

## 参考

- `doc/SDK_CONFLICT_RULES.md` — SDK 模式冲突规则详解
- `doc/API_MAPPING.md` — 120+ SDK 函数到 ROS2 接口的完整映射
- `doc/PORTING_MOTION_FIX.md` — ROS1→ROS2 移植完整记录 (13 轮调试)
- UR ROS2 Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Franka ROS2: https://github.com/frankaemika/franka_ros2
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
