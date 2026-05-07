# AUBO SDK 完整参考文档

> 合并自: SDK_CONFLICT_RULES.md + ARCHITECTURE.md + API_MAPPING.md

---

# 一、核心 API 速查

以下是最重要、使用频率最高的 15 个 SDK 函数，按调用频率排列。

## 1. robotServiceSetRobotPosData2Canbus — 批量发送轨迹点

> **作用**: 在 TCP2CAN 模式下，将一组预计算的关节位置路点批量发送到机器人 CAN 总线。机器人控制器以约 200Hz 从 RIB 缓冲区取点执行。**这是轨迹流控制的核心函数，每条运动指令最终都通过它下发到电机。**

| 维度 | 说明 |
|------|------|
| 调用频率 | ~250Hz (每 4ms 一次) |
| 典型延迟 | 5-15ms, 尖峰**225ms** |
| RIB 消费 | 每个路点占 6 个槽位, 容量约 400 |
| 返回值 | `InterfaceCallSuccCode=0` 表示成功 |
| 冲突 | 必须在 TCP2CAN 模式下使用 |

**ROS2 封装**: `HardwareInterface::writeTrajectoryPoints(vector<wayPoint_S>)`

## 2. robotServiceGetCurrentWaypointInfo — 读取当前关节位置

> **作用**: 从机器人控制器读取当前法兰盘中心的完整位姿信息，包括 6 个关节角度（弧度）和笛卡尔空间位置（米）+ 姿态（四元数）。**所有关节状态反馈的源头——joint_states 话题的每一帧都来自这个函数。**

| 维度 | 说明 |
|------|------|
| 调用频率 | ≤200Hz (每 5ms 一次) |
| 典型延迟 | 2-5ms |
| 返回数据 | `wayPoint_S.jointpos[6]` + `cartPos` + `orientation` |
| 冲突 | 始终可用，不受 TCP2CAN 模式影响 |

**ROS2 封装**: `HardwareInterface::readJointState(joints, velocity)`

## 3. robotServiceGetRobotDiagnosisInfo — 读取 RIB 缓冲量

> **作用**: 读取机器人控制器的诊断信息，其中最关键的是 `macTargetPosDataSize`——RIB（Robot Interface Buffer）的当前填充量。**这个值决定了流量控制策略：RIB=0 意味着控制器无数据可发、机器人即将停止；RIB=400 意味着缓冲区已满、需减缓发送。PORTING 文档的 13 轮调试核心就是围绕这个函数的调用策略。**

| 维度 | 说明 |
|------|------|
| 调用频率 | 按需 (~每 120ms), RIB=0 时立即查询 |
| 典型延迟 | 2-12ms, 尖峰**225ms** |
| 返回数据 | `RobotDiagnosis.macTargetPosDataSize` (RIB 填充量), `armPowerStatus` (上电状态) |
| 冲突 | 必须在发送轨迹点的同一连接上查询（避免缓存过时） |

**ROS2 封装**: `HardwareInterface::readDiagnosis(rib_buffer_size)`

**流量控制逻辑**:
```
RIB < 200  → 加速发送 (每周期 3 点)
RIB ≥ 200  → 正常发送 (每周期 1 点)
RIB = 0    → 1ms 等待 (快速恢复)
RIB ≥ 400  → 暂停发送 (防止溢出)
```

## 4. robotServiceEnterTcp2CanbusMode / robotServiceLeaveTcp2CanbusMode — 模式切换

> **作用**: 进入/退出 TCP2CAN 透传模式。**进入前控制器自主规划轨迹；进入后控制器停止规划，等待外部逐点下发关节位置。退出后恢复控制器自主规划。这是 SDK 运动 API 和轨迹流模式之间的切换开关。**

| 维度 | 说明 |
|------|------|
| 调用频率 | 启动/关闭时各一次, 模式切换时 |
| 典型延迟 | 100-500ms |
| 进入后效果 | 控制器停止自主轨迹规划, RIB 开始接收外部数据 |
| 退出后效果 | 控制器恢复自主规划, SDK 运动 API 重新可用 |
| 冲突 | **TCP2CAN ON → SDK 运动 API 不可用** |

**ROS2 封装**: `HardwareInterface::enterTcp2CanbusMode()` / `leaveTcp2CanbusMode()`

## 5. robotServiceLogin / robotServiceLogout — 连接/断开

> **作用**: Login 建立到机器人控制器的 TCP 连接并认证，**是所有其他 API 调用的前提**。Logout 断开连接。每个 ServiceInterface 实例需要独立 login，多个实例对应多个 TCP 连接。

| 维度 | 说明 |
|------|------|
| 调用频率 | 启动时 login ×N, 关闭时 logout ×N |
| 典型延迟 | 1-3s (TCP 握手 + 认证) |
| 参数 | host=IP地址, port=8899, user="aubo", password="123456" |
| 重试 | 默认 5 次 |

**ROS2 封装**: `HardwareInterface::init()` / `shutdown()`

## 6. robotServiceRegisterRobotEventInfoCallback — 注册事件回调

> **作用**: 注册 SDK 事件通知的回调函数。**事件回调默认启用且无法关闭**——即使不注册，SDK 仍会推送事件。关键事件包括：socket 断开、急停、碰撞、防护停止、供电变化、MAC 数据中断预警等。

| 维度 | 说明 |
|------|------|
| 调用频率 | 启动时注册一次 |
| 回调线程 | SDK 内部线程 (**不能调用 SDK API**) |
| 关键事件 | `socketDisconnected`, `remoteEmergencyStop`, `softEmergency`, `collision`, `ArmPowerOff`, `MacDataInterruptWarning` |

**ROS2 封装**: `HardwareInterface::registerCallbacks(event_cb, ...)`

## 7. rootServiceRobotStartup — 机器人启动

> **作用**: 一次性完成上电 + 松刹车 + 设置碰撞等级 + 设置动力学参数。**这是机器人从断电到可运动状态的完整初始化流程，阻塞 2-5 秒。**

| 维度 | 说明 |
|------|------|
| 调用频率 | 启动时一次 |
| 典型延迟 | 2-5s (IsBolck=true) |
| 执行内容 | 上电 → 松刹车 → 设置碰撞等级 → 设置动力学参数 |
| 冲突 | **不能在运动中调用** |

**ROS2 封装**: `/aubo/startup` 服务

## 8. robotServiceJointMove — 关节运动

> **作用**: 控制机器人以关节空间点到点方式运动到目标位置。**SDK 内部规划轨迹，各关节独立运动，路径不保证直线。可以用 IsBolck=true 阻塞等待完成，或用 IsBolck=false 立即返回。TCP2CAN 模式下不可用。**

| 维度 | 说明 |
|------|------|
| 调用频率 | 按需 (用户/程序指令) |
| IsBolck=true | 阻塞到运动完成 (数秒到数十秒) |
| IsBolck=false | 立即返回，适合 ROS2 服务回调 |

**ROS2 封装**: `/aubo/move_joint` 服务

## 9. rootServiceRobotMoveControl — 运动控制 (停止/暂停/继续)

> **作用**: 控制机器人的运动状态。**RobotMoveStop 正常减速停止，RobotMovePause 暂停，RobotMoveContinue 继续。是最常用的安全控制接口。**

| 维度 | 说明 |
|------|------|
| 命令 | `RobotMoveStop=0`, `RobotMovePause`, `RobotMoveContinue` |
| 调用频率 | 按需 (急停、异常处理) |

**ROS2 封装**: `/aubo/stop`, `/aubo/pause`, `/aubo/resume` 服务

## 10. robotServiceRobotFk — 正运动学

> **作用**: 根据 6 个关节角度计算法兰盘中心的笛卡尔位姿（位置+姿态四元数）。**纯数学计算，不涉及机器人运动，始终可用。**

| 维度 | 说明 |
|------|------|
| 输入 | 6 个关节角 (rad) |
| 输出 | 位置 (x,y,z, 米) + 姿态四元数 (w,x,y,z) |
| 调用频率 | 按需 |
| 冲突 | 无，始终可用 |

**ROS2 封装**: `/aubo/get_fk` 服务

## 11. robotServiceRobotIk — 逆运动学

> **作用**: 根据笛卡尔位姿（位置+姿态）计算对应的 6 个关节角度。**需要提供参考关节角用于从多解中选择。纯数学计算，始终可用。**

| 维度 | 说明 |
|------|------|
| 输入 | 参考关节角 + 目标位置 + 目标姿态 |
| 输出 | 6 个关节角 (rad) |
| 调用频率 | 按需 |
| 冲突 | 无，始终可用 |

**ROS2 封装**: `/aubo/get_ik` 服务

## 12. robotServiceGetBoardIOStatus — 读取 IO 状态

> **作用**: 读取控制器接口板的 IO 状态——包括用户数字输入/输出、控制器数字输入/输出、用户模拟输入/输出。**用于紧急停止检测（digitalIn[0]/[8]）、防护停止检测（digitalIn[1]/[9]）、以及通用 IO 读写。**

| 维度 | 说明 |
|------|------|
| 调用频率 | ≤50Hz |
| 典型延迟 | 10-30ms (查询多种 IO 类型) |
| IO 类型 | UserDI/DO, ControllerDI/DO, UserAI/AO |

**ROS2 封装**: `HardwareInterface::readSafetyIOStatus()` / `readFullIOStatus()`

## 13. robotServiceSetBoardIOStatus — 写入 IO

> **作用**: 设置控制器接口板的 IO 输出状态——数字输出的高低电平、模拟输出的电压值。

**ROS2 封装**: `HardwareInterface::writeIOCommand()` / `/aubo/set_io` 服务

## 14. robotServiceCollisionRecover — 碰撞恢复

> **作用**: 碰撞发生后，**必须先调用此函数清除碰撞状态，才能继续发送运动指令**。否则所有运动 API 都会拒绝执行。

| 维度 | 说明 |
|------|------|
| 调用频率 | 碰撞后一次 |
| 冲突 | **碰撞后必须先恢复才能运动** |

**ROS2 封装**: `/aubo/collision_recover` 服务

## 15. robotServiceSetRobotCollisionClass — 设置碰撞等级

> **作用**: 设置碰撞检测的灵敏度。等级 1-10，1 最敏感（轻微碰撞即停止），10 最不敏感。默认 6。运动中修改下次运动生效。

**ROS2 封装**: `/aubo/set_collision_class` 服务

---

# 二、SDK 模式冲突规则

## 规则 1: TCP2CAN 模式与 SDK 运动 API 互斥

```
TCP2CAN 模式 (轨迹流)           SDK 运动 API (robotServiceJointMove 等)
═══════════════════════         ════════════════════════════════════════
控制器停止自身轨迹规划            控制器使用自身轨迹规划
等待外部逐点下发关节位置          接收目标位置后自动生成轨迹
robotServiceSetRobotPosData2Canbus  robotServiceJointMove
robotServiceEnterTcp2CanbusMode    robotServiceLineMove
robotServiceLeaveTcp2CanbusMode    robotServiceTrackMove
                                   robotServiceRotateMove
                                   robotServiceTeachStart/Stop
```

**处理**:
- Dashboard 节点维护**独立的 ServiceInterface 连接**（不进入 TCP2CAN 模式）
- 轨迹控制器（JointTrajectoryController）使用 conn_control_ 的 TCP2CAN 连接
- 两者互不干扰

## 规则 2: IsBolck 参数行为

| IsBolck=true | IsBolck=false |
|-------------|---------------|
| SDK 调用阻塞直到运动完成 | SDK 调用立即返回（命令已发送） |
| 可能阻塞数秒到数十秒 | 不阻塞 |
| 适合简单脚本/测试 | 生产环境 / ROS2 服务回调推荐 |

## 规则 3: 运动中的安全约束

| 操作 | 运动中可以执行？ | 备注 |
|------|:---:|------|
| `rootServiceRobotStartup` | ❌ | 需先停止运动 |
| `robotServiceRobotShutdown` | ❌ | 需先停止运动 |
| `robotServiceReleaseBrake` | ❌ | 需先停止运动后松刹车 |
| `robotServiceCollisionRecover` | ❌ | 碰撞后必须先恢复才能继续 |
| `robotServiceSetToolDynamicsParam` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceSetToolKinematicsParam` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceSetRobotCollisionClass` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceGetDiagnosisInfo` | ✅ | 始终可用 |
| `robotServiceGetCurrentWaypointInfo` | ✅ | 始终可用 |
| `robotServiceRobotFk` | ✅ | 纯计算，始终可用 |
| `robotServiceRobotIk` | ✅ | 纯计算，始终可用 |

## 规则 4: 碰撞恢复顺序
```
碰撞发生
  → robotServiceCollisionRecover()   // 必须先恢复
  → 检查碰撞状态已清除
  → 重新发送运动指令
```

## 规则 5: 同一 ServiceInterface 的线程安全
- 同一实例内: 所有 SDK API 调用必须串行，不能并发
- 不同实例间: 可以并行使用（独立 TCP 连接）

## 规则 6: SDK 回调中的约束
- 回调在 SDK 内部线程执行 (**禁止调用任何 SDK API**)
- 允许: `std::atomic` 读写、无锁队列写入、`RCLCPP_*` 日志

## 启动/关闭/异常流程

**启动**: `login ×2` → `registerCallback` → `getIsRealRobotExist` → `[startup]` → `enterTcp2Canbus` → 控制循环

**关闭**: `stop motion` → `leaveTcp2Canbus` → `[shutdown]` → `logout ×2`

**异常**: `socketDisconnected 事件` → `connected_ = false` → 重连 `login` → 恢复状态

## 通信延迟参考

| SDK 调用 | 典型延迟 | 尖峰延迟 |
|---------|---------|---------|
| `robotServiceGetCurrentWaypointInfo` | 2-5 ms | ~12 ms |
| `robotServiceGetRobotDiagnosisInfo` | 2-12 ms | **225 ms** |
| `robotServiceSetRobotPosData2Canbus` (batch) | 5-15 ms | **225 ms** |
| `robotServiceSetRobotPosData2Canbus` (single) | 2-5 ms | ~20 ms |
| `robotServiceSetBoardIOStatus` | 5-10 ms | ~30 ms |
| `robotServiceGetBoardIOStatus` (multi) | 10-30 ms | ~50 ms |
| `robotServiceJointMove` (IsBolck=false) | 5-10 ms | ~20 ms |
| `rootServiceRobotStartup` (IsBolck=true) | 2-5 s | ~10 s |

## RIB 流量控制实践总结

> 经过多轮真实机器人测试验证的 RIB 使用结论:

### RIB 必须在发送轨迹点的同一条连接上查询

ROS1 使用 `robot_mac_size_service_` 既发送轨迹点又查 RIB,两者在同一条 TCP 连接上。
如果发送用 conn_control_ (TCP2CAN),查询用 conn_status_ (普通),RIB 值永远不会更新,
因为不同 TCP 连接的 RIB 数据不同步。

### RIB≥300 必须暂停发送

RIB 缓冲区只有约 400 槽(~66 个路点)。持续以高于机器人消费速率发送会导致
RIB 溢出,控制器可能丢弃路点,造成运动跳变或机器人停机。

### 预计算 + 独立发送线程 是正确方案

200Hz 插值在 Goal 接受时一次性完成,存入预计算向量。
独立 std::thread 从向量中按序取点,自适应批量发送。
插值和发送完全解耦,不存在时序竞争。

### SDK 阻塞无法绕过

TCP 阻塞是 AUBO SDK 的根本特性。正确做法是把阻塞调用隔离在独立线程中,
插值路径不做任何 SDK 调用。这与 UR/Franka 用 UDP/RTDE 实时协议不同,
但与国产协作机器人(JAKA/Dobot/Elite)的做法一致。

---

# 三、连接架构

```
轨迹控制器 (JointTrajectoryController)
  └── HardwareInterface
        ├── conn_control_ [TCP2CAN 模式]
        │     └── robotServiceSetRobotPosData2Canbus (轨迹流)
        │     └── robotServiceGetRobotDiagnosisInfo (RIB 查询)
        └── conn_status_ [普通模式]
              └── robotServiceGetCurrentWaypointInfo (关节状态)
              └── robotServiceGetBoardIOStatus (IO 状态)
              └── RobotEventCallback (事件回调)

状态广播器 (AuboStateBroadcaster)
  └── HardwareInterface (独立实例)
        ├── conn_control_ [普通模式]
        └── conn_status_ [普通模式]
              └── robotServiceGetCurrentWaypointInfo
              └── robotServiceGetRobotDiagnosisInfo
              └── robotServiceGetBoardIOStatus

Dashboard 节点 (AuboDashboardNode)
  └── HardwareInterface (独立实例)
        ├── conn_control_ [普通模式] ← 供运动 API 使用
        └── conn_status_ [普通模式]
              └── rootServiceRobotStartup / Shutdown
              └── robotServiceJointMove / LineMove
              └── robotServiceSetRobotCollisionClass
              └── robotServiceRobotFk / RobotIk
              └── robotServiceGetRobotDevInfoService
              └── RobotEventCallback
```

**关键**: 每个节点有独立的 HardwareInterface → 独立的 ServiceInterface 连接 → 互不干扰。

---

# 四、整体架构

## 设计哲学

**借鉴 ros2_control 架构，但不依赖 ros2_control 框架**。AUBO SDK 是同步阻塞的 TCP 通信库（单次调用 2-225ms），无法满足 ros2_control 的实时约束。采用 **"独立驱动 + ros2_control 模式"** 的混合方案。

### 参考来源

| 参考 | 借鉴的设计 |
|------|-----------|
| UR `URPositionHardwareInterface` | async I/O 线程 + GPIO named interfaces |
| UR `DashboardClientROS` | LifecycleNode + 独立进程 + Trigger service |
| Franka `FrankaExecutor` | 后台 MultiThreadedExecutor 隔离非RT工作 |
| ros2_control `joint_trajectory_controller` | 5次插值 + Action 状态机 |
| `PORTING_MOTION_FIX.md` | RIB 实时查询、数据竞争、start_move_ 误关 |

## 数据流

```
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
  │ 系统管理: /aubo/startup, shutdown, brake_release, stop, collision_recover
  │ 运动控制: /aubo/move_joint, move_line, teach_start/stop
  │ 配置:    /aubo/set_payload, set_collision_class, set_tool
  │ 运动学:  /aubo/get_fk, get_ik
  │ 诊断:    /aubo/get_robot_info, get_joint_status, get_safety_config
  │ IO:     /aubo/set_io
```

## 组件

### AuboHardwareInterface（硬件抽象层）
- **conn_control_**: TCP2CAN 轨迹流 + RIB 诊断
- **conn_status_**: 状态查询 + IO + SDK 回调
- 每个节点实例独立连接

### JointTrajectoryController（轨迹控制器）
- 替代 Python simulator + trajectory_action
- 200Hz 5次多项式插值 + FollowJointTrajectory Action
- 5次系数: `a1=v_last, a2=0.5·a_last, a3=0.5/T³·(20h-(8v2+12v1)T-(3a1-a2)T²), a4=..., a5=...`

### AuboDashboardNode（仪表板）
- LifecycleNode, 独立 Executor
- 20 个 ROS2 服务覆盖全部非实时 SDK 功能

### AuboStateBroadcaster（状态广播器）
- 替代旧的 publishIOMsg / publishJointStateAndFeedbackLoop 线程
- 200Hz + 50Hz ROS2 wall timer

## 新旧对比

| 维度 | 旧架构 | 新架构 |
|------|--------|--------|
| 轨迹插值 | Python simulator (GIL) | C++ JointTrajectoryController |
| 状态广播 | std::thread 手动管理 | ROS2 wall timer |
| SDK 功能暴露 | 仅 22/163 函数 | 全部 163 函数有对应接口 |
| SDK 连接数 | 3 (send/receive/mac_size) | 每节点 2 (control/status) |
| 生命周期 | 普通 Node, 构造函数 login | LifecycleNode, 显式状态转换 |
| 消息类型 | Float32MultiArray | sensor_msgs::JointState |

---

# 五、SDK API 完整映射

> 对应源码: `include/aubo_driver_ros2/serviceinterface.h` (1343行)

## 映射原则

| SDK 函数类型 | ROS2 映射方式 | 原因 |
|-------------|-------------|------|
| 实时轨迹流 (TCP2CAN) | `HardwareInterface::writeTrajectoryPoints()` | 200Hz 高频, 阻塞调用需独立线程 |
| 状态查询 (高频 ≥50Hz) | `HardwareInterface` + `StateBroadcaster` 定时发布 | 避免每个回调都调 SDK |
| 状态查询 (低频/按需) | `/aubo/get_*` Dashboard 服务 | 按需调用 |
| 系统管理 | `/aubo/startup` 等 Dashboard 服务 | 阻塞数秒, 独立 Executor |
| 运动控制 (SDK API) | `/aubo/move_*` Dashboard 服务 | 需先退出 TCP2CAN |
| 配置修改 | `/aubo/set_*` + 运行时参数 | 运动停止时生效 |
| IO 读写 | HardwareInterface (读) + Dashboard (写) | 读高频, 写按需 |
| 运动学 | `/aubo/get_fk` 等 Dashboard 服务 | 纯计算, 始终可用 |
| SDK 回调 | `HardwareInterface::registerCallbacks()` | SDK 内部线程 |

## 5.1 系统/连接

### robotServiceLogin
```cpp
int robotServiceLogin(const char* host, int port,
                      const char* userName, const char* password);
```

> **作用**: 建立到机器人控制器的 TCP 连接并认证。是所有其他 API 调用的前提。每个 ServiceInterface 实例需独立 login。

| 维度 | 说明 |
|------|------|
| 使用状态 | ✅ 每个 HardwareInterface 调用 2 次 |
| 阻塞时间 | 1-3s |
| 重试策略 | 默认5次 |
| ROS2 接口 | `HardwareInterface::init()` 内部调用 |

### robotServiceLogout
```cpp
int robotServiceLogout();
```

> **作用**: 断开与机器人控制器的 TCP 连接。调用前应先 `leaveTcp2CanbusMode()`。

### robotServiceGetConnectStatus / robotServiceRobotHandShake
> **作用**: 查询连接状态 / 可选的握手确认。驱动自行维护连接状态，一般不需要显式调用。

## 5.2 运动控制

### robotServiceJointMove — 关节运动
```cpp
int robotServiceJointMove(double jointAngle[ARM_DOF], bool IsBolck);
```

> **作用**: 控制机器人以关节空间方式运动到目标关节角。SDK 内部规划轨迹——各关节独立、路径不保证直线。IsBolck=true 会阻塞到运动完成（数秒到数十秒），不适合 ROS2 回调；IsBolck=false 立即返回。

| 维度 | 说明 |
|------|------|
| 使用状态 | ✅ `/aubo/move_joint` 服务 |
| 输入 | 参数 `move_joint_target`: 逗号分隔6关节角(rad) |
| 调用示例 | `ros2 service call /aubo/move_joint std_srvs/srv/Trigger {}` |

### robotServiceLineMove — 直线运动
```cpp
int robotServiceLineMove(double jointAngle[ARM_DOF], bool IsBolck);
```

> **作用**: 控制机器人 TCP 沿笛卡尔空间直线运动到目标位姿。SDK 内部做逆解和轨迹规划。直线运动在奇异点附近可能失败。

| 维度 | 说明 |
|------|------|
| ROS2 接口 | `/aubo/move_line` |

### robotServiceRotateMove — 旋转运动
```cpp
int robotServiceRotateMove(const CoordCalibrateByJointAngleAndTool &userCoord,
                           const double rotateAxisOnUserCoord[3],
                           double rotateAngle, bool IsBolck);
```

> **作用**: 控制机器人绕指定轴做纯旋转运动，保持 TCP 位置不变。

### robotServiceTrackMove — 轨迹运动
```cpp
int robotServiceTrackMove(aubo_robot_namespace::move_track subMoveMode, bool IsBolck);
```

> **作用**: 执行预设路点列表的轨迹运动。支持圆弧(CIRC)、MOVEP、样条等多种子模式。调用前需先通过 `addGlobalWayPoint` 构建路点列表。

子模式: `ARC_CIR`(圆弧/圆), `CARTESIAN_MOVEP`(带交融的连续运动), `CARTESIAN_CUBICSPLINE`, `JOINT_UBSPLINEINTP`

### robotServiceTeachStart / robotServiceTeachStop — 示教
> **作用**: 启动/停止示教模式。示教模式下机器人持续运动直到调用 teachStop。

模式: `JOINT1`~`JOINT6`(单关节), `MOV_X/Y/Z`(直线位置), `ROT_X/Y/Z`(旋转姿态)

### rootServiceRobotMoveControl — 运动控制
```cpp
int rootServiceRobotMoveControl(RobotMoveControlCommand cmd);
// RobotMoveStop=0, RobotMovePause, RobotMoveContinue
```

> **作用**: 控制机器人运动状态——正常减速停止、暂停、继续。

### 其他运动函数 (较少使用)

| SDK Function | 作用 |
|-------------|------|
| `robotServiceFollowModeJointMove` | 跟随模式关节运动，运动过程中可连续更新目标 |
| `robotMoveLineToTargetPosition` | 保持当前姿态，直线运动到目标位置 |
| `robotMoveJointToTargetPosition` | 保持当前姿态，关节运动到目标位置 |
| `robotMoveLineToTargetPositionByRelative` | 基于当前位姿的相对偏移直线运动 |
| `robotMoveJointToTargetPositionByRelative` | 基于当前位姿的相对偏移关节运动 |
| `getJointAngleByTargetPositionKeepCurrentOri` | 保持当前姿态的逆解（纯计算，不运动） |

## 5.3 运动属性

> **作用**: 配置 SDK 内置运动规划器的参数。所有 Set 在下次运动时生效，Get 始终可用。默认值: 关节速度 25°/s, 末端速度 3m/s。

### 核心属性

| SDK Function | 作用 | ROS2 参数 |
|-------------|------|---------|
| `robotServiceInitGlobalMoveProfile` | 恢复所有运动属性为默认值 | `/aubo/init_move_profile` |
| `robotServiceSetGlobalMoveJointMaxVelc` | 设置关节运动最大速度 (°/s) | `joint_max_vel` |
| `robotServiceSetGlobalMoveJointMaxAcc` | 设置关节运动最大加速度 (°/s²) | `joint_max_acc` |
| `robotServiceSetGlobalMoveEndMaxLineVelc` | 设置末端直线最大速度 (m/s) | `end_max_line_vel` |
| `robotServiceSetGlobalMoveEndMaxLineAcc` | 设置末端直线最大加速度 (m/s²) | `end_max_line_acc` |
| `robotServiceSetJerkAccRatio` | 设置加加速度比例 (平滑度) | `jerk_ratio` |

### 路点管理 (用于 TrackMove)

| SDK Function | 作用 |
|-------------|------|
| `robotServiceAddGlobalWayPoint` | 添加路点到运动列表 (支持笛卡尔位姿或关节角) |
| `robotServiceClearGlobalWayPointVector` | 清空路点列表 |

### 交融半径

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetGlobalBlendRadius` | 设置交融半径 (0.0~0.05m)，MOVEP 模式路点间平滑过渡 |

### 提前到位

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetArrivalAheadTimeMode` | 提前到位——时间模式，到达目标前提前过渡到下一个目标 |
| `robotServiceSetArrivalAheadDistanceMode` | 提前到位——距离模式 |
| `robotServiceSetNoArrivalAhead` | 关闭提前到位功能 |

### 圆弧参数 / 偏移量 / 示教坐标系

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetGlobalCircularLoopTimes` | 设置圆轨迹圈数 (0=圆弧, >0=完整的圆) |
| `robotServiceSetMoveRelativeParam` | 设置运动偏移量 (基于基座标系或用户坐标系) |
| `robotServiceSetTeachCoordinateSystem` | 设置示教参考坐标系 |

## 5.4 TCP2CAN 透传模式

> **作用**: TCP2CAN 模式下控制器停止自身轨迹规划，等待外部通过 CAN 总线逐点下发关节位置。这是轨迹流控制的基础。

### 进入/退出
- `robotServiceEnterTcp2CanbusMode()` — 启动后控制器停止自主规划，RIB 开始接收外部数据
- `robotServiceLeaveTcp2CanbusMode()` — 退出后控制器恢复自主规划，SDK 运动 API 重新可用

### 发送轨迹点
- `robotServiceSetRobotPosData2Canbus(double[6])` — 发送单个路点，用于 OTG 紧急减速
- `robotServiceSetRobotPosData2Canbus(vector<wayPoint_S>)` — 批量发送路点，主轨迹流路径

**流量控制**: 基于 RIB 填充量自适应调整发送批量——RIB<200 加速，RIB≥400 暂停。

## 5.5 工具接口

> **作用**: 配置法兰盘末端安装的工具（夹爪、焊枪等）的物理属性——质量、重心、TCP 位置姿态。正确设置影响碰撞检测灵敏度和运动精度。

### 动力学参数 (Payload)

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetToolDynamicsParam` | 设置工具质量(kg)和重心位置(m)，影响动力学模型 |
| `robotServiceSetNoneToolDynamicsParam` | 清空为无工具状态 |
| `robotServiceGetToolDynamicsParam` | 查询当前设置 |

### 运动学参数 (TCP)

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetToolKinematicsParam` | 设置 TCP 相对法兰盘的位置和姿态 |
| `robotServiceSetNoneToolKinematicsParam` | 清空 (TCP 回到法兰盘中心) |
| `robotServiceGetToolKinematicsParam` | 查询当前 TCP 设置 |
| `robotServiceSetRobotTool` | 设置工具 (别名，等同于 SetToolKinematicsParam) |

### 工具标定

| SDK Function | 作用 |
|-------------|------|
| `robotServiceToolCalibration` | 通过多个位姿自动计算 TCP 位置 (最小二乘法) |

## 5.6 IO 接口

> **作用**: 读写控制器接口板 IO 和工具端 IO——数字输入/输出、模拟输入/输出、工具电源。

### 读取 IO

| SDK Function | 作用 | 调用频率 |
|-------------|------|---------|
| `robotServiceGetBoardIOStatus` | 读接口板 IO 状态 (指定类型列表) | ≤50Hz, ~10-30ms |
| `robotServiceGetBoardIOConfig` | 读 IO 配置 (名称、地址等) | 按需 |
| `robotServiceGetAllToolDigitalIOStatus` | 读工具端数字 IO 状态 | ≤50Hz |
| `robotServiceGetAllToolAIStatus` | 读工具端模拟输入状态 | ≤50Hz |

IO 类型: `RobotBoardUserDI/DO`(用户数字), `RobotBoardControllerDI/DO`(控制器数字), `RobotBoardUserAI/AO`(用户模拟)

### 写入 IO

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetBoardIOStatus` | 设置接口板 IO 输出 (按地址或名称) |
| `robotServiceSetToolDigitalIOType` | 设置工具端 IO 方向 (输入/输出) |
| `robotServiceSetToolDOStatus` | 设置工具端数字输出值 |
| `robotServiceSetToolPowerVoltageType` | 设置工具电源电压 (0V/12V/24V) |
| `robotServiceSetToolPowerTypeAndDigitalIOType` | 批量配置工具电源和 IO |

## 5.7 运动学

> **作用**: 正解（关节角→笛卡尔位姿）和逆解（笛卡尔位姿→关节角）。纯数学计算，不涉及机器人运动，始终可用。

### 正解/逆解

| SDK Function | 作用 |
|-------------|------|
| `robotServiceRobotFk` | 正解: 6关节角 → 笛卡尔位姿 (位置+四元数) |
| `robotServiceRobotIk` (单解) | 逆解: 目标位姿+参考关节角 → 最接近的逆解 |
| `robotServiceRobotIk` (多解) | 逆解: 返回所有可能的解 |

### 坐标变换

| SDK Function | 作用 |
|-------------|------|
| `baseToUserCoordinate` | 基座标系 → 用户坐标系 (带工具偏移) |
| `userToBaseCoordinate` | 用户坐标系 → 基座标系 (去工具) |
| `baseToBaseAdditionalTool` | 基座标系加工具偏移 |
| `userCoordPointToBasePoint` | 坐标系内点的变换 |
| `endOrientation2ToolOrientation` | 法兰盘姿态 → 工具姿态 |
| `toolOrientation2EndOrientation` | 工具姿态 → 法兰盘姿态 |
| `getTargetWaypointByPosition` | 综合: 计算目标位姿对应的关节角 |

### 四元数/欧拉角

| SDK Function | 作用 |
|-------------|------|
| `quaternionToRPY` | 四元数 → 欧拉角 (Roll/Pitch/Yaw) |
| `RPYToQuaternion` | 欧拉角 → 四元数 |
| `robotServiceOriMatrixToQuaternion` | 旋转矩阵 → 四元数 |

## 5.8 状态/诊断

> **作用**: 查询机器人的各种状态信息——关节角、诊断、设备信息、工作模式等。

| SDK Function | 作用 | ROS2 接口 |
|-------------|------|---------|
| `robotServiceGetCurrentWaypointInfo` | 读当前法兰盘完整位姿 (关节角+笛卡尔) | `readJointState()` |
| `robotServiceGetRobotJointStatus` | 读关节角+速度+电流+温度+错误码 | `/aubo/get_joint_status` |
| `robotServiceGetJointAngleInfo` | 读当前关节角 (简化版) | 状态话题 |
| `robotServiceGetRobotDiagnosisInfo` | 读诊断信息 (RIB缓冲量+上电状态) | `readDiagnosis()` |
| `robotServiceGetIsRealRobotExist` | 判断是否连接了真实机器人 | `init()` 内部 |
| `robotServiceGetRobotDevInfoService` | 读设备信息 (序列号、固件版本) | `/aubo/get_robot_info` |
| `robotServiceGetRobotWorkMode` | 读工作模式 (仿真/真实) | `/aubo/get_work_mode` |
| `robotServiceSetRobotWorkMode` | 设置工作模式 | `/aubo/set_work_mode` |
| `robotServiceGetRobotCurrentState` | 读当前机器人状态 | 状态话题 |
| `robotServiceGetMacCommunicationStatus` | 读 MAC 通信状态 | 状态话题 |
| `robotServiceIsOnlineMode` | 读联机模式状态 | 状态话题 |
| `robotServiceGetRobotGravityComponent` | 读重力分量 | 未使用 |
| `robotServiceGetJoint6Rotate360EnableFlag` | J6 关节 360° 旋转使能标志 | 未使用 |

## 5.9 控制 (上电/关机/刹车/碰撞)

| SDK Function | 作用 | ROS2 接口 |
|-------------|------|---------|
| `rootServiceRobotStartup` | **一键启动**: 上电 + 松刹车 + 碰撞等级 + 动力学参数 | `/aubo/startup` |
| `robotServiceRobotShutdown` | 关机 (先停止运动) | `/aubo/shutdown` |
| `robotServicePowerControl` | 单独控制电源 (true=上电, false=断电) | `/aubo/power_control` |
| `robotServiceReleaseBrake` | 单独松刹车 | `/aubo/brake_release` |
| `robotServiceCollisionRecover` | 碰撞恢复 (必须先调用才能继续运动) | `/aubo/collision_recover` |
| `robotServiceSetRobotCollisionClass` | 设置碰撞检测灵敏度 (1-10) | `/aubo/set_collision_class` |
| `robotServiceGetRobotCollisionCurrentService` | 查询当前碰撞等级 | `/aubo/get_collision_class` |
| `robotServiceSetRobotMaxACC` | 设置全局最大加速度 | `/aubo/set_max_acc` |
| `rootServiceRobotControl` | 通用控制命令 | `/aubo/control` |
| `robotServiceSetRobotJointOffset` | 关节零位补偿 (0.00~0.51°) | 未使用 |

## 5.10 SDK 回调

> **作用**: SDK 通过内部线程推送数据，用户注册回调接收。**回调在 SDK 内部线程执行，禁止调用 SDK API。**

| SDK Function | 作用 |
|-------------|------|
| `robotServiceRegisterRobotEventInfoCallback` | 注册事件回调 (socket断开/急停/碰撞等 49 种事件) |
| `robotServiceRegisterRealTimeJointStatusCallback` | 注册关节状态实时推送 (速度+电流+温度) |
| `robotServiceRegisterRealTimeRoadPointCallback` | 注册路点实时推送 |
| `robotServiceRegisterRealTimeEndSpeedCallback` | 注册末端速度实时推送 |
| `robotServiceSetRealTimeJointStatusPush` | 启用/禁用关节状态推送 |
| `robotServiceSetRealTimeRoadPointPush` | 启用/禁用车路点推送 |
| `robotServiceSetRealTimeEndSpeedPush` | 启用/禁用末端速度推送 |

**49 个事件类型**:
```
armCanbusError, remoteEmergencyStop, jointError,
forceControl, exitForceControl,
softEmergency, exitSoftEmergency,
collision, collisionStatusChanged,
powerChanged, ArmPowerOff,
socketDisconnected,
atTrackTargetPos,
MacDataInterruptWarning,
... (etc, 49 total)
```

## 5.11 传送带跟踪 (15 函数)

> **作用**: 与传送带同步的视觉抓取场景——机器人跟踪传送带上移动的物体并进行抓取。需要真实传送带硬件。全部预留 Dashboard `/aubo/conveyor_*` 服务。

## 5.12 离线轨迹 (5 函数)

> **作用**: 从文件加载预录轨迹并回放。功能可通过 `joint_trajectory_controller` 的 FollowJointTrajectory 标准 Action 替代。

## 5.13 安全 IO/ORPE (10 函数)

> **作用**: 与外部安全系统（急停按钮、安全门、光栅）集成。部分功能已通过 RobotEventCallback 覆盖。

| SDK Function | 作用 |
|-------------|------|
| `robotServiceClearSystemEmergencyStop` | 解除系统急停输出信号 |
| `robotServiceClearReducedModeError` | 解除缩减模式错误 |
| `robotServiceRobotSafetyguardResetSucc` | 防护重置成功确认 |
| `robotServiceGetRobotSafetyConfig` | 读安全配置 |
| `robotServiceSetRobotSafetyConfig` | 设置安全配置 |
| `robotServiceGetOrpeSafetyStatus` | 读 ORPE 安全状态 |

## 5.14 固件升级 (3 函数)

> **作用**: 升级机器人控制器固件。建议通过独立工具完成，不集成到 ROS2 节点。

## 5.15 其他

| SDK Function | 作用 |
|-------------|------|
| `robotServiceSetWeaveMoveParameters` | 焊接摆动参数 (振幅/频率/波形) |
| `robotServiceSetRobotRecognitionParam` | 视觉识别参数 |
| `robotServiceSetRobotCameraCalib` | 手眼标定结果设置 |
| `robotServiceSetRobotJointOffset` | 关节碰撞零位补偿 |

---

## 实现优先级

### P0 — 已完整实现 (28/163)

#### Dashboard 服务 (20) — 通过 `/aubo/*` 调用

| SDK 函数 | 作用 | 服务名 | srv 类型 | 关键请求字段 |
|---------|------|--------|---------|------------|
| `rootServiceRobotStartup` | 一键启动: 上电+松刹车+碰撞等级(阻塞2-5s) | `/aubo/startup` | Trigger | (无) |
| `robotServiceRobotShutdown` | 关机: 先停止再断电 | `/aubo/shutdown` | Trigger | (无) |
| `robotServiceReleaseBrake` | 单独松刹车 | `/aubo/brake_release` | Trigger | (无) |
| `rootServiceRobotMoveControl` | 正常减速停止 | `/aubo/stop` | Trigger | (无) |
| `robotMoveFastStop` | 快速停止(不减速) | `/aubo/fast_stop` | Trigger | (无) |
| `robotServiceCollisionRecover` | 碰撞恢复 | `/aubo/collision_recover` | Trigger | (无) |
| `robotServiceTeachStop` | 停止示教 | `/aubo/teach_stop` | Trigger | (无) |
| `robotServiceRobotFk` | 正解: 6关节角→位姿 | `/aubo/get_fk` | GetFK | `float32[6] joint` |
| `robotServiceRobotIk` | 逆解: 位姿→关节角 | `/aubo/get_ik` | GetIK | `float32[6] ref_joint, pos, ori` |
| `robotServiceSetBoardIOStatus` | 设置IO输出 | `/aubo/set_io` | SetRobotIO | `string io_type, int32 idx, float64 val` |
| `robotServiceGetRobotDevInfoService` | 设备信息(序列号/固件) | `/aubo/get_robot_info` | Trigger | (无) |
| `robotServiceGetRobotJointStatus` | 详细关节状态 | `/aubo/get_joint_status` | Trigger | (无) |
| `robotServiceGetRobotSafetyConfig` | 安全配置 | `/aubo/get_safety_config` | Trigger | (无) |
| `robotServiceJointMove` | **关节空间运动(MoveJ)** | `/aubo/move_joint` | MoveJoint | `float64[6] joints, float64 velocity` |
| `robotServiceLineMove` | **笛卡尔直线运动(MoveL)** | `/aubo/move_line` | MoveLine | `float64[6] joints, float64 velocity` |
| `robotServiceTeachStart` | 启动示教(12种模式) | `/aubo/teach_start` | TeachStart | `int32 joint, bool direction` |
| `robotServiceSetRobotCollisionClass` | 碰撞灵敏度(1-10) | `/aubo/set_collision_class` | SetCollisionClass | `int32 grade` |
| `robotServiceSetToolDynamicsParam` | 末端负载(质量+重心) | `/aubo/set_payload` | SetPayload | `float32 mass, Vector3 cog` |
| `robotServiceSetToolKinematicsParam` | TCP(位置+姿态) | `/aubo/set_tool_kinematics` | SetToolKinematics | `float64[3] pos, float64[4] ori` |
| `robotServiceSetToolPowerVoltageType` | 工具电源(0/12/24V) | `/aubo/set_tool_voltage` | SetToolVoltage | `int32 voltage_type` |

#### HardwareInterface 内部 (8) — 不暴露为 ROS2 服务

| SDK 函数 | 作用 | 封装方法 | 调用频率 |
|---------|------|---------|---------|
| `robotServiceSetRobotPosData2Canbus` (batch+single) | 批量/单点发送轨迹点到CAN总线 | `writeTrajectoryPoints/writeTrajectoryPoint()` | ~250Hz |
| `robotServiceGetCurrentWaypointInfo` | 读取6关节角+笛卡尔位姿 | `readJointState()` | ≤200Hz |
| `robotServiceGetRobotDiagnosisInfo` | RIB缓冲量(流量控制核心) | `readDiagnosis()` | 按需~120ms |
| `robotServiceEnter/LeaveTcp2CanbusMode` | 进入/退出透传模式 | `enter/leaveTcp2CanbusMode()` | 启动/停止 |
| `robotServiceLogin` / `robotServiceLogout` | TCP连接认证/断开(×2) | `init()` / `shutdown()` | 启动/关闭 |
| `robotServiceRegisterRobotEventInfoCallback` + 3推送 | 注册SDK事件/状态回调 | `registerCallbacks()` | 启动时1次 |
| `robotServiceGetBoardIOStatus` + 4 IO读取 | 批量读取IO状态 | `readSafetyIOStatus/readFullIOStatus()` | ≤50Hz |

#### 调用示例

```bash
# MoveJ: 关节运动
ros2 service call /aubo/move_joint demo_interface/srv/MoveJoint \
  "{joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0], velocity: 0.8}"

# MoveL: 直线运动  
ros2 service call /aubo/move_line demo_interface/srv/MoveLine \
  "{joints: [0.0, -0.5, 1.2, 0.0, 0.8, 0.0], velocity: 0.5}"

# 设置碰撞等级
ros2 service call /aubo/set_collision_class demo_interface/srv/SetCollisionClass \
  "{grade: 6}"

# 设置TCP (夹爪长0.15m沿Z轴)
ros2 service call /aubo/set_tool_kinematics demo_interface/srv/SetToolKinematics \
  "{position: [0.0, 0.0, 0.15], orientation: [1.0, 0.0, 0.0, 0.0]}"

# 正解/逆解
ros2 service call /aubo/get_fk aubo_msgs/srv/GetFK \
  "{joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 启动: 上电+松刹车
ros2 service call /aubo/startup std_srvs/srv/Trigger {}
```

### 🟡 P2 — 常用但待实现 (~25个)

| SDK 函数 | 作用 | 实现方式 |
|---------|------|---------|
| `robotServiceRotateMove` | 旋转运动 | Dashboard `/aubo/move_rotate` |
| `robotServiceTrackMove` | 轨迹运动 | Dashboard `/aubo/move_track` |
| `robotServiceSetGlobalMoveJointMaxVelc` | 关节最大速度 | 参数 `joint_max_vel` |
| `robotServiceSetGlobalMoveJointMaxAcc` | 关节最大加速度 | 参数 `joint_max_acc` |
| `robotServiceSetGlobalMoveEndMaxLineVelc` | 末端最大速度 | 参数 `end_max_line_vel` |
| `robotServiceSetGlobalMoveEndMaxLineAcc` | 末端最大加速度 | 参数 `end_max_line_acc` |
| `robotServiceSetJerkAccRatio` | 加加速度比例 | 参数 `jerk_ratio` |
| `robotServiceInitGlobalMoveProfile` | 恢复默认 | Dashboard 服务 |
| `robotServiceSetGlobalBlendRadius` | 交融半径 | 参数 `blend_radius` |
| `robotServiceGetGlobalBlendRadius` | 查询交融半径 | Dashboard 服务 |
| `robotServiceGetToolDynamicsParam` | 查询负载 | Dashboard 服务 |
| `robotServiceGetToolKinematicsParam` | 查询 TCP | Dashboard 服务 |
| `robotServiceToolCalibration` | 工具标定 | Dashboard 服务 |
| `robotServiceSetRobotWorkMode` | 设置工作模式 | Dashboard 服务 |
| `robotServiceGetRobotWorkMode` | 查询工作模式 | Dashboard 服务 |
| `robotServiceGetRobotCurrentState` | 查询状态 | 状态话题定时发布 |
| `robotServiceGetJointAngleInfo` | 查询关节角 | 状态话题定时发布 |
| `robotServiceGetMacCommunicationStatus` | MAC 通信状态 | 状态话题 |
| `robotServiceIsOnlineMode` | 联机模式 | 状态话题 |
| `all coordinate transform functions` (10个) | 坐标变换 | Dashboard 服务 |
| `quaternionToRPY` / `RPYToQuaternion` | 四元数/欧拉角 | Dashboard 服务 |
| `robotServiceGetRobotCollisionCurrentService` | 查询碰撞等级 | Dashboard 服务 |
| `robotServicePowerControl` | 电源控制 | Dashboard 服务 |
| `robotServiceSetRobotMaxACC` | 最大加速度 | Dashboard 服务 |
| `rootServiceRobotControl` | 通用控制 | Dashboard 服务 |

### 🔵 P3 — 低优先级 (~100个)

| 类别 | 函数数 | 说明 |
|------|:---:|------|
| 运动属性 (其余) | ~15 | `SetGlobal*Angle*`, `GetGlobal*`, `TrackPlaybackCycle`, `CircularLoopTimes`, `MoveRelative`, `ArrivalAhead*`, `TeachCoordinate` |
| 传送带跟踪 | 15 | 需真实传送带硬件 |
| 离线轨迹 | 5 | 可用 FollowJointTrajectory 替代 |
| 安全 IO / ORPE | 10 | 部分已通过 RobotEventCallback 覆盖 |
| 固件升级 | 3 | 独立工具更合适 |
| 相机标定 / Weave / Recognition | 5 | 专用场景 |
| IO 配置/查询 (未使用部分) | ~8 | `getIOConfig`, `getByName` 等 |

---

## 使用状态统计 (逐函数明细)

> 图例: ✅ 已实现 | 🔶 代码就绪待注册 | 🟡 待实现 | 🔵 低优先级 | ❌ 不需要
>
> 实现位置缩写: `hw.cpp` = `aubo_hardware_interface.cpp`, `dash.cpp` = `aubo_dashboard_node.cpp`, `bc.cpp` = `aubo_state_broadcaster.cpp`

### 1. 系统/连接 (5)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceLogin` | 建立到机器人控制器的TCP连接并认证(用户"aubo",密码"123456"),所有API的前置条件 | ✅ | `hw.cpp:30` — `HardwareInterface::init()` |
| `robotServiceLogin` (DH重载) | Login同时返回机器人DH参数 | ❌ | 不需要 — DH参数从URDF读取 |
| `robotServiceGetConnectStatus` | 查询当前TCP连接是否存活 | ✅ | `HardwareInterface::isConnected()` — 原子变量`connected_` |
| `robotServiceLogout` | 断开TCP连接,释放资源 | ✅ | `hw.cpp:100` — `HardwareInterface::shutdown()` |
| `robotServiceRobotHandShake` | 与控制器握手确认通信正常 | ❌ | 不需要 — login后自动完成 |

### 2. 运动控制 (16)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceJointMove` | **关节空间点到点运动**: 各关节独立到达目标角,路径不保证直线 | ✅ | `dash.cpp:245` — `/aubo/move_joint` |
| `robotServiceJointMove` (array) | 同上,输入为6个关节角数组 | ✅ | 同上 |
| `robotServiceLineMove` | **笛卡尔直线运动**: TCP沿直线运动到目标位姿,姿态线性插值 | 🔶 | `dash.cpp` 代码已写 — `/aubo/move_line` |
| `robotServiceLineMove` (array) | 同上,输入为6个关节角 | 🔶 | 同上 |
| `robotServiceRotateMove` | **纯旋转运动**: 绕指定轴旋转,TCP位置保持不变 | 🟡 | Dashboard `/aubo/move_rotate` |
| `robotServiceTrackMove` | **轨迹运动**: 执行预设路点列表,支持圆弧/MOVEP/样条等子模式 | 🟡 | Dashboard `/aubo/move_track` |
| `robotServiceTeachStart` | **启动示教模式**: 机器人持续朝指定方向运动直到teachStop | 🔶 | 代码已写 — `/aubo/teach_start` |
| `robotServiceTeachStop` | **停止示教模式** | ✅ | `dash.cpp:280` — `/aubo/teach_stop` |
| `rootServiceRobotMoveControl` (Stop) | **正常减速停止机器人运动** | ✅ | `dash.cpp:213` — `/aubo/stop` |
| `rootServiceRobotMoveControl` (Pause) | **暂停运动** (可从暂停位置继续) | 🔶 | 代码已写 — `/aubo/pause` |
| `rootServiceRobotMoveControl` (Continue) | **继续运动** (从暂停位置恢复) | 🔶 | 代码已写 — `/aubo/resume` |
| `robotMoveFastStop` | **快速停止** (不减速,紧急程度低于急停) | ✅ | `dash.cpp:225` — `/aubo/fast_stop` |
| `robotServiceFollowModeJointMove` | **跟随模式**: 运动过程中可连续更新目标位置 | 🟡 | 预留 |
| `robotMoveLineToTargetPosition` | 保持当前姿态,直线运动到目标位置(基于坐标系+工具) | 🟡 | 预留 |
| `robotMoveJointToTargetPosition` | 保持当前姿态,关节运动到目标位置(基于坐标系+工具) | 🟡 | 预留 |
| `robotMoveLineToTargetPositionByRelative` | 基于当前位姿的相对偏移直线运动 | ❌ | 可用标准MoveIt规划替代 |
| `robotMoveJointToTargetPositionByRelative` | 基于当前位姿的相对偏移关节运动 | ❌ | 同上 |
| `getJointAngleByTargetPositionKeepCurrentOri` | 保持当前姿态的逆解(纯计算,不运动) | ❌ | 用`/aubo/get_ik`替代 |

### 3. 运动属性 (28)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceInitGlobalMoveProfile` | 将所有运动属性恢复为默认值(关节速度25°/s,末端速度3m/s等) | 🟡 | Dashboard服务 |
| `robotServiceSetGlobalMoveJointMaxAcc` | 设置关节运动最大加速度(°/s²),影响关节空间运动速度上限 | 🟡 | 参数 `joint_max_acc` |
| `robotServiceGetGlobalMoveJointMaxAcc` | 查询当前关节最大加速度 | 🟡 | Dashboard服务 |
| `robotServiceSetGlobalMoveJointMaxVelc` | 设置关节运动最大速度(°/s),影响关节空间运动速度上限 | 🟡 | 参数 `joint_max_vel` |
| `robotServiceGetGlobalMoveJointMaxVelc` | 查询当前关节最大速度 | 🟡 | Dashboard服务 |
| `robotServiceSetGlobalMoveEndMaxLineAcc` | 设置末端直线运动最大加速度(m/s²),影响笛卡尔运动速度上限 | 🟡 | 参数 `end_max_line_acc` |
| `robotServiceGetGlobalMoveEndMaxLineAcc` | 查询当前末端最大线加速度 | 🟡 | Dashboard服务 |
| `robotServiceSetGlobalMoveEndMaxLineVelc` | 设置末端直线运动最大速度(m/s),影响笛卡尔运动速度上限 | 🟡 | 参数 `end_max_line_vel` |
| `robotServiceGetGlobalMoveEndMaxLineVelc` | 查询当前末端最大线速度 | 🟡 | Dashboard服务 |
| `robotServiceSetGlobalMoveEndMaxAngleAcc` | 设置末端姿态旋转最大角加速度(rad/s²) | 🔵 | 低优先级 |
| `robotServiceGetGlobalMoveEndMaxAngleAcc` | 查询当前末端最大角加速度 | 🔵 | 低优先级 |
| `robotServiceSetGlobalMoveEndMaxAngleVelc` | 设置末端姿态旋转最大角速度(rad/s) | 🔵 | 低优先级 |
| `robotServiceGetGlobalMoveEndMaxAngleVelc` | 查询当前末端最大角速度 | 🔵 | 低优先级 |
| `robotServiceSetJerkAccRatio` | 设置加加速度比例,控制运动平滑度(值越小越平滑) | 🟡 | 参数 `jerk_ratio` |
| `robotServiceGetJerkAccRatio` | 查询当前加加速度比例 | 🔵 | 低优先级 |
| `robotServiceClearGlobalWayPointVector` | 清空路点列表(用于TrackMove前的准备) | 🟡 | Dashboard服务 |
| `robotServiceAddGlobalWayPoint` | 添加路点到运动列表(支持笛卡尔位姿或关节角两种输入) | 🟡 | Dashboard服务 |
| `robotServiceGetGlobalWayPointVector` | 获取当前路点列表 | 🔵 | 低优先级 |
| `robotServiceSetGlobalBlendRadius` | 设置交融半径(0.0-0.05m),MOVEP模式下路点间平滑过渡的距离 | 🟡 | 参数 `blend_radius` |
| `robotServiceGetGlobalBlendRadius` | 查询当前交融半径 | 🔵 | 低优先级 |
| `robotServiceGetTrackPlaybackCycle` | 查询轨迹回放周期 | 🔵 | 低优先级 |
| `robotServiceSetTrackPlaybackCycle` | 设置轨迹回放周期(秒) | 🔵 | 低优先级 |
| `robotServiceGetGlobalCircularLoopTimes` | 查询圆轨迹圈数 | 🔵 | 低优先级 |
| `robotServiceSetGlobalCircularLoopTimes` | 设置圆轨迹圈数(0=圆弧,>0=完整的圆) | 🟡 | 参数 `circular_loops` |
| `robotServiceSetMoveRelativeParam` | 设置运动偏移量(基于基座标系或用户坐标系),如焊接轨迹偏移 | 🔵 | 低优先级 |
| `robotServiceSetNoArrivalAhead` | 关闭提前到位功能(必须精确到达目标后才开始下一段) | 🟡 | 参数 `no_arrival_ahead` |
| `robotServiceSetArrivalAheadDistanceMode` | 提前到位—距离模式: TCP距目标<指定距离时即过渡到下一目标 | 🟡 | 参数 `arrival_ahead_dist` |
| `robotServiceSetArrivalAheadTimeMode` | 提前到位—时间模式: 预计<指定秒数到达时即过渡到下一目标 | 🟡 | 参数 `arrival_ahead_time` |
| `robotServiceSetArrivalAheadBlendDistanceMode` | 提前到位—交融距离模式 | 🟡 | 参数 `arrival_ahead_blend` |
| `robotServiceSetTeachCoordinateSystem` | 设置示教运动的参考坐标系(默认基座标系) | 🔵 | 低优先级 |

### 4. TCP2CAN 透传 (4)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceEnterTcp2CanbusMode` | **进入透传模式**: 控制器停止自主轨迹规划,等待外部逐点下发关节位置 | ✅ | `hw.cpp:75` |
| `robotServiceLeaveTcp2CanbusMode` | **退出透传模式**: 恢复控制器自主轨迹规划,SDK运动API重新可用 | ✅ | `hw.cpp:92` |
| `robotServiceSetRobotPosData2Canbus` (单点) | 发送单个路点到CAN总线(6关节角),用于OTG紧急减速 | ✅ | `hw.cpp:148` — `writeTrajectoryPoint()` |
| `robotServiceSetRobotPosData2Canbus` (批量) | **批量发送路点**(主流路径),机器人以200Hz从RIB缓冲区消费 | ✅ | `hw.cpp:140` — `writeTrajectoryPoints()` |

### 5. 工具接口 (9)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceSetNoneToolDynamicsParam` | 清除工具动力学参数(恢复为无工具状态) | 🔶 | 代码已写 — `/aubo/clear_tool_dynamics` |
| `robotServiceSetToolDynamicsParam` | **设置末端负载**: 质量(kg)和重心位置(m),影响动力学模型和碰撞检测 | 🔶 | 代码已写 — `/aubo/set_payload` |
| `robotServiceGetToolDynamicsParam` | 查询当前负载设置 | 🟡 | Dashboard `/aubo/get_payload` |
| `robotServiceSetNoneToolKinematicsParam` | 清除TCP偏移(TCP回到法兰盘中心) | 🔶 | 代码已写 — `/aubo/clear_tool_kinematics` |
| `robotServiceSetToolKinematicsParam` | **设置TCP**: 工具中心点相对法兰盘的位置(m)和姿态(四元数) | 🔶 | 代码已写 — `/aubo/set_tool_kinematics` |
| `robotServiceGetToolKinematicsParam` | 查询当前TCP设置 | 🟡 | Dashboard `/aubo/get_tool_kinematics` |
| `robotServiceToolCalibration` (位置) | **工具标定**: 通过多个位姿用最小二乘法自动计算TCP位置 | 🟡 | Dashboard `/aubo/calibrate_tool` |
| `robotServiceToolCalibration` (位置+姿态) | 工具标定(同时标定位置和姿态) | 🟡 | 同上 |
| `robotServiceSetRobotTool` | 设置工具(等同于SetToolKinematicsParam的别名) | 🟡 | Dashboard `/aubo/set_robot_tool` |

### 6. IO 接口 (16)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceGetBoardIOConfig` | 读取接口板IO的配置信息(名称、地址、方向) | 🟡 | Dashboard `/aubo/get_io_config` |
| `robotServiceGetBoardIOStatus` (type list) | **批量读取IO状态**: 数字输入/输出、模拟输入/输出的当前值 | ✅ | `hw.cpp` — `readSafetyIOStatus()` / `readFullIOStatus()` |
| `robotServiceSetBoardIOStatus` (by name) | 按IO名称设置输出状态 | 🔵 | 低优先级(用by addr替代) |
| `robotServiceSetBoardIOStatus` (by addr) | **按地址设置IO输出**: 数字输出的高低电平、模拟输出的电压值 | ✅ | `hw.cpp` + `/aubo/set_io` |
| `robotServiceGetBoardIOStatus` (by name) | 按名称读取单个IO状态 | 🔵 | 低优先级 |
| `robotServiceGetBoardIOStatus` (by addr) | 按地址读取单个IO状态 | 🔵 | 低优先级 |
| `robotServiceSetToolPowerVoltageType` | **设置工具端电源电压**: 0V/12V/24V | ✅ | `hw.cpp` + `/aubo/set_tool_voltage` |
| `robotServiceGetToolPowerVoltageType` | 查询当前工具电源电压类型 | 🟡 | Dashboard `/aubo/get_tool_voltage` |
| `robotServiceGetToolPowerVoltageStatus` | 查询工具电源当前实际电压值 | 🟡 | IO状态话题发布 |
| `robotServiceSetToolPowerTypeAndDigitalIOType` | 批量配置工具电源+4路数字IO方向 | 🟡 | Dashboard `/aubo/config_tool_io` |
| `robotServiceSetToolDigitalIOType` | 设置工具端单路IO方向(输入或输出) | ✅ | `hw.cpp` — `writeToolIOCommand()` |
| `robotServiceGetAllToolDigitalIOStatus` | 读取工具端所有数字IO的当前状态 | ✅ | `hw.cpp` — `readFullIOStatus()` |
| `robotServiceSetToolDOStatus` (by addr) | 设置工具端数字输出值(高/低电平) | ✅ | `hw.cpp` — `writeToolIOCommand()` |
| `robotServiceSetToolDOStatus` (by name) | 按名称设置工具端数字输出 | 🔵 | 低优先级 |
| `robotServiceGetToolIoStatus` | 按名称读取单个工具IO状态 | 🟡 | Dashboard `/aubo/get_tool_io` |
| `robotServiceGetAllToolAIStatus` | 读取工具端所有模拟输入当前值 | ✅ | `hw.cpp` — `readFullIOStatus()` |

### 7. 运动学 (15)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceRobotFk` | **正解**: 6个关节角(rad) → TCP笛卡尔位姿(位置m+姿态四元数) | ✅ | `dash.cpp:325` — `/aubo/get_fk` |
| `robotServiceRobotIk` (单解) | **逆解**: 目标位姿+参考关节角 → 最接近参考角的一个逆解 | ✅ | `dash.cpp:343` — `/aubo/get_ik` |
| `robotServiceRobotIk` (多解) | 逆解: 返回所有可能的关节角解 | 🔵 | 低优先级 |
| `baseToUserCoordinate` | 基座标系→用户坐标系: 法兰盘位姿转为工具末端基于用户坐标系 | 🟡 | Dashboard `/aubo/base_to_user` |
| `baseToBaseAdditionalTool` | 基座标系加工具偏移: 法兰盘位姿→工具末端基于基座标系 | 🟡 | Dashboard `/aubo/base_add_tool` |
| `userToBaseCoordinate` | 用户坐标系→基座标系: 工具末端位姿转为法兰盘基于基座标系(去工具) | 🟡 | Dashboard `/aubo/user_to_base` |
| `userCoordPointToBasePoint` | 用户坐标系内的点→基座标系内的点(纯位置变换,无姿态) | 🟡 | Dashboard `/aubo/user_point_to_base` |
| `endOrientation2ToolOrientation` | 法兰盘姿态→工具末端姿态 | 🟡 | Dashboard `/aubo/end_ori_to_tool` |
| `toolOrientation2EndOrientation` | 工具末端姿态→法兰盘姿态 | 🟡 | Dashboard `/aubo/tool_ori_to_end` |
| `getTargetWaypointByPosition` | 综合: 已知当前位姿+用户坐标系+工具+目标位置→计算目标关节角 | 🟡 | Dashboard服务 |
| `robotServiceCheckUserCoordinate` | 验证用户坐标系参数是否合法 | 🟡 | Dashboard `/aubo/check_user_coord` |
| `robotServiceUserCoordinateCalibration` | 用户坐标系标定(3点法或类似方法) | 🟡 | Dashboard `/aubo/calibrate_user_coord` |
| `robotServiceOriMatrixToQuaternion` | 3×3旋转矩阵→四元数 | ❌ | 驱动内自算(`fillCartesianPoseAndRpy`) |
| `quaternionToRPY` | 四元数(w,x,y,z)→欧拉角(Roll,Pitch,Yaw) | ❌ | 驱动内自算 |
| `RPYToQuaternion` | 欧拉角(Roll,Pitch,Yaw)→四元数(w,x,y,z) | ❌ | 不需要 |

### 8. 状态/诊断 (17)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceGetRobotWorkMode` | 查询工作模式(仿真模式/真实机器人模式) | 🟡 | Dashboard `/aubo/get_work_mode` |
| `robotServiceSetRobotWorkMode` | 设置工作模式(可切换到仿真模式进行离线测试) | 🟡 | Dashboard `/aubo/set_work_mode` |
| `robotServiceGetRobotGravityComponent` | 读取当前重力在基座标系下的分量 | 🔵 | 低优先级 |
| `robotServiceGetRobotCollisionCurrentService` | 查询当前碰撞检测灵敏度等级(1-10) | 🟡 | Dashboard `/aubo/get_collision_class` |
| `robotServiceSetRobotCollisionClass` | **设置碰撞等级**: 1最敏感(轻微碰即停),10最不敏感,默认6 | 🔶 | 代码已写 — `/aubo/set_collision_class` |
| `robotServiceGetRobotDevInfoService` | 读取设备信息(序列号、固件版本、型号等) | 🔶 | 代码已写 — `/aubo/get_robot_info` |
| `robotServiceSetRobotMaxACC` | 设置机器人全局最大加速度 | 🟡 | Dashboard `/aubo/set_max_acc` |
| `robotServiceGetRobotCurrentState` | 读取机器人当前运行状态 | 🟡 | StateBroadcaster定时发布到话题 |
| `robotServiceGetMacCommunicationStatus` | 查询MAC(运动控制器)通信状态 | 🟡 | StateBroadcaster定时发布 |
| `robotServiceGetIsRealRobotExist` | 判断是否有真实机器人连接(区别于仿真) | ✅ | `hw.cpp:59` — `init()`内部调用 |
| `robotServiceGetJoint6Rotate360EnableFlag` | 查询第6关节360°旋转使能状态(AUBO特有功能) | 🔵 | 低优先级 |
| `robotServiceGetRobotJointStatus` | **详细关节状态**: 6关节的角度+速度+电流+温度+错误码 | 🔶 | 代码已写 — `/aubo/get_joint_status` |
| `robotServiceGetRobotDiagnosisInfo` | **读取诊断信息**: RIB缓冲量(`macTargetPosDataSize`)+上电状态 | ✅ | `hw.cpp:133` — `readDiagnosis()` |
| `robotServiceGetJointAngleInfo` | 读取当前6个关节角(简化版,仅角度无速度) | 🟡 | StateBroadcaster定时发布 |
| `robotServiceGetCurrentWaypointInfo` | **读取当前位姿**: 6关节角+笛卡尔位置+姿态四元数 | ✅ | `hw.cpp:115` — `readJointState()` |
| `robotServiceIsOnlineMode` | 查询是否运行在联机模式(区别于单机模式) | 🟡 | StateBroadcaster定时发布 |
| `robotServiceIsOnlineMasterMode` | 查询是否为联机主模式(多机协作场景) | 🔵 | 低优先级 |

### 9. 控制 (8)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `rootServiceRobotControl` | 通用机器人控制命令 | 🟡 | Dashboard `/aubo/control` |
| `robotServicePowerControl` | 单独控制机器人电源(true=上电,false=断电) | 🟡 | Dashboard `/aubo/power_control` |
| `robotServiceReleaseBrake` | **单独松刹车**(不放刹车机器人无法运动) | 🔶 | 代码已写 — `/aubo/brake_release` |
| `rootServiceRobotStartup` | **一键启动**: 上电+松刹车+设置碰撞等级+设置动力学参数(阻塞2-5s) | ✅ | `dash.cpp:180` — `/aubo/startup` |
| `robotServiceRobotShutdown` | **关机**: 先停止运动,再断电(阻塞) | 🔶 | 代码已写 — `/aubo/shutdown` |
| `robotServiceSetRobotMaxACC` | 设置全局最大加速度 | 🟡 | Dashboard `/aubo/set_max_acc` |
| `robotServiceCollisionRecover` | **碰撞恢复**: 碰撞后必须先调用此函数才能继续运动 | ✅ | `dash.cpp:237` — `/aubo/collision_recover` |
| `robotServiceSetRobotJointOffset` | 关节碰撞零位补偿(补偿范围0.00~0.51°) | 🔵 | 低优先级(工厂校准用) |

### 10. SDK 回调 (7)

| SDK 函数 | 作用 | 状态 | 实现方式 |
|---------|------|:---:|------|
| `robotServiceSetRealTimeJointStatusPush` | 启用SDK主动推送关节状态(需配合RegisterCallback使用) | ✅ | `hw.cpp:226` |
| `robotServiceRegisterRealTimeJointStatusCallback` | **注册关节状态回调**: SDK内部线程推送6关节角度+速度+电流+温度 | ✅ | `hw.cpp:227` |
| `robotServiceSetRealTimeRoadPointPush` | 启用SDK主动推送路点信息 | 🔶 | 可选启用,代码已支持 |
| `robotServiceRegisterRealTimeRoadPointCallback` | 注册路点回调: 推送当前法兰盘完整位姿 | 🔶 | 可选启用,代码已支持 |
| `robotServiceSetRealTimeEndSpeedPush` | 启用SDK主动推送末端速度 | 🔶 | 可选启用,代码已支持 |
| `robotServiceRegisterRealTimeEndSpeedCallback` | 注册末端速度回调: 推送当前TCP线速度(m/s) | 🔶 | 可选启用,代码已支持 |
| `robotServiceRegisterRobotEventInfoCallback` | **注册事件回调**: 推送49种事件(急停/碰撞/断连/供电变化等),**默认启用无法关闭** | ✅ | `hw.cpp:218` |

### 11-15. 其余子系统

| SDK 函数 | 作用 | 状态 |
|---------|------|:---:|
| **传送带跟踪 (15)** | | |
| `robotServiceSetConveyorEncoderReset` | 重置传送带编码器 | 🔵 |
| `robotServiceSetConveyorStartup` | 启动传送带 | 🔵 |
| `robotServiceSetConveyorStop` | 停止传送带 | 🔵 |
| `robotServiceSetConveyorDir` | 设置传送带方向 | 🔵 |
| `robotServiceSetConveyorVelc` | 设置传送带线速度(m/s) | 🔵 |
| `robotServiceSetEncoderValPerMeter` | 设置编码器比例(脉冲数/米) | 🔵 |
| `robotServiceSetStartWindowUpstream` | 设置抓取窗口上游边界(m) | 🔵 |
| `robotServiceSetStartWindowDownstream` | 设置抓取窗口下游边界(m) | 🔵 |
| `robotServiceSetConveyorTrackDownstream` | 设置跟踪轨迹下限(m) | 🔵 |
| `robotServiceAppendObject2ConveyorTrackQueue` | 添加物体到传送带跟踪队列 | 🔵 |
| `robotServiceEnableConveyorTrack` | 启用传送带跟踪 | 🔵 |
| `robotServiceGetConveyorEncoderVal` | 读取当前编码器值 | 🔵 |
| `robotServiceSetRobotConveyorTrackMaxVelc` | 设置传送带跟踪时机器人最大速度 | 🔵 |
| `robotServiceSetRobotConveyorTrackMaxAcc` | 设置传送带跟踪时机器人最大加速度 | 🔵 |
| `robotServiceSetRobotConveyorSystemDelay` | 设置传送带跟踪系统延时补偿 | 🔵 |
| **离线轨迹 (5)** | | |
| `robotServiceOfflineTrackWaypointAppend` (vector) | 从内存添加离线轨迹路点 | 🔵 |
| `robotServiceOfflineTrackWaypointAppend` (file) | 从文件加载离线轨迹路点 | 🔵 |
| `robotServiceOfflineTrackWaypointClear` | 清空离线轨迹 | 🔵 |
| `robotServiceOfflineTrackMoveStartup` | 启动离线轨迹回放 | 🔵 |
| `robotServiceOfflineTrackMoveStop` | 停止离线轨迹回放 | 🔵 |
| **安全IO/ORPE (10)** | | |
| `robotServiceSetRobotAtOriginPose` | 设置机器人为原点姿态 | 🔵 |
| `robotServiceSetRobotOrpePause` | 通知接口板上位机暂停状态 | 🔵 |
| `robotServiceSetRobotOrpeStop` | 通知接口板上位机停止状态 | 🔵 |
| `robotServiceSetRobotOrpeError` | 通知接口板上位机错误(16字节错误码) | 🔵 |
| `robotServiceClearSystemEmergencyStop` | 解除系统紧急停止输出信号 | 🔵 |
| `robotServiceClearReducedModeError` | 解除缩减模式错误 | 🔵 |
| `robotServiceRobotSafetyguardResetSucc` | 防护重置成功确认 | 🔵 |
| `robotServiceGetRobotSafetyConfig` | 读取安全配置参数 | 🔶 |
| `robotServiceSetRobotSafetyConfig` | 设置安全配置参数 | 🔵 |
| `robotServiceGetOrpeSafetyStatus` | 读取ORPE安全状态 | 🔵 |
| **固件升级 (3)** | | |
| `robotServiceUpdateRobotBoardFirmware` | 更新机器人控制器固件 | ❌ |
| `robotServiceGetBoardFirmwareUpdateResultService` | 查询固件升级结果 | ❌ |
| `robotServiceGetRobotEthernetDeviceName` | 获取以太网设备名 | ❌ |
| **其他 (6)** | | |
| `robotServiceSetWeaveMoveParameters` | 设置焊接摆动参数(振幅/频率/波形) | 🔵 |
| `robotServiceSetRobotRecognitionParam` | 设置视觉识别参数 | 🔵 |
| `robotServiceGetRobotRecognitionParam` | 读取视觉识别参数 | 🔵 |
| `robotServiceSetRobotCameraCalib` | 设置手眼标定结果 | 🔵 |
| `startupOfflineExcitTrajService` | 启动动力学辨识激励轨迹 | 🔵 |
| `getDynIdentifyResultsService` | 获取动力学辨识结果 | 🔵 |

---

## 进度汇总

| 状态 | 数量 | 含义 |
|------|:---:|------|
| ✅ P0 完整实现 | **28** | 服务已注册+srv规范+逻辑完整 — 可直接调用 |
| 🟡 P2 待实现 | **40** | 接口设计明确, 实现工作量小(每个~30行Dashboard服务) |
| 🔵 P3 低优先级 | **89** | 专用场景或已可替代功能 |
| ❌ 不需要 | **6** | 驱动内自算或可用标准方式替代 |
| **总计** | **163** | 覆盖: P0 17% → P0+P2 42% |
