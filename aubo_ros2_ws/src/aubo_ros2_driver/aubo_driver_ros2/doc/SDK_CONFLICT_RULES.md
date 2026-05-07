# AUBO SDK 模式冲突规则

本文档记录了 AUBO SDK (ServiceInterface/libauborobotcontroller) 在使用中的所有模式冲突和约束。

---

## 核心冲突

### 规则 1: TCP2CAN 模式与 SDK 运动 API 互斥

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

**后果**: TCP2CAN 模式下调用 SDK 运动 API 会返回错误或行为不可预测。

**处理**: 
- Dashboard 节点维护**独立的 ServiceInterface 连接**（不进入 TCP2CAN 模式）
- Dashboard 的运动 API 服务（`/aubo/move_joint` 等）使用自己的普通模式连接
- 轨迹控制器（JointTrajectoryController）使用 conn_control_ 的 TCP2CAN 连接
- 两者互不干扰

### 规则 2: IsBolck 参数行为

| IsBolck=true | IsBolck=false |
|-------------|---------------|
| SDK 调用阻塞直到运动完成或故障 | SDK 调用立即返回（命令已发送） |
| 可能阻塞数秒到数十秒 | 不阻塞 |
| 不能在 ROS2 回调中直接使用 | 适合 ROS2 服务回调 |
| 适合简单脚本/测试 | 生产环境推荐 |

**处理**: 所有 Dashboard 服务统一使用 `IsBolck=false`。

### 规则 3: 运动中的安全约束

| 操作 | 运动中可以执行？ | 备注 |
|------|:---:|------|
| `rootServiceRobotStartup` | ❌ | 需先停止运动 |
| `robotServiceRobotShutdown` | ❌ | 需先停止运动 |
| `robotServiceReleaseBrake` | ❌ | 需先停止运动后松刹车 |
| `robotServiceCollisionRecover` | ❌ | 碰撞后必须先恢复才能继续 |
| `robotServiceSetToolDynamicsParam` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceSetToolKinematicsParam` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceSetRobotCollisionClass` | ⚠️ | 可调用但下次运动才生效 |
| `robotServiceGetRobotDiagnosisInfo` | ✅ | 始终可用 |
| `robotServiceGetCurrentWaypointInfo` | ✅ | 始终可用 |
| `robotServiceRobotFk` | ✅ | 纯计算，始终可用 |
| `robotServiceRobotIk` | ✅ | 纯计算，始终可用 |

### 规则 4: 碰撞恢复顺序

```
碰撞发生
  → robotServiceCollisionRecover()   // 必须先恢复
  → 检查碰撞状态已清除
  → 重新发送运动指令
```

碰撞后未恢复就发送运动指令，SDK 会拒绝执行。

### 规则 5: 同一个 ServiceInterface 实例的线程安全

```
同一个 ServiceInterface 实例内:
  - 所有 SDK API 调用必须串行
  - 不能从多个线程并发调用
  - SDK 内部没有线程安全保护

不同 ServiceInterface 实例间:
  - 可以并行使用（独立 TCP 连接）
  - 不会相互干扰
```

**处理**: 
- conn_control_ → 仅轨迹发送线程使用
- conn_status_ → 仅状态查询/IO 线程使用
- Dashboard 有自己独立的 HardwareInterface → 仅 Dashboard 服务线程使用

### 规则 6: SDK 回调中的约束

```
SDK 回调执行上下文:
  - 在 SDK 内部线程执行（非 ROS2 线程）
  - 使用 pthread_mutex_t 保护回调注册状态

回调中禁止:
  - 调用任何 SDK API（会死锁或行为不可预测）
  - 执行长时间操作（会阻塞 SDK 内部接收线程）
  - 分配大量内存

回调中允许:
  - std::atomic 变量的读写
  - 写入无锁队列
  - RCLCPP_* 日志（ROS2 日志是线程安全的）
```

---

## 启动/关闭顺序

### 正常启动流程

```
1. robotServiceLogin("host", port, "aubo", "123456") ×2
2. robotServiceRegisterRobotEventInfoCallback()
3. robotServiceGetIsRealRobotExist() → 确认连接状态
4. [可选] rootServiceRobotStartup() → 上电+松刹车+碰撞等级
5. robotServiceEnterTcp2CanbusMode() → 进入轨迹流模式
6. 开始轨迹流控制循环
```

### 正常关闭流程

```
1. rootServiceRobotMoveControl(RobotMoveStop) → 停止运动
2. robotServiceLeaveTcp2CanbusMode() → 退出轨迹流模式
3. [可选] robotServiceRobotShutdown() → 关机
4. robotServiceLogout() ×2 → 断开连接
```

### 异常恢复流程

```
检测到 RobotEvent_socketDisconnected:
  → connected_ = false
  → 尝试 robotServiceLogin() 重连
  → 重连成功后恢复之前的状态（TCP2CAN 模式等）
```

---

## 连接架构

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

## 通信延迟参考

| SDK 调用 | 典型延迟 | 尖峰延迟 | 备注 |
|---------|---------|---------|------|
| `robotServiceGetCurrentWaypointInfo` | 2-5 ms | ~12 ms | 50Hz 轮询可行 |
| `robotServiceGetRobotDiagnosisInfo` | 2-12 ms | **225 ms** | TCP 往返，有尖峰 |
| `robotServiceSetRobotPosData2Canbus` (batch) | 5-15 ms | **225 ms** | 批量发送，有尖峰 |
| `robotServiceSetRobotPosData2Canbus` (single) | 2-5 ms | ~20 ms | 单点发送较快 |
| `robotServiceSetBoardIOStatus` | 5-10 ms | ~30 ms | IO 写入 |
| `robotServiceGetBoardIOStatus` (multi) | 10-30 ms | ~50 ms | 查询多种 IO 类型 |
| `robotServiceJointMove` (IsBolck=false) | 5-10 ms | ~20 ms | 仅发送命令 |
| `rootServiceRobotStartup` (IsBolck=true) | 2-5 s | ~10 s | 包含上电+松刹车 |

数据来源: `doc/PORTING_MOTION_FIX.md` 和 `doc/MOTION_JITTER_FIX_SUMMARY_2026-03-18.md`

---

## 参考

- `doc/PORTING_MOTION_FIX.md` — ROS1→ROS2 移植 13 轮调试完整记录
- `doc/MOTION_JITTER_FIX_SUMMARY_2026-03-18.md` — 运动抖动修复总结
- `serviceinterface.h` — AUBO SDK C++ API 头文件 (1343 行)
- `AuboRobotMetaType.h` — SDK 数据类型定义
