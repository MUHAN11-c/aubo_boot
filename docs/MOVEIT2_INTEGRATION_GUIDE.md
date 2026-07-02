# MoveIt2 完整接入：自定义 JTC 改造方案

> 基于对 GitHub 真实案例 (UR/Franka/FANUC)、ros2_control 源码、
> AUBO SDK 全接口、MoveIt2 execute() 源码的全面研究，输出最终改造方案。

---

## 一、研究结论

### 1.1 GitHub 真实案例：非实时硬件的 ros2_control 集成

| 项目 | 通信方式 | 核心模式 | 链接 |
|------|---------|---------|------|
| **UR Driver** | TCP/RTDE | 后台线程 + `non_blocking_read` + 哨兵值 (NaN="无新命令") | github.com/UniversalRobots/Universal_Robots_ROS2_Driver |
| **Franka Driver** | TCP/libfranka | lock-free `AsyncBuffer` v3.3.0（用于 `franka_robot_state_broadcaster` 状态发布管道）+ `libfranka` 实时控制回调（`readOnce()`/`writeOnce()` 硬件通信）+ 独立 Executor 线程 | github.com/frankarobotics/franka_ros2 |
| **FANUC CRX** | TCP (Ethernet/IP 或 RMI JSON) | ~0.2s (Ethernet/IP, 已确认) / ~0.6s (RMI, 来源标注 "To be tested"，非最终确认值) 延迟，写入执行缓冲区 | github.com/paolofrance/ros2_fanuc_interface |

> **AUBO 官方 ROS2 driver (`AuboRobot/aubo_ros2_driver`) 不作为参考** — 该仓库针对**新 SDK + 新机械臂**，与本项目使用的旧 SDK (v2.5.3) + 旧机械臂不兼容。其 `write()` 中 `Servoj()` RPC 同步阻塞循环等待的设计恰好说明了直接在 ros2_control `write()` 中做同步 TCP 调用的反面模式，但具体 API/架构与本地 SDK 完全不同，不可参考。

**社区共识**: 非实时硬件应在 `on_activate()` 启动后台线程，`read()/write()` 只做内存拷贝。ros2_control **Jazzy** 版本引入了原生 `is_async` + `AsyncComponentThread` 支持，但 **Humble 没有，必须手动实现后台线程**。

关键参考: [StackExchange: read() 最佳实践](https://robotics.stackexchange.com/questions/107761)

### 1.2 MoveIt2 execute() 对 Action 结果的处理（源码分析）

**核心发现：MoveIt2 只检查 `rclcpp_action::ResultCode`，不检查 `error_code` 具体值。**

```cpp
// moveit_simple_controller_manager/action_based_controller_handle.h (line 217-231)
// 本地源码: /home/mu/ws_moveit2/src/moveit2/moveit_plugins/moveit_simple_controller_manager/...
void finishControllerExecution(const rclcpp_action::ResultCode& state)
{
    if (state == rclcpp_action::ResultCode::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else if (state == rclcpp_action::ResultCode::ABORTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    else if (state == rclcpp_action::ResultCode::CANCELED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    else if (state == rclcpp_action::ResultCode::UNKNOWN)
      last_exec_ = moveit_controller_manager::ExecutionStatus::UNKNOWN;
    else
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
}
// JTC Result 中无论 error_code 是何值，最终都通过 action ResultCode 传递
// PATH_TOLERANCE_VIOLATED(-4) → JTC setAborted() → ResultCode::ABORTED → ExecutionStatus::ABORTED
// GOAL_TOLERANCE_VIOLATED(-5) → JTC setAborted() → ResultCode::ABORTED → ExecutionStatus::ABORTED
```

**MoveIt2 不处理 Feedback 内容** — 它只等待 result。Feedback 的 `desired/actual/error` 供 rviz2/foxglove 工具使用。

### 1.3 AUBO SDK 停止机制

| 函数 | 延迟 | TCP2CAN 下可用 | RIB 影响 |
|------|------|:---:|------|
| `rootServiceRobotMoveControl(RobotMoveStop)` | 5-15ms | ✅ | **丢弃 RIB 残留** |
| `robotMoveFastStop()` | 5-15ms | ✅ | 快速停止（SDK 错误码 `ErrCode_moveControlFastStopFailed=11026`），不依赖 TCP2CAN 模式 |
| `robotServiceLeaveTcp2CanbusMode()` | 5-15ms | ✅ | 退出透传。SDK_CONFLICT_RULES 关闭流程要求在此之前先 stop，否则 RIB 残留可能继续执行 |
| `robotServiceEnterTcp2CanbusMode()` | 5-15ms | — | 重入透传，RIB 不自动清空 |

> **参考**: SDK `serviceinterface.h:559` (`rootServiceRobotMoveControl`) / `serviceinterface.h:561` (`robotMoveFastStop`)，`AuboRobotMetaType.h:364-369` (`RobotMoveControlCommand` 枚举)，错误码 `ErrCode_moveControlSlowStopFailed=11025` / `ErrCode_moveControlFastStopFailed=11026`。
>
> **SDK 版本说明**: 以上行号来自项目实际编译链接的安装版 SDK（`include/aubo_driver_ros2/` + `lib/`，`serviceinterface.h` 1342 行）。`doc/references/aubo_sdk/` 是较新版本的参考副本（1551 行，多了 `robotMoveStop()` 等新 API），仅供 API 演进参考，**以安装版为准**。
>
> **延迟数据**: 表中 5-15ms 延迟来自 `SDK_CONFLICT_RULES.md` 通信延迟参考表中间接依据——`robotServiceSetRobotPosData2Canbus` batch 模式典型延迟 5-15ms（尖峰 225ms），`robotServiceJointMove` IsBolck=false 典型延迟 5-10ms（尖峰 ~20ms）。`rootServiceRobotMoveControl` 和 `robotMoveFastStop` 未在延迟表中独立列出，延迟值为同级别 SDK TCP 调用的合理估算。

**正确停止顺序**（依据 `aubo_driver_ros2/doc/SDK_CONFLICT_RULES.md` 关闭流程第1-2步）: `RobotMoveStop` → `LeaveTcp2CanbusMode` → `EnterTcp2CanbusMode`（如需恢复）

### 1.4 现有 JTC 的问题

| # | 问题 | 证据 |
|---|------|------|
| 1 | `handleCancel()` 只 return ACCEPT，不停止运动 | `joint_trajectory_controller.cpp:65-66` |
| 2 | `blendToFirstPoint()` 头文件声明但**从未实现** | `joint_trajectory_controller.h:165-168` 声明，`.cpp` 无实现 |
| 3 | `feedback_pub_` 创建但**从未 publish()** | `.cpp:38` 创建，全文无 publish 调用 |
| 4 | `deactivate()` 不调用 SDK stop | RIB 残留路点继续执行，安全风险 |
| 5 | 容差硬编码 `goal_tolerance_ = 0.02` | `.h:229`，不读 goal 中的 goal_tolerance |
| 6 | `precomputed_` 跨线程无锁读写 | sendLoop 线程 + action 回调线程并发访问 |
| 7 | 6 个成员变量声明但从不使用 | `current_traj_`, `current_point_index_`, `trajectory_start_time_`, `wall_clock_start_`, `last_goal_point_`, `has_last_goal_` |
| 8 | `handleGoal()` 只检查轨迹非空 | `.cpp:61` — 不验证关节名/字段维度/时间戳，非法 goal 也被接受 |

---

## 二、MoveIt2 接口契约

```
MoveIt2 发送 (Goal):               JTC 必须响应:
══════════════════                 ══════════════
trajectory                         → ① 验证后执行 (非法 → REJECT，runtime 失败 → ABORT + error_code)
path_tolerance[]                   → ② 执行中偏离 > tol → ABORT PATH_TOLERANCE_VIOLATED(-4)
goal_tolerance[]                   → ③ |actual-goal| < tol 判到位
goal_time_tolerance                → ④ 轨迹结束后再等这段时间
Cancel 请求                         → ⑤ SDK 硬件急停 → CANCELED
新 Goal                            → ⑥ 抢占+过渡点 (blendToFirstPoint)
                                   → ⑦ SUCCEED: error_code=0
                                   → ⑧ 失败: error_code=-1/-2/-3/-4/-5 + error_string
                                   → ⑨ Feedback: desired/actual/error (rviz2用)
```

---

## 三、架构设计

### 3.1 为什么不用 ros2_control plugin

AUBO SDK 单次 TCP 调用 2-225ms。标准 ros2_control 的 `hw.write()` 在控制循环中周期调用（典型 100-1000Hz），必须在下个周期开始前返回。AUBO 若放在 `write()` 中会阻塞整个 control loop，导致控制频率崩溃。

**改造方案: 保留自定义 rclcpp::Node 架构，参照标准 JTC 全部控制逻辑升级。**

### 3.2 改造前后架构对比

```
【改造前】                             【改造后】
handleGoal → 只查 empty                handleGoal → 5项验证 → REJECT/ACCEPT
handleCancel → 只 return ACCEPT        handleCancel → ACCEPT + stopAndClear()
handleAccepted → 简单预计算             handleAccepted → 抢占+过渡点+容差+预计算
update() → 混合 安全检查+到位判断        controlLoop(200Hz) → 安全检查
  (到位检查每 10 次=50ms 周期,          monitorLoop(50Hz) → 到位判断+Cancel+抢占
   且仅在预计算全部发送完毕后才开始)       (独立的显式频率，不依赖 sendLoop 进度)
sendLoop → 无锁                        sendLoop → precomputed_mutex_ 保护
abortActiveGoal → 默认 Result          abortActiveGoal(int error_code)
```

> **改造前 update() 的到位检查延迟问题**: 当前代码 `if (precomputed_idx_ < precomputed_.size()) return;` 意味着**轨迹发送完毕前完全不做到位检查**。对于长轨迹，可能在数秒内对目标偏差毫无感知。改造后 `monitorLoop` 独立于 sendLoop 运行，无论预计算发送进度如何，都能持续监控到位状态和安全条件。

#### handleGoal 验证项（对齐 ros2_controllers `validate_trajectory_msg` 标准）

参考 [ros2_controllers 源码](https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp)，标准 JTC 在 `validate_trajectory_msg()` 中执行 8 个独立检查块（行 1461-1592）。AUBO JTC 改造采纳其中 5 项核心检查，跳过以下不适用的检查（详见下方"跳过的检查"）：

| # | 验证项 | 阶段 | 失败处理 | 备注 |
|---|--------|------|---------|------|
| 1 | 轨迹非空 | handleGoal (GoalResponse) | `trajectory.points.empty()` → **REJECT** | 标准 JTC 检查块 #3 (行 1482-1486) |
| 2 | 关节名匹配 | handleGoal (GoalResponse) | goal 中任一 joint_name 不在控制器 `joint_names_` 中 → **REJECT** | 标准 JTC 检查块 #5 (行 1506-1518) |
| 3 | 字段维度一致 | handleGoal (GoalResponse) | 每个 point 的 `positions` 大小 ≠ `joint_names` 数量 → **REJECT** | 标准 JTC 检查块 #8 子项 (行 1578-1585) |
| 4 | time_from_start 单调递增 | handleGoal (GoalResponse) | 任一点 `time_from_start` ≤ 前一点 → **REJECT** | 标准 JTC 检查块 #7 (行 1535-1547) |
| 5 | 轨迹未过期 | handleGoal (GoalResponse) | `trajectory.header.stamp + points.back().time_from_start` < 当前时间 → **REJECT** | 标准 JTC 检查块 #4 (行 1488-1504) |

> **架构说明**: rclcpp_action 协议中 `GoalResponse::REJECT` 无法携带 error_code——GoalHandle 仅在 ACCEPT 后才创建，`abort()`/`canceled()` 才能设置 error_code。标准 ros2_controllers JTC 对所有验证失败统一执行 REJECT（无 error_code），文档中的 error_code（INVALID_GOAL=-1 / INVALID_JOINS=-2 / OLD_HEADER_TIMESTAMP=-3）仅用于 ACCEPT 后的运行时终止场景。**本方案与标准 JTC 行为一致**：handleGoal 阶段验证失败 → REJECT，不使用 error_code。

> **跳过的检查**（标准 JTC 中 AUBO 不适用的 3 个检查块）:
>
> | 检查块 | 标准 JTC 行号 | 跳过原因 |
> |--------|:---:|------|
> | 检查块 #1: `allow_partial_joints_goal` 关节数不匹配校验 | 1465-1474 | AUBO 不允许部分关节 goal（所有 6 轴必须指定），且 `remapJointNames()` 在 `handleAccepted` 中完成关节名重排 |
> | 检查块 #6: 末速度非零检查（`allow_nonzero_velocity_at_trajectory_end`） | 1520-1533 | AUBO 驱动器内部 PID 独立管理速度过渡，末速度非零不影响安全性；且标准 JTC 此检查已被标记为 deprecated |
> | 检查块 #8 子项: effort 接口字段拒绝 | 1587-1592 | AUBO 仅使用 position 接口，不涉及 effort/velocity/acceleration 字段校验 |
>
> `fill_partial_goal`（部分关节补全）和 `sort_to_local_joint_order`（关节名重排）在 `handleAccepted` 中通过现有 `remapJointNames()` 实现。

### 3.3 Cancel 延迟分析

```
Step1: handleCancel → ACCEPT (<1ms)
Step2: stopAndClear()
  → precomputed_.clear()
  → rootServiceRobotMoveControl(RobotMoveStop)  (~5-15ms TCP)
→ 机械臂停止, sendLoop 检测 precomputed_ 为空后自然进入等待状态
→ 总延迟: 5-15ms

对比标准 ros2_control: preempt_active_goal() 也是在 action_monitor 周期(50ms)内完成
```

---

## 四、改造清单

### 4.1 文件范围

| 文件 | 改动 | 新增行 |
|------|------|--------|
| `aubo_hardware_interface.h` (include/aubo_driver_ros2/) | +1 方法声明 `stopMotion()` | +3 |
| `aubo_hardware_interface.cpp` (src/) | +1 方法实现 | +15 |
| `joint_trajectory_controller.h` (include/aubo_driver_ros2/) | +9 成员 (`std::mutex precomputed_mutex_` + `std::atomic<bool> has_active_goal_` + 7), -6 废弃, ~5 签名修改 | +15 |
| `joint_trajectory_controller.cpp` (src/) | +4 新函数, ~7 改造, ~2 删除 | +300 |

**新增成员变量清单**:

| 新增成员 | 类型 | 用途 |
|---------|------|------|
| `precomputed_mutex_` | `std::mutex` | 保护 `precomputed_[]`/`precomputed_idx_` 跨线程访问 |
| `has_active_goal_` | `std::atomic<bool>` (原 `bool`) | 4 线程安全可见性 |
| `goal_target_point_` | `trajectory_msgs::msg::JointTrajectoryPoint` | 保存目标点用于到位判断 |
| `goal_tolerances_[]` | `double[6]` | per-joint 目标容差 |
| `goal_time_tolerance_` | `double` | goal 中指定的额外等待时间 |
| `goal_hold_start_` | `rclcpp::Time` | 轨迹结束时刻（用于 goal_time_tolerance 计时） |
| `stopped_velocity_tolerance_` | `double` | 判定静止的速度阈值 |

### 4.2 分步实施

#### PR1 — 安全与生命周期 (P0)

| # | 改动 | 说明 |
|---|------|------|
| 1 | 新增 `AuboHardwareInterface::stopMotion()` | conn_control_ 上调用 RobotMoveStop + 备用 robotMoveFastStop |
| 2 | 新增 `JointTrajectoryController::stopAndClear()` | 统一停止: precomputed_.clear(), hw_->stopMotion()。注意不设置 send_running_=false（仅 deactivate 中设置），sendLoop 检测 precomputed_ 为空后自然进入等待状态 |
| 3 | `handleCancel()` 改造 | +stopAndClear() + `gh->canceled()` (外部取消请求 → canceled，对齐标准 JTC `goal_cancelled_callback`) |
| 4 | `deactivate()` 改造 | +stopAndClear() + `gh->abort()` (server 端终止 → abort，对齐标准 JTC PR #1517 已合并共识); leaveTcp2Canbus 由 main.cpp 显式调用 |
| 5 | `sendLoop()` 加 `precomputed_mutex_` | batch 提取时短暂持锁，RIB 查询在锁外 |
| 6 | `has_active_goal_` → `std::atomic<bool>` | 4 线程并发访问 (action 回调 / update 定时器 / 主线程 deactivate)，非原子存在数据竞争 |
| 7 | 删除 `feedback_pub_` | 从未使用，MoveIt2 不处理 feedback (标准 JTC 用于 rviz2 轨迹监控面板；本项目由 aubo_state_broadcaster 替代) |
| 8 | 删除 6 个未用成员 | current_traj_, current_point_index_, trajectory_start_time_, wall_clock_start_, last_goal_point_, has_last_goal_ |

**验证**: 执行中 cancel → 机器人 5-15ms 停止 + MoveIt2 收到 PREEMPTED (非 ABORTED); deactivate → RIB 无残留; colcon build 通过; stress test → 无 segfault/死锁

#### PR2 — 抢占与过渡 (P0)

| # | 改动 | 说明 |
|---|------|------|
| 1 | 实现 `blendToFirstPoint()` | quintic S-curve `10t³-15t⁴+6t⁵`, 30步×5ms=150ms, diff<0.01rad 跳过 |
| 2 | `handleAccepted()` 重写 | 检测 active goal → `canceled()` (抢占语义 = 取消旧 goal，对齐标准 JTC `preempt_active_goal`) → stopAndClear → 读当前位置 → blend → 插入过渡点到预计算开头 |
| 3 | `abortActiveGoal(int error_code)` | 增加错误码参数 |

**验证**: 连续 2 个 goal → 平滑过渡无瞬跳

#### PR3 — 容差与错误码 (P1-P2)

| # | 改动 | 说明 |
|---|------|------|
| 1 | 新增 `readGoalTolerances()` | 从 goal 解析 per-joint 容差; 未设置用 ROS 参数默认 |
| 2 | `withinGoalConstraints()` 改造 | 用 goal_tolerances_[i] 替代硬编码 0.02 |
| 3 | 新增 `checkPathTolerance()` | 实际 vs 预计算期望偏差; 超出 → PATH_TOLERANCE_VIOLATED(-4); **默认禁用**（原因见下方注释） |
| 4 | 所有 Result 正确 error_code | SUCCESS=0, INVALID_GOAL=-1, INVALID_JOINS=-2, OLD_HEADER_TIMESTAMP=-3, PATH_TOLERANCE_VIOLATED=-4, GOAL_TOLERANCE_VIOLATED=-5 |
| 5 | 新增 ROS 参数 | `constraints.goal_time` (特殊值: 0.0=使用默认, -1.0=无限等待, >0=使用指定值), `constraints.stopped_velocity_tolerance`, `constraints.<joint>.goal/trajectory` |

> **注意**: `OLD_HEADER_TIMESTAMP(-3)` 在 `control_msgs/action/FollowJointTrajectory.action` 中已定义但标准 ros2_controllers JTC 未使用（过期轨迹直接 REJECT 无 error_code）。本项目将其用于 `handleAccepted` 中 ACCEPT 后检测到轨迹过期的 abort 场景，属于**超越标准 JTC 的增强**喵~

> **checkPathTolerance 默认禁用原因**: `wayPoint_S` 结构体（`AuboRobotMetaType.h:471-479`）不含时间戳字段——仅含 `cartPos`、`orientation`、`jointpos[ARM_DOF]`。路径容差需要在执行过程中将**当前实际关节角与对应时刻的期望关节角**进行比较，但预计算后的 `wayPoint_S` 丢失了时间信息，无法建立"当前时刻 → 对应期望点"的映射。
>
> 未来如需启用 path_tolerance，有两种方案：(a) 扩展 `wayPoint_S` 增加 timestamp 字段（需修改 SDK 头文件，上游不兼容）；(b) 维护并行的 `vector<JointTrajectoryPoint>` 期望轨迹数组（保留时间戳），路径检查时二分查找对应点。

**验证**: goal_tolerance 设置生效; 手动阻挡 → PATH_TOLERANCE_VIOLATED(-4)

---

## 五、核心代码逻辑

### 5.1 stopMotion() — AUBO SDK 停止封装

> **连接选择**: 在 `conn_control_` (TCP2CAN 连接) 上调用停止命令。Dashboard 节点在 `conn_status_` 上调用（因 Dashboard 不进入 TCP2CAN 模式），但 JTC 处于 TCP2CAN 模式下，`conn_control_` 是正确的选择。TCP2CAN 连接上的 `RobotMoveStop` 可停止当前 RIB 缓冲中的路点执行（`SDK_CONFLICT_RULES.md` 关闭流程第 1 步；RIB 丢弃效果为实测经验推断，非 SDK 文档明确保证）。SDK 参考副本（`doc/references/aubo_sdk/`，较新版本）提供 `robotMoveStop()` 便捷封装，但**当前安装版 SDK 不含此函数**，不可使用。

```cpp
// aubo_hardware_interface.cpp
bool AuboHardwareInterface::stopMotion() {
    if (!connected_ || !tcp2can_mode_) return false;
    // 主路径: RobotMoveStop 丢弃 RIB (SDK_CONFLICT_RULES 关闭流程第1步)
    int ret = conn_control_.rootServiceRobotMoveControl(
        aubo_robot_namespace::RobotMoveStop);
    // 备用: robotMoveFastStop 硬件急停
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        ret = conn_control_.robotMoveFastStop();
    }
    return (ret == aubo_robot_namespace::InterfaceCallSuccCode);
}
```

### 5.2 stopAndClear() — 统一停止入口

```cpp
// joint_trajectory_controller.cpp
void JointTrajectoryController::stopAndClear() {
    // 注意: 不设置 send_running_=false！
    // sendLoop 检测到 precomputed_ 为空后自然进入等待状态 (avail==0 → sleep 4ms 循环)
    // 仅 deactivate() 中才设置 send_running_=false 并 join 线程
    {
        std::lock_guard<std::mutex> lock(precomputed_mutex_);
        precomputed_.clear();
        precomputed_idx_ = 0;
    }
    if (hw_ && hw_->isConnected()) {
        hw_->stopMotion();  // SDK 硬件 stop + 丢弃 RIB
    }
}
```

### 5.3 handleCancel() 改造

> **设计决策**: 使用 `gh->canceled()` 而非 `gh->abort()`。虽然 rclcpp_action C 状态机允许 CANCELING→ABORTED 转换（已通过汇编级反编译验证），但 `canceled()` 产生 ResultCode::CANCELED → ExecutionStatus::PREEMPTED，MoveIt2 可继续发送新轨迹；`abort()` 产生 ABORTED → MoveIt2 视为控制失败停止执行。标准 ros2_control JTC 在取消场景统一使用 `canceled()`（二进制反汇编验证），仅错误条件（路径/目标容差违规、超时）使用 `abort()`。

```cpp
rclcpp_action::CancelResponse JointTrajectoryController::handleCancel(
    const std::shared_ptr<GoalHandle> gh) {
    RCLCPP_INFO(get_logger(), "Cancel requested — immediate stop");
    stopAndClear();                             // 硬件停止 + 清空预计算
    if (active_goal_ && active_goal_ == gh) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Canceled by user";
        gh->canceled(result);                   // canceled() 而非 abort()
        has_active_goal_ = false;
        active_goal_.reset();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}
```

### 5.4 deactivate() 改造

> **设计决策**: 使用 `gh->abort()` 而非 `gh->canceled()`。**ros2_controllers PR #1517 已于 2025-02-04 合并至 master 并 backport 到 humble（PR #1521）**——标准 JTC 在 `on_deactivate()` 中改为 `setAborted()`（原为 `setCanceled()`），理由是 deactivate 是 server 端主动终止（非外部 action client 取消请求），ROS 2 action 状态机规定 server 端终止应使用 ABORTED。
>
> **注意**: ros2_controllers humble 分支（SHA `8388ce7`）中 `preempt_active_goal()` 使用 **`setCanceled()`**（非 `setAborted()`）。PR #1517 仅修改了 `on_deactivate()`（→ `setAborted()`），未修改 `preempt_active_goal()`。标准 JTC 的抢占语义是"旧 goal 被新 goal 取消"→ `setCanceled()`，而 deactivate 是"server 端主动终止"→ `setAborted()`。
>
> **社区决议总结**（humble 分支源码，SHA `8388ce7`）:
> - `on_deactivate()` → `setAborted()` ✅ (PR #1517, 2025-02-04)
> - `preempt_active_goal()` → `setCanceled()` ✅ (未被 PR #1517 修改，原样保留)
> - `goal_cancelled_callback()` → `setCanceled()` ✅ (仅外部取消请求)
> - 容差违规 → `setAborted()` ✅
>
> `leaveTcp2CanbusMode()` 不在 deactivate 内调用——由 `main.cpp` 显式控制，保持 deactivate 职责单一（仅停止轨迹流）。
>
> **注意**: `succeed()`/`abort()`/`canceled()` 内部均有 `rcl_handle_mutex_` 保护（rclcpp_action 汇编验证），从 update 定时器线程或主线程调用是线程安全的。但需注意 rclcpp_action 官方文档**未正式记录此线程安全保证**——内部 `rcl_handle_mutex_` 是实现细节，非公开 API 契约。

```cpp
void JointTrajectoryController::deactivate() {
    send_running_ = false;                       // 通知 sendLoop 退出
    stopAndClear();                              // 停止运动 + 清空预计算
    if (send_thread_.joinable()) send_thread_.join();
    if (has_active_goal_) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = "Controller deactivated during execution";
        active_goal_->abort(result);             // abort() — 对齐标准 JTC (PR #1517)
        has_active_goal_ = false;
        active_goal_.reset();
    }
    // leaveTcp2CanbusMode() 由 main.cpp 调用，不在此处
    if (update_timer_) update_timer_->cancel();
}
```

> **deactivate() 阻塞时间分析**: `deactivate()` 显式设置 `send_running_=false` (<1ms) → `stopAndClear()` 清空预计算并 SDK 停止 → `send_thread_.join()` 等待发送线程退出。若 sendLoop 正在 `writeTrajectoryPoints()` 内部阻塞（TCP 尖峰延迟可达 225ms，见 `SDK_CONFLICT_RULES.md` 通信延迟表），deactivate() 将同步等待。**最坏情况阻塞 ~230ms**（225ms TCP + 5ms 清理）。对于非实时生命周期管理来说可接受，但在紧急停止场景应优先通过 `stopMotion()` 硬件急停，不依赖 deactivate 的 join 时序。

### 5.5 blendToFirstPoint() — quintic smoothstep 过渡（C² 连续）

> **状态: 待实现（PR2）** — 以下为设计伪代码。当前头文件 `joint_trajectory_controller.h:165-168` 有声明但 `.cpp` 无实现，是 PR2 的待开发内容。

五次多项式 `s(t) = 10t³ - 15t⁴ + 6t⁵` 提供 C² 连续（位置/速度/加速度在首尾端均为零），30 步 × 5ms = 150ms 过渡时间。关节角差异 < 0.01 rad 时跳过过渡。

**C² 连续性数学证明**（quintic Hermite spline 在 v₀=v₁=a₀=a₁=0 时的特例）:

| 函数 | 表达式 | t=0 | t=1 |
|------|--------|-----|-----|
| 位置 s(t) | `10t³ - 15t⁴ + 6t⁵` | `s(0) = 0` | `s(1) = 10-15+6 = 1` |
| 速度 s'(t) | `30t² - 60t³ + 30t⁴` = `30t²(1-t)²` | `s'(0) = 0` | `s'(1) = 0` |
| 加速度 s''(t) | `60t - 180t² + 120t³` = `60t(1-t)(1-2t)` | `s''(0) = 0` | `s''(1) = 0` |

- **C⁰ 连续**: s(0)=0, s(1)=1 → 位置在起止点连续（平滑过渡从当前到目标）
- **C¹ 连续**: s'(0)=s'(1)=0 → 速度在起止点均为零（无速度突变，过渡不引入 jerk）
- **C² 连续**: s''(0)=s''(1)=0 → 加速度在起止点均为零（无加速度突变，过渡曲线二阶光滑）

**5ms 来源**: 本项目 JTC 的 `update_period_ = 0.005`（200Hz），定义在 `joint_trajectory_controller.h:234`。30 步 × 5ms/步 = 150ms 过渡时间。

**标准 JTC 对比**: 标准 ros2_controllers JTC 使用 `VARIABLE_DEGREE_SPLINE` 插值方法（`trajectory.cpp:312`），其五次样条系数公式 `c₃=(-20Δp-3a₀T²+a₁T²-12v₀T-8v₁T)/(2T³)` 用于**轨迹采样**（在标称轨迹路径点之间插值），不是抢占过渡用的 smoothstep blending。`blendToFirstPoint()` 是**针对 AUBO 的增强**，标准 JTC 中无对应实现。五次多项式是标准 Hermite spline 在 v₀=v₁=a₀=a₁=0 的退化特例。

```cpp
int JointTrajectoryController::blendToFirstPoint(
    const double current[6],
    const trajectory_msgs::msg::JointTrajectoryPoint& target,
    std::vector<aubo_robot_namespace::wayPoint_S>& out) {
    // 距离 < 0.01 rad → 不需要过渡
    double max_diff = 0;
    for (int i = 0; i < kNJoint; i++)
        max_diff = std::max(max_diff, std::fabs(current[i] - target.positions[i]));
    if (max_diff < 0.01) return 0;

    const int kBlendSteps = 30;  // 150ms @200Hz
    for (int s = 0; s < kBlendSteps; s++) {
        double t = (s + 1.0) / kBlendSteps;
        double s_curve = 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t;  // C2 连续
        wayPoint_S wp{};
        for (int i = 0; i < kNJoint; i++)
            wp.jointpos[i] = current[i] + (target.positions[i] - current[i]) * s_curve;
        out.push_back(wp);
    }
    return kBlendSteps;
}
```

### 5.6 handleAccepted() 重写要点

```
① 检测 has_active_goal_ → active_goal_->canceled() → stopAndClear()
② 读当前位置 hw_->readJointState(current)
③ blendToFirstPoint(current, traj.points.front()) → 生成过渡点
④ 预计算轨迹 (过渡点 + 原始轨迹), 写入 precomputed_
⑤ 解析 goal->goal_tolerance → readGoalTolerances()
⑥ 解析 goal->goal_time_tolerance → 保存用于 update() 计时
⑦ 保存 goal_target_point_ (目标点) → 用于到位判断
⑧ active_goal_ = gh; has_active_goal_ = true
```

> **关于 preempt 使用 `canceled()` 而非 `abort()`**: ros2_controllers 标准 JTC 在 `preempt_active_goal()` 中使用 `setCanceled()`（humble 分支，SHA `8388ce7`）。抢占（新 goal 替换旧 goal）的语义是"旧 goal 被新 goal 取代"——这是一种取消行为，应使用 `canceled()` 产生 `ResultCode::CANCELED` → `ExecutionStatus::PREEMPTED`，使 MoveIt2 可继续发送后续轨迹。`abort()` 产生 `ABORTED` → MoveIt2 视为控制失败并停止后续执行。
>
> **MoveIt2 `finishControllerExecution()` 实际映射**（源码 `/home/mu/ws_moveit2/...` `action_based_controller_handle.h:217-231`）:
> - `ResultCode::SUCCEEDED` → `ExecutionStatus::SUCCEEDED`
> - `ResultCode::ABORTED`   → `ExecutionStatus::ABORTED`
> - `ResultCode::CANCELED`  → `ExecutionStatus::PREEMPTED`
> - `ResultCode::UNKNOWN`   → `ExecutionStatus::UNKNOWN`
> - 其他                    → `ExecutionStatus::FAILED`
>
> **MoveIt2 中 ABORTED 的实际影响**（`ExecutionStatus` 传递链）:
> - `TrajectoryExecutionManager::executePart()`: 非 SUCCEEDED 一律终止 → 停止所有后续轨迹
> - `PlanExecution::executeAndMonitor()`: ABORTED → `CONTROL_FAILED`, PREEMPTED → `PREEMPTED`（不同 error_code）
> - MoveGroup action 服务: ABORTED → `CONTROL_FAILED`（中断重规划），PREEMPTED → `PREEMPTED`（客户端已取消，不中断）
> - **相同实际行为**: 两者均不继续后续轨迹、不触发重规划（但 MoveGroup 返回的最终 error_code 不同）
>
> **`goal_time_tolerance` 特殊值** (来自 ros2_controllers `tolerances.hpp:130-155` `resolve_tolerance_source()`):
> - `0.0` (或未设置): 使用控制器默认参数 `constraints.goal_time`
> - `-1.0`: **擦除 (erase)** — 解析为 `0.0`，`update()` 中 `goal_time_tolerance != 0.0` 检查失败 → 跳过时间限制 → 效果等同"无限等待直到关节误差落入 goal_tolerance"
> - `> 0.0`: 使用 goal 中指定的值
> - 其他负值: 抛出 `std::runtime_error("Illegal tolerance value.")` → 回退到默认参数
>
> 源码参考: `/opt/ros/humble/include/joint_trajectory_controller/joint_trajectory_controller/tolerances.hpp`

### 5.7 sendLoop() 线程安全

```cpp
void JointTrajectoryController::sendLoop() {
    while (send_running_ && rclcpp::ok()) {
        // RIB 查询 (锁外 — sendLoop 独占)
        int rib; hw_->readDiagnosis(rib);
        if (rib >= 300) { sleep(4ms); continue; }

        // batch 提取 (锁内 — 最小化持锁时间)
        std::vector<wayPoint_S> batch;
        {
            std::lock_guard<std::mutex> lock(precomputed_mutex_);
            size_t avail = (precomputed_idx_ < precomputed_.size())
                ? precomputed_.size() - precomputed_idx_ : 0;
            // ... 自适应批量计算 ...
            if (n > 0) {
                batch.assign(precomputed_.begin() + precomputed_idx_,
                             precomputed_.begin() + precomputed_idx_ + n);
                precomputed_idx_ += n;
            }
        }
        if (!batch.empty()) hw_->writeTrajectoryPoints(batch);
        // 自适应睡眠...
    }
}
```

---

## 六、线程安全设计

| 资源 | sendLoop 线程 | Action 回调线程 | update 定时器线程 | 主线程 (deactivate) |
|------|:---:|:---:|:---:|:---:|
| `precomputed_[]` / `precomputed_idx_` | 读+写 (加锁) | 写 (加锁) | 只读 (加锁) | — |
| `send_running_` | 只读 | — | — | 写 (仅 deactivate) |
| `has_active_goal_` | — | 读写 | 读 | 读写 |
| `active_goal_` | — | 读写 | 读 (调用 succeed/abort) | 读写 (调用 abort) |
| `goal_target_point_` | — | 写 | 读 | — |
| `goal_tolerances_[]` | — | 写 | 读 | — |
| `goal_hold_count_` | — | 写 (abort) | 读写 (递增/清零) | — |

**关键说明**:
- `send_running_` 和 `has_active_goal_` 使用 `std::atomic<bool>`，保证跨线程可见性
- `send_running_` 仅主线程的 `deactivate()` 写入，action 回调线程的 `handleCancel()` 通过 `precomputed_.clear()` 让 sendLoop 自然进入等待状态（avail==0），不直接写 `send_running_`
- `precomputed_mutex_` 是唯一自定义锁，作用域严格限定 `precomputed_`/`precomputed_idx_`
- `active_goal_->succeed()`/`canceled()`/`abort()` 内部有 `rcl_handle_mutex_` (rclcpp_action) 保护，可在锁外调用
- `shared_ptr<GoalHandle>` 的引用计数操作是线程安全的，但解引用后调用方法需要确保 goal 未被其他线程重置

**死锁预防**: `precomputed_mutex_` 与 rclcpp_action 内部的 `rcl_handle_mutex_` 无依赖关系 → 无死锁风险。RIB 查询在 `precomputed_mutex_` 锁外执行。

---

## 七、功能检查表

| # | MoveIt2 需求 | 改造前 | 改造后 | 验证 |
|---|-------------|--------|--------|------|
| 1 | 非法轨迹 REJECT | ⚠️ 只查 empty | ✅ 5项验证 (非空+关节名+字段维度+时间戳+过期) | 发非法 goal→REJECT |
| 2 | Cancel 停止 | ❌ 不停 | ✅ RobotMoveStop + `canceled()` (外部取消请求) | 运动中 cancel→5-15ms停 + MoveIt2 收到 PREEMPTED |
| 3 | Goal 抢占过渡 | ❌ 跳变 | ✅ blendToFirstPoint | 连续2个goal→无瞬跳 |
| 4 | deactivate 安全 | ❌ RIB残留 | ✅ stopAndClear + `abort()` (server 端终止，对齐标准 JTC PR #1517) → leaveTcp2Canbus (main.cpp 显式调用) | rosbag查RIB |
| 5 | 到位容差(per-joint) | ⚠️ 硬编码0.02 | ✅ goal覆盖+ROS参数 | setGoalTol→生效 |
| 6 | 速度容差 | ❌ | ✅ stopped_velocity_tol | 到位时速度>tol→不判到位 |
| 7 | goal_time_tolerance | ❌ | ⚠️ PR3 待实现 (handleAccepted 解析 + update 计时) | 轨迹结束再等N秒后判到位/超时 |
| 8 | path_tolerance | ❌ | ✅ checkPathTolerance (默认禁用) | 手动阻挡→-4 |
| 9 | error_code/string | ❌ 默认 | ✅ 6种+描述 (SUCCESSFUL/INVALID_GOAL/INVALID_JOINTS/OLD_HEADER_TIMESTAMP/PATH_TOLERANCE_VIOLATED/GOAL_TOLERANCE_VIOLATED + error_string) | ros2 action result |
| 10 | 线程安全 | ❌ 无锁 | ✅ precomputed_mutex_ + atomic members | stress test |

---

## 八、不改的部分

| 组件 | 原因 |
|------|------|
| MoveIt2 / move_group | 标准包，零修改 |
| demo_driver / latte_backend / tool_changer | 通过 MoveGroupInterface 调用，接口不变 |
| aubo_dashboard_node | 独立运维服务，不归 JTC 管 |
| aubo_state_broadcaster | 独立状态发布，保留 |
| quinticInterpolate / remapJointNames | 已验证，逻辑正确 |
| sendLoop RIB 流控 + 自适应批量 | AUBO TCP2CAN 核心，保留 |
| 启动脚本 / launch | 不增删进程 |

---

*创建日期: 2026-06-25 | 最后审查: 2026-06-30*

> **研究来源 (本文件内容来源)**:
> - UR Driver: [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
> - Franka Driver: [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2)
> - ros2_control Humble 源码: [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) (humble 分支, SHA `8388ce7`)
> - MoveIt2 源码: `/home/mu/ws_moveit2/src/moveit2/` (本地 Humble) — `action_based_controller_handle.h:217-231`
> - AUBO SDK `serviceinterface.h`: `/home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/include/aubo_driver_ros2/serviceinterface.h`
> - AUBO SDK `AuboRobotMetaType.h`: 同上 `include/` 目录 — `wayPoint_S` (line 471-479), `RobotDiagnosis`, `JointStatus`
> - SDK_CONFLICT_RULES.md: `/home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/doc/SDK_CONFLICT_RULES.md`
> - 本地 JTC 源码: `aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/src/joint_trajectory_controller.cpp`
> - **SDK 权威数据源**: `include/aubo_driver_ros2/` + `lib/`（安装版，项目实际编译链接）。`doc/references/aubo_sdk/` 为较新 SDK 版本参考副本，**仅供 API 参考，以安装版为准**（参考副本比安装版多 `robotMoveStop()` 等函数，不可在项目中使用）

## 附录：源码验证结果

> 以下验证基于 `doc/references/` 目录下的参考源码副本（ros2_controllers humble 分支 SHA `8388ce7`，MoveIt2 humble 分支，control_msgs humble 分支）。

### A.1 MoveIt2 finishControllerExecution() 映射

**源文件**: `references/moveit2/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h:217-231`

| 文档声明 | 源码行号 | 验证 |
|----------|---------|------|
| SUCCEEDED → SUCCEEDED | 220-221 | ✅ |
| ABORTED → ABORTED | 222-223 | ✅ |
| CANCELED → PREEMPTED | 224-225 | ✅ |
| UNKNOWN → UNKNOWN | 226-227 | ✅ |
| 其他 → FAILED | 228-229 | ✅ |

### A.2 ExecutionStatus 枚举

**源文件**: `references/moveit2/moveit_core/controller_manager/include/moveit/controller_manager/controller_manager.h:49-100`

枚举值：`UNKNOWN`, `RUNNING`, `SUCCEEDED`, `PREEMPTED`, `TIMED_OUT`, `ABORTED`, `FAILED`（7 个值）。文档引用的 5 个值全部存在。

### A.3 FollowJointTrajectory.action error_code

**源文件**: `references/control_msgs/control_msgs/action/FollowJointTrajectory.action:33-39`

| error_code | 源码值 | 文档值 | 验证 |
|-----------|--------|--------|------|
| SUCCESSFUL | 0 | 0 | ✅ |
| INVALID_GOAL | -1 | -1 | ✅ |
| INVALID_JOINTS | -2 | -2 | ✅ |
| OLD_HEADER_TIMESTAMP | -3 | -3 | ✅ |
| PATH_TOLERANCE_VIOLATED | -4 | -4 | ✅ |
| GOAL_TOLERANCE_VIOLATED | -5 | -5 | ✅ |

### A.4 标准 JTC preempt_active_goal()

**源文件**: `references/ros2_controllers/joint_trajectory_controller/src/joint_trajectory_controller.cpp:1603-1614`

- 文档声称使用 `setCanceled()` ✅
- 源码第 1611 行: `active_goal->setCanceled(action_res);`
- 设置 error_code=INVALID_GOAL, error_string="Current goal cancelled due to new incoming action."

### A.5 标准 JTC on_deactivate()

**源文件**: 同上 `:1055-1106`

- 文档声称使用 `setAborted()` (PR #1517) ✅
- 源码第 1065 行: `active_goal->setAborted(action_res);`
- 设置 error_code=INVALID_GOAL, error_string="Current goal cancelled during deactivate transition."

### A.6 标准 JTC goal_cancelled_callback()

**源文件**: 同上 `:1273-1295`

- 文档声称使用 `setCanceled()` ✅
- 源码第 1288 行: `active_goal->setCanceled(action_res);`
- 同时调用 `add_new_trajectory_msg(set_hold_position())` 进入保持位姿模式

### A.7 标准 JTC goal_accepted_callback() 执行顺序

**源文件**: 同上 `:1297-1331`

文档 5.6 节声明顺序 → 实际源码：

| 步骤 | 文档 | 源码行 | 验证 |
|------|------|--------|------|
| ① | preempt_active_goal (setCanceled) | 1305 | ✅ |
| ② | add_new_trajectory_msg | 1306-1311 | ✅ |
| ③ | 设置 RealtimeGoalHandle | 1314-1317 | ✅ |
| ④ | get_segment_tolerances 更新容差 | 1321-1322 | ✅ |

### A.8 validate_trajectory_msg() 全部检查项

**源文件**: 同上 `:1461-1595`

| # | 检查项 | 源码行 | 文档 3.2 节对应 |
|---|--------|--------|----------------|
| 1 | 可选的 partial joints 大小检查 (`allow_partial_joints_goal`) | 1465-1474 | ⚠️ 未单列（属于"关节名匹配"的上层条件） |
| 2 | 关节名非空 | 1476-1480 | —（文档未列此项） |
| 3 | 轨迹点非空 | 1482-1486 | ✅ 第 1 项 (轨迹非空) |
| 4 | 轨迹结束时间未过期 (非零 timestamp 场景) | 1492-1504 | ✅ 第 5 项 (轨迹未过期) |
| 5 | 关节名在控制器白名单中 | 1506-1518 | ✅ 第 2 项 (关节名匹配) |
| 6 | 末点速度为零 (`allow_nonzero_velocity_at_trajectory_end`) | 1520-1533 | ⚠️ 跳过（AUBO 不适用） |
| 7 | time_from_start 单调递增 | 1535-1547 | ✅ 第 4 项 |
| 8 | 字段维度 + effort 拒绝 (逐点) | 1549-1592 | ✅ 第 3 项 (字段维度) |

> **注意**: 文档 3.2 节已修正为 **8 个独立检查块**。若将 `allow_integration_in_goal_trajectories` 分支内部的子检查（position/velocity/acceleration 各有独立验证）单独计数可达更多，但作为函数顶层逻辑块计数为 8。AUBO 采纳其中 5 项核心检查，跳过 3 项（详见 3.2 节"跳过的检查"表）。

### A.9 容差违规处理（源码中的使用位置）

| 类型 | 触发条件 | 源码行 | error_code |
|------|---------|--------|-----------|
| PATH_TOLERANCE_VIOLATED | `tolerance_violated_while_moving == true` | 349-364 | -4, `setAborted()` |
| SUCCESSFUL | `!outside_goal_tolerance` (轨迹结束且到位) | 366-383 | 0, `setSucceeded()` |
| GOAL_TOLERANCE_VIOLATED | `!outside_goal_tolerance && !within_goal_time` | 384-402 | -5, `setAborted()` |

### A.10 tolerances.hpp resolve_tolerance_source()

**源文件**: `references/ros2_controllers/joint_trajectory_controller/include/joint_trajectory_controller/tolerances.hpp:130-155`

| goal_value | 行为 | 源码行 | 文档描述 | 验证 |
|-----------|------|--------|---------|------|
| `> 0.0` | return goal_value | 142-144 | 使用 goal 中指定的值 | ✅ |
| `-1.0` (erase) | return 0.0 | 146-149 | 解析为 0.0，跳过 | ✅ |
| `< 0.0` (非 -1) | throw runtime_error | 150-152 | 抛出 `std::runtime_error("Illegal tolerance value.")` | ✅ |
| `0.0` (或未设置) | return default_value | 154 | 使用控制器默认参数 | ✅ |

### A.11 get_segment_tolerances() 两个重载

**源文件**: 同上 `:96-128` 和 `:166-289`

| 重载 | 源码行 | 用途 | 验证 |
|------|--------|------|------|
| `get_segment_tolerances(logger, params)` | 96-128 | 从 ROS 参数加载默认容差 | ✅ |
| `get_segment_tolerances(logger, default, goal, joints)` | 166-289 | 从 action goal 的 path_tolerance/goal_tolerance 覆盖 | ✅ |

第二个重载中，`goal.goal_time_tolerance` 通过 `resolve_tolerance_source()` 处理，非法值回退到 default_tolerances。

### A.12 check_state_tolerance_per_joint()

**源文件**: 同上 `:298-345`

逻辑：只检查 `state_tolerance.xxx > 0.0` 的维度。容差值为 0.0 表示该维度不检查。position/velocity/acceleration 三维度独立。

---

*附录版本: 2026-06-30 | 基于 doc/references/ 目录参考源码副本验证*
