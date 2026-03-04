# 机械臂规划→执行数据链与抖动/卡顿分析

## 1. 数据流总览

```
MoveIt (规划)
    ↓ FollowJointTrajectory Goal
aubo_ros2_trajectory_action
    ↓ 一次发布完整轨迹 JointTrajectory
joint_path_command (topic)
    ↓
aubo_robot_simulator_ros2 (200Hz 插值，5 次多项式)
    ↓ 按节拍发布 JointTrajectoryPoint，且当 rib > minimum_buffer_size 时阻塞
moveItController_cmd (topic)
    ↓
aubo_driver_ros2::moveItPosCallback
    ↓ 入队
buf_queue_
    ↓ updateControlStatus (500Hz)，仅当 rib_buffer_size_ < 300 时
setRobotJointsByMoveIt → ros_motion_queue_
    ↓ publishWaypointToRobot 线程 (每 4ms)
tryPopWaypoint → robotServiceSetRobotPosData2Canbus
    ↓
机器人控制器 (缓冲 400，按 6 为单位消费)
```

- **rib 含义**：driver 内 `rib_buffer_size_` = 控制器当前缓冲量 `macTargetPosDataSize`；对 simulator 发布的 `rib_status_.data[0]` = **buf_queue_.size()**（与 ROS1 语义一致，simulator 用其做背压）。

## 2. 可能引起抖动/卡顿的环节

### 2.1 规划与轨迹发布（trajectory_action）

- 一次发布整条轨迹到 `joint_path_command`，无节拍抖动。
- 笛卡尔重采样、速度缩放与 ROS1 对齐，非主要嫌疑。

### 2.2 插值节点（aubo_robot_simulator_ros2）

- **200Hz 固定节拍**（约 5ms）做 5 次多项式插值并发布 `moveItController_cmd`。
- **背压阻塞**：`_move_to()` 中当 `rib_buffer_size > minimum_buffer_size`（2000）时 `time.sleep(throttle_sleep)`，即当 driver 的 **buf_queue_.size()** 超过 2000 时，simulator 会停发。
- **影响**：若 driver 端 `buf_queue_` 经常接近或超过 2000，simulator 会周期性阻塞→放开，导致 **moveItController_cmd 呈“成批到达”**，下游看起来像突发+间歇，易表现为卡顿/抖动。
- motion_log 对比：移植后 segment 间 ts 间隔约 2–3ms，移植前约 0–2ms，与“背压更常触发”一致。

### 2.3 Driver 主线程（updateControlStatus + spin_some）

- **取点条件**：`start_move_ && rib_buffer_size_ < MINIMUM_BUFFER_SIZE(300)` 时才从 `buf_queue_` 取一点放入 `ros_motion_queue_`。
- **rib_buffer_size_ 跨线程**：由 `publishWaypointToRobot` 线程写（约每 4ms 从控制器诊断更新），由主线程读，**无原子/互斥**，存在数据竞争与可见性延迟。
- **影响**：主线程可能长期看到“过时”的 rib（如一直认为已满 400 或一直为 0），导致取点节奏不均匀：要么长时间不取、要么连续猛取，进而 **buf_queue_ 与 ros_motion_queue_ 的流量都呈突发**，与卡顿/抖动相符。

### 2.4 送机线程（publishWaypointToRobot）

- 每 **4ms** 轮询，当 `current_macsz < 400` 且 `ros_motion_queue_` 非空时，按 `ceil((400 - current_macsz) / 6)` 一批发送。
- 与 ROS1 一致，单次可送较多点（最多约 66 个），属设计如此；若上游（取点节奏）已因 2.3 呈突发，这里会把这些突发原样送给控制器，放大观感。

### 2.5 控制器与反馈

- 控制器缓冲 400，driver 以 50Hz 发布 joint_states/feedback；若送点节奏不均匀，实际执行会呈现“一段一段”的加减速，听起来/看起来像卡顿或“卡顿声”。

## 3. 根因归纳

| 假设 | 描述 | 证据/逻辑 |
|------|------|-----------|
| **H_rib_race** | rib_buffer_size_ 非原子，主线程读到的值滞后或不稳定 | 写线程：publishWaypointToRobot；读线程：updateControlStatus；无同步 → 取点节奏不稳 |
| **H_simulator_backpressure** | simulator 因 rib（buf_queue_.size()）> 2000 频繁阻塞 | motion_log 移植后 segment 间隔略大；buf_queue_ 易因 H_rib_race 堆积 |
| **H_batch_send** | 送机单批点数多，放大上游突发 | 需结合 H_rib 日志看送批大小与时间间隔是否成簇 |

## 4. 已做/建议修改

1. **rib_buffer_size_ 改为 std::atomic<int>**（已实现，`aubo_driver.h` + `aubo_driver_ros2.cpp`）  
   - 消除跨线程读写竞争：`publishWaypointToRobot` 写、`updateControlStatus` 读，主线程始终看到送机线程最近一次更新的控制器缓冲量，取点条件更稳定，减少“长时间不取/突然猛取”。

2. **保留现有调试埋点**（H_batch / H_rib / timer/feedback 等）  
   - 复现后根据 `debug-624408.log` 再确认：pop 的 interval_ms、rib 与 buf_queue 关系、send_batch 的 batch_size 与时间分布。

3. **若仍卡顿，可进一步**  
   - 在 simulator 侧对 `minimum_buffer_size` 或 throttle 策略做小幅放宽或平滑（需结合日志确认 rib 是否经常 >2000）。  
   - 视日志决定是否对单次送机批大小做上限（仅在有明确“成批卡顿”证据时考虑）。

## 5. 复现与验证

- 复现步骤见每次回复末尾的 `<reproduction_steps>`。  
- 验证：对比修改前后 `debug-624408.log` 中 H_rib 的 `interval_ms`、`rib`、`buf_queue` 是否更平稳，以及 motion_log 的 segment 间隔是否接近移植前。
