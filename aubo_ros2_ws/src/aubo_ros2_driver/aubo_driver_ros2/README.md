# aubo_driver_ros2：MoveIt2 到真实机械臂控制完整流程与修复总结

本文档全面描述从 **MoveIt2 规划** 到 **控制真实 Aubo 机械臂** 的完整数据流、逻辑细节、调试过程中发现的所有问题与解决方法、修改原因、测试与诊断方法，以及每个修改的理论与实践依据。

---

## 一、完整数据流概览

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 1. MoveIt2 规划层                                                                 │
│    - MoveGroupInterface::plan() 规划轨迹                                          │
│    - MoveGroupInterface::execute() 转为 FollowJointTrajectory Action Goal          │
└──────────────────────────────────────────┬──────────────────────────────────────┘
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 2. aubo_ros2_trajectory_action                                                   │
│    - handleGoal/handleAccept: 接受 Goal 后一次性发布完整 JointTrajectory           │
│    - 发布到 joint_path_command (trajectory_msgs/JointTrajectory)                  │
│    - 订阅 aubo/feedback_states 等待执行反馈；watchDog 2s 无反馈则 abort               │
└──────────────────────────────────────────┬──────────────────────────────────────┘
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 3. aubo_robot_simulator_ros2 (Python)                                            │
│    - 订阅 joint_path_command                                                      │
│    - 5 次多项式插值，200Hz 固定节拍（5ms）发布 JointTrajectoryPoint                  │
│    - Wall-clock 累积误差补偿，保证发布间隔均匀                                      │
│    - 发布到 moveItController_cmd (trajectory_msgs/JointTrajectoryPoint)           │
│    - 可选：订阅 /aubo_driver/rib_status 做背压节流                                  │
└──────────────────────────────────────────┬──────────────────────────────────────┘
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 4. aubo_driver_ros2 (C++)                                                        │
│    moveItPosCallback: 订阅 moveItController_cmd → buf_queue_ (mutex)               │
│    feedToRosMotionLoop 线程 (200Hz/5ms): buf_queue_ → ros_motion_queue_ (批量)     │
│    publishWaypointToRobot 线程 (~250Hz/4ms):                                      │
│       - 实时查询 RIB (robotServiceGetRobotDiagnosisInfo)                           │
│       - tryPopWaypoint → robotServiceSetRobotPosData2Canbus                       │
└──────────────────────────────────────────┬──────────────────────────────────────┘
                                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 5. 机器人控制器                                                                   │
│    - RIB 缓冲 400 点，按 6 关节/点消费                                             │
│    - 执行关节轨迹，发布 joint_states / aubo/feedback_states                        │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 1.1 话题与消息类型

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `follow_joint_trajectory` | action | MoveIt → trajectory_action | FollowJointTrajectory Goal |
| `joint_path_command` | JointTrajectory | trajectory_action → simulator | 完整轨迹 |
| `moveItController_cmd` | JointTrajectoryPoint | simulator → driver | 200Hz 插值点 |
| `aubo/feedback_states` | FollowJointTrajectory_Feedback | 控制器 → driver → trajectory_action | 执行反馈 |
| `joint_states` | JointState | driver | 当前关节角 |
| `/aubo_driver/rib_status` | Int32MultiArray | driver | data[0]=buf_queue_.size() |
| `/aubo_driver/real_pose` | Float32MultiArray | driver | 真实位姿 |

### 1.2 驱动内部关键数据结构

| 名称 | 类型 | 含义 | 写入 | 读取 |
|------|------|------|------|------|
| `buf_queue_` | `std::queue<PlanningState>` | 来自插值节点的轨迹点 | moveItPosCallback | feedToRosMotionLoop |
| `ros_motion_queue_` | `readerwriterqueue` | 待送机的关节点（无锁） | setRobotJointsByMoveIt | tryPopWaypoint |
| `rib_buffer_size_` | `std::atomic<int>` | 控制器 RIB 缓冲量 | publishWaypointToRobot、timerCallback | feedToRosMotionLoop |
| `start_move_` | `bool` | 运动标志 | updateControlStatus、feedToRosMotionLoop | feedToRosMotionLoop |
| `buffer_size_` | `int` (400) | 预缓冲阈值 | 构造函数 | moveItPosCallback |

---

## 二、问题现象与根因定位（基于 debug 日志）

### 2.1 问题现象总结

| 现象 | 描述 | 主观/客观判定 |
|------|------|----------------|
| 连续卡顿 | 运动“一顿一顿”，可闻机械/电气噪音 | 与 ROS1 对比；轨迹执行时间明显变长 |
| 突然加速 | 某时刻机械臂明显突然变快 | time_diff 过小或超速检测触发 |
| 末端抖动 | 末端小幅高频晃动，无卡顿噪音 | 关节反馈曲线有小幅振荡 |
| 偶发卡顿 | 大部分平滑，偶发短暂停顿 | RIB 或 ros_motion_queue_ 曾耗尽 |
| 单条轨迹内停顿 | 同一条轨迹执行中数秒级停顿 | 两次 SetRobotPosData2Canbus 间隔 2～4s |
| MoveIt 报 -4 | execute() 返回 CONTROL_FAILED | action 被 abort |
| MoveIt 报 -6 | execute() 返回 TIMED_OUT | 超时或二次 goal |

### 2.2 根因与日志判定依据

调试时使用 NDJSON 写入 `.cursor/debug-*.log`，下列字段用于判定根因：

| 根因 | 日志 message | 关键 data 字段 | 判定逻辑 |
|------|--------------|----------------|----------|
| 取点间隔抖动 | feeder_stats | interval_ms, ros_motion_queue_sz, buf_queue_sz | interval_ms > 10ms ⇒ 取点与主线程竞争 |
| RIB 缓存过时 | feeder_stats | rib_buffer_size, cached_rib, current_macsz | 运动中 cached_rib 大但 RIB 已耗尽 ⇒ 缓存滞后 |
| publisher 吞吐不足 | tcp_send_stats | send_avg_ms, count, rmq_sz | 送点速率 < 200 pts/s 且 RIB 下降 ⇒ 吞吐不足 |
| TCP 延迟尖峰 | tcp_send_stats | send_max_ms | send_max_ms > 50ms ⇒ 单次 TCP 阻塞 |
| RIB 初始过冲 | tcp_send_stats | current_macsz | RIB > 400 ⇒ 缓存 0 时大量送点 |
| start_move_ 误关 | ros_motion_queue_starvation | starvation_count, buf_queue_sz | buf_queue_sz > 0 却 starvation ⇒ feeder 已停 |
| 数据竞争 | 代码审查 | — | timerCallback 与 publishWaypointToRobot 同时写 robot_diagnosis_info_ |

### 2.3 关键指标与阈值

| 指标 | 正常范围 | 异常判定 |
|------|----------|----------|
| 取点间隔 | ~2ms (500Hz) | 12～80ms 尖峰 |
| 插值发布间隔 | ~5ms (200Hz) | max > 20ms 或 4s 级尖峰 |
| RIB | 几十～400 | 长期 0 或个位数 |
| buffer_size_ | 400 (ROS1 一致) | 5、60 等过小值 |
| TCP 送点 | 平均 ms～十数 ms | 尖峰 > 50ms |

---

## 三、问题与解决方法详表

### 3.1 连续卡顿与卡顿噪音（Fix1）

**根因**：`setRobotJointsByMoveIt()` 与主线程 `spin_some()` 同线程执行。回调密集时主线程被占用，取点间隔出现 12～80ms 尖峰（理想 ~2ms），导致控制器执行不连续。

**依据**：debug 日志中 `interval_ms` 多次 > 10ms；对比 ROS1 单线程下取点节奏稳定。

**解决**：
- 新增独立线程 `feedToRosMotionLoop`（200Hz，5ms 周期）
- `buf_queue_` 使用 `buf_queue_mutex_` 保护
- `rib_buffer_size_` 改为 `std::atomic<int>`

**修改文件**：`aubo_driver_ros2.cpp`，`aubo_driver.h`

---

### 3.2 插值发布不均匀（Fix2）

**根因**：Python 插值使用浮点步数，`n_steps` 在 19/20 之间抖动；无 wall-clock 补偿，累积误差导致发布间隔漂移。

**依据**：motion_log 中 segment 间 ts 间隔不均匀；理论 5ms 与实际 perf_counter 差增大。

**解决**：
- `n_steps = max(1, round(T / update_duration_sec))` 整数步数
- 每 segment 记录 `_timing_wall_start`，每步 `_wait = _step_idx * update_duration_sec - (perf_counter - _timing_wall_start)`，sleep(_wait) 补偿

**修改文件**：`aubo_robot_simulator_ros2/aubo_robot_simulator_node.py`

---

### 3.3 start_move_ 误关（Fix3、Fix9）

**根因**：`buf_queue_` 短暂为空即关闭 `start_move_`，导致 feeder 停止、ros_motion_queue 断供。

**依据**：`buf_queue_sz > 0` 却出现 `ros_motion_queue_starvation`；`start_move_` 在轨迹未结束时变 false。

**解决**：
- 路径 B 条件改为 `buf_queue_` 与 `ros_motion_queue_` 双空才关
- 超时：`empty_streak > 500`（500×5ms ≈ 2.5s）才关闭 `start_move_`

**修改文件**：`aubo_driver_ros2.cpp` feedToRosMotionLoop

---

### 3.4 buffer_size_ 过小（Fix4、Fix7、Fix10）

**根因**：曾将 `buffer_size_` 设为 5、60 等，预缓冲不足，`start_move_` 频繁切换。

**依据**：start_move_ 与 rib 日志显示频繁开关；RIB 易耗尽。

**解决**：`buffer_size_ = 400`，与 ROS1 一致，提供约 10s 缓冲深度。

**修改文件**：`aubo_driver_ros2.cpp` 构造函数

---

### 3.5 RIB 缓存过时与数据竞争（Fix8、Fix12）

**根因**：
1. 用 50Hz timerCallback 更新 `rib_buffer_size_`，publisher 使用缓存做流控，实际 RIB 已耗尽却误判为“已满”
2. `rs.robot_diagnosis_info_` 被 timerCallback 与 publishWaypointToRobot 并发读写，无保护

**依据**：运动中 `cached_rib` 大但 RIB 实际耗尽；对比 ROS1 在 publishWaypointToRobot 热循环内实时查 RIB。

**解决**：
- 在 `publishWaypointToRobot` 每次循环内调用 `robotServiceGetRobotDiagnosisInfo`，获取真实 RIB
- 使用局部变量 `pub_diag`，不再共享 `rs.robot_diagnosis_info_`

**修改文件**：`aubo_driver_ros2.cpp` publishWaypointToRobot

---

### 3.6 单条轨迹内异常停顿（Fix13）

**根因**：
1. RIB=0 时 sleep 10ms，多轮叠加导致送机间隔达数秒
2. feedToRosMotionLoop 每 5ms 仅取 1～2 点，送机线程在 RIB=0 时一次可送 ~66 批点，瞬间抽空 `ros_motion_queue_`，补点需数百次 5ms，形成长 gap

**依据**：driver 侧记录两次 SetRobotPosData2Canbus 墙钟间隔 2～4s 且 rib=0；simulator 段间间隔 <1ms，排除插值侧。

**解决**：
- RIB=0 时 `sleep_for(10ms)` → `sleep_for(1ms)`
- feedToRosMotionLoop 每周期 `while(setRobotJointsByMoveIt() && batch < 150)` 批量转点，单周期最多 150 点

**修改文件**：`aubo_driver_ros2.cpp` publishWaypointToRobot、feedToRosMotionLoop

注：当前代码中 feedToRosMotionLoop 为 `feed_count` 控制（rib<200 时 3 次），批量 150 的上限见历史迭代；若仍有停顿可考虑恢复 150 上限。

---

### 3.7 ROS1 与 ROS2 启动延迟差异（路径 A/B）

**现象**：同一 buffer_size_=400，ROS1 约 100ms 开动，纯 ROS2 约 2s。

**根因**：
- 路径 A：`buf_queue_.size() > 400` 时置位，401 点 @ 200Hz ≈ 2s
- 路径 B：`data_count_ == MAXALLOWEDDELAY` 且 `bq >= 50` 时置位，理论 50×2ms=100ms
- ROS2 多线程下路径 B 的 50 次与数据到达错位，路径 B 未先触发

**解决**：
- `data_count_` 改为 `std::atomic<int>`
- moveItPosCallback 首点入队时 `data_count_.store(0)`
- updateControlStatus 移到 driver 节点的 500Hz wall timer，由 driver executor 执行

**修改文件**：`aubo_driver_ros2.cpp`，`aubo_driver.h`，driver_node_ros2

---

### 3.8 MoveIt 错误码 -4 / -6（move_to_pose 等）

**-4 (CONTROL_FAILED)**：FollowJointTrajectory action 被 abort。

| abort reason | 含义 | 排查方向 |
|--------------|------|----------|
| client_cancel | MoveIt 或客户端主动 cancel | 起点校验超容差；执行超时 |
| watchdog_no_feedback_2s | 2s 内无 aubo/feedback_states | 反馈话题/桥接 |
| trajectory_execution_event_stop | 收到 stop 事件 | 谁在发 trajectory_execution_event |
| new_goal_received | 新 goal 到达时旧 goal 未完成 | 重复/并发请求 |

**-6 (TIMED_OUT)**：允许时间内未收到 SUCCESS，MoveIt 主动取消。常见诱因：**同一次操作触发了两次 ExecuteTrajectory**，第二个 goal 一直未完成。

**诊断**：查看 aubo_ros2_trajectory_action 日志中的 `abort, reason=XXX`。

---

## 四、如何编写调试代码判断问题

### 4.1 日志约定

- **格式**：NDJSON，每行一条 JSON
- **路径**：如 `/home/mu/IVG2.0/.cursor/debug-<session>.log`
- **字段**：`timestamp`、`message`、`data`（对象）

### 4.2 C++ 驱动打点位置

| 位置 | 目的 |
|------|------|
| feedToRosMotionLoop 循环内 | feeder_stats：interval_ms, buf_queue_sz, rib, start_move_ |
| publishWaypointToRobot 内 | tcp_send_stats：send_ms, current_macsz, batch |
| RIB 不足且 ros_motion_queue_ 为空 | ros_motion_queue_starvation |

### 4.3 Python 插值打点位置

| 位置 | 目的 |
|------|------|
| _publish_cmd 内 | publish_interval_stats：min/max/avg_ms |
| _move_to throttle 分支 | throttle_event |
| _motion_worker segment 结束 | segment_timing |

### 4.4 示例代码（C++）

```cpp
if (dbg_count % 1000 == 0) {
    size_t rmq = ros_motion_queue_.size_approx();
    size_t bq = 0;
    { std::lock_guard<std::mutex> lk(buf_queue_mutex_); bq = buf_queue_.size(); }
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"timestamp\":%lld,\"message\":\"feeder_stats\",\"data\":{"
        "\"ros_motion_queue_sz\":%zu,\"buf_queue_sz\":%zu,\"rib\":%d}}\n",
        (long long)ts, rmq, bq, (int)rib_buffer_size_.load());
    FILE* f = fopen("/home/mu/IVG2.0/.cursor/debug.log", "a");
    if (f) { fputs(buf, f); fclose(f); }
}
```

### 4.5 注意事项

- 仅追加（append），不截断
- 控制频率（如每 200～1000 次一条）
- C++ 中先持锁读变量，释放锁后再写文件
- 用 `#region agent log` 包裹便于移除

---

## 五、迭代修复历程总表

| 迭代 | 主要修改 | 结果 |
|------|----------|------|
| Fix1 | 分离 feedToRosMotionLoop 线程 + buf_queue_mutex_ + rib_buffer_size_ atomic | 连续卡顿消除 |
| Fix2 | 整数步数 + wall-clock 补偿 | 插值间隔稳定 |
| Fix3 | start_move_=false 条件改为双空 | 减少误关 |
| Fix4 | buffer_size_=5 | 严重抖动（回退）|
| Fix5 | 移除 setRobotJointsByMoveIt 中 start_move_=false | start_move_ 稳定 |
| Fix6 | RIB 查询 100ms + publisher 2ms | RIB 先冲后空（回退）|
| Fix7 | buffer_size_=60 + 三重条件 | RIB 查询阻塞（回退）|
| Fix8 | RIB 缓存 50Hz + publisher 2ms | RIB 耗尽（回退）|
| Fix9 | 批量排空 + 1s 超时 | 数据流通 |
| Fix10 | buffer_size_=200 | 仍偶发卡顿 |
| Fix11 | local_sent + min_batch=5 | RIB 过冲缓解 |
| Fix12 | 实时 RIB 查询 + buffer_size_=400 + 4ms + 局部 pub_diag | 连续/偶发卡顿解决 |
| Fix13 | RIB=0 时 1ms sleep + 批量 150 点 | 单条轨迹内停顿消除 |

---

## 六、线程架构

| 线程 | 频率 | 职责 |
|------|------|------|
| 主线程 / driver 主循环 | — | spin_some、updateControlStatus（由 timer 触发）|
| feedToRosMotionLoop | 200Hz (5ms) | buf_queue_ → ros_motion_queue_ |
| publishWaypointToRobot | ~250Hz (4ms) | 实时查 RIB、送点；RIB<200 时循环末 1ms |
| timerCallback | 50Hz | 状态查询、robot_status/rib_status 发布 |
| publishJointStateAndFeedbackLoop | 50Hz | joint_states、feedback_states |

---

## 七、关键经验

1. **勿“优化”已验证的硬件协议设计**：ROS1 在热循环内实时查 RIB 看似低效，但保证流控准确；缓存引入滞后。
2. **预缓冲与 ROS1 一致**：`buffer_size_=400`。
3. **数据竞争即使“看似只读”也有害**：用局部变量避免多线程写共享结构。
4. **feeder 批量排空且单周期有上限**：避免补点过慢或单周期过长。
5. **start_move_ 关闭需容忍度**：短暂空队列不代表运动结束。
6. **RIB=0 时不宜长 sleep**：10ms 会叠加成数秒，改为 1ms。

---

## 八、相关文档

- `doc/PORTING_MOTION_FIX.md`：移植与运动修复详细说明
- `../demo_driver/docs/MOVE_TO_POSE_AND_ERROR_MINUS4_ANALYSIS.md`：move_to_pose 与 -4/-6 分析
- `../../../docs/motion_jitter_analysis.md`：抖动/卡顿分析（工作区根 docs）

---

*文档版本：与当前 aubo_driver_ros2、aubo_robot_simulator_ros2、aubo_ros2_trajectory_action 代码一致。*
