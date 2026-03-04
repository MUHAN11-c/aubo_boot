# aubo_driver_ros2 移植与运动修复总结（合一文档）

本文档合并原有多份总结，统一说明：从 ROS1 `aubo_driver` 移植到 ROS2 `aubo_driver_ros2` 及插值节点 `aubo_robot_simulator_ros2` 的过程中，为消除**连续卡顿**、**卡顿噪音**、**末端抖动**及**单条轨迹内异常停顿**所做的问题定位、迭代修复与最终方案；并含调试日志约定与插值节点要点，便于后续维护与参考。

---

## 一、问题描述

### 1.1 现象

- **ROS1（移植前）**：机械臂轨迹运动平滑，无异常噪音。
- **ROS2（移植后、修复前）**：
  1. 运动出现**连续卡顿**和**卡顿噪音**，末端偶发抖动
  2. 偶发**突然加速**
  3. 运动过程中**末端小幅度抖动**
  4. **单条轨迹内异常停顿**：同一条轨迹执行过程中出现明显停顿（数秒级），RIB=0 时送机间隔过长、取点线程补点不足

### 1.2 约束

- 与 ROS1 行为对齐：定时器频率、启动延迟、送点逻辑、发布频率、缓冲区管理等均需与 ROS1 一致。
- 根因需基于**运行时日志**确认，而非仅凭代码猜测。

---

## 二、数据流与流程

### 2.1 规划到执行的整体流程

```
MoveIt (规划)
    ↓ FollowJointTrajectory Goal
aubo_ros2_trajectory_action
    ↓ 一次发布完整轨迹 JointTrajectory
joint_path_command (topic)
    ↓
aubo_robot_simulator_ros2 (200Hz 五次多项式插值)
    ↓ 发布 JointTrajectoryPoint (wall-clock 累积误差补偿定时)
moveItController_cmd (topic)
    ↓ moveItPosCallback
aubo_driver_ros2
    ├─ buf_queue_ (入队，mutex 保护)
    ├─ feedToRosMotionLoop (500Hz，批量排空 buf_queue_ → ros_motion_queue_)
    └─ publishWaypointToRobot 线程
           ↓ 实时查询 RIB → robotServiceSetRobotPosData2Canbus
       机器人控制器 (RIB 缓冲 400，按 6 为单位消费)
```

### 2.2 驱动内部关键量

| 名称 | 含义 | 写入处 | 读取处 |
|------|------|--------|--------|
| `buf_queue_` | 来自插值节点的轨迹点队列 | moveItPosCallback | feedToRosMotionLoop、updateControlStatus、timerCallback |
| `rib_buffer_size_` | 控制器 RIB 当前缓冲量（`std::atomic<int>`）| publishWaypointToRobot（实时查询）、timerCallback | feedToRosMotionLoop |
| `ros_motion_queue_` | 待送机的关节点无锁队列 | setRobotJointsByMoveIt | publishWaypointToRobot（tryPopWaypoint） |
| `start_move_` | 运动标志 | updateControlStatus、feedToRosMotionLoop | feedToRosMotionLoop、publishWaypointToRobot |
| `buffer_size_` | 预缓冲阈值（构造函数中设为 400）| 构造函数 | moveItPosCallback |

---

## 三、判断问题的依据

本节记录**如何判定“有问题”**以及**各根因的判定依据**（指标、阈值、日志字段），便于复现验证或在新环境中做同类诊断。

### 3.1 现象是否成立的判定

| 现象 | 判定方式 | 备注 |
|------|----------|------|
| **连续卡顿** | 主观：运动“一顿一顿”，可闻机械/电气噪音；客观：轨迹执行时间明显长于规划时间或出现多次短暂停顿 | 与 ROS1 同场景对比 |
| **突然加速** | 主观：某时刻机械臂明显突然变快；客观：`time_diff` 过小或超速检测触发（`overspeed_triggered`）| 与插值理论步长 5ms 对比 |
| **末端抖动** | 主观：末端在运动过程中小幅度高频晃动，无卡顿噪音；客观：关节反馈/位置曲线有小幅振荡 | 区别于“卡顿”（卡顿伴随噪音与停顿）|
| **偶发卡顿** | 主观：运动大部分平滑，偶发一次短暂停顿后继续；客观：RIB 或 `ros_motion_queue_` 曾耗尽（见下）| 需多次复现取日志 |

### 3.2 关键指标与阈值

| 指标 | 正常/期望范围 | 异常判定 | 来源 |
|------|----------------|----------|------|
| 取点间隔 `interval_ms` | 稳定约 2ms（500Hz）| 出现 12ms～80ms 甚至更高尖峰 | feeder 循环内打点时间戳差 |
| 插值发布间隔 | 稳定约 5ms（200Hz）| `max_ms` > 20ms 或出现 4s 级尖峰 | Python `_publish_cmd` 相邻两次 `perf_counter` 差 |
| RIB 缓冲量 `current_macsz` / `cached_rib` | 运动中维持在几十～400，不应长期为 0 | 运动过程中降至 0 或个位数并持续 | `robotServiceGetRobotDiagnosisInfo` 或 timer 更新值 |
| 预缓冲 `buffer_size_` | ROS1 为 400，对应约 10s 数据量 | 过小（如 5、60）导致 start_move_ 频繁开关或 RIB 易耗尽 | 构造函数/配置 |
| TCP 送点单次耗时 | 平均数 ms～十数 ms | 尖峰 > 50ms（如 225ms）可导致本周期内无法再送点，RIB 被快速消耗 | `robotServiceSetRobotPosData2Canbus` 前后计时 |
| `ros_motion_queue_` 与 `buf_queue_` | 运动中有数据流动；`buf_queue_` 先积压后下降 | `buf_queue_` 有数据但 `ros_motion_queue_` 长期为 0（数据卡在 feeder）| 两队列的 size 采样 |
| `start_move_` | 单次轨迹内保持 true 直至轨迹结束 | 轨迹未结束即变 false（误关）| 与 feeder/starvation 日志对照 |

### 3.3 日志证据与根因对应关系

调试时曾向 `.cursor/debug-671533.log` 写入 NDJSON，下列 message 与 data 字段即**判断各根因的依据**：

| 根因 | 日志 message | 关键 data 字段 | 判定逻辑 |
|------|---------------|----------------|----------|
| 取点间隔抖动 | `feeder_stats` / 取点间隔统计 | `interval_ms`、`ros_motion_queue_sz`、`buf_queue_sz` | `interval_ms` 多次 > 10ms ⇒ 取点与主线程竞争 |
| RIB 缓存过时 | `feeder_stats`、`ros_motion_queue_starvation` | `rib_buffer_size`、`cached_rib`、`current_macsz`、`buf_queue_sz` | 运动过程中 `cached_rib` 仍大但实际 RIB 已耗尽 ⇒ 缓存滞后 |
| publisher 吞吐不足 | `tcp_send_stats`、队列 size | `send_avg_ms`、`count`、`rmq_sz`、RIB 变化 | 送点速率 < 200 pts/s 且 RIB 持续下降 ⇒ 吞吐不足 |
| TCP 延迟尖峰 | `tcp_send_stats` | `send_max_ms`、`max_batch` | `send_max_ms` > 50ms（如 225ms）⇒ 单次 TCP 阻塞导致本周期少送点 |
| RIB 初始过冲 | `tcp_send_stats` 或 RIB 采样 | `current_macsz`、发送批大小 | 运动刚开始时 RIB > 400（如 870）⇒ 用缓存 0 大量送点导致过冲 |
| start_move_ 误关 | `ros_motion_queue_starvation` | `starvation_count`、`buf_queue_sz` | `buf_queue_sz` > 0 但出现 starvation ⇒ feeder 已停，start_move_ 被误关 |
| 数据竞争 | 代码审查（两处写 `robot_diagnosis_info_`）| — | `timerCallback` 与 `publishWaypointToRobot` 同时写同一结构体 ⇒ 数据竞争 |

### 3.4 与 ROS1 行为对齐的判定

以下以 **ROS1 源码与脚本** 为基准，作为“正确行为”的判定依据：

| 项目 | 依据来源 | 判定方式 |
|------|----------|----------|
| RIB 在送点循环内实时查询 | ROS1 `aubo_driver.cpp` 中 `publishWaypointToRobot` | 循环内是否调用 `robotServiceGetRobotDiagnosisInfo` |
| buffer_size_ = 400 | ROS1 构造函数/头文件 | 与 ROS1 常量对比 |
| 送点循环 4ms 间隔 | ROS1 `sleep_for(4ms)` | 与 ROS1 一致 |
| RIB=0 时多等 10ms | ROS1 同一函数内分支 | 与 ROS1 一致 |
| 启动与通信参数 | `start_IVG_before_migration.sh` 及 ROS1 launch | 端口、频率、缓冲区相关环境/参数一致 |

### 3.5 如何编写日志代码获得判断依据

为复现问题或在新环境中做同类诊断，可在下述位置插入**最小化**的写文件日志，输出 NDJSON 到同一文件，便于后续用脚本/工具分析。

#### 日志输出约定

| 项目 | 说明 |
|------|------|
| **文件路径** | 例如工作区内固定路径：`/home/mu/IVG/.cursor/debug-671533.log`（可按会话改文件名）。 |
| **格式** | 每行一条 **NDJSON**（一行一个完整 JSON 对象），便于按行解析。 |
| **建议字段** | `timestamp`（毫秒）、`message`（事件类型）、`data`（对象，放具体指标）。可选：`runId`、`hypothesisId` 便于区分多次运行。 |

#### C++ 驱动（aubo_driver_ros2）

**位置与目的：**

| 位置 | 文件 | 目的 |
|------|------|------|
| `feedToRosMotionLoop()` 循环内 | `src/aubo_driver_ros2.cpp` | 统计取点节奏、队列长度、`start_move_`、`empty_streak`（对应 3.2 的取点间隔、3.3 的 `feeder_stats`）。 |
| `publishWaypointToRobot()` 循环内 | 同上 | 在每次 `robotServiceGetRobotDiagnosisInfo` 前后计时→`query_*_ms`；在 `robotServiceSetRobotPosData2Canbus` 前后计时→`send_*_ms`；可选记录 `current_macsz`、`rmq_sz`（对应 `tcp_send_stats`）。 |
| `publishWaypointToRobot()` 分支：RIB 不足且 `ros_motion_queue_` 为空 | 同上 | 记录 `starvation` 次数、`current_macsz`、`buf_queue_sz`（对应 `ros_motion_queue_starvation`）。 |
| `tryPopWaypoint` / `setRobotJointsByMoveIt` 内（可选）| 同上 | 超速时记录关节、速度、`time_step`（对应 `overspeed_triggered`）；或 `time_diff` 分布（对应 `time_diff_stats`）。 |

**C++ 写 NDJSON 示例（追加、单行）：**

```cpp
// 在需要打点的位置：先准备好 data 所需数值，再 snprintf 一整行 JSON，最后 append 写文件。
#include <cstdio>
#include <chrono>

// 示例：每 N 次循环写一条 feeder_stats
if (dbg_log_count % 1000 == 0) {
    size_t rmq_sz = ros_motion_queue_.size_approx();
    size_t bq_sz = 0;
    { std::lock_guard<std::mutex> lk(buf_queue_mutex_); bq_sz = buf_queue_.size(); }
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"timestamp\":%lld,\"message\":\"feeder_stats\",\"data\":{"
        "\"ros_motion_queue_sz\":%zu,\"buf_queue_sz\":%zu,\"rib_buffer_size\":%d,\"start_move\":%d}}\n",
        (long long)ts, rmq_sz, bq_sz, (int)rib_buffer_size_.load(), (int)start_move_);
    FILE* f = fopen("/home/mu/IVG/.cursor/debug-671533.log", "a");
    if (f) { fputs(buf, f); fclose(f); }
}
```

**TCP 耗时示例（在 `publishWaypointToRobot` 内）：**

```cpp
auto t0 = std::chrono::steady_clock::now();
robot_mac_size_service_.robotServiceSetRobotPosData2Canbus(wayPointVector);
double tcp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
// 与其它字段一起凑成一条 tcp_send_stats 写入同一 log 文件（可每 200 次汇总一次 avg/max）
```

#### Python 插值节点（aubo_robot_simulator_ros2）

**位置与目的：**

| 位置 | 文件 | 目的 |
|------|------|------|
| `_publish_cmd()` 内，每次 publish 前 | `aubo_robot_simulator_ros2/aubo_robot_simulator_node.py` | 用 `time.perf_counter()` 计算与上次发布的时间差，收集到列表，每 200 次输出一条 `publish_interval_stats`（min/max/avg_ms），对应 3.2 的插值发布间隔。 |
| `_move_to()` 内，throttle 分支（等待 RIB 后）| 同上 | 记录 `throttle_event`：block_count、rib_buffer_size_at_start、total_sleep_ms 等。 |
| `_motion_worker()` 内，每个 segment 结束后 | 同上 | 记录 `segment_timing`：T_sec、actual_wall_ms、n_steps 等，用于对比理论时间与墙钟时间。 |

**Python 写 NDJSON 示例：**

```python
import json
import time

_LOG_PATH = "/home/mu/IVG/.cursor/debug-671533.log"

def _dbg(msg: str, data: dict) -> None:
    entry = {"timestamp": int(time.time() * 1000), "message": msg, "data": data}
    try:
        with open(_LOG_PATH, "a") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except Exception:
        pass

# 在 _publish_cmd 中，每 200 次发布写一条
# _dbg("publish_interval_stats", {"min_ms": ..., "max_ms": ..., "avg_ms": ..., "count": 200})
```

#### 注意事项

- **只追加、不截断**：统一用 **append 模式**（C++ 的 `"a"`，Python 的 `"a"`），避免多进程/多线程写同一文件时互相覆盖；若需新一次运行清空，在**启动前**由脚本或人工删除该 log 文件。
- **控制频率**：feeder / publisher 循环频率高，不要每次循环都写；可每 200～1000 次或每 100ms 写一条汇总，减少 I/O 对时序的影响。
- **线程安全**：C++ 中写文件前不要长时间持锁；可先持锁读出要记录的变量，释放锁后再 snprintf + fopen/fputs/fclose。
- **不依赖 ROS 日志**：上述方式为**直接写文件**，不依赖 `RCLCPP_INFO` / `rclpy.logging`，便于在未改 ROS 日志级别的情况下采集数据。

---

## 四、问题根因（基于运行时日志逐步定位）

### 4.1 第一阶段：连续卡顿与卡顿噪音

**根因**：取点函数 `setRobotJointsByMoveIt()` 与主线程 `spin_some()` 同线程执行。当订阅回调大量到达时，主线程被回调占用，取点间隔出现 12～80ms 尖峰（理想约 2ms）。送入 `ros_motion_queue_` 的节奏不均匀，导致控制器执行不连续。

**解决**：将取点逻辑移至独立的 `feedToRosMotionLoop` 线程（500Hz），用 `sleep_until` 稳定周期。

### 4.2 第二阶段：末端抖动与偶发卡顿

经过第一阶段修复后，连续卡顿消除，但仍存在偶发卡顿和末端运动中小幅度抖动。通过 12 轮迭代调试，逐步定位到以下根因：

| 根因 | 日志证据 | 影响 |
|------|----------|------|
| **RIB 缓存值过时** | `timerCallback`（50Hz）更新 `rib_buffer_size_`，publisher 使用缓存值做流控，实际 RIB 已耗尽但 publisher 误判为"已满" | 偶发卡顿 |
| **publisher 吞吐不足** | 去掉热循环中实时 RIB 查询后，publisher 有效发送率仅 ~154 pts/s，低于机器人消费 200 pts/s | RIB 耗尽导致卡顿 |
| **TCP 延迟尖峰** | `robotServiceSetRobotPosData2Canbus` 平均 12ms 但尖峰达 225ms | 送点中断导致抖动 |
| **RIB 初始过冲** | 缓存值为 0 时一次性灌入过多点，RIB 冲到 870（超过 400 上限 2.175 倍）| 运动不平滑 |
| **`start_move_` 误关** | `buf_queue_` 短暂为空即关闭 `start_move_`，导致 feeder 停止、ros_motion_queue 断供 | 数据饥饿 |
| **数据竞争** | `rs.robot_diagnosis_info_` 被 `timerCallback` 和 `publishWaypointToRobot` 并发读写，无保护 | 不可预测行为 |

### 4.3 关键发现：对比 ROS1 原始代码

对比 ROS1 `aubo_driver.cpp` 发现 ROS2 移植中偏离了 ROS1 的关键设计：

| 项目 | ROS1 原始 | ROS2 移植后（修复前） |
|------|-----------|----------------------|
| RIB 查询方式 | 在 `publishWaypointToRobot` 热循环中**实时调用** `robotServiceGetRobotDiagnosisInfo` | 用 50Hz `timerCallback` 缓存，publisher 读取缓存值 |
| `buffer_size_` | 400 | 曾低至 5～60 |
| publisher 循环间隔 | 4ms（250Hz）| 曾改为 2ms |
| RIB=0 处理 | 额外 sleep 10ms | 无 |
| 流控基准 | 实时 `current_macsz` | 缓存 `rib_buffer_size_` + 估算值 |

**结论**：ROS1 的实时 RIB 查询虽然引入一次 TCP 往返（~2-12ms），但确保了流控的**准确性**——publisher 始终基于真实 RIB 大小决定送多少点。ROS2 移植时为了"优化"而引入的缓存方案，反而破坏了流控精度。

---

## 五、最终解决方案

### 5.1 核心原则

**忠实还原 ROS1 的通信架构**，不做"优化"偏离。

### 5.2 aubo_robot_simulator_ros2（Python 插值节点）

| 修改点 | 说明 |
|--------|------|
| **整数步数控制** | 使用 `n_steps = max(1, round(T / update_duration_sec))` 代替浮点循环，避免 num_steps 在 19/20 之间抖动 |
| **Wall-clock 累积误差补偿** | 记录 segment 起始 `perf_counter()`，每步计算 `_expected_elapsed`，sleep 差值，保证 200Hz 均匀发布 |

### 5.3 aubo_driver_ros2（C++ 驱动核心）

#### 构造函数

- `buffer_size_` 设为 **400**（与 ROS1 一致），确保足够的预缓冲

#### feedToRosMotionLoop()（200Hz 取点线程，周期 5ms）

- **批量排空**：每次循环用 `while(setRobotJointsByMoveIt() && batch < max_batch_per_cycle)` 将 `buf_queue_` 中最多 **150 点**转入 `ros_motion_queue_`，避免 RIB=0 时送机线程一次抽空 `ros_motion_queue_` 后长时间补不上导致单条轨迹内停顿（Fix13）
- **超时关闭 `start_move_`**：当 `ros_motion_queue_` 持续为空超过 500 次循环（~2.5 秒）才关闭 `start_move_`，防止短暂空队列导致误关

#### setRobotJointsByMoveIt()

- 返回 `bool`：`buf_queue_` 为空时返回 `false`，支持上层批量排空
- 不再负责 `start_move_` 状态管理

#### publishWaypointToRobot()（送机线程，与 ROS1 对齐）

```cpp
while(rclcpp::ok()) {
    // 1. 实时查询 RIB 大小（TCP 调用）
    aubo_robot_namespace::RobotDiagnosis pub_diag;  // 局部变量，避免数据竞争
    if(0 == robot_mac_size_service_.robotServiceGetRobotDiagnosisInfo(pub_diag)) {
        rib_buffer_size_ = pub_diag.macTargetPosDataSize;
        current_macsz = pub_diag.macTargetPosDataSize;
        if(current_macsz == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));   // RIB=0 时 1ms，减少单条轨迹内停顿（原 10ms 会叠加成数秒间隔，Fix13）
    }
    // 2. 按 RIB 空间送点
    if(current_macsz < expect_macsz && 0 != ros_motion_queue_.size_approx()) {
        cnt = ceil((expect_macsz - current_macsz) / 6.0);
        wayPointVector = tryPopWaypoint(cnt);
        if(!wayPointVector.empty())
            robot_mac_size_service_.robotServiceSetRobotPosData2Canbus(wayPointVector);
        wayPointVector.clear();
    }
    // 3. 4ms 间隔（~250Hz，与 ROS1 一致）
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
}
```

关键设计要点：
1. **实时 RIB 查询**：每次循环调用 `robotServiceGetRobotDiagnosisInfo` 获取真实缓冲区大小，确保流控精确
2. **局部 `pub_diag`**：避免与 `timerCallback` 共享 `rs.robot_diagnosis_info_` 的数据竞争
3. **4ms 循环间隔**：与 ROS1 一致的 250Hz 发送频率
4. **RIB=0 时 1ms 等待**：修复单条轨迹内停顿时由 10ms 改为 1ms，避免 RIB 持续为 0 时叠加成数秒送机间隔（Fix13）
5. **`expect_macsz = 400`**：与 ROS1 相同的 RIB 目标填充值
6. **RIB<50 时循环末 1ms、否则 4ms**：低 RIB 时加快灌点，减少等点卡顿

#### 线程安全

| 资源 | 保护方式 |
|------|----------|
| `buf_queue_` | `buf_queue_mutex_`（所有 push/pop/size/clear 操作均加锁）|
| `rib_buffer_size_` | `std::atomic<int>` |
| `ros_motion_queue_` | `moodycamel::ConcurrentQueue`（无锁队列）|
| `robot_diagnosis_info_` | `publishWaypointToRobot` 使用局部 `pub_diag` 而非共享的 `rs.robot_diagnosis_info_` |
| `current_joints_` / `target_point_` | `joints_mutex_` |

---

## 六、线程架构总览

| 线程 | 频率 | 职责 |
|------|------|------|
| **主线程** | — | `spin_some` + `updateControlStatus`（计数、`start_move_` 置位、`delay_clear_times` 处理，**不取点**）|
| **feedToRosMotionLoop** | 200Hz（5ms）| 每周期最多 150 点从 `buf_queue_` 转入 `ros_motion_queue_`；超时管理 `start_move_` |
| **publishWaypointToRobot** | ~250Hz | 实时查询 RIB → 按缺额送点 → RIB<50 时 1ms 否则 4ms |
| **timerCallback** | 50Hz | 状态查询、`robot_status` / `rib_status` 发布 |
| **publishJointStateAndFeedbackLoop** | 50Hz | 发布 `joint_states` / `feedback_states` |

---

## 七、迭代修复历程

以下为完整的调试迭代过程，供参考：

| 迭代 | 主要修改 | 结果 |
|------|----------|------|
| Fix1 | 分离取点线程 `feedToRosMotionLoop`（500Hz）+ `buf_queue_mutex_` + `rib_buffer_size_` 改 atomic | 连续卡顿消除，突然加速消除 |
| Fix2 | Python 插值：整数步数控制 + wall-clock 累积误差补偿 200Hz | 发布间隔稳定 |
| Fix3 | `start_move_=false` 条件改为 buf_queue_ + ros_motion_queue_ 双空 | 减少误关 |
| Fix4 | 移除 feeder 的 `rib < MINIMUM_BUFFER_SIZE` 条件 + `buffer_size_=5` | 严重抖动（buffer_size_ 过小，start_move_ 频繁切换）|
| Fix5 | 移除 setRobotJointsByMoveIt 中 start_move_=false | start_move_ 稳定，但 publisher 吞吐不足 |
| Fix6 | RIB 查询移到 100ms 周期 + publisher 改 2ms | 去掉流控导致 RIB 先冲后空 |
| Fix7 | 回退：buffer_size_=60 + 三重条件关 start_move_ + 恢复 publisher 流控 4ms | RIB 查询再次阻塞 publisher |
| Fix8 | RIB 查询移到 timerCallback 50Hz + publisher 用 cached rib + 2ms | 缓存过时导致 RIB 耗尽 |
| Fix9 | feeder 批量排空 + 1s 超时关 start_move_ | 数据流通，偶发卡顿 |
| Fix10 | buffer_size_ 60→200 | 预缓冲增加，仍偶发卡顿 |
| Fix11 | local_sent 追踪 + min_batch=5 智能批量 | RIB 过冲缓解，TCP 尖峰 225ms |
| **Fix12** | **完全恢复 ROS1 架构：实时 RIB 查询 + buffer_size_=400 + 4ms 间隔 + 局部 pub_diag** | 连续/偶发卡顿解决 |
| **Fix13** | **单条轨迹内停顿**：RIB=0 时 sleep 由 10ms 改为 1ms；feedToRosMotionLoop 每周期最多转 150 点（批量排空），避免送机抽空 ros_motion_queue_ 后补点不足导致数秒间隔 | 单条轨迹内异常停顿消除 |

---

## 八、单条轨迹内异常停顿（Fix13 补充）

### 8.1 现象与根因

- **现象**：同一条轨迹执行过程中出现数秒级明显停顿，观感“卡一下再动”。
- **日志证据**：在 driver 侧记录“同轨迹内两次 SetRobotPosData2Canbus 的墙钟间隔”，出现 gap_sec 2～4s 且 rib=0；simulator 侧段间发布间隔均 <1ms，排除插值侧。
- **根因**：  
  1. RIB=0 时原逻辑 sleep 10ms，多轮叠加导致同轨迹内两次送机间隔达数秒。  
  2. feedToRosMotionLoop 每 5ms 仅取 1～2 点，送机线程在 RIB=0 时一次可送出约 400/6 批点，瞬间抽空 `ros_motion_queue_`，补点需数百次 5ms 周期，期间送机线程空转，形成长 gap。

### 8.2 修复

- **publishWaypointToRobot**：`current_macsz == 0` 时由 `sleep_for(10ms)` 改为 `sleep_for(1ms)`。  
- **feedToRosMotionLoop**：每周期 `while (setRobotJointsByMoveIt() && batch < 150)` 批量转点，单周期最多 150 点，保证送机抽空后能快速补足。

---

## 九、关键经验

1. **不要“优化”硬件通信协议的已验证设计**：ROS1 在 publisher 热循环中实时查询 RIB 看似低效（每次循环一次 TCP 往返），但这确保了流控的准确性。用缓存值替代实时查询会引入不可预测的滞后。

2. **预缓冲量要与 ROS1 一致**：`buffer_size_=400` 提供约 10 秒的缓冲深度，在 TCP 延迟尖峰期间保持 RIB 充足。

3. **数据竞争即使在"看似只读"场景中也有害**：`robot_diagnosis_info_` 被两个线程写入（`timerCallback` 和 `publishWaypointToRobot`），即使只用一个字段也会导致不可预测行为。解决方案是使用局部变量。

4. **feeder 应批量排空且单周期有上限**：每周期最多转 150 点，既避免单点转移造成补点太慢（单条轨迹内停顿），又避免单周期占用过长。

5. **`start_move_` 关闭要有足够容忍度**：短暂的队列为空不代表运动结束，500×5ms 超时能有效避免误关。

6. **RIB=0 时不宜长 sleep**：原 10ms 在 RIB 持续为 0 时会叠加成数秒送机间隔，改为 1ms 可显著减轻单条轨迹内停顿。

---

## 十、桥接 ROS1 与纯 ROS2 的启动延迟差异（同为 400 时）

`aubo_driver_ros2` 由 ROS1 `aubo_driver` 移植，逻辑一致：都有两条置位 `start_move_` 的路径：

| 路径 | 条件 | 理论触发时间 |
|------|------|----------------|
| **A：moveItPosCallback** | `buf_queue_.size() > buffer_size_`（400） | 401 点 @ 200Hz ≈ **2 s** |
| **B：updateControlStatus** | `data_count_ == MAXALLOWEDDELAY(50)` 且 `bq > 0` | 50×2ms = **100 ms** |

- **ROS1**：通常单进程、单线程 spin（或回调与“主循环”同一 executor）。收到首点后，下一次 spin 就会跑 `updateControlStatus`，计数与数据到达对齐较好，**路径 B 容易先触发**，约 100 ms 就开动，体感几乎无延迟。
- **ROS2**：`driver_node_ros2` 中 **主线程只跑** `updateControlStatus()`，**不跑** driver 的 spin；`moveItPosCallback` 在**独立 listener 线程**。主线程的 50 次计数与 listener 写入 `buf_queue_` 的时机可能错位，实测中常出现“先到 401 点（路径 A）才置位”，导致约 2 s 启动延迟。

因此：**同一套 400 逻辑下，桥接 ROS1 时“无延迟”是因为路径 B 在 ROS1 里先触发；纯 ROS2 时延迟明显是因为路径 B 未先触发、实际依赖路径 A。**

**解决方案（已实现）**：

1. **首点对齐**：`data_count_` 改为 `std::atomic<int>`；`moveItPosCallback` 在首点入队时 `data_count_.store(0)`；`updateControlStatus` 用 `fetch_add(1)`/`store(0)`，使路径 B 的 50 次从轨迹起点起算。
2. **ROS2 多线程调度**：将 `updateControlStatus()` 从主线程的 while 循环移到 **driver 节点的 500 Hz wall timer**（`update_control_timer_`），由 driver 线程的 executor 在 `spin_some()` 中执行。这样 50 次边界与 50 Hz 状态定时器等在同一 executor 下被调度，避免主线程在轨迹下发阶段被挤占导致路径 B 迟迟不触发；主线程仅做 `sleep` 等待进程退出。
3. **driver_node_ros2.cpp**：主线程不再调用 `updateControlStatus()`，改为 `while (rclcpp::ok()) { sleep(100ms); }`；`updateControlStatus` 仅由 timer 在 driver 线程内触发。

`buffer_size_` 可保持 400，预缓冲作兜底；路径 B 在约 100ms 内触发后即可开动。

---

## 十一、调试日志约定（可选复现问题时使用）

- **目的**：用统一 NDJSON 日志（每行一条 JSON）在链路各阶段打点，便于对比 ts_ms、buf_queue_size、rib_macsz 等，定位卡顿发生的阶段。
- **建议阶段 (stage)**：`action`（accept/published/success）、`sim`（trajectory_received、publish_interval、segment_done）、`driver_cb`（每次 moveItPosCallback）、`driver_feed`（取点周期）、`driver_send`（每次送批）。可选字段：`ts_ms`、`message`、`data`（含 buf_queue_size、rib_macsz、batch 等）。
- **假设示例**：H1 插值 200Hz 是否稳定 → sim 的 dt_ms；H2 回调是否成批 → driver_cb 相邻 ts_ms 差；H3/H4 缓冲是否见底 → buf_queue_size、rib_macsz。
- **实现**：写文件用 append 模式、控制打点频率（如每 200 次或每 100ms 一条），C++/Python 用 `#region agent log` 包裹便于后续移除。详见上文第三节“如何编写日志代码”。

---

## 十二、插值节点 aubo_robot_simulator_ros2 要点

- **作用**：在 ROS2 侧实现原 ROS1 `aubo_robot_simulator` 的轨迹插值，使 MoveIt2 轨迹经插值后通过 ros1_bridge 下发给 ROS1 `aubo_driver`。
- **话题**：订阅 `joint_path_command`（JointTrajectory），发布 `moveItController_cmd`（JointTrajectoryPoint）；可选订阅 `/aubo_driver/rib_status`、`/aubo_driver/real_pose`。
- **插值**：相邻路径点 5 次多项式插值，按 `motion_update_rate`（默认 200 Hz）步进，每步 `_move_to` + `_publish_cmd()`；新轨迹以 `time_from_start` 不递增判断。
- **参数**：`motion_update_rate`（200）、`minimum_buffer_size`（launch 中常设 600）、`joint_names` 与 MoveIt 一致。
- **与驱动配合**：节流、关节顺序重排、motion_buffer 单线程 worker；迁移过程中的 T4/T5 系数、_pad 长度、异常处理等已保留，见包内 README。

---

*文档版本：与当前 `aubo_driver_ros2` 及 `aubo_robot_simulator_ros2` 代码一致（含 Fix13）。若后续修改线程、缓冲或送点逻辑，请同步更新本文档。*
