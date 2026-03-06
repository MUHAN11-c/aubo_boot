# 移植前后运动日志对比说明

## 本次运行结论（基于 motion_log_before / motion_log_after）

- **throttle**：移植前、移植后均为 **0 次**，说明插值端 RIB 节流未触发，抖动不是“缓冲满导致等待”造成。
- **segment / num_steps**：同段时长 T 下，移植前后步数一致（如 T=0.095 约 19–20 步，短段 4/6 步），插值逻辑等价。
- **trajectory_received**：点数、时长、min/max/avg_interval 一致，输入轨迹一致。

**结论：抖动原因不在 simulator 内部逻辑，而在数据路径。**  
移植后多了一跳 **ROS2 节点 → ros1_bridge → ROS1 aubo_driver**。桥接会带来：
- 跨进程、跨 DDS/ROS1 的序列化与转发延迟；
- 可能的消息成批到达（burst）或时间间隔不均匀；

导致 driver 收到的 `moveItController_cmd` 时间序列不如移植前（ROS1 直连）均匀，从而表现为抖动。

**建议**：  
1. 若可行，在 ROS1 端对 `moveItController_cmd` 做轻量平滑或小缓冲（不改变逻辑，仅匀化到达时间）；  
2. 或尝试减少 bridge 负载（少桥无关话题）、保证 bridge 进程优先级，观察是否有改善。

---

## 日志文件

- **移植前**：`/home/mu/IVG/.cursor/motion_log_before.ndjson`（由 `start_IVG_before_migration.sh` 启动，ROS1 aubo_robot_simulator 写入，每行一条 JSON，`side: "before"`）
- **移植后**：`/home/mu/IVG/.cursor/motion_log_after.ndjson`（由 `start_IVG_after_migration.sh` 启动，ROS2 aubo_robot_simulator_ros2 写入，`side: "after"`）

## 事件类型（两边格式一致，便于对比）

| event | 含义 | 典型字段 |
|-------|------|----------|
| `trajectory_received` | 收到一条轨迹并已入队 | `num_points`, `duration_sec`, `min_interval`, `max_interval`, `avg_interval` |
| `segment` | 两路径点间完成一段 5 次多项式插值 | `T`（段时长/s）, `num_steps`（插值步数） |
| `throttle` | 因 RIB 缓冲满而节流等待 | `block_count`, `rib_buffer_size`, `sleep_ms`, `dur_sec` |
| `publish_dt` | 每 200 次发布的间隔采样 | `dt_ms`, `count`（约 1s 一次，用于看发布节奏是否稳定） |

## 如何对比、分析抖动

1. **同一条轨迹**：在移植前、移植后各执行一次（例如同一条关节空间或笛卡尔轨迹），再对比两次的日志。
2. **重点看**：
   - **throttle**：移植后是否明显更多、`block_count` 更大、`sleep_ms` 更长 → 若明显更多，说明移植后更常因 RIB 满而等待，易造成卡顿。
   - **segment**：同一段时长 `T` 下，移植前后 `num_steps` 是否接近（理论应接近 200*T）；若移植后步数少或波动大，可能是插值周期或节流导致。
   - **publish_dt**：`dt_ms` 是否稳定在约 1000 ms（每 200 次约 1s）；若移植后波动大或偏大，说明发布节奏不稳或变慢。
   - **trajectory_received**：`min_interval` / `avg_interval` 移植前后是否一致；若移植后更密且未抽稀，可能产生更多短段和节流。
3. **可能原因归纳**：
   - 移植后 throttle 明显增多 → 桥接或 ROS2 发布时序导致 aubo_driver 端缓冲更容易满，需从缓冲策略或发布节奏排查。
   - 移植后 segment 步数偏少或不稳 → 插值周期、线程调度或节流导致实际步进变慢。
   - 移植后 publish_dt 波动大 → ROS2 定时/调度与 ROS1 不同，或节流导致发布不均匀。

## 简单命令行对比示例

```bash
# 只看 throttle 次数与总 sleep
grep '"event":"throttle"' /home/mu/IVG/.cursor/motion_log_before.ndjson | wc -l
grep '"event":"throttle"' /home/mu/IVG/.cursor/motion_log_after.ndjson | wc -l

# 看 throttle 的 block_count、sleep_ms
grep '"event":"throttle"' /home/mu/IVG/.cursor/motion_log_after.ndjson
```
