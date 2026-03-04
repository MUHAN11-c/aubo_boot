# MoveIt 轨迹执行超时根因分析

## 现象

使用 `aubo_moveit_pure_ros2.launch.py` 时，若不在 launch 中传入较大的 `trajectory_execution` 参数，会出现：

- `waitForExecution timed out`
- `Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was ~7.027 seconds)`
- 机械臂实际已运动且能到位，但 MoveGroup 在超时后取消，报 `PATH_TOLERANCE_VIOLATED: aborted`
- **补充**：MoveIt 取消后，机械臂会**继续执行**当前轨迹并最终到位，说明执行链路和轨迹本身正常，问题仅在于「允许执行时间」过短导致提前取消。

## 根因

1. **超时如何计算**  
   MoveIt 的 `trajectory_execution_manager` 用「规划轨迹的时长」算允许执行时间：
   - 公式：`allowed_time = trajectory_duration × allowed_execution_duration_scaling + allowed_goal_duration_margin`
   - 默认/小 scaling（如 1.2）时，约 6s 的轨迹 → 允许约 7.7s（6×1.2+0.5）

2. **实际执行为何更慢**  
   - 规划出的轨迹是「时间最优」或较激进，`time_from_start` 总长约等于轨迹时长（如 ~6s）。
   - 真实机械臂受速度/加速度限制、控制器缓冲与通信延迟影响，**物理到位时间**常大于该规划时长（实测可达 ~9s 或更多）。
   - 因此：**规划时长 < 实际到位所需时间**，在默认允许时间内无法完成。

3. **为何会判为「超时」**  
   - trajectory action 只有在收到 `aubo/feedback_states` 且 `checkReachTarget`（实际位与目标位差 < 0.01 rad）时才调用 `succeed()`。
   - 在 ~7s 时 MoveIt 已到允许上界并取消 goal，此时机械臂往往仍在接近目标（例如最大关节误差仍在 0.5 rad 以上），故不会触发 `succeed()`，只会被 cancel → 报超时与 PATH_TOLERANCE_VIOLATED。

结论：**根因是「允许执行时间」按规划时长算得太紧，而真实机械臂到位时间更长**。要消除误超时，需要让 `allowed_execution_duration_scaling` 与 `allowed_goal_duration_margin` 足够大，使允许时间覆盖实际到位时间。

## 正确做法

在 **launch** 中为 move_group 传入合理的 `trajectory_execution` 参数（本仓库在 `aubo_moveit_pure_ros2.launch.py` 中设置）：

- `allowed_execution_duration_scaling`: 1.5（约 50% 余量）
- `allowed_goal_duration_margin`: 1.0（再加 1s 余量）
- `allowed_start_tolerance`: 0.15

使允许执行时间覆盖真实到位时间，又不过大，避免异常时等待过久。

## 其他可选手段（不推荐作为主方案）

- **在 trajectory action 里做速度缩放**（如 `velocity_scale_factor < 1`）：会拉长**下发给仿真/机械臂**的轨迹时间，但 **MoveIt 侧看到的仍是原始 goal 的时长**，超时仍按原始轨迹算，因此无法单靠速度缩放消除超时，只能减轻一点压力。
- **放宽到位容差**（如 0.01 → 0.05 rad）：会提前判「到位」并 `succeed()`，可能在不完全到位时就结束，一般不推荐。
