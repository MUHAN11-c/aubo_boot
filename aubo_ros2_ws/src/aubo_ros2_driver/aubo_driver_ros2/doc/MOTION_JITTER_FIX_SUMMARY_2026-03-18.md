# 运动卡顿与位置突变问题总结（2026-03-18）

本文档用于快速回顾本次“运动过程卡顿、位置突变”问题的定位与修复结果，便于后续维护与回归验证。

---

## 1. 现象与影响

- 机械臂在连续轨迹执行中出现卡顿，体感不匀速。
- 局部阶段出现位置突变（关节瞬时跳变），影响轨迹平滑性与稳定性。
- 在多段轨迹切换场景（新轨迹接续旧轨迹）下问题更容易触发。

---

## 2. 关键根因（最终确认）

### 根因 A：新轨迹回调中的状态覆写导致瞬时跳变

在 `aubo_robot_simulator_ros2` 的 `trajectory_callback` 中，收到新轨迹时曾直接将内部关节状态强制写为新轨迹首点。  
该行为绕过了正常时间推进与边界过渡，造成位置瞬变。

**修复策略：**

- 移除“回调里直接覆写 joint state”的行为。
- 保持由 `motion_worker` 按轨迹时间推进与边界平滑处理状态。

---

### 根因 B：驱动发送链路存在突发与阻塞耦合，导致节拍锯齿

`aubo_driver_ros2` 下发链路在特定场景出现“短时大量/突发批次 + SDK 调用阻塞”，导致发送节拍不均匀，进而出现卡顿和反馈跳变。

**修复策略：**

1. **限制过速分段规模**
   - 对 overspeed 分段数设置上限，避免一次性扩张为极大批次。

2. **发送侧自适应节流**
   - 基础发送批量较小；
   - 队列积压时按阈值提升批量；
   - 保持发送批次硬上限，防止再次出现超大突发。

3. **降低热路径阻塞影响**
   - 诊断查询降频（有队列与空闲场景采用不同间隔）；
   - 保持发送优先，减少“查询阻塞导致的发送空窗”。

4. **批量入队改进**
   - `setRobotJointsByMoveIt` 支持每次弹出多点并批量入 `ros_motion_queue_`，
     提高发送线程的可用队列深度。

---

## 3. 已保留的有效改动（当前状态）

- `aubo_robot_simulator_node.py`
  - 保留：不在 `trajectory_callback` 中强制覆写当前关节状态。

- `aubo_driver_ros2.cpp`
  - 保留：overspeed 分段上限；
  - 保留：发送批次上限保护；
  - 保留：按队列积压与发送耗时特征进行自适应批量策略；
  - 保留：诊断刷新降频策略；
  - 保留：批量出队/入队路径优化。

---

## 4. 已清理内容

- 所有临时调试埋点（`agent log`/`dbg_log`/`_dbg` 等）已从相关源码中移除。
- 保留了最终验证有效的功能修复逻辑，避免“为清理日志而回退修复”。

---

## 5. 验证结论

- 大幅位置瞬跳问题已被切断（对应根因 A）。
- 大批次突发问题已显著收敛，发送链路更平稳（对应根因 B 的一部分）。
- 在高负载或多段轨迹连续切换下，整体卡顿明显降低，运动连续性改善。

---

## 6. 后续维护建议

1. 保持 `simulator` 侧“状态只在 worker 时间推进中更新”的原则，避免在回调中做突变赋值。
2. 发送链路参数（最小批量/最大批量/积压阈值/诊断刷新间隔）建议集中配置化，便于现场调优。
3. 后续若出现“卡顿回归”，优先检查：
   - `publishWaypointToRobot` 实际发送节拍；
   - SDK 调用耗时分布；
   - `ros_motion_queue_` 是否长期高水位或周期性抽空。
4. 建议在 CI 或联调脚本中增加“多段轨迹连续切换”回归场景，降低回归风险。

---

## 7. 涉及文件（参考）

- `aubo_ros2_ws/src/aubo_ros2_driver/aubo_robot_simulator_ros2/aubo_robot_simulator_ros2/aubo_robot_simulator_node.py`
- `aubo_ros2_ws/src/aubo_ros2_driver/aubo_driver_ros2/src/aubo_driver_ros2.cpp`
- `aubo_ros2_ws/src/aubo_ros2_driver/aubo_ros2_trajectory_action/src/aubo_ros2_trajectory_action.cpp`
- `aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/src/moveit_gripper_io_base.cpp`
- `aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/src/publish_grasps_client_worker.cpp`

