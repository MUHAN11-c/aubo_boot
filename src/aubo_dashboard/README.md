# aubo_dashboard

柜侧慢操作节点（上电、抱闸、停机、FK/IK、负载）。**bringup 不启动本节点。** 包保留在仓库，作业不使用。

## 现场约定（暂时）

**禁止调用本功能包**（不要 `ros2 run` / 不要调 `/aubo_dashboard/*`）。替代：

- 上电、断电、抱闸、碰撞恢复、急停 → **示教器 / 控制柜面板**
- 运动学规划、FK/IK → **MoveIt**
- 停轨 → 透传 `FollowJointTrajectory` 取消 → 硬件 `abort` → `ioLoop` 发 `RobotMoveStop`（失败再 `robotMoveFastStop`）。**不依赖本包。**

`auto_power_on` 必须为 false。`tools/fk_ik_check.py` 依赖本节点，作业不要跑。
