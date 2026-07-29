# Passthrough 架构移植说明（2026-07-27）

本工作区已从"标准 JTC 100Hz 流式打点"架构迁移到**一次性下发（passthrough）**架构，
核心逻辑完全遵循 Humble 实测驱动（aubo_boot），框架镜像 `/home/mu/Music/e`
（UR `PassthroughTrajectoryController` 写法的 Jazzy 完成版），仅做适配本项目的最小改动。

## 变更清单

- **新增包**：`aubo_e5_controllers`（AuboPassthroughTrajectoryController +
  AuboIOController，自写控制器插件）、`aubo_dashboard`（上电/断电/停止/FK/IK/负载服务节点）。
- **替换包**：`aubo_e5_hardware`（passthrough GPIO + 发送/IO 双线程 + RIB 流控 + sim 插件；
  vendor SDK 与旧版同一二进制，md5 校验一致）、`aubo_e5_bringup`（三模式 launch +
  controllers.yaml）、`aubo_e5_moveit_config`（controllers.yaml 指向 passthrough 控制器、
  joint_limits 蓝本值、trajectory_execution 5.0/10.0/0.15）。
- **合并包**：`aubo_msgs`（+RobotStatus.msg，其余消息/服务/action 不变）；
  `aubo_description`（保留工作单元 table/camera/quick_changer，注入
  `aubo_e5.ros2_control.xacro`，顶层 xacro 新增 `hardware_mode`/`robot_ip` 参数）；
  `aubo_e5.srdf`（保留 home/camera_pose，新增 zero）。
- **归档**：旧流式架构（`aubo_ros2_system.cpp` 等）整体移至 `src_legacy/`
  （含 COLCON_IGNORE，不参与构建）。
- **脚本**：`named_pose_controller.py` 的 action 名改为
  `/aubo_passthrough_trajectory_controller/follow_joint_trajectory`。
  （**2026-07-29 后续**：该脚本从未从 `src_legacy/` 移植进 `src/`，launch 与
  文档中的 `named_pose_enabled` 引用已一并移除；home/camera_pose 仍作为
  SRDF named states 存在，可经 MoveIt 使用。）
- **launch 参数变更**：旧的 `use_mock_hardware`/`enable_real_hardware`/
  `allow_motion_commands`/`start_moveit`/`server_host` 由
  `hardware_mode`（mock|sim|real）、`robot_ip`、`moveit_enabled` 取代。

## 已验证（2026-07-27，本机 Jazzy）

- `colcon build` 全工作区通过。
- sim 模式：三控制器 active；FollowJointTrajectory goal 执行成功
  （goal-hold confirmed）；执行中发新 goal 正确抢占（旧 goal CANCELED、新 goal SUCCEEDED）；
  SetIO 服务链路通（sim 插件不模拟板载 IO 回写，返回 success=false 属蓝本预期行为）。
- mock 模式：JTC active，FollowJointTrajectory goal 成功（SUCCEEDED）。
- 分析工具：`tools/passthrough_traj_client.py`、`tools/motion_analyzer.py`（run 模式）
  在 sim 实测通过（sine_shoulder 墙钟/标称比 1.08，终点误差 0，joint_states 200Hz）。

## 历史文档

`docs/` 下其余测试报告（NIC 事件、SDK 2.5.3 测试、realtime 配置等）针对已归档的
流式架构，仅供历史参考；手动测试清单 `manual_testing.md` 中的启动命令已过时，
以根 README 为准。
