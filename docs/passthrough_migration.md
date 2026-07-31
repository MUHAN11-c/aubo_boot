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
  （含 COLCON_IGNORE，不参与构建；**2026-07-29 后续**：商业化精简时已整体
  删除，历史见 git 记录与 `docs/archive/`）。
- **脚本**：`named_pose_controller.py` 的 action 名改为
  `/aubo_passthrough_trajectory_controller/follow_joint_trajectory`。
  （**2026-07-29 后续**：该脚本从未移植进 `src/`（已随 `src_legacy/` 一并
  删除），launch 与文档中的 `named_pose_enabled` 引用已一并移除；
  home/camera_pose 仍作为 SRDF named states 存在，可经 MoveIt 使用。）
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

## 后续演进

- **2026-07-29 商业化交付结构重组**：`src_legacy/` 整体删除（见上文"归档"注记）；
  文档分流为 `docs/`（现行）与 `docs/archive/`（历史参考）；新增交付打包脚本
  `scripts/package_workspace.sh`。
- **real 模式取消 RT 预检**（2026-07-29 起）：普通 generic 内核直接启动，不再要求
  lowlatency/RT 内核与 SCHED_FIFO 预检（背景见 `docs/nic_driver_incident.md`
  后续补记）；每次开机仍需手动关网卡 offload + 设 governor（命令见
  `docs/nic_driver_incident.md`"持久化注意事项"）。
- **2026-07-31 全项目 ROS 2 官方标准规范化**（commit `957d62a`）：
  - 全部 11 个项目包接入 lint 测试：ament_cmake 包经 `ament_lint_auto`
    （copyright/cpplint/uncrustify/lint_cmake/xmllint），ament_python 包带官方模板
    `test_flake8.py`/`test_pep257.py`；`colcon test` 强制风格合规（C++ 100 列、
    `*`/`&` 居中对齐、BSD copyright 头；Python 99 列、单引号）。厂商代码经
    `vendor/AMENT_IGNORE` 跳过 lint，不影响编译。
  - `aubo_e5_hardware` **刻意保留** Jazzy 已 deprecated 的旧式
    `export_state_interfaces()`/`export_command_interfaces()` 导出——这是 UR 验证过、
    能让硬件把 passthrough 状态机回写进 command 接口的写法；其
    `CMakeLists.txt` 以 `-Wno-deprecated-declarations` 屏蔽告警，**不要"升级"为
    新式接口**。
  - 项目 Python 环境统一为工作区 venv `aubo_py3.12`（依赖锁定在根目录
    `requirements.txt`，venv 内 numpy 必须 <2）；依赖 torch/open3d 的节点经包内
    `scripts/` 包装脚本进 venv 运行，launch 一律用标准 `Node()`。

## 历史文档

`docs/archive/` 下测试报告（NIC 事件之外的 SDK 2.5.3 测试、realtime 配置等）
针对已归档的流式架构，仅供历史参考；手动测试清单
`docs/archive/manual_testing.md` 中的启动命令已过时，以根 README 为准。
