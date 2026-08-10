# 源码审查记录（2026-08-10）

> 本文是整合前的历史审计快照，因此保留当时 16 包和 fusion 测试记录。
> 同日后续已将 peach_pose_fusion_ros2 移除，能力收敛到
> peach_reconstruction_ros2；当前结构以 AGENTS.md 和
> docs/peach_perception_progress.md 为准。

## 范围与方法

本轮覆盖工作区 16 个 ROS 2 包及根目录工具/配置：逐包清点源码与依赖，
执行 Release 全量构建和全量 `colcon test`，并用项目 venv 单独执行
hand-eye、scene-recon、peach-pose、peach-reconstruction、peach-fusion
业务测试。厂商 SDK 二进制、vendor/generated 代码与模型权重只核对集成边界，
不改写其实现；真机运动因缺少现场急停/限位确认而未执行。

人工审查按风险聚焦以下跨层路径：

1. FollowJointTrajectory action → passthrough 控制器 → GPIO → 硬件发送线程；
2. RGB-D/CameraInfo → 点云构建 → 多视角帧栈 → session 落盘；
3. 单帧目标注册 → 重建候选选择 → final pose 融合与 frame_id 契约；
4. 参数 yaml/default、console_scripts/venv、README/命令与源码的一致性。

## 参考基线

- ROS 2 `FollowJointTrajectory` 官方 action 契约：
  <https://docs.ros.org/en/ros2_packages/jazzy/api/control_msgs/action/FollowJointTrajectory.html>
- ros2_controllers Jazzy JointTrajectoryController：
  <https://github.com/ros-controls/ros2_controllers/tree/jazzy/joint_trajectory_controller>
- UR passthrough 上游实现：
  <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_controllers/src/passthrough_trajectory_controller.cpp>
- ros2_control Jazzy hardware interface 文档：
  <https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1SystemInterface.html>
- rclcpp Duration Jazzy API：
  <https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Duration.html>
- NumPy 有限值检查：
  <https://numpy.org/doc/stable/reference/generated/numpy.isfinite.html>
- Open3D 官方点云写盘 API：
  <https://www.open3d.org/docs/release/python_api/open3d.io.write_point_cloud.html>

项目继续保留 Jazzy 已标记 deprecated 的旧式 ros2_control interface export，
因为本驱动需要把 passthrough 状态机回写到 command interface；这是已验证的
项目契约，不在本轮做表面“升级”。

## 已修复问题

| 风险 | 原问题 | 修复 |
|---|---|---|
| 真机运动输入 | NaN/Inf、非法或非递增时间可进入重采样/SDK；effort/path tolerance 会被静默忽略 | action 边界统一拒绝，并校验重复/非有限 goal tolerance |
| 时间转换 | 手写 sec/nanosec 拆装存在归一化与舍入边界 | 改用 Jazzy `rclcpp::Duration` 官方 API |
| 身份注册 | 零轴/NaN 轴可破坏“单位轴或 None”契约，NaN 位置/时间可污染匹配/LRU | 使用 `numpy.isfinite` 门禁，退化轴按不可用处理 |
| 自动重建 | 全部候选均为 REJECT 时仍取第一项启动 | 只允许 ACCEPT 或非 REJECT 候选 |
| RGB-D | 非有限/非正焦距会在 Open3D 深处异常 | 反投影前校验内参；建云异常转为明确拒帧 |
| session | 秒级目录名 + `exist_ok=True` 会覆盖同秒结果；PNG 写失败不报错 | 微秒目录名、禁止复用、检查 `cv2.imwrite` 返回值 |
| 融合坐标系 | frame 不符只告警，却把未变换候选重新标成 output_frame | 选中来源 frame 为空/不符时转 INCONSISTENT 并发布空 pose |
| 文档命令 | reconstruction 测试缺少 sibling `peach_pose_ros2` PYTHONPATH | 根 README、包 README、AGENTS 同步为可直接运行命令 |
| 冗余信息 | README 固定测试数/源码行数很快过期 | 删除此类重复计数，以测试输出和源码为准 |

## 验证与剩余边界

验证已覆盖新增身份注册与 session 回归测试、三个 peach 包 venv 全套测试，
以及 16 包 Release 全量构建。全量 `colcon test-result` 为 262 tests、0 errors、
0 failures、1 skipped（系统 Python 无 Open3D 的既有可选跳过；venv Open3D 套件
已通过）。sim 下 `wave_shoulder 3` 三次均 SUCCEEDED（单次约 6.40 s，标称 6.00 s）；
两个点使用相同 `time_from_start` 的非法 goal 在 action 边界按预期 REJECT。

本轮不替换 vendor SDK、PCL/Open3D/ROS 官方已有能力，也未引入新依赖。未验证项
只有依赖现场条件的真机分阶段测试；执行时必须遵循 `docs/usage.md` 第 7 节，
先状态只读，再 0.1 缩放低速小轨迹。
