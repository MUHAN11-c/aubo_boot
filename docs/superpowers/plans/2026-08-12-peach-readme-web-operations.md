# 桃子采摘功能包 README 与 Web 操作说明实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让六个采摘业务包的 README 与当前源码、Web 控制逻辑和 2026-08-12 真机测试结果一致。

**Architecture:** 每个包只说明自身职责和接口，同时统一链接到总入口 `harvest_system.launch.py`。Web README 作为现场操作手册，明确自动 action 与手动 Trigger 的不同 arm 语义、维护模式、乐观锁、恢复确认和分阶段真机测试。

**Tech Stack:** ROS 2 Jazzy、MoveIt 2、rclcpp/rclpy、标准库 HTTP、Markdown。

## Global Constraints

- 始终使用中文。
- 真机默认 `execution/grasp/tool=false`，首次运动速度和加速度缩放不超过 0.1，当前为 0.05。
- 不调用 `/aubo_dashboard/startup`，上电、松刹车和安全恢复只允许现场人员完成。
- 不修改冻结的真机驱动、控制器、Dashboard 和 bringup。

---

### Task 1: 更新四个算法与操作包 README

**Files:**
- Modify: `src/peach_pose_ros2/README.md`
- Modify: `src/peach_reconstruction_ros2/README.md`
- Modify: `src/peach_approach_grasp/README.md`
- Modify: `src/peach_perception_web/README.md`

- [ ] 校对职责、总入口、单包入口、话题、服务和参数文件。
- [ ] 区分自动 action 自动一次 arm 与手动 Trigger 人工 arm。
- [ ] 为 Web 写出仿真、真机、自动、手动和恢复测试流程。

### Task 2: 补齐调度与接口包 README

**Files:**
- Create: `src/peach_harvest_orchestrator/README.md`
- Create: `src/peach_harvest_msgs/README.md`

- [ ] 说明状态机、revision 乐观锁、策略依赖和总 launch 参数。
- [ ] 列出 msg/srv/action 常量与生产者、消费者。

### Task 3: 验证文档

**Files:**
- Test: all six README files

- [ ] 搜索过期的“Web 只读”与错误参数名。
- [ ] 运行各 launch 的 `--show-args` 并核对 README。
- [ ] 运行 Web pytest 与相关包 lint，确认文档修改没有破坏安装和测试。
