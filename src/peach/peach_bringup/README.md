# peach_bringup

整栈组合入口。Include 只读 `aubo_e5_bringup`，再拉相机前端（`camera_frontend:=stereo` 默认 / `percipio` / `replay`）、大脑（`peach_harvester` brain：感知+重建+调度一进程三节点）、技能 / 观测 / 场景障碍（`peach_scene_obstacles`）、可选 `serial_force`（5 点力传感器）、`nav2_lifecycle_manager` 托管（名单：场景 → 重建 → 技能 → 调度）。**默认不自动 RunHarvest**；`autostart:=true` 为部署选配档——栈就绪且硬等 `selfcheck_passed=true` 后自动开批（授权=操作员发起 real launch）。

## 公有入口

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

bag 回放另加 `use_sim_time:=true`，并先 `ros2 bag play --clock`。其余常用参数（`tool_profile`、`bond_timeout` 默认 0 关、`skip_reconstruction`（两模式相同显式选择，默认false）、`task_params_file`（同一作业YAML） 、`brain_params_overlay`、`arm_params_overlay`、`observability_port` 默认 0=YAML 8090 等）见 `--show-args` 与 [doc/architecture.md](doc/architecture.md) §4；注意 **stereo include 必须先于 aubo include**（参数泄漏坑，launch 内已按此序排好，勿在别处重排）。

本包自带小组件（setup.py console_scripts）：`peach_lifecycle_flag_bridge`（nav2_lm `is_active` → 闩锁 `/peach/lifecycle/managed_nodes_activated`）与 `peach_autostart_client`（autostart 档开批客户端，参数 `config/bringup.yaml`）。

旧 harvester 整栈/观测转发入口与自研生命周期管理器已归档；整栈仅从本包启动。预检 `peach_bringup.preflight.running_stack_pids`（argv0 或 `.../lib/<pkg>/<node>`，不是 colcon 参数名；RSP/rviz/MoveIt 等同名二进制须带 aubo/peach 标记，不拦同机其他机器人）。参数直接使用 `peach_common.yaml_params`（harvester / vegetation 同），部署值 `config/bringup.yaml`。

## 构建 / 测试 / 许可

随工作区 `colcon build --packages-select peach_bringup`。`colcon test` 跑官方 flake8 / pep257。BSD-3-Clause。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。实现级流程见 [源码设计](doc/design.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。

RViz由应用入口使用 `config/harvest.rviz` 启动，驱动原RViz在作用域内关闭；实时相机raw与debug_image_raw默认可见。`real_web.launch.py`与mock共用业务默认和同一task_params_file，不提供高速/跳质量门的成功档。
