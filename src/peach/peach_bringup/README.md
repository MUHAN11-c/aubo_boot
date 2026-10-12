# peach_bringup

整栈组合入口。Include 只读 `aubo_e5_bringup`，再拉相机前端（`camera_frontend:=stereo` 默认 / `percipio` / `replay`）、大脑（`peach_harvester` brain：感知+重建+调度一进程三节点）、技能 / 观测 / 场景障碍（`peach_scene_obstacles`）、可选 `serial_imu` 与 `imu_follow`（仅 adaptive 档）、`nav2_lifecycle_manager` 托管（名单：场景 → 重建 → 技能 → 调度）。**默认不自动 RunHarvest**；`autostart:=true` 为部署选配档——栈就绪且硬等 `selfcheck_passed=true` 后自动开批（授权=操作员发起 real launch）。

## 公有入口

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

bag 回放另加 `use_sim_time:=true`，并先 `ros2 bag play --clock`。其余常用参数（`tool_profile`、`bond_timeout` 默认 0 关、`skip_reconstruction`、`brain_params_overlay` 等）见 `--show-args` 与 [doc/architecture.md](doc/architecture.md) §4；注意 **stereo include 必须先于 aubo include**（参数泄漏坑，launch 内已按此序排好，勿在别处重排）。

本包自带小组件（setup.py console_scripts）：`peach_lifecycle_flag_bridge`（nav2_lm `is_active` → 闩锁 `/peach/lifecycle/managed_nodes_activated`）与 `peach_autostart_client`（autostart 档开批客户端，参数 `config/bringup.yaml`）。

旧 harvester 整栈/观测转发入口与自研生命周期管理器已归档；整栈仅从本包启动。预检 `peach_bringup.preflight.running_stack_pids`（argv0 或 `.../lib/<pkg>/<node>`，不是 colcon 参数名；RSP/rviz/MoveIt 等同名二进制须带 aubo/peach 标记，不拦同机其他机器人）。参数直接使用 `peach_common.yaml_params`（harvester / vegetation 同），部署值 `config/bringup.yaml`。

## 构建 / 测试 / 许可

随工作区 `colcon build --packages-select peach_bringup`。零 ROS 预检：`peach_system_tests/test/test_preflight.py`。BSD-3-Clause。
