# peach_bringup

整栈组合入口。Include 只读 `aubo_e5_bringup`，启动相机前端（`camera_frontend:=stereo` 默认 / `percipio` 官方 18 图案 / `replay`；launch 期互斥，禁止两驱动同图）、各自独立进程的感知 / 重建 / 调度，以及机械臂 / 观测 / 场景障碍、可选 `serial_force`（5 点力传感器）。`nav2_lifecycle_manager` 默认托管名单为场景 → 重建 → 机械臂 → 调度；关闭重建进程时从名单移除。**默认不自动 RunHarvest**；`autostart:=true` 为部署选配档——栈就绪且默认等待 `selfcheck_passed=true` 后自动开批；它只自动发批，运动/接触/工具仍须独立使能与授权。

## 公有入口

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

bag 回放另加 `use_sim_time:=true`，并先 `ros2 bag play --clock`。其余常用参数（`tool_profile`、`bond_timeout` 默认 0 关、`reconstruction_enabled` 默认 true、`skip_reconstruction` 在 mock 默认 true、real 默认 false、`task_params_file`、`brain_params_overlay`、`arm_params_overlay`、`observability_port` 默认 0=YAML 8090 等）见 `--show-args` 与 [包架构](doc/architecture.md)。mock/real 共用同一作业 YAML；显式 `skip_reconstruction` 优先于模式默认，只控制本批策略，进程开关由 `reconstruction_enabled` 决定。每个 include 使用独立 scoped `GroupAction`，隔离 `params_file`/`autostart` 等子参数，同时继承父级时钟和启动事实。

本包自带小组件（setup.py console_scripts）：`peach_lifecycle_flag_bridge`（nav2_lm `is_active` → 闩锁 `/peach/lifecycle/managed_nodes_activated`）与 `peach_autostart_client`（autostart 档开批客户端，参数 `config/bringup.yaml`）。

旧 harvester 整栈/观测转发入口与自研生命周期管理器已归档；整栈仅从本包启动。预检 `peach_bringup.preflight.running_stack_pids`（argv0 或 `.../lib/<pkg>/<node>`，不是 colcon 参数名；RSP/rviz/MoveIt 等同名二进制须带 aubo/peach 标记，不拦同机其他机器人）。Python 参数使用各包 GPL schema/`ParamListener`（legacy vegetation 仍用 `peach_common.yaml_params`），部署值 `config/bringup.yaml`。

## 构建 / 测试 / 许可

随工作区 `colcon build --packages-up-to peach_bringup`。首次部署 Web 时先按依赖包说明构建浏览器资源。整栈使用 `ros2_web_link` 的 `bridge_node.py` 和 SDK，`peach_harvester` 仅为无代码 metapackage。WebSocket 默认监听 `127.0.0.1:9090`，可设 `rosbridge_port` 或用 `rosbridge_enabled:=false` 单独保留 HTTP 操作台。`colcon test` 跑官方 flake8 / pep257。BSD-3-Clause。见 [ros2_web_link 集成说明](../../ros2_web_link/doc/integration.md)。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。实现级流程见 [源码设计](doc/design.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。

RViz由应用入口使用 `config/harvest_app.rviz` 启动，驱动原RViz在作用域内关闭；实时相机raw与debug_image_raw默认可见。`real_web.launch.py`与mock共用业务默认和同一task_params_file，不提供高速/跳质量门的成功档。
