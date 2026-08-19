# AUBO E5 ROS 2 Jazzy

套袋桃采摘工作区：手臂透传 + 感知 → 局部重建 → 单目标技能 → 批次编排。

- 流程：[docs/flow.md](docs/flow.md)
- 用法：[docs/usage.md](docs/usage.md)
- 代理约束：[AGENTS.md](AGENTS.md)

验收看实机和过程数据（`web_runs/`、`_archive/runs/`）。`colcon test` 只检查语法、lint 和少量纯逻辑。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_harvest_orchestrator harvest_system.launch.py \
  hardware_mode:=sim camera_enabled:=false
```

默认不上电、不派发运动、不打工具 IO。监控 `http://127.0.0.1:8090`。

| 路径 | 内容 |
|------|------|
| `src/` | ROS 包 |
| `tools/` | 轨迹客户端、运动分析、回放 |
| `diagnostics/` | SDK 零运动探针 |
| `peach_profiles/` | 操作策略 |
| `docs/` | 流程与用法（按源码维护） |
| `_archive/runs/` | 历史过程数据（勿删） |
| `_archive/` | 旧文档、厂商资料、快照 |
| `aubo_py3.12/` | Python 3.12 venv |
