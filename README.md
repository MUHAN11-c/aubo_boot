# AUBO E5 ROS 2 Jazzy

套袋桃采摘工作区：手臂透传 + 场景感知 → 目标重建 → 技能 → 显式任务执行。

- 流程：[docs/flow.md](docs/flow.md)
- 用法：[docs/usage.md](docs/usage.md)
- 代理约束：[AGENTS.md](AGENTS.md)

验收看实机和过程数据（`web_runs/`、`_archive/runs/`）。`colcon test` 只跑 ROS 2 默认 lint。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=sim camera_enabled:=false
```

默认不上电、不派发运动、不打工具 IO、**不自动开批**。监控 `http://127.0.0.1:8090`。

## 功能包

逻辑与接口写在各包 README。

**采摘**

| 包 | 职责 |
|----|------|
| [peach_interfaces](src/peach_interfaces/README.md) | 唯一跨包 IDL |
| [peach_common_py](src/peach_common_py/README.md) | 共享纯 Python |
| [peach_scene_perception](src/peach_scene_perception/README.md) | RGB-D 观测与身份 |
| [peach_target_reconstruction](src/peach_target_reconstruction/README.md) | 单目标 TSDF / 精化 |
| [peach_manipulation_skills](src/peach_manipulation_skills/README.md) | SurveyScene + ExecuteTarget |
| [peach_task_executor](src/peach_task_executor/README.md) | 显式批次，唯一所有者 |
| [peach_observability](src/peach_observability/README.md) | 只读 Web / JSONL |

**手臂与相机**（驱动栈只读：hardware / controllers / dashboard / ros2_control xacro / bringup / controllers.yaml）

| 包 | 职责 |
|----|------|
| [aubo_msgs](src/aubo_msgs/README.md) | 柜侧状态与 IO 接口 |
| [aubo_description](src/aubo_description/README.md) | URDF / xacro |
| [aubo_e5_hardware](src/aubo_e5_hardware/README.md) | ros2_control 插件 |
| [aubo_e5_controllers](src/aubo_e5_controllers/README.md) | 透传与 IO 控制器 |
| [aubo_dashboard](src/aubo_dashboard/README.md) | 慢操作；禁 startup |
| [aubo_e5_bringup](src/aubo_e5_bringup/README.md) | 手臂唯一 launch |
| [aubo_e5_moveit_config](src/aubo_e5_moveit_config/README.md) | E5 MoveIt |
| [aubo_hand_eye_calibration](src/aubo_hand_eye_calibration/README.md) | 手眼 TF |
| [percipio_camera](src/percipio_camera/README.md) | 图漾驱动 |

架子机 [peach_gantry_description](src/peach_gantry_description/README.md)、[peach_moveit_config](src/peach_moveit_config/README.md) 不在本链路。

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
