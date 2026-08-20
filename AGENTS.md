# AGENTS.md — 编码代理约束

权威：源码、各包 `config/*.yaml`、[docs/flow.md](docs/flow.md)（含阅读地图与调用关系）、[docs/usage.md](docs/usage.md)。旧手册只在 `_archive/`。

## 红线

- 真机驱动栈只读：`aubo_e5_hardware`、`aubo_e5_controllers`、`aubo_dashboard`、`aubo_e5.ros2_control.xacro`、`bringup.launch.py`、对应 `controllers.yaml`。
- bringup **不起** `aubo_dashboard`；禁止调用该包。`auto_power_on=false`。柜侧用示教器，规划/FK/IK 用 MoveIt，停轨走透传取消 + 硬件 `RobotMoveStop`。
- 未授权不得真机运动或 SetIO。
- Python：`aubo_py3.12`；numpy **1.26.4**；禁止 pip 装 opencv-python / scipy。
- 关节顺序：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。
- 启动前：`pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'`
- 不向 `build/`、`install/`、`log/`、`_archive/` 提交。
- **不要删过程数据**（`_archive/runs/`、现场 `web_runs/`）。

## 测试

各包 `test/` 只保留 ROS 2 默认 lint（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`）。不要写业务用例、gtest、DDS 假现场或 launch_testing。语法与流程由审查核对，对错以实机为准。

## 技术

C++17；参数走 yaml + `generate_parameter_library` / `declare_parameter`。`peach_task_executor` 是批次唯一所有者；**launch 绝不自动 RunHarvest**。能力端运动绑定 Lifecycle **Active**。重建只用精确时间戳 TF。默认 `execution/grasp/tool=false`。

## 入口

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch peach_task_executor harvest_system.launch.py hardware_mode:=sim camera_enabled:=false
```
