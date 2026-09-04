# AGENTS.md — 编码代理约束

权威：源码、各包 `config/*.yaml`、三份活文档——[docs/architecture.md](docs/architecture.md)（设计架构）、[docs/io.md](docs/io.md)（输入输出）、[docs/testing.md](docs/testing.md)（测试流程与命名）。**设计和改动依赖这三份。源码与文档互相更新：改一边必须同一改动内改另一边，始终一致。** 不要在 `docs/` 再加第四份活文档。[docs/testing-log.md](docs/testing-log.md) 只记真机/审查轮次，不写怎么跑、不驱动现行设计：改行为改 testing.md + 源码；补实测只追加 testing-log。旧手册与暂不用的根目录项只在 `_archive/`（含 `parked_2026-08-24/`）。不把监控或底盘驱动拆成新 peach 包。

改行为 / yaml / IDL / launch 必须改对应文档；改文档里的现行描述必须兑现到源码或 yaml（标成「缺口 / 预留 / 归档」的除外）。注释不得与这三份或现行源码打架。发现不一致：两边一起改到一致再继续，禁止只改一边。

## 红线

- 真机驱动栈只读：`aubo_e5_hardware`、`aubo_e5_controllers`、`aubo_dashboard`、`aubo_e5.ros2_control.xacro`、`bringup.launch.py`、对应 `controllers.yaml`。
- bringup **不起** `aubo_dashboard`；禁止调用该包。`auto_power_on=false`。柜侧用示教器，规划/FK/IK 用 MoveIt，停轨走透传取消 + 硬件 `RobotMoveStop`。
- 未授权不得真机运动或 SetIO。
- Python：`aubo_py3.12`。依赖分层 venv-first：ROS 2 依赖走 Jazzy apt，其余第三方一律 `requirements.txt` 钉版本装进 venv；numpy **1.26.4**（<2，cv_bridge ABI 硬约束）。
- 关节顺序：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。
- 启动前：`pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'`
- 不向 `build/`、`install/`、`log/`、`_archive/` 提交。
- **不要删过程数据**（`_archive/runs/`、现场 `runs/`）。

## 测试

怎么跑、命名、验收门：[docs/testing.md](docs/testing.md)。真机/审查轮次：[docs/testing-log.md](docs/testing-log.md)（只追加，不驱动现行设计）。

各包 `test/` 只保留 ROS 2 默认 lint（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`）。不要写业务用例、gtest、DDS 假现场或 launch_testing。语法与流程由审查核对，对错以实机为准。

## 技术

C++17；参数走 yaml + `generate_parameter_library` / `declare_parameter`。采摘 ROS 包四个，作用不得串（详细：[docs/architecture.md](docs/architecture.md) §3）：

- `peach_interfaces`：跨包唯一 IDL，不跑节点
- `peach_perception`：视觉算法（两节点：看场景 + 建当前目标），不发运动、不选下一颗、不写账本；积分不用 latest TF；球只作袋内果实包络先验
- `peach_manipulation`：机械臂执行（`SurveyScene` / `ExecuteTarget`：视点、预抓取、套入、刀具、原路撤退），不写账本、不调重建 Trigger；`GraspDecision.allowed` 是套入/剪切唯一权威（`PREGRASP_ONLY` 不要求 `allowed`；方向定位以预抓取真机实测为准）
- `peach_executor`：整栈调度（含 lifecycle、只读监控 Web 与**鉴权手动调试操作面**——融合 8090，`debug.enabled`/`token`/运动类 `motion_enabled` 三重门默认全关，操作全审计；见决策 0013）；**launch 绝不自动 RunHarvest**；`execute_pregrasp_only` 默认 true（停预抓取不回 stow；套入前改 false）

导航适配 `peach_navigation` 已归档 `_archive/parked_2026-09/`（固定座核心栈四包不含它）；`NavigateToWorksite` 等 IDL 保留标预留，真底盘授权后恢复。

臂/相机九包职责与只读范围同 architecture §3 驱动层。

Python 模块名与包名一致；launch/config/可执行文件跟职责名对齐（见 architecture §3 命名与文件树）。图名（节点/话题/动作）保持契约，不因整理文件而改。能力端运动绑定 Lifecycle **Active**。重建只用精确时间戳 TF。默认 `execution/grasp/tool=false`。

## 入口

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
```
