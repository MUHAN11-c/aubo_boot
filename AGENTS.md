# AGENTS.md — 编码代理约束

权威：源码、各包 `config/*.yaml`、三份活文档——[docs/architecture.md](docs/architecture.md)（设计架构）、[docs/io.md](docs/io.md)（输入输出）、[docs/testing.md](docs/testing.md)（测试流程与命名）。**设计和改动依赖这三份。源码与文档互相更新：改一边必须同一改动内改另一边，始终一致。** 不要在 `docs/` 再加第四份活文档。过程记录不驱动现行设计、不写怎么跑：

- [docs/testing-log.md](docs/testing-log.md) — 真机/审查轮次；补实测只追加
- [docs/REFACTORING.md](docs/REFACTORING.md) — 工程整理过程与文件映射

改行为改 testing.md + 源码。旧手册与暂不用的根目录项只在 `_archive/`（`parked_2026-08-24/` 架子机/工具，`parked_2026-09/` 导航适配）。不把监控或底盘驱动拆成新 peach 包。

改行为 / yaml / IDL / launch 必须改对应文档；改文档里的现行描述必须兑现到源码或 yaml（标成「缺口 / 预留 / 归档」的除外）。注释不得与这三份或现行源码打架。发现不一致：两边一起改到一致再继续，禁止只改一边。

## 红线

- 真机驱动栈只读：`aubo_e5_hardware`、`aubo_e5_controllers`、`aubo_dashboard`、`aubo_e5.ros2_control.xacro`、`bringup.launch.py`、对应 `controllers.yaml`。
- bringup **不起** `aubo_dashboard`；禁止调用该包。`auto_power_on=false`。柜侧用示教器，规划/FK/IK 用 MoveIt，停轨走透传取消 + 硬件 `RobotMoveStop`。
- 未授权不得真机运动或 SetIO。
- Python：`aubo_py3.12`。依赖分层 venv-first：ROS 2 依赖走 Jazzy apt，其余第三方一律 `requirements.txt` 钉版本装进 venv；numpy **1.26.4**（<2，cv_bridge ABI 硬约束）。
- 关节顺序：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。
- 启动前：`pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'`
- 不向 `build/`、`install/`、`log/`、`_archive/` 提交。不要提交 GPL 生成的 `*_parameters.py`（gitignore）。
- **不要删过程数据**（`_archive/runs/`、现场 `runs/`）。

## 测试

怎么跑、命名、验收门：[docs/testing.md](docs/testing.md)。真机/审查轮次：[docs/testing-log.md](docs/testing-log.md)（只追加，不驱动现行设计）。

各包 `test/` 保留 ROS 2 默认 lint（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`），并允许**零 ROS 纯核 pytest**（不 import rclpy / 不造 DDS 现场）。现行纯核：`peach_executor/test/test_harvest_fsm.py`（`react` 表）、`peach_perception/test/test_runtime_core.py`（`ManualClock` / `BoundedWorker`）、`graspnet_ros2/test/test_grasp_core.py`（NMS/碰撞；torch `importorskip`）。`peach_interfaces` 的 `check_interface_manifest.py` 进 colcon test。不要写业务用例、gtest、DDS 假现场、launch_testing 或采摘仿真测。`colcon test` 不等于采摘验收。语法与流程由审查核对，对错以实机为准。

## 技术

C++17；参数走 yaml + `generate_parameter_library`（GPL 声明 yaml=默认/校验/描述单一事实源；同名运行 yaml 只写部署覆盖；规约见 docs/architecture.md「参数分层」）。感知与调度/监控/lifecycle 的 GPL 生成模块同时写入源码包（gitignore），避免 `PYTHONPATH` 指向 `src/` 时挡住 install。采摘 ROS 包四个，作用不得串（详细：[docs/architecture.md](docs/architecture.md) §3）：

- `peach_interfaces`：跨包唯一 IDL，不跑节点。清单 consumers 须对上真实订阅：调度只订 `target_observations` 与 `managed_nodes_activated`，不要把 `initial_pose` / `grasp_decision` 写成调度订阅。
- `peach_perception`：视觉算法（两节点：看场景 + 建当前目标），不发运动、不选下一颗、不写账本；积分不用 latest TF；球只作袋内果实包络先验。单实现直接构造；仅袋/果管线与柱/球 refitter 留 yaml `*.impl` 映射。
- `peach_manipulation`：机械臂执行（`SurveyScene` / `ExecuteTarget`：视点、预抓取、套入、刀具、原路撤退），不写账本、不调重建 Trigger；`GraspDecision.allowed` 是套入/剪切唯一权威（`PREGRASP_ONLY` 不要求 `allowed`；方向定位以预抓取真机实测为准）。节点持 GPL `Params` 快照，运动/接触/扫描 Config 从快照直构。`grasp_hypothesis` 走 LifecyclePublisher，须 `on_activate`。不要为 `stages.cpp` 再加 Manager。
- `peach_executor`：整栈调度（含 lifecycle、只读监控 Web 与**鉴权手动调试操作面**——融合 8090，`debug.enabled`/`token`/运动类 `motion_enabled` 三重门默认全关，操作全审计；见决策 0013）；**launch 绝不自动 RunHarvest**；`execute_pregrasp_only` 默认 true（停预抓取不回 stow；套入前改 false）。`harvest_fsm.react` 是批次纯核，节点禁止手写 `batch_state`。`peach_lifecycle_manager` 走 GPL `lifecycle_manager_parameters.yaml`（ParamListener；名单场景 → 重建 → 技能 → 调度；observability 不进名单、不加 bond）。Web 不做成产品（不加登录/RBAC/独立前端工程）。

旁路视觉抓取三包（`ivg_interfaces` / `visual_pose_estimation_python` / `graspnet_ros2`）**不是** peach 包：不进 `harvest_system`、不进 lifecycle、不订 peach 话题、不改驱动栈。GraspNet 后端不用 AnyGrasp 许可证。采摘跨包契约仍只走 `peach_interfaces`。

导航适配 `peach_navigation` 已归档 `_archive/parked_2026-09/`（固定座核心栈四包不含它）；`NavigateToWorksite` 等 IDL 保留标预留，真底盘授权后恢复。

臂/相机九包职责与只读范围同 architecture §3 驱动层。

Python 模块名与包名一致；launch/config/可执行文件跟职责名对齐（见 architecture §3 命名与文件树）。图名（节点/话题/动作）保持契约，不因整理文件而改；拆文件时旧路径留 shim，须 re-export **模块级常量**（不只 class/def）。能力端运动绑定 Lifecycle **Active**。重建只用精确时间戳 TF。默认 `execution/grasp/tool=false`。

## 入口

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
```
