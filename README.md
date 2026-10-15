# AUBO E5 ROS 2 Jazzy

套袋桃采摘：到位（预留）→ 场景里有哪些桃 → 这一颗的局部模型 → 臂怎么动 → 调度开批。

- 设计架构：[src/peach/peach_bringup/doc/architecture.md](src/peach/peach_bringup/doc/architecture.md)
- 输入输出：[src/peach/peach_bringup/doc/io.md](src/peach/peach_bringup/doc/io.md)
- 测试流程指导：[src/peach/peach_system_tests/doc/testing.md](src/peach/peach_system_tests/doc/testing.md)（v1）；[src/peach2/peach2_system_tests/doc/testing.md](src/peach2/peach2_system_tests/doc/testing.md)（v2）
- 完整项目目标设计（非活文档）：[peach_autonomous_harvest_design/README.md](peach_autonomous_harvest_design/README.md)，包含自主导航采摘、Web与调试设计。原docs中的v1已按模块[整理合并](peach_autonomous_harvest_design/10_docs_merge_and_baseline.md)，v2及配套图件[仅归档](peach_autonomous_harvest_design/archive/docs_2026-10-13/README.md)、暂不采用；docs旧路径保留兼容导航。
- 代理约束：[AGENTS.md](AGENTS.md)

前两份（architecture / io）是活文档，与源码互相更新：改一边须同一轮改另一边。测试流程随两测试包 `doc/testing.md` 维护（2026-10-10 自 docs/testing.md 拆写入包，用户裁定）。过程记录类（testing-log / REFACTORING）已于 2026-10-10 清理退役，历史在 git。`docs/` 不另增活文档。

验收看实机和过程数据（`runs/`、`_archive/runs/`）。`colcon test` 跑包内 lint、pytest/gTest 与 isolated launch_testing，不等于采摘验收。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

默认不上电、不派发运动、不打工具 IO、**不自动开批**（`autostart` 默认关）。监控 `http://127.0.0.1:8090`。

## 采摘应用包

详细作用、入口、禁止项、命名与文件树：[src/peach/peach_bringup/doc/architecture.md](src/peach/peach_bringup/doc/architecture.md) §3。跨包契约：[src/peach/peach_bringup/doc/io.md](src/peach/peach_bringup/doc/io.md)。能力包不互发批次命令；只有调度当客户端。节点之间只走 `peach_interfaces`。

| 包 | 作用 | 含节点 | 不做什么 |
|----|------|--------|----------|
| [peach_interfaces](src/peach/peach_interfaces/README.md) | 跨包唯一 IDL：批次/观测/重建/抓取（导航名预留） | 无 | 不跑节点、不设算法参数、不 launch |
| [peach_harvester](src/peach/peach_harvester/README.md) | 大脑一进程三节点：vision（场景观测+目标重建）+ supervisor（批次 FSM/选果/视点两档/批次策略/操作台服务/账本+补采清单） | `peach_scene_perception_node`、`peach_target_reconstruction_node`、`peach_supervisor` | 不发关节命令、不做 IK（问臂）；感知不选下一颗、不写账本 |
| [peach_arm](src/peach/peach_arm/README.md) | 拍照、主动视点、质量/安全门、MTC 接触、工具 SetIO、撤退 | `peach_arm` | 不写账本、不调重建 Trigger、不 `BeginScene`/`RunHarvest` |
| [peach_bringup](src/peach/peach_bringup/README.md) | 整栈入口、预检、lifecycle 托管、autostart 客户端、生命周期桥 | `peach_lifecycle_flag_bridge`、`peach_autostart_client` | 不含业务；launch 不自动开批 |
| [peach_observability](src/peach/peach_observability/README.md) | 8090 过程页/单步调试、会话 bag、`peach_bag_report` | `peach_observability` | 不发运动 |
| [peach_web](src/peach_web/README.md) | 8090 过程页静态资源（顶层资源包，HTTP 托管在 peach_observability） | 无 | 不跑节点、不进 launch/lifecycle |
| [peach_vegetation](src/peach/peach_vegetation/README.md) | GPU 枝/叶 2D 掩膜（独立 launch） | `peach_vegetation` | 不写 PlanningScene、不进 harvest_system |
| [peach_system_tests](src/peach/peach_system_tests/README.md) | isolated mock launch_testing + 回放塔 | （launch_testing） | 不进运行 launch |

作业目标以调度 `/peach_supervisor/state` 的 `target_id` 为准。感知 `harvest_plan` 只做收齐锁定窗。导航已归档 `_archive/parked_2026-09/`，到位一步调度直通 `NAV_OK`（`NavigateToWorksite` 预留）。

## 手臂与相机

驱动栈只读：hardware / controllers / dashboard / ros2_control xacro / bringup / controllers.yaml。

| 包 | 职责 |
|----|------|
| [aubo_msgs](src/aubo_msgs/README.md) | 柜侧状态与 IO 接口 |
| [aubo_description](src/aubo_description/README.md) | URDF / xacro |
| [aubo_e5_hardware](src/aubo_e5_hardware/README.md) | ros2_control 插件 |
| [aubo_e5_controllers](src/aubo_e5_controllers/README.md) | 透传与 IO 控制器 |
| [aubo_dashboard](src/aubo_dashboard/README.md) | 柜侧慢操作；bringup 不起，作业禁用 |
| [aubo_e5_bringup](src/aubo_e5_bringup/README.md) | 手臂唯一 launch |
| [aubo_e5_moveit_config](src/aubo_e5_moveit_config/README.md) | E5 MoveIt |
| [aubo_hand_eye_calibration](src/aubo_hand_eye_calibration/README.md) | 手眼标定（侧车） |
| [percipio_camera](src/percipio_camera/README.md) | 图漾驱动 |
| [peach_stereo](src/peach/peach_stereo/README.md) | 可选相机前端（`camera_frontend:=stereo`，主机单图案立体，话题同构） |

架子机 URDF / MoveIt 在 `_archive/parked_2026-08-24/`，不在本链路。

## 交付树

| 路径 | 内容 |
|------|------|
| `src/` | v1 采摘栈九包（`src/peach/`，含公共库与相机前端）+ v2 孵化十二包（`src/peach2/`）+ IVG 演示九包（`src/ivg/`）+ 臂/相机驱动九包 + 过程页静态资源 `peach_web` 1 + 仿真 `peach_sim` 1 + 可选 USB IMU / IMU 跟随 2 + 夹爪描述 `robotiq_description` 1 |
| `docs/` | v1/v2 方案书与架构图交付件（非活文档）；现行架构、IO 与测试指导在各包 `doc/` |
| `scripts/` | 工作区打包与架构图生成工具；业务、采集与测试入口在所属包 |
| `runs/` | 过程数据唯一根（文本按 `.gitignore` 可入库，bag/图像等二进制留本地） |
| `_archive/runs/` | 历史过程数据（勿删） |
| `_archive/code_cleanup_2026-10-12/` | 冻结设计书、历史 A/B 实验与退役源码；manifest.json 可校验和恢复（本地，不随仓） |
| `_archive/parked_2026-08-24/` | 暂不用：tools / diagnostics / profiles / scripts / 架子机 / 过程文档 |
| `_archive/parked_2026-09/` | 已归档 `peach_navigation`（真底盘授权后恢复） |
| `aubo_py3.12/` | Python 3.12 venv（本机，不入库） |

## 打包迁移

```bash
scripts/pack_workspace.sh --dry-run   # 预览包含/排除与体积
scripts/pack_workspace.sh             # 生成 ../aubo_e5_jazzy_ws-src-<时间戳>.tar.zst
```

排除归档（`_archive/`）、根目录编译产物与包内残留的 colcon `build/ install/ log/`、venv、会话输出与 IDE/AI 工具本地态；ivg 模型权重、Blender 发行版、YCB meshes 等**可重下**大件不入包（目标环境按各包 README、`fetch_*.sh` 重取，`colcon build` 重建）；`reconstruction/output/` 只随包携带 git 已跟踪的验证产物，未跟踪的重渲大件重渲即得。`src/` 源码资产（含 `peach_harvester/model` 自训练模型、驱动 vendor、web 前端 three.js/robotwebtools 发行件）、`.cursor/rules`、`scripts/` 与 `.git` 历史默认入包，`--no-git` 可去掉。
