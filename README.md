# AUBO E5 ROS 2 Jazzy

套袋桃采摘：到位（预留）→ 场景里有哪些桃 → 这一颗的局部模型 → 臂怎么动 → 调度开批。

- 设计架构：[docs/architecture.md](docs/architecture.md)
- 输入输出：[docs/io.md](docs/io.md)
- 测试流程与命名：[docs/testing.md](docs/testing.md)
- 测试过程记录：[docs/testing-log.md](docs/testing-log.md)
- 代理约束：[AGENTS.md](AGENTS.md)

前三份（architecture / io / testing）是活文档，与源码互相更新：改一边须同一轮改另一边。`testing-log.md` 只记轮次，不驱动现行设计。`docs/` 不另增活文档。

验收看实机和过程数据（`runs/`、`_archive/runs/`）。`colcon test` 只跑 ROS 2 默认 lint。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

默认不上电、不派发运动、不打工具 IO、**不自动开批**。监控 `http://127.0.0.1:8090`。

## 采摘四个能力包

详细作用、入口、禁止项、命名与文件树：[docs/architecture.md](docs/architecture.md) §3。跨包契约：[docs/io.md](docs/io.md)。能力包不互发批次命令；只有调度当客户端。节点之间只走 `peach_interfaces`。

| 包 | 作用 | 含节点 | 不做什么 |
|----|------|--------|----------|
| [peach_interfaces](src/peach_interfaces/README.md) | 跨包唯一 IDL：批次/观测/重建/抓取（导航名预留） | 无 | 不跑节点、不设算法参数、不 launch |
| [peach_perception](src/peach_perception/README.md) | 看场景（身份+锁定集）+ 建当前目标（TSDF/`GraspDecision`） | `peach_scene_perception_node`、`peach_target_reconstruction_node` | 不选下一颗、不运动、不写账本；积分不用 latest TF |
| [peach_manipulation](src/peach_manipulation/README.md) | 拍照、主动视点、质量/安全门、MTC 接触、工具 SetIO、撤退 | `peach_manipulation_node` | 不写账本、不调重建 Trigger、不 `BeginScene`/`RunHarvest` |
| [peach_executor](src/peach_executor/README.md) | 开批、选果、账本；lifecycle 四节点；只读 Web/jsonl；整栈 launch | `peach_executor`、`peach_lifecycle_manager`、`peach_observability` | 不做视觉、不规划接触/导航、Web 不发运动、launch 不自动开批 |

作业目标以执行器 `~/state.target_id` 为准。感知 `harvest_plan` 只做收齐锁定窗。导航已归档 `_archive/parked_2026-09/`，到位一步调度直通 `NAV_OK`（`NavigateToWorksite` 预留）。

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

架子机 URDF / MoveIt 在 `_archive/parked_2026-08-24/`，不在本链路。

## 交付树

| 路径 | 内容 |
|------|------|
| `src/` | 采摘 4 包 + 臂/相机 9 包 + 可选 USB IMU 1 |
| `docs/` | `architecture.md` 设计架构；`io.md` 输入输出；`testing.md` 测试流程与命名；`testing-log.md` 测试过程记录 |
| `runs/` | 过程数据唯一根（gitignore：账本、观测、session、MCAP） |
| `_archive/runs/` | 历史过程数据（勿删） |
| `_archive/parked_2026-08-24/` | 暂不用：tools / diagnostics / profiles / scripts / 架子机 / 过程文档 |
| `_archive/parked_2026-09/` | 已归档 `peach_navigation`（真底盘授权后恢复） |
| `aubo_py3.12/` | Python 3.12 venv（本机，不入库） |
