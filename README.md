# AUBO E5 ROS 2 Jazzy

固定座 AUBO E5 套袋桃采摘工作区：RGB-D 感知 → 目标重建与许可 → 批次调度 → 机械臂执行 → Web 观测与过程记录。本文按 2026-11-06 当前源码整理。

从 GitHub 新克隆部署及运行随仓真实 RGB-D＋mock 样例，见 [部署说明](docs/deployment.md)。

## 阅读入口

- [整栈架构](src/peach/peach_bringup/doc/architecture.md)、[跨包输入输出](src/peach/peach_bringup/doc/io.md)、[实现流程](src/peach/peach_bringup/doc/design.md)。
- [接口清单](src/peach/peach_interfaces/config/interface_manifest.yaml)：图名、QoS、生产消费方；字段与数值常量以 IDL 为准。
- [构建、mock、冻结回放与验收](src/peach/peach_system_tests/doc/testing.md)。
- [专题文档与 ADR](docs/README.md)、[项目约束](AGENTS.md)。
- [自主导航采摘目标设计](peach_autonomous_harvest_design/README.md)：设计提案，须与当前实现区分。

包级 `doc/architecture.md`、`doc/io.md` 和 README 随实现维护。历史测试记录必须保留日期、配置及证据路径；lint、mock 完成级别和真实剪切分别判定。

## 构建与无硬件启动

在工作区根执行，依赖使用 Jazzy apt 与 `aubo_py3.12`，`numpy==1.26.4`：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
colcon test --packages-select peach_arm peach_supervisor --return-code-on-test-failure
colcon test-result --verbose
npm --prefix src/ros2_web_link run check

# 启动前盘点进程；本次关闭相机、串口、窗口和自动开批。
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false force_enabled:=false \
  rviz:=false autostart:=false
```

默认 `autostart=false`，执行、接触和工具使能关闭；`execute_pregrasp_only=true`。相机默认关闭，开启时默认 `camera_frontend=stereo`，也支持 percipio 或 replay。`force_enabled` 原默认为 true，无硬件开发应如上显式关闭。mock 默认跳过重建，real 默认完整重建；进程开关 `reconstruction_enabled` 与本批策略 `skip_reconstruction` 分别控制组合与处理流程。

HTTP 操作台为 [8090](http://127.0.0.1:8090)；整栈默认另起回环 ROSbridge（`ws://127.0.0.1:9090`），供 SDK 原生 ROS 通信。ROSbridge 服务/动作有 allowlist；HTTP 调试门与审计只覆盖 HTTP 路径，最终运动/工具授权在 ROS 节点内复检。具体安装和资源构建见 [Web 包](src/peach/peach_web/README.md) 与 [SDK](src/ros2_web_link/README.md)。

测试后按本轮 PID/进程组清理，再盘点进程。自动检查主要是官方 ament lint；感知包另有既存图像解码 pytest。它们不建立现场采摘正确性。修改 IDL/接线后执行：

```bash
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

## 采摘应用源码

| 目录 | 当前职责 |
|---|---|
| [peach_scene_perception](src/peach/peach_scene_perception/README.md) | RGB-D 检测、分割、身份跟踪、锁定集与候选观测 |
| [peach_target_reconstruction](src/peach/peach_target_reconstruction/README.md) | 按目标融合、模型精化、能力与 GraspDecision |
| [peach_supervisor](src/peach/peach_supervisor/README.md) | 批次 FSM、选果、动作编排、结算、账本与补采 |
| [peach_arm](src/peach/peach_arm/README.md) | 拍照/视点、预抓取、套入、工具、撤退与最后授权门 |
| [peach_scene_obstacles](src/peach/peach_scene_obstacles/README.md) | Survey 触发的 PlanningScene world 快照；部署默认禁用建图 |
| [peach_interfaces](src/peach/peach_interfaces/README.md) | 跨包消息、服务、动作与清单，无节点 |
| [peach_common](src/peach/peach_common/README.md)、[peach_vision_core](src/peach/peach_vision_core/README.md) | 共享基础设施与视觉纯核 |
| [peach_bringup](src/peach/peach_bringup/README.md) | 预检、独立能力进程组合、Nav2 生命周期管理、可选 autostart |
| [peach_observability](src/peach/peach_observability/README.md)、[peach_web](src/peach/peach_web/README.md) | HTTP/API、诊断、自检、会话 bag、分析和静态页面 |
| [peach_stereo](src/peach/peach_stereo/README.md) | PS800-E1 主机立体深度前端，与 percipio 互斥 |
| [peach_rviz_plugins](src/peach/peach_rviz_plugins/README.md) | RViz 图像显示兼容插件 |
| [peach_vegetation](src/peach/peach_vegetation/README.md) | 独立二维枝叶分割，未接入 harvest_system |
| [peach_system_tests](src/peach/peach_system_tests/README.md) | 手工探针、冻结回放、联合 runner 与官方 lint |
| [peach_harvester](src/peach/peach_harvester/README.md) | 兼容 metapackage，无代码、配置、模型或组合进程 |

感知、重建和调度由各包 launch 分别启动。跨能力只走 ROS 接口；调度基于 `target_observations` 选果，`GraspDecision` 用作许可。PlanningScene world 归 scene_obstacles，ACM diff 归 arm。五点力用于观测、记录及独立实验，不参与主链运动许可。

## 其他源码与资产

| 路径 | 当前范围 |
|---|---|
| `src/aubo_*` | 柜侧 IDL、模型、硬件/控制器、MoveIt 与驱动 bringup；只读区旧说明见 [现状差异](src/peach/peach_bringup/doc/architecture.md#只读驱动文档的现状差异) |
| [aubo_hand_eye_calibration](src/aubo_hand_eye_calibration/README.md) | 独立手眼/彩色内参标定、激活外参和静态 TF；当前采集/默认结果边界见包级 architecture/io |
| [src/percipio_camera](src/percipio_camera/README.md) | 图漾厂商相机驱动 |
| [src/serial_force](src/serial_force/README.md) | 五点串口驱动、兼容力仿真源与独立 CoP 实验工具 |
| [src/ros2_web_link](src/ros2_web_link/README.md) | 共享浏览器 SDK、ROSbridge 与独立自检工具 |
| [src/ivg](src/ivg/README.md) | 演示/估姿/抓取/快换/拉花及其 Web、桥接、仿真和组合包，独立于 Peach 主链 |
| [src/peach_sim](src/peach_sim/README.md) | 六包仿真子工作区；独立运行器，Gazebo 当前暂停，恢复须用户明确要求 |
| `runs/` | 本机运行与测试证据，不入库 |
| `_archive/` | 本机历史归档，不入库；本轮记录见 归档清单（仅本机；公开历史见 Git 提交记录） |

驱动源码入口：[aubo_msgs](src/aubo_msgs/README.md)、[aubo_description](src/aubo_description/README.md)、[aubo_e5_hardware](src/aubo_e5_hardware/README.md)、[aubo_e5_controllers](src/aubo_e5_controllers/README.md)、[aubo_dashboard](src/aubo_dashboard/README.md)、[aubo_e5_bringup](src/aubo_e5_bringup/README.md)、[aubo_e5_moveit_config](src/aubo_e5_moveit_config/README.md)。

驱动 hardware/controllers、dashboard、ros2_control xacro、driver bringup 和控制器配置只读。真机运动和 SetIO 须操作员授权；硬件急停不经 ROS。导航与 peach2 历史实现已归档，不是当前源码入口。

## 打包迁移

```bash
scripts/pack_workspace.sh --dry-run --out /tmp
scripts/pack_workspace.sh
# 按需附带本机归档或最近运行证据：
scripts/pack_workspace.sh --include-archive --records-days 2
```

实际包含/排除规则以 [打包脚本](scripts/pack_workspace.sh) 为准，恢复步骤见 [迁移说明](README_PACKAGE.md)。默认排除构建/安装/日志、venv、node_modules、运行证据、工具缓存和 `_archive/`；感知模型实际位于 `src/peach/peach_scene_perception/model/`。
