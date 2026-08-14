# AGENTS.md — AUBO E5 ROS 2 Jazzy 工作区

> 面向 AI 编码代理的项目说明。内容以源码、各包 README 与 `docs/` 为权威源，
> 包级细节先查对应 README。项目文档与代码注释以中文为主。
> 当前交付快照（2026-08-13）：采摘批次闭环（拍照前置→收齐锁定→逐目标分级记账→
> 复扫递减集→HarvestSummary）已落地且测试全绿；整机集成与真机验证仍未完成。

## 1. 项目概述

ROS 2 Jazzy（Ubuntu 24.04 / noble）工作区，共 19 个包：为 AUBO E5 六轴机械臂提供
ros2_control 驱动（老控制器固件、vendored 旧 SDK v1.3.1，无源码构建），并叠加桃子
采摘的感知/重建/抓取/编排全链路。

驱动核心为 **passthrough（一次性下发）** 架构，2026-07-27 起替代流式 JTC（旧实现
见 `docs/archive/`）；蓝本为 Humble 实测驱动 aubo_boot，写法参考 UR
`PassthroughTrajectoryController`：

```
MoveIt → FJT goal
  → AuboPassthroughTrajectoryController（按名重排、首点 smoothstep 融合，每周期经
    trajectory_passthrough GPIO 传 1 个设定点，状态机 6→1→2→…→3）
  → AuboE5Hardware::write()（设定点入 SPSC 队列，回写状态机）
  → 发送线程（非 RT，4ms）：五次重采样为 5ms 点 → RIB 水位流控
    → SetRobotPosData2Canbus 批量透传 → 接口板 5ms/点消费
  → 队列排空 → DONE(5) → 控制器 goal_hold → action succeed
```

停止原语三类：正常完成=队列自然排空；取消/抢占=清双队列 + `RobotMoveStop` 主动丢弃
RIB；急停/防护停=仅停发清队（本体安全回路主导）。

## 2. 技术栈与 Python 环境

- C++17（硬件/控制器/dashboard/编排），Python（launch、tools、感知/重建）；
  colcon + ament_cmake / ament_python。
- 全项目 Python 统一用 `aubo_py3.12`（Python 3.12 venv，带 `--system-site-packages`）。
  tools/、diagnostics/ 及一切 Python 调用都用 `aubo_py3.12/bin/python`；依赖锁定在
  根目录 `requirements.txt`（`aubo_py3.12/bin/pip install -r requirements.txt` 可复现）。
- **venv 内 numpy 必须保持 1.26.4（<2）**；禁止 pip 安装 opencv-python/scipy（会
  shadow 系统版，导致 cv2 / cv_bridge 崩溃），详见 requirements.txt 头注释。
- 节点解释器分两层：
  - 纯 ROS 节点（aubo_hand_eye_calibration、peach_perception_web）：系统 python3 +
    console_scripts；
  - torch/open3d 节点（aubo_scene_recon、peach_pose_ros2、peach_reconstruction_ros2）：
    console_scripts + setup.py `build_scripts.executable` 指向 venv（构建期
    `_resolve_python()` 相对解析，不写死路径）；launch 一律标准 `Node()`。
- MoveIt 2（ompl + pilz）不在 apt，来自源码 overlay `~/ws_moveit`（~/.bashrc 自动
  source；新机部署必须同步构建）。

## 3. 仓库结构

### 驱动栈（冻结只读，见第 7 节）

| 包 | 职责 |
|---|---|
| aubo_msgs | 自定义 msg/srv/action（IO、RobotStatus、FK/IK、SetPayload、手眼标定） |
| aubo_description | E5 工作单元 URDF + ros2_control xacro（接口契约与硬件参数表） |
| aubo_e5_hardware | SystemInterface 真机插件（双 SDK 连接 + TCP2CAN 发送线程）与 sim 板级模拟器；vendor SDK |
| aubo_e5_controllers | passthrough FJT 控制器 + IO 控制器（状态/RIB/diagnostics 发布、set_io） |
| aubo_dashboard | 非运动服务节点（上电/断电/停止/FK/IK/负载） |
| aubo_e5_moveit_config | MoveIt 配置与唯一 launch `moveit.launch.py`（controllers.yaml / controllers_mock.yaml） |
| aubo_e5_bringup | 唯一入口 `bringup.launch.py` + controllers.yaml |
| percipio_camera | 厂商相机驱动（仅 launch 默认值与内参配置被项目化；不参与 lint） |

### 手眼标定与场景重建

- `aubo_hand_eye_calibration`：17 预定义位姿 + 多算法求解 + Web；系统 python3。
- `aubo_scene_recon`：Open3D TSDF 点云场景重建；venv 节点。

### 桃子采摘链路

- `peach_pose_msgs` / `peach_harvest_msgs`：感知与采摘编排的类型化 msg/srv/action
  （含 RunHarvest / RunTargetCycle、ControlHarvest、SetOperationPolicy）。
- `peach_pose_ros2`：YOLO+MobileSAM 位姿估计；target_id 在 base_link 世界系空间匹配，
  消失重现 ID 不变；venv。
- `peach_reconstruction_ros2`：连续运动局部重建（精确时刻 TF/FK + 有界 ICP + 在线
  TSDF + 精化拟合）；session 落盘；venv。
- `peach_approach_grasp`：球面自适应视点 + 行为树 + MTC 抓取；周期状态 `CycleState`
  枚举流转、action 终局 outcome 分级（SUCCEEDED/SKIPPED_QUALITY/SKIPPED_UNREACHABLE/
  FAILED/recovery）；`~/go_to_photo_pose` 服务移动到 SRDF 命名状态
  `global_photo_pose`（Pilz PTP+OMPL 兜底）；默认只规划，运动需人工 arm，抓取与
  工具 IO 默认关闭。
- `peach_perception_web`：感知/重建/抓取**只读**监控台（2026-08-13 起移除全部
  Web 写入口：控制/调试/策略/档案代理取消，只留 GET /api/state）：批次过程线
  （就绪→拍照位姿→感知锁定→观察规划→质量验证→靠近抓取→工具动作→撤离收尾→
  完成）+ 过程数据 + 各节点当前参数只读镜像（3s 轮询 get_parameters）；
  系统 python3，默认 127.0.0.1:8090。
- `peach_harvest_orchestrator`：批次唯一所有者（生命周期状态机、四路就绪门、三级策略
  execution→grasp→tool 两阶段提交、RunHarvest action + HarvestSummary、Web 代理类型化
  命令）；批次闭环：拍照前置（go_to_photo_pose→reset_global_targets）→ 感知收齐锁定 →
  按优先级逐目标派发、outcome 记账自动推进 → 复扫递减集循环（`harvest.max_rounds`
  上限）→ COMPLETED；SKIP_TARGET/CANCEL_NOW 真取消活动 goal；
  `config/orchestrator.yaml` 默认 auto_start=true 但 execution/grasp/tool=false；
  整栈入口 `harvest_system.launch.py`。
- `peach_gantry_description` / `peach_moveit_config`：【暂不参与开发/运行】架子式
  采摘机器人模型与 MoveIt 配置。

### 非包目录

- `tools/`：分析/测试脚本（passthrough_traj_client、motion_analyzer、
  peach_dataset_replayer、peach_validation_recorder、fk_ik_check、
  joint_trace_recorder、m2_m3_retest、SDK 检查程序；不随 colcon 构建）。
- `diagnostics/`：SDK 探针（COLCON_IGNORE，独立 CMake，零运动）。
- `hand_eye/`、`test_results/`、`harvest_runs/`、`peach_profiles/`：标定结果、
  运动分析输出、采摘批次结果、操作策略档案。
- `docs/`、`scripts/package_workspace.sh`、`SDK资料/`（厂商资料，勿外传）；
  `build/` `install/` `log/` 为 colcon 产物，勿改勿提交。

19 个包中 17 个接入 lint（12 个 ament_cmake + 5 个 ament_python）；`percipio_camera`
（厂商）与 `peach_moveit_config`（生成配置）除外；vendor 目录含 `AMENT_IGNORE`。

## 4. 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release   # 全量；--packages-select <pkg> 单包
source install/setup.bash
```

三种模式（bringup 参数 `hardware_mode`）：

| 模式 | 硬件插件 | 控制器 | 用途 |
|---|---|---|---|
| mock | mock_components/GenericSystem | 标准 JTC | ros2_control 回归 |
| sim | AuboE5SimHardware | passthrough + IO | 无真机闭环验证首选 |
| real（默认） | AuboE5Hardware | passthrough + IO + dashboard | 真机（普通内核即可） |

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<IP>
ros2 launch aubo_e5_moveit_config moveit.launch.py                      # 只起 MoveIt+rviz2
ros2 launch peach_harvest_orchestrator harvest_system.launch.py         # 采摘整栈
ros2 launch aubo_scene_recon recon.launch.py
ros2 launch peach_pose_ros2 peach_pose.launch.py
ros2 launch peach_reconstruction_ros2 reconstruction.launch.py          # params_file:=<路径> 可覆盖
ros2 launch peach_approach_grasp approach_grasp.launch.py
ros2 launch peach_perception_web peach_perception_web.launch.py
```

bringup 其他参数：`robot_ip`（默认 169.254.10.98）、`moveit_enabled`（true）、
`camera_enabled`（true；sim/mock 无相机建议显式 false）、`extrinsics_enabled`
（true）、`hand_eye_enabled` / `hand_eye_web_enabled`（false）。任意 launch 可用
`--show-args` 查看全部参数及中文说明。

sim 插件与真机契约一致：传输状态机、五次重采样、虚拟接口板 200Hz 每周期消费 1 个
5ms 点、`rib_level`、abort 丢弃队列，执行耗时与真实轨迹一致；sim 不模拟板载 IO，
`set_io` 返回 success=false 属预期。

## 5. 测试与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon test && colcon test-result --verbose    # 全包 lint + 接入的测试

cd src/aubo_hand_eye_calibration && ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/aubo_scene_recon && ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_pose_ros2 && PYTHONPATH=peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_reconstruction_ros2 && PYTHONPATH=.:../peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
cd src/peach_perception_web && ../../aubo_py3.12/bin/python -m pytest test/ -q
```

硬件/控制器验证流程（启动任何 launch/测试/探针前必查旧进程，防止旧二进制与设备独占
干扰判断）：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

1. sim 闭环冒烟：`hardware_mode:=sim camera_enabled:=false` 起 bringup，然后
   `tools/passthrough_traj_client.py wave_shoulder 3`、`sine_shoulder` 压测，
   `tools/motion_analyzer.py run sine_shoulder` 分析（输出 test_results/）。
   已验证基准：三控制器 active、goal-hold 成功、抢占正确、sine 墙钟/标称 ≈1.08、
   终点误差 0、joint_states 200Hz。
2. mock 回归：标准 JTC goal 成功。
3. 真机分阶段（`docs/usage.md` 第 7 节）：只核对 joint_states → 用户现场手动上电并
   确认 → 0.1 缩放低速小轨迹 → 取消/抢占 → MoveIt 整机轨迹 + trace。验收指标：
   点吞吐 ≈200 点/s、RIB 不饿死不溢出（0<level<400）、墙钟/标称 ≈1.0、
   终点误差 <0.02 rad。

## 6. 关键参数与契约

- 权威关节顺序（goal 的 joint_names 会被控制器 remap 到此顺序）：
  `shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。
- hardware 参数（`aubo_description/urdf/aubo_e5.ros2_control.xacro`）：
  `send_period_ms=4`、`rib_target=400`、`rib_slowdown_1/2=300/350`、
  `batch_min/max=2/8`、`ema_alpha=0.1`、`stop_retry_ms=20`、`auto_power_on=false`、
  `max_joint_velocity/acceleration` 等。
- 控制器参数（`aubo_e5_bringup/config/controllers.yaml`）：
  `goal_tolerance_rad=0.02`、`goal_vel_tolerance=0.01`、`goal_hold_frames=5`、
  `goal_check_ms=50`、`goal_time=0.0`（禁用超时）、`blend_threshold_rad=0.01`、
  `blend_steps=30`；controller_manager `update_rate: 200`。
- GPIO 三组接口：`trajectory_passthrough`（transfer_state 0–6 + 设定点 + abort +
  trajectory_size）、`aubo_io`（IO + RIB）、`speed_scaling`（恒 1.0，不做执行期缩放）。
- MoveIt：`controller_names=aubo_passthrough_trajectory_controller` +
  `action_ns=follow_joint_trajectory`；goal 容差可覆盖默认；携带 path_tolerance 会
  拒 goal；速度缩放由 MoveIt 时间参数化完成。
- 采摘链路参数（权威源：各包 `config/*.yaml`，全表见
  `docs/peach_harvest_operations.md` 参数节）：编排器 `photo_pose.*`（拍照前置开关/
  重试/冷却/90s 超时/完成回位）、`harvest.rescan_until_empty`/`harvest.max_rounds=3`
  （复扫递减集）、`harvest.stall_timeout_s=30`（无可选目标停滞提前收口）、
  `dispatch.retry_delay_s=2.0`/`dispatch.max_retries=4`（goal 拒绝冷却重试，
  只在感知 selected 跨帧真变时复位）、`dispatch.max_consecutive_rejections=6`
  （连续拒绝熔断进 RECOVERY_REQUIRED）、5 个跨包接口名；能力端
  `photo_pose_named_target='global_photo_pose'`，
  速度双档 `moveit.velocity_scaling=0.05`（接触段 MTC）/
  `moveit.transit_velocity_scaling=0.10`（自由空间转移；0.01 蠕行曾在高重力矩姿态
  持续过久而关节过流，低速档不得让高重力矩姿态持续过久）、
  `scan.maximum_moves=5`/`scan.min_camera_height_m=0.06`（桌面保护平面过滤）、
  `scan.time_budget_s=5.0`（观察段时间盒，到期强制 finalize 走降级链；
  暂时测试设置，实际工况再调）、
  质量门验证档 `quality.minimum_views=3`/`minimum_baseline_deg=15`/
  `maximum_refined_rmse_m=0.01`（精化未达标时回退候选锚点降级抓取，非极端必抓）；
  感知 `harvest.min_collect_frames=10`/`lock_settle_frames=5`/`max_collect_s=25.0`/
  `priority_prefer_lower_first=true`（收齐窗口与优先级；max_collect_s 运行期按实测
  帧间隔 EMA 自适应伸缩、有未确认记录在攒帧就不关窗防锁空集）、`yolo_conf`/
  `min_detection_conf=0.25`（逆光 0.3 全滤真实目标；不宜再低，误检由确认机制兜底）、
  `target_memory.recovery_scale=3.5`（跨视角锚点偏差
  实测 15.7cm，恢复半径 21cm 兜底）、`target_memory.tentative_ttl_frames=5`
  （未确认目标存活按帧计，帧率以运行状态为准）；帧率自适应超时——能力端
  `scan.frame_wait_s`/`execution.target_observation_max_age_s` 均按观测话题实测
  帧间隔 EMA 伸缩（配置值为回退/上限）；重建节点 1Hz 活性心跳（status/diagnostics/
  grasp_decision，无心跳时编排器重建就绪门会因 2s 新鲜度永远不满足）；视角覆盖
  指标按机位聚类（同机位连帧不稀释基线）。
- Python 节点参数以各包 `config/*.yaml` 为权威源，代码内 `declare_parameter` 默认值
  必须逐一对齐。

## 7. 开发约定与安全红线

- **真机驱动栈冻结只读，禁止修改**：`src/aubo_e5_hardware/`、
  `src/aubo_e5_controllers/`、`src/aubo_dashboard/`、
  `src/aubo_description/urdf/aubo_e5.ros2_control.xacro`、
  `src/aubo_e5_bringup/launch/bringup.launch.py`、
  `src/aubo_e5_bringup/config/controllers.yaml`，以及
  `src/aubo_e5_moveit_config/config/controllers.yaml` 中的真机映射。
  允许读/构建/测试/报告，缺陷只能记录风险与建议、等待用户授权；感知、重建、工具、
  测试、文档可改，但不得改变或绕过真机驱动接口。
- **上电只能由用户现场手动完成**：AI、launch、脚本、测试禁止调用
  `/aubo_dashboard/startup`；`auto_power_on=false` 必须保持不变；急停/防护停由本体
  安全回路主导，软件只停发清队，不做“恢复”。
- 真机任何运动测试先压速度/加速度缩放到 0.1，确认安全后再放宽；现场先确认急停、
  限位、碰撞等级、低速模式。
- passthrough 语义优先于自由发挥：一次性下发、goal_hold、RIB 流控等不得改回流式；
  硬件插件保留旧式 export_state/command_interfaces（UR 验证写法，勿“升级”）。
- 控制器参数用 generate_parameter_library（改
  `src/aubo_e5_controllers/src/*_parameters.yaml`，不手写参数解析）。
- peach 视觉包四层架构（params.py / interfaces.py 注册表 / 数据层零 ROS / 编排
  节点），纯核子包零 ROS import 由单测强制；重建只用精确时间戳 TF、禁止 latest TF
  回退、ICP 只做有界小修正；`peach_harvest_orchestrator` 是批次唯一所有者，抓取只
  执行稳定 target_id。
- 风格由 `colcon test` 强制：C++（uncrustify/cpplint，100 列）、Python（flake8，
  99 列/单引号）、CMake（lint_cmake）；改代码保持 lint 全绿并同步注释与 docs。
- `aubo_e5_hardware` 公共头 include 了 vendor SDK 头且 include 路径为 PRIVATE，
  不支持下游 include（未来需要先移出 SDK 类型）。
- 不向 `build/` `install/` `log/` 提交改动。

## 8. 部署坑点

- **libprotobuf.so.9**：SDK 强依赖 protobuf 2.6.1，vendor 在
  `aubo_e5_hardware/vendor/lib64/`；链接强制 DT_RPATH，手动跑二进制需设
  LD_LIBRARY_PATH 指向 install 下 vendor 目录（经 launch 启动无此问题）。
- **SDK 按进程 CWD 读配置**：`./config/auborobot.conf` 与 `tracelog.properties`；
  launch 已把 `ros2_control_node` 与 dashboard 的 cwd 设为各自 share 目录。
- **TCP2CAN 独占**：activate 后 SDK 运动 API 被独占、示教器运动暂停；deactivate 恢复。
- 已取消 RT 要求（普通内核直接运行；NVIDIA 595 无 PREEMPT_RT 模块）；网卡 r8126
  DKMS 曾致 SDK 通道秒级停滞，见 `docs/nic_driver_incident.md`。
- **每次开机必做（不持久）**：`sudo ethtool -K enp130s0 gro off gso off tso off` +
  `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`，
  否则 SDK 推送链路可能 >200ms 停滞触发 read() FAULT。
- MoveIt 不在 apt：依赖 `~/ws_moveit` 源码 overlay。
- tf2 静态 TF 不接受同发布者覆盖：更新 wrist3_Link→camera_link 后必须重启
  extrinsics_publisher（或整个标定 launch）。
- SDK 凭据走 xacro 参数，dashboard 需单独传 `sdk_password`；不要把真实凭据、
  `SDK资料/` 或 vendor 二进制提交/外传。

## 9. 参考

- `README.md` — 项目入口与当前交付状态
- `docs/usage.md` — 完整命令手册与排障表
- `docs/passthrough_migration.md` — 驱动架构迁移说明
- `docs/peach_harvest_operations.md` — 采摘编排/Web/类型化接口手册
- `docs/peach_pose_reconstruction_integration.md` — 感知与重建联动契约
- `docs/peach_auto_grasp_overview.md` — 自动抓取全流程图解与细节说明（mermaid + 静态 PNG）
- `docs/code_review_peach_grasp.md` — 2026-08-13 自动抓取链路代码审查报告（P0/P1 修复清单）
- `docs/peach_perception_progress.md`、`docs/peach_perception_reconstruction_logic.md`
  — 感知/重建进度台账与逻辑图解
- `docs/source_audit_2026-08-10.md` — 源码审查记录
- `docs/superpowers/specs/` — 功能设计文档（含
  `2026-08-12-peach-commercial-orchestration-design.md`、
  `2026-08-14-peach-harvest-integral-design.md` 整体分层设计/状态机/接口契约/
  安全门分层/Refinement Journal 规划/问题登记册）
- `docs/archive/` — 旧架构文档（仅供历史参考）
- 蓝本：aubo_boot（`/home/mu/Music/e`，不随交付分发）；写法参考 UR
  `PassthroughTrajectoryController`
