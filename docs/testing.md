# 测试流程与命名

现行系统（SNAPSHOT）：源码。与 [architecture.md](architecture.md)、[io.md](io.md) 构成仅有的三份活文档；**源码与本文互相更新，改启动/验收口径或改本文须同一轮改另一边**。**如何演化**以 [AGENTS.md](../AGENTS.md) 为准：非完美适配当前真机/产品则跟 ROS 2 / 优秀 GitHub 主流。

真机轮次、量化基线、审查记录写在 [testing-log.md](testing-log.md)；工程整理过程写在 [REFACTORING.md](REFACTORING.md)（二者都是过程记录，不驱动现行设计）。改行为只改本文 + 源码；补一条实测时追加 testing-log，不把轮次散文写回本文。

各包 `test/` **保留 ROS 2 默认 lint，并允许零 ROS 纯核 pytest**（Python：`test_flake8.py` / `test_pep257.py` + 不 import rclpy 的表驱动；CMake：`ament_lint_auto`）。现行纯核：`peach_executor/test/test_harvest_fsm.py`（`react` 表）、`peach_observability/test/test_bag_report.py`（bag 流→报告合成、验收门、回收选择，零 ROS）、`peach_perception/test/test_runtime_core.py`（`ManualClock` / `BoundedWorker` capacity=1 drop_oldest）、`peach_perception/test/test_tool_profiles.py`（工具档案解析结构校验）、`peach_manipulation/test/test_contact_monitor.py`（合成电流序列编译 `contact_monitor.hpp`）、`ivg_graspnet/test/test_grasp_core.py`（`GraspList` NMS/碰撞；torch 算子 `importorskip`）、`serial_imu/test/test_protocol.py`（切帧/协方差）与 `test_frame.py`（倒装 Rx + parent 对齐）、`imu_follow/test/test_core.py`（姿态增量/死区锥钳/平滑/关节步长/插入推进）。现行测试面以 lint + 零 ROS 纯核为主（决策 0006，**UNWIND**：不是套袋工艺的完美适配）。**新测试按 [AGENTS.md](../AGENTS.md) 测试塔与官方 / Nav2 / Autoware 主流**：允许 gtest、launch_testing（isolated `ROS_DOMAIN_ID`）、`mock_components` 集成。独立系统测包 `peach_system_tests` 已落地（mock `harvest_system`，不发 `RunHarvest`）；Gazebo/Isaac 物理仿真仍缺口。采摘方向 / 接触对错仍以真机 `runs/` + [testing-log.md](testing-log.md) 为最终权威（KEEP）；`colcon test` 绿不是田间验收。语法与流程由审查核对。套入剪切软件门看 flake8 / pep257 / uncrustify 与纯核表；`peach_manipulation` 整测项跳过 cpplint（其 legal/copyright 与 Google include 顺序检查同本项目「文件头版权块项目结束再补」「include own-first」约定冲突，CMake 已 `set(ament_cmake_cpplint_FOUND TRUE)`），C++ 风格门以 uncrustify 为准、静态分析走 cppcheck。`ament_xmllint` 会拉 `package_format3.xsd`，网络卡住超时不阻塞本产品路径。

不要删 `_archive/runs/` 与现场 `runs/` 的文本与账本；bag 二进制例外——observability 按 `record.max_total_bag_gb` 预算自动回收最旧的 `session_*/bag` 与旧 `mcap_*`（解析总结 `bag_report.md/json`、账本与一切文本保留，回收逐条写 `runs/retention_audit.jsonl`）。未授权不得真机运动或 SetIO。硬件急停在示教器/柜，不经 ROS。launch **不自动** `RunHarvest`。采摘应用七包职责见 [architecture.md](architecture.md) §3。旁路视觉抓取四包（`ivg_interfaces` / `ivg_utils` / `ivg_pose_estimation` / `ivg_graspnet`）不进整栈 launch。`serial_imu` 随 `harvest_system` 起（`imu_enabled` 默认 true），不进 lifecycle、不进只读 bringup。`imu_follow`（IMU 姿态跟随）独立 launch、不随整栈，`motion.enabled` 默认 false 只算不发。

---

## 命名

批次 `request_id` 同时是账本目录名 `runs/<request_id>/`，**不得复用**。开批前按当时本地时间填 `YYYYMMDD` 与 `HHMM`。

| 用途 | 格式 | 例 |
|------|------|-----|
| 预抓取真机 | `field_pregrasp_<YYYYMMDD>_<HHMM>` | `field_pregrasp_20260901_1757` |
| 接触干跑（不开刀） | `field_full_<YYYYMMDD>_<HHMM>` | `field_full_20260825_1851` |
| 只扫不运动 | `field_dry`；或预抓取名 + `intent: 2` | `intent: 2` = SURVEY_ONLY |
| 开发机 mock | `dev` | `intent` 默认 0 |
| 当日综述 | `runs/field_test_<YYYYMMDD>/log.md` | `runs/field_test_20260901/log.md` |
| 监控会话 | `runs/session_<YYYYMMDD>_<HHMMSS>/bag/`（观测自动，随栈启停开合） | 内含 `bag_0.mcap` 与自动生成的 `bag_report.md/json`，与账本互引 |

场景键 `scene_key` 现行实验室用 `lab`。`profile_id` 现行 `default`。

写记录：当场把结论写入 `runs/field_test_<日期>/log.md`，并追加 [testing-log.md](testing-log.md) 对应轮次。`runs/` 结构化文本（jsonl/json/csv/md/yaml/txt/log）入库随仓推送，克隆即可离线复算/分析；过程 bag（`session_*/bag` 的 mcap）、图像/点云等二进制仍只留本地（.gitignore 白名单）。会话报告由 observability 在停栈时自动生成，也可随时手动复跑：`ros2 run peach_observability peach_bag_report runs/session_*/bag`。

---

## 1. 构建与开发机

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

R0+ 纯核门（不启 DDS、不动臂；`colcon test` 绿 ≠ 套袋验收）：

```bash
bash scripts/r0_gate.sh
# source Jazzy 后：
# colcon test --packages-select peach_interfaces peach_perception peach_executor peach_manipulation peach_bringup peach_observability peach_system_tests
```

现行纯核另含：`param_rules` / `identity` / `tool_budget` / `harvest_fsm`（EventHold） / `idl_constants`（HarvestState/ControlTask/ManageLifecycleNodes 数值对账） / `path_metrics` / `domain`（reducer、ledger、watchdog、model_contract、evidence） / `pregrasp_level` gtest。Python 键名冻结测试对照 yaml（感知两节点 + 调度/观测/lifecycle）；C++ 新合同字段走 generate_parameter_library。`.github/workflows/jazzy.yaml`：`peach-core` 跑同一纯核门 + numpy 1.26.4；`industrial_ci` 在 Docker 里 `colcon` 编测驱动与 peach（`COLCON_IGNORE` IVG 四包、`imu_follow`、`percipio_camera`，无真机 job）。scipy 不进 `package.xml`（venv-first KEEP）；ICI 用 apt `python3-scipy` / `python3-pytest` / `python3-yaml`（`ros:jazzy` numpy 已 1.26.4，不再 Docker 内 pip）。`peach_system_tests` isolated launch_testing 起 mock `harvest_system`（`camera_enabled:=false` `imu_enabled:=false`，`QT_QPA_PLATFORM=offscreen`，`AUBO_RUNS_DIR` 指临时目录），断言 `/joint_states` 含 MUST 六关节名（JSB 的 name 数组常为字母序，按下标当 MUST 序会拧腕）与 lifecycle Active，**不**发 `RunHarvest`。headless 下 `move_group`/`rviz2` 退出码不纳入 peach 进程门。本机已有栈残留时预检拒测（只认节点 argv0 或 `.../lib/<pkg>/<node>`，不认 colcon 包名参数）。

全新机器 / 新环境自检与部署：`scripts/env_bootstrap.sh check|install|all`（幂等；check 零改动、退出码=缺失项数，`SMOKE=1` 追加 mock 冒烟）。脚本内 apt/venv/udev 清单是依赖事实源之一，变更依赖须四处同步：package.xml、requirements.txt、脚本清单、本节。

有残留按 PID 补杀。clangd：上述 `CMAKE_EXPORT_COMPILE_COMMANDS` 让每个 CMake 包在 `build/<pkg>/compile_commands.json` 留下编译命令；工作区 `.clangd` 按包指向这些文件。驱动栈 CMakeLists 只读，不在那些包里写 `set(CMAKE_EXPORT_COMPILE_COMMANDS)`。改完 CMake 或新编一包后 **Clangd: Restart language server**。Python：`aubo_py3.12`。依赖分层（venv-first）：ROS 2 依赖走 Jazzy apt；其余第三方（numpy/scipy/opencv/PyYAML/open3d/torch 等）一律由工作区 `requirements.txt` 钉版本装进 venv（对同名 apt 包需 `pip install --ignore-installed -r requirements.txt` 才真正落入 venv）。**numpy 必须 ==1.26.4**（<2）：Jazzy 的 cv_bridge 二进制按 numpy 1.x 编译，numpy 2.x 会 `import cv2` 报错、`import cv_bridge` 段错误；该版本同时是 apt python3-numpy 的版本，双路径一致。感知身份分配与手眼标定共用 scipy（venv 内 1.11.4）；`peach_perception` 的 package.xml 只声明 ROS 键与 `python3-numpy`（ABI 边界），数值库不走 rosdep。感知与调度/监控/lifecycle 手写参数模块（`params.py`）落在 `install/` 与源码包 `peach_perception/peach_perception/`、`peach_executor/peach_executor/`（随包提交，非生成物）；从源码树直接跑脚本时把 `PYTHONPATH` 指到 `src/peach_*` 需带 venv 的 ROS 依赖，否则节点会在 import 期退出，lifecycle 拉不齐 Active。本机若 venv 抢了 `PYTHONPATH`，launch 前先清再只留 Jazzy site-packages 并重新 `source` 两份 setup（见 §4 复现命令）。跨包轴向后撤只改 `src/peach_perception/config/grasp_standoffs.yaml` 两行；不要把它当 ROS `ParameterFile` 直接喂节点（rcl 不允许 `ros__parameters` 之前出现裸值）。能力 launch 读入后注入已声明参数。

```bash
# 开发机：无相机、不运动
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
# 等价薄转发：ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false

# mock 轨迹回放（1757/1740 过程坐标；官方 GenerateGraspPose+LIN Fallbacks，不开批）
python3 scripts/replay_field_pregrasp.py --case 1757
python3 scripts/replay_field_pregrasp.py --case 1740 --planner ptp   # OMPL/PTP 绕行对照
# 过护栏后再下发 mock 控制器：加 --execute（不动真机）

# mock 全链路回放（09-09 现场逐目标坐标驱动真实技能节点 PREGRASP_ONLY；
# 注入感知/重建话题 + robot_status + 光学系 TF。回拍照位走周期内
# goToPhotoPose（接近原路返程，否则 PTP），与正式接触段同一 C++ 函数，脚本不直连 JTC）
python3 scripts/sim_field_targets.py --list
python3 scripts/sim_field_targets.py --case all
# 现场典型包络随机位姿（axis_z≥0.70 且 |entry|≤1.02；感知允许水平，压测加 --envelope algorithm）
python3 scripts/sim_field_targets.py --random 16 --seed 20260910
# 接近轨迹形状对照（typical；seed 与 09-11 审查轮一致；--velocity 1.0 仅 mock）
python3 scripts/sim_field_targets.py --random 30 --seed 20260911 --velocity 1.0
# 大样本 + 仿真提速（速度/加速度缩放设 1.0，仅 mock；100 例约 10 min）
python3 scripts/sim_field_targets.py --random 100 --seed 20260910 --velocity 1.0
# 感知算法包络压测（含近水平；seed 与 09-11 对照轮一致）
python3 scripts/sim_field_targets.py --random 100 --envelope algorithm --seed 20260911 --velocity 1.0
# 接近失败根因探针：G/预抓取/staging 逐滚转 IK + 直弦 fraction（只读诊断）
python3 scripts/sim_approach_probe.py --random 100 --seed 20260910
# 解析覆盖（不执臂）：感知包络 + TCP 测地线 + 果实胶囊；10000 分层位姿约 2 s
python3 scripts/analyze_approach_envelope.py --n 10000 --seed 20260911
# 解析约束不变量验证（不执臂）：I1–I5（果实胶囊有限圆柱）+ 姿态门闭式 + 拒发结合度
python3 scripts/analytic_constraints.py --n 10000 --seed 20260911
# 只读可达性验证（不执臂、不开周期）：同批位姿在 staging/预抓取逐档查 /compute_ik
python3 scripts/analytic_reachability.py --n 10000 --seed 20260911
# 实时轨迹 watchdog：执行中 FK 监测；默认只记录。技能节点笛卡尔门 1.8/0.25/0.08
python3 scripts/trajectory_watchdog.py            # 只记录，不停轨
python3 scripts/trajectory_watchdog.py --ratio 1.8 --dev 0.25 --recede 0.08

# 真机（须显式 real；示教器上电；bringup 不起 aubo_dashboard）
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

旁路视觉抓取（独立 launch，不进上面这条整栈）。lint/纯核走 colcon；Web 回归与 GraspNet torch 算子须 `aubo_py3.12`。GraspNet 权重 `src/ivg_graspnet/models/checkpoint-rs.tar` 随库；估姿 rembg 的 `u2net.onnx`（约 168MB）超远程单文件上限不入库，clone 后执行 `src/ivg_pose_estimation/ivg_pose_estimation/models/fetch_u2net.sh`（或首次抠图时 rembg/pooch 下载）：

```bash
colcon test --packages-select ivg_interfaces ivg_pose_estimation ivg_graspnet
# GraspNet 纯核（torch 在 venv）
./aubo_py3.12/bin/python -m pytest src/ivg_graspnet/test/test_grasp_core.py
# 估姿 Web 回归（fastapi/httpx 在 venv；系统 python 下整文件 skip）
./aubo_py3.12/bin/python -m pytest src/ivg_pose_estimation/ivg_pose_estimation/test/test_web_app.py
# 接触检测纯核（g++ 编译 contact_monitor.hpp，零 ROS）
python3 -m pytest src/peach_manipulation/test/test_contact_monitor.py
```

监控：`http://127.0.0.1:8090`。参数 `peach_executor/config/observability.yaml`。过程页：作业票（发现→完成）、事件、落盘目录、TCP 俯视（绕行比、Δz、对照预抓取/入口/弦）、本场目标、柜侧硬件（TCP xyz/rpy、六轴角/速度、电流 SDK 原单位、温度、跟随误差）。`/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor` / `robot`（`status` / `tcp` / `joints`）/ `metrics` / `record` / `params` / `job` / `debug`。默认不上电、不派发运动、不打工具 IO、不自动开批。

### Web 单步调试（决策 0018）

Tab「调试」＝向**既有**动作/服务发请求的纯客户端，页面只留本管线：BeginScene、SurveyScene、Build/finalize、ExecuteTarget、去拍照位、RunHarvest、CANCEL_NOW。无令牌。`debug.enabled` 默认 true（false 时 POST 503）。运动类另需 `debug.motion_enabled=true`（false=423）。审计 `runs/debug_audit/<日期>.jsonl`。

动臂（真机手调预抓取等）：

```bash
# yaml：debug.motion_enabled: true
# （技能侧 execution/grasp/tool 仍须另行打开，ExecutionAuthority 照常复核）
# 浏览器 8090 → 调试 Tab
```

门控口径：无令牌、无 401。`motion_enabled=false` 时 Survey、ExecuteTarget 非 PREVIEW（含 OBSERVE_ONLY）、go_to_photo_pose、非 SURVEY_ONLY 的 RunHarvest 一律 `423`；PREVIEW/BeginScene/finalize 不拦。未知端点 `404`。Web **绕不过** `ExecutionAuthority`、调度使能与重建门。

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | mock | mock / real |
| `robot_ip` | 169.254.10.98 | 仅 real |
| `tool_profile` | adaptive_cylinder_v1 | 末端工具档案（URDF TCP、感知许可内径、消息标签统一随档案切换）；固定圆柱显式 `tool_profile:=hollow_cylinder_v1`。切换须整栈重启，RSP 与 move_group 同 arg |
| `camera_enabled` | false | 有相机时设 true |
| `imu_enabled` | true | USB IMU；挂 tcp 并对齐。无设备时节点重试。关掉：`false` |
| `extrinsics_enabled` | true | wrist3 → camera_link |
| `moveit_enabled` | true | move_group + RViz |
| `hand_eye_enabled` | false | 标定流程 |
| `hand_eye_web_enabled` | false | 标定 Web `:8088` |
| 调度 `execute_pregrasp_only` | true | 接触段 `PREGRASP_ONLY`：停预抓取不回 stow；套入前改 false |

过程录制不再有 launch 参数：observability 的 `record.enabled`（默认 true）随栈开合会话 bag，栈停自动出报告；`record.bag_topics`/`record.max_total_bag_gb` 见 `config/observability.yaml`。

完整列表：`--show-args`。只起手臂：`ros2 launch aubo_e5_bringup bringup.launch.py …`。

USB IMU（随整栈，不进 lifecycle）：手册 [`src/serial_imu/README.md`](../src/serial_imu/README.md)（udev、协议、TF、RViz 插件项、dialout/`newgrp`）。`harvest_system` 默认 `imu_enabled:=true`（mock / real 相同）：`tf_parent_frame:=tcp`、`align_to_parent:=true`、不起 IMU 自己的 RViz。画面在 MoveIt RViz **Peach → Imu**（订 `/imu/data`）。无 USB 时每 2 s 重试串口，不挡整栈。关掉：`imu_enabled:=false`。不要另起 `serial_imu.launch.py` 与整栈并行。摘要：

```bash
sudo usermod -aG dialout $USER && newgrp dialout
# 随 mock / real 整栈（默认已开）
ros2 launch peach_executor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
ros2 topic echo /imu/data
# 只看 IMU、不起采摘：
ros2 launch serial_imu serial_imu.launch.py
```
`usermod` 后必须 `newgrp`（或重新登录）。缺插件：`sudo apt install ros-jazzy-imu-tools`。整栈里 Fixed Frame 用 `base_link`；姿态看 `/imu/data`（Rx(180°) + 对齐到 tcp）。单独 launch 时 Fixed Frame `world`。静置：`linear_acceleration.z` 为正（~+9.6）。再对齐：`ros2 service call /imu/align_to_parent std_srvs/srv/Trigger`。健康：`ros2 topic echo /diagnostics`。纯核：`PYTHONPATH=src/serial_imu pytest src/serial_imu/test/test_protocol.py src/serial_imu/test/test_frame.py`。协方差：未提供 `[0]=-1`，未知全 0。话题 QoS Reliable。

IMU 姿态跟随（`imu_follow`，独立工具包不随整栈；**servo 主路径** 09-14 mock 全指标过：15°→目标 0.2618/关节 0.50、锥钳 0.3500、死区归零、回位 0.0002、位置零漂移、拒收告警 0）：

```bash
# 1) bringup mock + 导拍照位（冷启动全零位 IK 无解 -31，必做）
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=mock camera_enabled:=false
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [shoulder_joint, \
  upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint], \
  points: [{positions: [0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621], \
  time_from_start: {sec: 6}}]}}"
# 2) servo + 跟随节点（默认只算不发；moveit_servo 用 Jazzy apt）
ros2 launch imu_follow imu_follow_servo.launch.py
# 3) 假 IMU（无 USB 时用独立话题，launch 传 imu_topic；真 IMU 走默认 /imu/data）
ros2 topic pub -r 20 /imu_data_fake sensor_msgs/msg/Imu \
  '{header: {frame_id: imu_link}, orientation: {w: 1.0}}'   # 另起时加 imu_topic:=/imu_data_fake
# 4) enable（自动激活 servo）→ dry 检查 → 开门
ros2 service call /imu_follow/enable std_srvs/srv/Trigger
ros2 topic echo /imu_follow/target_pose --once
ros2 param set /imu_follow motion.enabled true
ros2 topic echo /moveit_servo/status --once       # 0=No warnings
# 停 / fjt 备选
ros2 service call /imu_follow/disable std_srvs/srv/Trigger
ros2 param set /imu_follow motion.backend fjt
```

验收口径：enable 后静置目标=参考；转动输入源目标增量=IMU 相对增量（锥 0.35 rad 内、死区 0.02 rad 外），开门后 `/joint_states` 随动、`/moveit_servo/status`=0。**servo 三坑（已修在包内，换环境重查）**：twist 输入必须 BEST_EFFORT 发布（可靠 QoS 收不到）；此版 servo 未 `switch_command_type(TWIST)` 拒收（enable 自动调）；其参数名自带 `moveit_servo.` 前缀（yaml 按此写）。`/imu/data` 勿与假发布器混流。纯核：`PYTHONPATH=src/imu_follow pytest src/imu_follow/test/test_core.py`。

**插入推进模式（自适应圆柱套入，2026-09-15）**——目的：视觉给的袋轴/入口不够准时，柔性筒+IMU 姿态跟随弥补，保证套入。编排为人工分段（peach 套入 LIN 段不动、与 imu_follow 零耦合；勿在 peach MTC 执行期间同时开门）：

```bash
# 前置同上（bringup + servo + enable + motion.enabled true）
# peach 侧 execute_pregrasp_only=true 停在预抓取（默认），臂静止后：
ros2 service call /imu_follow/insert_start std_srvs/srv/Trigger   # 沿工具开口 0.01 m/s 推进，行程钳 0.20 m
ros2 topic echo /imu_follow/target_pose --once                    # 位置目标沿推进前移、姿态跟 IMU
ros2 service call /imu_follow/insert_stop std_srvs/srv/Trigger    # 停推进（跟随保持）；disable 全停
```

验收口径：`insert_start` 后 `~/target_pose` 位置沿锁定方向匀速前移（0.01 m/s，到 0.20 m 自动停并告警）；姿态仍只跟 IMU 增量；`insert_stop`/`disable`/断流即停推进；停跟用 `~/disable`（不要只 param set false）。切刀/撤退仍走 peach Web 单步（须 `debug.motion_enabled`）。

```bash
ros2 action send_goal /peach_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'dev', scene_key: 'lab', profile_id: 'default'}"
ros2 topic echo /peach_executor/state
ros2 service call /peach_executor/control peach_interfaces/srv/ControlTask \
  "{command: 0, expected_state_seq: 0}"
# 预抓取看完方向/定位后 ACK（命令 6），才允许再 Survey
ros2 service call /peach_executor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0}"
```

打开真运动须同时改调度 `execution_enabled` 与技能 `execution.enabled`，并经人工授权。到预抓取还须 `grasp.enabled=true`、`tool.enabled=false`。调度侧用 `ros2 param set` 即可（开批与 `HarvestState` 会刷新快照）；技能侧 `ros2 param set` 空闲态全量生效、运行中拒改（execution→grasp→tool 依赖链由节点校验）。调参分层：改默认值改 `params.py`/`params.hpp` 并同步 `config/<节点>.yaml` 同键（决策 0017，须重编）；改本机部署值只改 `config/<节点>.yaml`；运行期临时改参用 `ros2 param set`。

---

## 2. 真机干跑（默认不运动、不 SetIO）

`auto_power_on` 必须为 false。柜侧用示教器；规划/FK/IK 用 MoveIt；停轨走透传取消 + 硬件 `RobotMoveStop`（应用停轨，**不是** ISO 13850 急停）。禁止调用 `aubo_dashboard`。真机授权前：示教器急停手能摸到；工作空间无人或已隔离；使能保持默认关。急停或保护停止之后：处理现场 → 示教器复位 → 取消并丢弃 ROS 在途 goal → 重新授权后再下发，**不要 resume 原轨迹**（UR ROS2 Driver 同类警告）。分层见 [AGENTS.md](../AGENTS.md) 第 2 章。

过程数据：新记录在工作区 `runs/`。08-20～08-24 在 `_archive/runs/root_2026-08-24/`。每次干跑把结论写进 `runs/field_test_<日期>/log.md`，并追加 [testing-log.md](testing-log.md)。过程录制为会话 bag（决策 0019）：observability 随栈开合 `runs/session_<时间戳>/bag/`（events/state/感知/重建/许可/技能/`/tf`/关节量/图像点云/job/metrics 全流 MCAP），栈停自动生成 `bag_report.md`（头部含验收门对照：帧率 ≥2.0 / tf_failures=0 / 到预抓取停住 ≥1，口径见下；按 request_id 分批还原 outcome 与阶段耗时，并与 `runs/<request_id>/ledger.json` 互引）；随时可 `ros2 run peach_observability peach_bag_report <bag>` 复跑。终局事件 `message` 带 `failure_code`，原因列直接可读。恢复等待（Hold 等 ACK）期间照常录制；bag 体积超 `record.max_total_bag_gb` 时停栈后自动回收最旧 bag（总结/账本/文本保留，审计在 `runs/retention_audit.jsonl`）。

### 档位（干跑默认）

| 项 | 值 |
|----|----|
| `hardware_mode` | real |
| `camera_enabled` | true |
| 调度 `execution_enabled` | false |
| 技能 `execution.enabled` / `grasp.enabled` / `tool.enabled` | false |
| launch | 不自动 RunHarvest |

### 冒烟

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /aubo_io_controller/robot_status
ros2 topic hz /camera/color/image_raw
ros2 topic echo --once /peach_executor/state
ros2 topic echo --once /peach/perception/target_observations
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/state
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/trajectory
ros2 lifecycle get /peach_observability
ros2 lifecycle get /peach_scene_perception_node
ros2 lifecycle get /peach_target_reconstruction_node
ros2 lifecycle get /peach_manipulation_node
ros2 lifecycle get /peach_executor
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

关节名必须是：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。把 `/joint_states` 对照 SRDF `global_photo_pose`（`src/aubo_e5_moveit_config/config/aubo_e5.srdf` 的 `group_state`）。launch / lifecycle **不到**拍照位；开执行后第一次 `SurveyScene` 才 PTP 过去。`execution.enabled=false` 时 Survey 仍核**当前**关节：停在袋口开批须 `termination_reason=survey_failed`，不得把袋口 FOV 收进本批锁定集。上一轮若停在 HoldPregrasp，当前多半还在袋口——差值大时先目视/示教器确认再开 `execution`。`ros2 topic hz /camera/color/image_raw` 的订阅 QoS 须与发布端一致。Percipio launch 默认 `color_qos:=default`（RELIABLE）；感知订户也是 RELIABLE depth 10。若改成 `SENSOR_DATA`（BEST_EFFORT），默认可靠的 `hz` 会误报未发布。帧率以 Percipio `fps ≈ 2.43` 与感知注册表为准。四节点须 Active。重建有时停在 inactive：`ros2 lifecycle set /peach_target_reconstruction_node activate`。固定座无导航动作（`NavigateToWorksite` 预留，调度 `_cmd_navigate` 直通 `NAV_OK`）。

显式只扫（仍不运动）：

```bash
ros2 action send_goal /peach_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
```

`intent: 2` = SURVEY_ONLY。技能 `execution.enabled=false` 时 Survey **仍核当前关节**（只规划不够）。调度会先 Survey、再 Begin、再 WAIT_LOCK，然后结算（不选果）。默认 intent 0 且调度 `execution_enabled` 关时同样：拍照位失败则 `survey_failed`；成功则锁定后直接结算（不选果、不记 `SKIPPED_QUALITY`）。全流程到预抓取须两边 `execution=true` 且技能 `grasp.enabled=true`。

停栈：launch 终端 Ctrl+C，再 `pgrep`。

### 透传冒烟（仅 real）

```bash
aubo_py3.12/bin/python _archive/parked_2026-08-24/tools/passthrough_traj_client.py wave_shoulder
```

### 手眼

日常采摘不跑标定，但必须有 `src/aubo_hand_eye_calibration/hand_eye/active.yaml`（gitignore）。没有则名义 TF 平移 2 cm、单位四元数，光学系会偏约 10 cm。现场副本 `_archive/runs/hand_eye/`。平移应接近 `[0.045, 0.108, 0.002]`，不是 `[0, 0, 0.020]`。

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
# 浏览器只开回环 http://127.0.0.1:8088
ros2 service call /hand_eye_extrinsics_publisher/reload std_srvs/srv/Trigger {}
```

### 授权后真运动（本页不写默认使能）

须同时打开调度与技能 `execution`。`RunHarvest` / `ExecuteTarget` / `SurveyScene` 动作入口会自动 arm；手动 Trigger 才 `~/set_execution_armed`。抓取再开 `grasp.enabled`；工具再开 `tool.enabled`。卸果站为预留（`DepositToStation` 已删，`DepositResult` 恒 `deposited=false`，无 `deposit_pose_named_target` 参数）。使能顺序：先关 `grasp` 再关 `execution`（依赖链 execution→grasp→tool）。

---

## 3. RViz（`aubo_e5_moveit_config/rviz/moveit.rviz`）

Fixed Frame 用 **`base_link`**，不要用未接上的 `world`。改显示配置后须重装 `aubo_e5_moveit_config` 并重启 RViz。

现场优先看 **Debug Image** 与 **Perception Markers**；重建开始后看 **TSDF Cloud** 与 **Reconstruction Markers**。不要同时开 Camera Points 和 Detection Cloud。

| 显示名 | 默认 | 话题 | 含义 |
|--------|------|------|------|
| Perception Markers | 开 | `/peach/perception/markers` | ns `scene_perception`。锁定目标 3D。绿 ACCEPT、黄 REOBSERVE、红 REJECT |
| Detection Cloud | 关 | `/peach/perception/single_cloud` | 检测框深度反投影，对 TF/深度，不是重建 |
| Camera Points | 关 | `/camera/depth_registered/points` | 整幅配准点云，很密；move_group octomap ③层同订（`sensors_3d.yaml`） |
| TSDF Cloud | 开 | `/peach/reconstruction/tsdf_cloud` | 绑定目标 TSDF 表面 |
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 未融体的拼接点 |
| Reconstruction Markers | 开 | `/peach/reconstruction/markers` | 主 ns `target_reconstruction`；精化 `peach_reconstruction/refined`。相机轨迹与精化示意 |
| Planned Views | 开 | `/peach_manipulation_node/planned_views` | 候选拍照位；`execution.enabled=false` 时仍会出，不代表已走到 |
| Camera Color | 关 | `/camera/color/image_raw` | 原彩图 |
| Debug Image | 开 | `/peach/perception/debug_image` | 检/分割叠加。灰框=未满 confirm_frames |
| Imu | 开 | `/imu/data` | `rviz_imu_plugin`：TCP 上的灰盒子 / RGB 轴 / 黄比力。姿态不写进 `imu_link` TF |

---

## 4. 验收门（产品）

**适用范围：本节验收门针对现行代码**（阶段执行器 + ExecutionAuthority + CycleContext 重构后）。历史量化基线与现场轮次在 [testing-log.md](testing-log.md)，**仅供排障参考，不构成对现行代码的验证**；现行有效性以按本节就绪单完成的真机验证为唯一依据。

### M1 单果观察（不接触、不开工具）

档位：两边 `execution=true`，`grasp.enabled=false`，`tool.enabled=false`。确认 `motion_possible=1` `e_stop=0`。同一可见目标连续 3 次 PICK_ALL 或 OBSERVE_ONLY。每次须：独立机位 `view_count >= capture.min_views`（默认 2）；`captured_views` 是积分帧数，可大于机位数；TSDF 点数 > 0，refit `ok=True`；`ExecuteTarget.outcome=SUCCEEDED` 且 Build 成功；失败时 ledger 有 `failure_code`（不得计数器全 0 且无告警）。

### 单目标完整抓取（运动、接触，不开工具 IO）

档位：`execution=true`、`grasp.enabled=true`、`tool.enabled=false`。禁止为提速放宽质量门。

- Build 接收后 2 s 内反馈 COLLECTING/READY；未绑定时臂不得环绕。超时取消后须等该 Build 结束再派下一颗。
- 观察：覆盖达标或 `maximum_moves` 用尽才停（不做完位姿序列不收口）；`time_budget_s` 只进日志，不按移动+等帧 EMA 预测收口。拍照位 + 当前位采帧；下一视点沿当前相机直线截到 `max_camera_step_m`（默认 0.15 m，~0.7 m 处一跨过 8°），只 LIN，失败换候选（绕行看 4.0 rad / 单轴 1.5 rad，不按时长）。到位后等新机位再判覆盖，同机位连帧不算。时长随 ~2.5 FPS 等帧浮动。
- 机位数 `view_count >= capture.min_views`（默认 2），基线/深度/RMSE/内点率过门。`captured_views` 是积分帧数。
- 无精化不得宣称方向准确。
- MTC 接近、直线套入、同轴撤离均须 goal-hold。预抓取先回拍照位（有记录的接近则原路返程，否则 PTP 0.5 s / 失败 OMPL 3.0 s），再走接近主路径：**staging 转移——最近构型 PTP（`select_goal_joints` 各滚转并行 IK：keep-roll 及 ±30°/±60° × 当前+4随机种子、自碰过滤、最近 5 候选逐个试）落到预抓取正下方轴上，再沿轴 LIN 升到预抓取**（已齐 LIN 段加相对目标 20° 姿态约束）；执行路径 MTC `plan(1)`，不凑满 `mtc_max_solutions`。staging 不可用且起点已在袋底侧、直连不穿囊时兜底直连 LIN（未齐先 LIN 原地对齐工具 Z；keep-roll 自碰换滚转）。G/under 单弦档已删（2026-09-10：photo→G 弦 fraction 均值 0.77、同 seed 100 随机位姿基线 9/100；`sim_approach_probe.py` 复核）。解析覆盖不执臂：`python3 scripts/analyze_approach_envelope.py --n 10000 --seed 20260911`（感知包络 + TCP 测地线 + 果实胶囊；PTP 行程/弧绕行仍须规划）。不走 CIRC/STOMP/OMPL。失败 `skipped_unreachable`，不进 OMPL。再一段沿轴 LIN；反向同轨迹（含 staging 段）回预抓取后 PTP `harvest_stow`。接近绕腕护栏 **累计 12 rad / 单轴 6.1 rad**；口侧/上方看果实胶囊，逐段审查（工具有限圆柱 vs 感知胶囊；反爬 s ≤ 本段起点 max(s,0)+2 cm，staging 首段 PTP 弧只查筒体接触）；笛卡尔绕行比 1.8 / 偏离 0.25 m / 回退 0.08 m；TCP 姿态行程绝对 110°（相对起止余量 20°；0=不查）。不按时长；近果 LIN 0.05、自由空间 0.10。mock 回放：`python3 scripts/replay_field_pregrasp.py --case 1757`（现场坐标，不开批）；全链路逐目标回放 `python3 scripts/sim_field_targets.py --case all`（回拍照位走周期内 `goToPhotoPose`，与正式接触段同一函数）；包络内随机位姿 `python3 scripts/sim_field_targets.py --random 16 --seed 20260910`（默认 typical：`axis_z≥0.70` 且 `|entry|≤1.02`，与现场多数袋一致；感知算法允许水平，压测加 `--envelope algorithm`；轨迹形状对照 `--random 30 --seed 20260911 --velocity 1.0`，仅 mock；`--velocity > 0` 时用例间隔 0.05 s）；实时记录 `python3 scripts/trajectory_watchdog.py`（默认不按绕行比停轨）。接近轨迹形状以 mock 为准（下节）；方向/定位仍以真机目视。
- 日志不得出现 SetIO。`harvest.grasped=false`（未开工具不得宣称采摘成功）。
- 单目标目标 45–60 s；失败必须有 `failure_code`，不得停在 `RUNNING + action_active=false`。

接触干跑历史轮次见 [testing-log.md](testing-log.md)。实验室两果均应走到接触干跑。方向准不准以现场目视为准。带工具采摘未做。

mock 接近轨迹形状（不开批、不代替真机方向验收）：`hardware_mode:=mock` 整栈 + `scripts/sim_field_targets.py`，与正式接触段同一 C++ 接近。默认 `--envelope typical` 对齐现场多数袋（`axis_z≥0.70` 且 `|entry|≤1.02`）；`--envelope algorithm` 含近水平袋，只作压测。09-11 对照轮（seed `20260911`、typical、`--velocity 1.0`）：笛卡尔门与 TCP 姿态 90° 打开后，**从拍照位出发的成功接近**绕行比 ≤1.70、姿态测地线 ≤71°，无 1740 式先抬再落；30 例 26 到位、4 例护栏拒发（绕腕累计 12.7–12.9 rad ×2、回退 0.14 m、keepout 穿囊），不放宽 12 rad / 8 cm 回退 / 12 cm 半径。看 `from_photo=true` 行与技能日志 `MTC 接近笛卡尔/姿态审查`、`接近：… 原语`；`--velocity 1.0` 时脚本 0.6 s 静止切段常把回拍照位与接近并成一段，失败例的 `from_photo=false` 路径往往是上一段返程，不得当接近绕行。记录 `runs/sim_field_targets_20260911_102546.jsonl`。规划加速后感知算法包络 100 随机（同 seed、`--velocity 1.0`）见 [testing-log.md](testing-log.md) 09-11 151609（墙钟 14.5 min，单例中位 4.6 s，护栏未放宽）。解析覆盖 10000 分层位姿见 [testing-log.md](testing-log.md) 09-11 154345（90° 门）与 155716（110°/余量 20°、`|entry|≤1.16`，闭式 10014/10014）。轮次过程见 [testing-log.md](testing-log.md) 09-11。

树干/粗枝进 PlanningScene 是预留。无人工确认的无粗枝通道时不接触。

### 套袋套入剪切软件门（P0–P8；真机剪切未做）

全程默认 `execution/grasp/tool=false`。launch 不自动 `RunHarvest`。

| 门 | 怎么验 | 现行 |
|----|--------|------|
| P0 可构建 + 工具帧 | 干净 `build/install/log` 后 colcon；URDF 有 `tool_axis` / `sleeve_mouth` / `cutting_plane` / `tool_body_link` | TCP 在圆柱顶部，**按当前 `tool_profile`**：hollow_cylinder_v1 `(0, 47.90, 151.07) mm` / adaptive_cylinder_v1 `(0, 47, 168.66) mm`，`Rx(-90°)`：Z=开口、XY=刀口；筒沿 −Z 200 mm。**固定圆柱等价门**：改 `aubo_description` 后 `xacro src/aubo_description/urdf/aubo_e5.urdf.xacro hardware_mode:=mock tool_profile:=hollow_cylinder_v1` 输出与改动前 diff 须为空；`ros2 param get /peach_target_reconstruction_node tool.profile_id` 须与 launch `tool_profile` 一致 |
| P1 几何基线 | `runs/` 写 `geometry.jsonl`；复算脚本已归档（需要时 `_archive/offline_2026-09/` 下以模块方式运行） | 离线脚本已归档 |
| P2 袋模型 | 观测 `occlusion_class`；球 marker ns=`prior`；裸果不入 `next_target_id`；`branch_blocked`/`neighbor_overlap`/`damaged_or_wet` 不得 `allowed` | 沿袋长轴半径剖面，窄头为口、宽头为底，箭头袋底→袋口；斜袋保持长轴不对成竖轴；袋底→袋口只许上半球（从下往上，左右最多水平，禁止朝下）；分割两端比沿轴朝外框边贴合，更贴边的一端为口（竖缝贴左边）；剪切参考在袋口/分割贴框极限，果距不足只否决 `allowed` 不挪刀；两端贴合差不够才用 3D 窄头/逆重力 |
| P3 重建权威 | `allowed` 须袋融合预算才套入；无 budget 不得接触；圆柱/TSDF 不定轴；包络轴只否决，扁袋不打 12°；35° 只诊断 | FULL 时 `allowed=false` → `SKIPPED_QUALITY`；`PREGRASP_ONLY` 不要求 `allowed` |
| P4 预抓取 | 默认 `execute_pregrasp_only=true`；停预抓取（入口在拟合袋底，预抓取沿 −axis 后撤 30 mm）；无 SetIO；ACK 后再 Survey。**方向/定位是否可用与精度以到位后真机目视/测量为准**，不以预算或 2°/3 mm 残差代替 | 残差未过门也 Hold；`allowed=false` 不拦预抓取。停袋底对照轮次见 [testing-log.md](testing-log.md) 1757 |
| P5 套入干跑 | `grasp=true` `tool=false`；套入与反向撤退均须先过 `PlanSleeve` 规划；到预抓取只走直线/插值 | 软件路径已接线；失败不改 PTP 绕行 |
| P6 刀具 | `ToolActuator`：SetIO ACK ≠ `cut_confirmed`；无硬件反馈时不得 `CUT_CONFIRMED` | 已实现；真刀未接 |
| P7 成功语义 | `harvest.grasped` 仅 `cut_confirmed && retreat_confirmed`；tool 关干跑可 `outcome=SUCCEEDED` 但 grasped=false | 软件已钉死 |
| P8 导航适配（预留） | 导航包已归档（`_archive/parked_2026-09/`）；四个 IDL 名在 manifest `reserved_interfaces`；调度 `_cmd_navigate` 直通 `NAV_OK` | 固定座现行；清单脚本核对预留区 |
| P9 发现开窗 | 袋口开批（干跑或不在拍照位）须 `termination_reason=survey_failed`，不 Begin。通过：`photo_pose_reached` 先于 `round_locked`，且锁定集 `scene_epoch` 与 Begin 返回值对齐 | 调度首巡 Survey→Begin→WAIT_LOCK |

```bash
python3 src/peach_interfaces/scripts/check_interface_manifest.py
```

### PREGRASP_ONLY 判定口径（运动、不开工具）

档位：两边 `execution=true`，`grasp.enabled=true`，`tool.enabled=false`，调度 `execute_pregrasp_only=true`（现行默认）。到预抓取后看筒口是否对袋轴、侧向是否偏、剪切紫点是否落在袋口（分割贴检测框极限）。**这些对错只在现场评**，监控里的 `allowed`/余量/RMSE 只作记录，不作为通过条件。残差 2°/3 mm 未过也停住。任何路径无 SetIO。**结束后停在预抓取**，不回 `harvest_stow`。`ExecuteTarget` 终局须 `SUCCEEDED` 且 `recovery_required`（作业票停在靠近、等 ACK）；不得记 `FAILED` 或「须现场人工撤离」。看完后 `ControlTask` `ACKNOWLEDGE_RECOVERY`（命令 6）才允许再 Survey / 下一颗。`grasp.enabled=false` 到不了预抓取。判断/执行全图：`runs/field_test_20260828/pregrasp_only_flow.mmd`。

轮次事实（08-28→1757、重构前后）见 [testing-log.md](testing-log.md)。现行停袋底以 **1757** 为对照起点，不把旧数字当现行合格证据。

### 预抓取全程测试就绪单（真机前逐项核对）

> 历史轮次见 [testing-log.md](testing-log.md)；**以本单流程与当场实测为准。**

前置（上电/起栈前）：

1. `pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'` 无残留，有则按 PID 补杀。整栈 launch 预检同样拦 `extrinsics_publisher`（多代会叠发 `wrist3→camera_link`）。
2. 示教器上电、抱闸释放；`auto_power_on=false` 保持不动；不调 `aubo_dashboard`。
3. 手眼 TF 在位：`ros2 run tf2_ros tf2_echo wrist3_Link camera_link` 有输出（缺失则点云相对臂偏 ~10 cm 且轴不对，不得开批）。
4. TF 帧核查：`ros2 run tf2_ros tf2_echo base_link tip` 应报**不存在**——存在即有旧实例污染（link1/link2/tip 链）；launch 预检已拦，现场仍见则按 PID 清理后重启。
5. 相机出图且深度配准：`ros2 topic hz /camera/color/image_raw` ≈2.5 FPS。
6. 当前关节对照 SRDF `global_photo_pose`。launch 不自动到位；开执行后第一段运动是 Survey 回拍照位（进程内无接近记录则 PTP）。停在预抓取重启时这一段行程大，须现场确认再开批。

档位（预抓取全程）：

| 项 | 值 | 说明 |
|----|----|----|
| 调度 `execution_enabled` | true | 运行期可 `ros2 param set` |
| 技能 `execution.enabled` / `grasp.enabled` | true / true | 到不了预抓取先查这两档 |
| 技能 `tool.enabled` | **false** | 全程不得出现 SetIO |
| 调度 `execute_pregrasp_only` | true | 现行默认；到位后停住等 ACK |
| 观察 `observe_max_total_joint_travel_rad` | 4.0 | yaml 已固化，勿再运行时改 |
| 轴向后撤 | `grasp_standoffs.yaml` 现行 `0.0` / `0.03` | 入口=拟合袋底；预抓取沿 −axis 后撤 30 mm |

复现命令（`request_id` 按「命名」节换新，勿复用；停袋底对照批次 `field_pregrasp_20260901_1757`）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_perception peach_manipulation peach_executor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
# 有残留按 PID 补杀。停在预抓取的重启：示教器先回到拍照位
# （SRDF global_photo_pose ≈ 0.425, 0.195, 1.678, 1.462, -0.500, 0.039 rad）。
# go_to_photo_pose 行程门 6 / 2.5，从袋口回去可能被拒，故 1757 用示教器。

unset PYTHONPATH
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
source /opt/ros/jazzy/setup.bash
# 2026-09-15 起迁全 apt（2026-09-08 的 ~/ros2_ws、~/ws_moveit 铺层要求退役，
# 两目录删除；缺失症状原文存档于 REFACTORING.md）：moveit 全家 2.12.4、
# MTC 0.1.8（core/msgs/capabilities）、moveit_servo、pilz、stomp、
# moveit_configs_utils 均 ros-jazzy 包；open3d/torch 仍在 aubo_py3.12 venv。
export PYTHONPATH="/home/mu/Desktop/aubo_e5_jazzy_ws/aubo_py3.12/lib/python3.12/site-packages:${PYTHONPATH:-}"
source /home/mu/Desktop/aubo_e5_jazzy_ws/install/setup.bash
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 等到 rosout：managed nodes Active（仍须显式 RunHarvest）

# 冒烟：五节点 Active；standoffs 入口 0 / 预抓取 0.03；
# /aubo_io_controller/robot_status drives_powered=1 motion_possible=1；
# tf2_echo wrist3_Link camera_link 平移 ≈ [0.045, 0.108, 0.002]；
# tf2_echo base_link tip 应失败；关节对照拍照位。
ros2 param set /peach_manipulation_node execution.enabled true
ros2 param set /peach_manipulation_node grasp.enabled true
ros2 param set /peach_executor execution_enabled true
# 确认 tool.enabled 仍为 false、execute_pregrasp_only 仍为 true。不要改仓库 yaml。

ros2 action send_goal -f /peach_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_pregrasp_YYYYMMDD_HHMM', scene_key: 'lab', profile_id: 'default'}"
```

停预抓取后现场评方向/定位。看完：

```bash
ros2 service call /peach_executor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0}"
```

异常先命令 4（`CANCEL_NOW`）。过程数据在工作区 `runs/`（gitignore，不入库）；结论写进 [testing-log.md](testing-log.md) 与 `runs/field_test_<日期>/log.md`。

选果约束（有效深度 + TCP IK 可达，09-01 定稿）：

- **可达性权威 = TCP IK 预检**：SELECT 把各目标感知入口（`entry_pose`，Z=袋轴）批量送技能 `CheckReachability`。服务端换成与 MovePregrasp 同一停位再 `setFromIK`（位置沿袋轴后撤 `mtc_approach_along_axis_m`，现行 0.03 m；姿态=`alignFrameZ` 保留当前 TCP 滚转，不抄感知四元数；种子=当前关节，与 MTC 同一运动学）。无解过滤并留 `ik_no_solution` 归因。
- **有效深度窗**：相机距离 0.30–1.60 m（`selection_depth_min/max_m`）；逐帧掩膜有效深度仍由采集门 `min_mask_depth_ratio` 把关。
- 服务不可用（mock/技能未起）回退标定半径窗 0.88（成功 0.830–0.840 / MTC 0 解 ≥0.917），事件里带 `reach_check` 说明。
- 摆位建议：袋底距基座 0.5–0.8 m（参考成功标定区间）。

流程与判定：

1. `ros2 launch peach_executor harvest_system.launch.py hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98`（先 moveit_enabled 默认 true）。
2. Active 后先对照拍照位，再 `ros2 param set` 开执行/抓取（`tool.enabled` 保持 false），再发 `RunHarvest`（launch 不自动开批）。完整命令见上「复现命令」。
3. 每颗期望链：首巡 Survey → Begin → WAIT_LOCK → SELECT → Build+OBSERVE 并行 → `PREGRASP_ONLY` 停在预抓取（作业票停在「靠近」）→ 现场目视评方向/定位（筒口对袋轴？侧向偏多少？剪切点落袋口？）→ `ControlTask` 命令 6 ACK → 回访 Survey（不 Begin）→ 下一颗。
4. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`（不得 `FAILED`）；全程无 SetIO；这些对错只在现场评，`allowed`/余量只作记录。Hold 等 ACK 期间不写 summary（结算只认批次终局）；若需当场核对，以技能 `[SUCCEEDED] PREGRASP_ONLY` 日志与现场停位为准。

中断与异常：

- 异常先 `ControlTask` 命令 4（CANCEL_NOW），臂停后人工撤离；恢复等待未 ACK 前调度不会 Survey/派下一颗。
- 停轨走透传取消 + 硬件 `RobotMoveStop`；避障绕行只看行程护栏（观察 4/1.5、接触 12/6.1、拍照 6/2.5）。
- 观察失败高发项（历史）：`selected_target_stale`/`selected_target_changed`（身份新鲜度）、`missing_mask`（有效深度不足）——summary 原因列现在直接给出，先看原因再调参。量化复算与归档数字见 [testing-log.md](testing-log.md)。
