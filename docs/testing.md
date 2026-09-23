# 测试流程与命名

现行系统（SNAPSHOT）：源码。与 [architecture.md](architecture.md)、[io.md](io.md) 构成仅有的三份活文档；**源码与本文互相更新，改启动/验收口径或改本文须同一轮改另一边**。**如何演化**以 [AGENTS.md](../AGENTS.md) 为准：非完美适配当前真机/产品则跟 ROS 2 / 优秀 GitHub 主流。

真机轮次、量化基线、审查记录写在 [testing-log.md](testing-log.md)；工程整理过程写在 [REFACTORING.md](REFACTORING.md)（二者都是过程记录，不驱动现行设计）。改行为只改本文 + 源码；补一条实测时追加 testing-log，不把轮次散文写回本文。

各包 `test/` **保留 ROS 2 默认 lint，并允许零 ROS 纯核 pytest**（Python：`test_flake8.py` / `test_pep257.py` + 不 import rclpy 的表驱动；CMake：`ament_lint_auto`）。**测试文件名必须匹配 pytest 默认收集（`test_*.py`）**——2026-09-18 前 `peach_harvester` 的 `vision_test_*` / `supervisor_test_*` 共 24 个模块不被收集（假绿），已改名 `test_vision_*` / `test_supervisor_*` 并清偿全部 lint 债（中文 docstring 首行 ASCII 句号、import 序），lint 测试从此真实生效。现行纯核：`peach_common/test/` 四文件（`test_yaml_params` / `test_param_rules` / `test_paths` / `test_qos`，W1 单源组；qos 用例无 rclpy 环境整体 skip）、`peach_harvester/test/test_supervisor_harvest_fsm.py`（`react` 表）、`peach_observability/test/test_bag_report.py`（bag 流→报告合成、验收门、回收选择，零 ROS）、`peach_harvester/test/test_vision_runtime_core.py`（`ManualClock` / `BoundedWorker` capacity=1 drop_oldest）、`peach_harvester/test/test_vision_tool_profiles.py`（工具档案解析结构校验）、`peach_arm/test/test_contact_monitor.py`（合成电流序列编译 `contact_monitor.hpp`）、`ivg_graspnet/test/test_grasp_core.py`（`GraspList` NMS/碰撞；torch 算子 `importorskip`）、`serial_imu/test/test_protocol.py`（切帧/协方差）与 `test_frame.py`（倒装 Rx + parent 对齐）、`imu_follow/test/test_core.py`（姿态增量/死区锥钳/平滑/关节步长/插入推进）。`peach_arm` 另有纯核 gtest 十套（W0 六套 + W5 三套 + `test_pregrasp_level`；见 §1 gtest 清单）。现行测试面 = lint + 零 ROS 纯核 + gtest + isolated launch_testing（决策 0006，**UNWIND**：不是套袋工艺的完美适配）。**新测试按 [AGENTS.md](../AGENTS.md) 测试塔与官方 / Nav2 / Autoware 主流**：允许 gtest、launch_testing（isolated `ROS_DOMAIN_ID`）、`mock_components` 集成。独立系统测包 `peach_system_tests` 已落地（mock `harvest_system`，不发 `RunHarvest`）＋回放塔＋`perf_baseline.json` 性能对拍锚点；Gazebo/Isaac 物理仿真仍缺口。采摘方向 / 接触对错仍以真机 `runs/` + [testing-log.md](testing-log.md) 为最终权威（KEEP）；`colcon test` 绿不是田间验收。语法与流程由审查核对。套入剪切软件门看 flake8 / pep257 / uncrustify 与纯核表；`peach_arm` 整测项跳过 cpplint（其 legal/copyright 与 Google include 顺序检查同本项目「文件头版权块项目结束再补」「include own-first」约定冲突，CMake 已 `set(ament_cmake_cpplint_FOUND TRUE)`），C++ 风格门以 uncrustify 为准、静态分析走 cppcheck。`ament_xmllint` 会拉 `package_format3.xsd`，网络卡住超时不阻塞本产品路径。

不要把现行精华账本当一次性文件删（网格 jsonl、live SURVEY/PREGRASP 目录、testing-log）。分析完成后可 `python3 scripts/purge_analyzed_bags.py --all-reported runs` 只删 MCAP。未授权不得真机运动或 SetIO。硬件急停在示教器/柜，不经 ROS。launch **不自动** `RunHarvest`。采摘应用九包职责见 [architecture.md](architecture.md) §3。旁路视觉抓取三包（`ivg_interfaces` / `ivg_pose_estimation` / `ivg_graspnet`）不进整栈 launch。`serial_imu` 随 `harvest_system` 起（`imu_enabled` 默认 true），不进 lifecycle、不进只读 bringup。`imu_follow`（IMU 姿态跟随）仅 `tool_profile:=adaptive_cylinder_v1` 随整栈 Include、`motion.enabled` 默认 false 只算不发；空心末端不起。peach 不自动开门。

---

## 命名

批次 `request_id` 同时是账本目录名 `runs/<request_id>/`，**不得复用**。开批前按当时本地时间填 `YYYYMMDD` 与 `HHMM`。

| 用途 | 格式 | 例 |
|------|------|-----|
| 预抓取真机 | `field_pregrasp_<YYYYMMDD>_<HHMM>` | `field_pregrasp_20260901_1757` |
| 接触干跑（不开刀） | `field_full_<YYYYMMDD>_<HHMM>` | `field_full_20260825_1851` |
| 只扫不运动 | `field_dry`；或预抓取名 + `intent: 2` | `intent: 2` = SURVEY_ONLY |
| 实验室只扫（mock 臂+真相机） | `e2e_survey_<YYYYMMDDTHHMMSS>` | `intent: 2`；不评套袋方向 |
| 跳过重建接触验证 | `e2e_unrefined_<YYYYMMDDTHHMMSS>` | `intent: 0` + `skip_reconstruction:=true`；预抓取默认，套入干跑另 `execute_pregrasp_only:=false` |
| 开发机 mock | `dev` | `intent` 默认 0 |
| 当日综述 | `runs/field_test_<YYYYMMDD>/log.md` | `runs/field_test_20260901/log.md` |
| 监控会话 | `runs/session_<YYYYMMDD>_<HHMMSS>/bag/`（观测自动，随栈启停开合） | 内含 `bag_0.mcap` 与自动生成的 `bag_report.md/json`，与账本互引 |

场景键 `scene_key` 现行实验室用 `lab`。`profile_id` 现行 `default`。

写记录：当场把结论写入 `runs/field_test_<日期>/log.md`，并追加 [testing-log.md](testing-log.md) 对应轮次。`runs/` 结构化文本（jsonl/json/csv/md/yaml/txt/log）入库随仓推送，克隆即可离线复算/分析；过程 bag（`session_*/bag` 的 mcap）、图像/点云等二进制仍只留本地（.gitignore 白名单）。会话报告由 observability 在停栈时自动生成，也可随时手动复跑：`ros2 run peach_observability peach_bag_report runs/session_*/bag`。

**单轮复盘完备集（2026-09-17 起）**：① 会话 bag 录 `/rosout` 全量节点日志（`record.rosout` 默认开，stamp/level/logger 可回放：`ros2 bag play` 后 `ros2 topic echo /rosout`）；② ros2 自动日志在 `~/.ros/log/<launch 时间戳>/launch.log`（含全部进程 stdout，目录时间戳=起栈时刻）；③ `runs/<request_id>/` 账本与事件流。三者按运行窗口归集：`python3 scripts/collect_round.py <request_id>` → 生成 `runs/<rid>/round_report.md`（逐目标 outcome 表 + 感知/重建时间轴 + 自动日志关键行 + bag 互链）。

**全量录制（`record.level`，2026-09-17 起）**：`std`（默认长跑）=计算图通配发现订阅域内全部话题自动进会话 bag（新话题无需改代码即被录），相机 raw 大流限 1Hz；`all`=同上但大流不限速（失败短抓 / 仿真复现；stereo 前端约 50MB/s，100 GB 预算约 30+ 分钟，超 `max_total_bag_gb` 靠 retention 回收）；`core`=仅固定订阅集（镜像订阅+派生 job/metrics；2026-09-20 起 `bag_topics` 键已删）。运行时切档：改 `observability.yaml` 后重启，或 `ros2 param set /peach_observability record.level all` 再重启节点生效（订阅在 activate 期建立）。harvest RViz 窗录像：`scripts/record_rviz_harvest.sh <request_id> [duration_s]`（产物 `runs/<request_id>/rvizwin.mp4`；录前把 MoveIt RViz 提到前台，x11grab 录屏幕像素，挡住会进别的窗）。停栈须出 `bag_report`：SIGINT 先停通配订阅再有界关 bag（不再 `queue.join` 永久挂死）。

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
# colcon test --packages-select peach_common peach_interfaces peach_harvester peach_arm \
#   peach_bringup peach_observability peach_vegetation peach_system_tests
```

`r0_gate.sh` 现行测试组：harvester vision 组（含 W3 `test_vision_plan_updater`、W4 `test_reconstruction_{session,refine_result,session_recorder,icp_cache}` / `test_refit_orchestrator`）、supervisor 组（含 W2/S3 `test_supervisor_param_rules` 使能依赖全组合）、**peach_common 组（W1 新增：`test_yaml_params` / `test_param_rules` / `test_paths` / `test_qos` 四文件，`PYTHONPATH` 前置 `src/peach_common`）**、cycle_core 视点/批次策略组、observability / vegetation / system_tests preflight 组，末尾跑接口清单核对。现行纯核另含：`param_rules` / `identity` / `tool_budget` / `harvest_fsm`（EventHold） / `idl_constants`（HarvestState/ControlTask/ManageLifecycleNodes 数值对账） / `path_metrics` / `domain`（reducer、model_contract、evidence；supervisor 侧 ledger/watchdog 死码已删，W6-B） / `pregrasp_level` gtest。`peach_arm` gtest 十套（`test_pregrasp_level` / `test_grasp_geometry` / `test_trajectory_guard` / `test_view_planner` / `test_gates` / `test_reconfirm_policy` / `test_target_cache`（W0 六套 54 例）+ `test_frame_timeouts` / `test_staging_selector` / `test_pregrasp_residual`（W5））：纯核链 `${PROJECT_NAME}_core`，现场案册夹具 `test/fixtures/field_pregrasp_cases.yaml`（W0 自 config 迁入，并安装到 `share/peach_arm/test/fixtures` 供系统测消费）；`peach_system_tests` 另有 `test_perf_baseline.py` 守卫 `perf_baseline.json`（性能对拍锚点：仓内实测数字带 `_provenance` 来源，供优化前后对照，只锁 schema 与量纲不锁数值）。Python 键名冻结测试对照 yaml（感知两节点 + 调度/观测/lifecycle）；C++ 新合同字段走 generate_parameter_library。`.github/workflows/jazzy.yaml`：`peach-core` 跑同一纯核门 + numpy 1.26.4；`industrial_ci` 在 Docker 里 `colcon` 编测驱动与 peach（`COLCON_IGNORE` IVG 三包、`imu_follow`、`percipio_camera`、`camera_calibration`，无真机 job）。scipy 不进 `package.xml`（venv-first KEEP）；ICI 用 apt `python3-scipy` / `python3-pytest` / `python3-yaml`（`ros:jazzy` numpy 已 1.26.4，不再 Docker 内 pip）。`peach_system_tests` isolated launch_testing 起 mock `harvest_system`（`camera_enabled:=false` `imu_enabled:=false`，`QT_QPA_PLATFORM=offscreen`，`AUBO_RUNS_DIR` 指临时目录），断言 `/joint_states` 含 MUST 六关节名（JSB 的 name 数组常为字母序，按下标当 MUST 序会拧腕）与 lifecycle Active，**不**发 `RunHarvest`。headless 下 `move_group`/`rviz2` 退出码不纳入 peach 进程门。本机已有栈残留时预检拒测（只认节点 argv0 或 `.../lib/<pkg>/<node>`，不认 colcon 包名参数）。

全新机器 / 新环境自检与部署：`scripts/env_bootstrap.sh check|install|all`（幂等；check 零改动、退出码=缺失项数，`SMOKE=1` 追加 mock 冒烟）。脚本内 apt/venv/udev 清单是依赖事实源之一，变更依赖须四处同步：package.xml、requirements.txt、脚本清单、本节。

有残留按 PID 补杀。clangd：上述 `CMAKE_EXPORT_COMPILE_COMMANDS` 让每个 CMake 包在 `build/<pkg>/compile_commands.json` 留下编译命令；工作区 `.clangd` 按包指向这些文件。驱动栈 CMakeLists 只读，不在那些包里写 `set(CMAKE_EXPORT_COMPILE_COMMANDS)`。改完 CMake 或新编一包后 **Clangd: Restart language server**。Python：`aubo_py3.12`。依赖分层（venv-first）：ROS 2 依赖走 Jazzy apt；其余第三方（numpy/scipy/opencv/PyYAML/open3d/torch 等）一律由工作区 `requirements.txt` 钉版本装进 venv（对同名 apt 包需 `pip install --ignore-installed -r requirements.txt` 才真正落入 venv）。**numpy 必须 ==1.26.4**（<2）：Jazzy 的 cv_bridge 二进制按 numpy 1.x 编译，numpy 2.x 会 `import cv2` 报错、`import cv_bridge` 段错误；该版本同时是 apt python3-numpy 的版本，双路径一致。感知身份分配与手眼标定共用 scipy（venv 内 1.11.4）；`peach_harvester`（vision） 的 package.xml 只声明 ROS 键与 `python3-numpy`（ABI 边界），数值库不走 rosdep。Python peach 参数由 `config/<节点>.yaml` 直读（`yaml_params.attach`），源码在 `peach_harvester/{yaml_params.py,vision/*/params.py,supervisor/params.py}`；从源码树直接跑脚本时把 `PYTHONPATH` 指到 `src/peach_*` 需带 venv 的 ROS 依赖，否则节点会在 import 期退出，lifecycle 拉不齐 Active。本机若 venv 抢了 `PYTHONPATH`，launch 前先清再只留 Jazzy site-packages 并重新 `source` 两份 setup（见 §4 复现命令）。跨包轴向后撤只改 `src/peach_harvester/config/grasp_standoffs.yaml` 两行；不要把它当 ROS `ParameterFile` 直接喂节点（rcl 不允许 `ros__parameters` 之前出现裸值）。能力 launch 读入后注入已声明参数。

```bash
# 开发机：无相机、不运动
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
# 实验室：mock 臂 + 真立体相机（物理相机不跟随 mock TF）
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=true camera_frontend:=stereo autostart:=false
# 同上并跳过重建、用锁定集场景观测验证接触（不评套袋方向；真机 KEEP false）：
#   追加 skip_reconstruction:=true

# 真机 RGB-D bag 回放：先 play --clock 再起栈（感知身份要精确 stamp TF）
# ros2 bag play /home/mu/Pictures/pipeline_replay_20260918_162704/bag --clock
# ros2 launch peach_bringup harvest_system.launch.py \
#   hardware_mode:=mock camera_enabled:=false imu_enabled:=false use_sim_time:=true

# GPU 枝/叶分割（独立 launch，不进 harvest_system，不写 PlanningScene）
ros2 launch peach_vegetation vegetation.launch.py
# ros2 topic echo /peach/vegetation/status --once

# mock 轨迹回放（1757/1740 过程坐标；官方 GenerateGraspPose+LIN Fallbacks，不开批）
python3 scripts/replay_field_pregrasp.py --case 1757
python3 scripts/replay_field_pregrasp.py --case 1740 --planner ptp   # OMPL/PTP 绕行对照
# 过护栏后再下发 mock 控制器：加 --execute（不动真机）

# mock 全链路回放（09-09 现场逐目标坐标驱动真实技能节点；默认 PREGRASP_ONLY。
# --mode full 套入干跑：须 PROFILE_FULL；1757 为工作空间内对照。注入感知/重建
# 话题 + robot_status + 光学系 TF。回拍照位走周期内 goToPhotoPose）
python3 scripts/sim_field_targets.py --list
python3 scripts/sim_field_targets.py --case all
python3 scripts/sim_field_targets.py --mode full --case 1757 --velocity 1.0
# 现场典型包络随机位姿（axis_z≥0.70 且 |entry|≤1.02；感知允许水平，压测加 --envelope algorithm）
python3 scripts/sim_field_targets.py --random 16 --seed 20260910
# 接近轨迹形状对照（typical；seed 与 09-11 审查轮一致；--velocity 1.0 仅 mock）
python3 scripts/sim_field_targets.py --random 30 --seed 20260911 --velocity 1.0
# 大样本 + 仿真提速（速度/加速度缩放设 1.0，仅 mock；100 例约 10 min）
python3 scripts/sim_field_targets.py --random 100 --seed 20260910 --velocity 1.0
# 感知算法包络压测（含近水平；seed 与 09-11 对照轮一致）
python3 scripts/sim_field_targets.py --random 100 --envelope algorithm --seed 20260911 --velocity 1.0
# 执行超时护栏（2026-09-23）：`execute_timeout_s`（默认 90s）内单条轨迹/MTC 解未收口即下发 stop 并按失败记；`MTC execution timeout` / 「执行超时」一律按失败收口，不 resume 原轨迹（UR Driver 红线）。
# 工具档案注入（2026-09-22 起）：--tool-profile 写 GraspDecision/ExecuteTarget 档案标签，
# 启动时与 /peach_arm tool.profile_id 比对，错配即拒跑；GraspDecision 径向预算按档案
# D_inner×注入袋径复算（超内径注入得到 allowed=False，网格 expect=deny_decision 验收）。
# 两档案轴向余量在当前误差常数下结构性为负（blade_capture<固定误差+安全余量），见战役 analysis。
python3 scripts/sim_field_targets.py --grid --mode full --tool-profile hollow_cylinder_v1 --velocity 1.0
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
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 相机前端改 peach_stereo（压掉 bringup 内 percipio；本包 RViz 关，画面在 MoveIt RViz）
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true camera_frontend:=stereo robot_ip:=169.254.10.98
```

旁路视觉抓取（独立 launch，不进上面这条整栈）。lint/纯核走 colcon；Web 回归与 GraspNet torch 算子须 `aubo_py3.12`。GraspNet 权重 `src/ivg_graspnet/models/checkpoint-rs.tar` 随库；估姿 rembg 的 `u2net.onnx`（约 168MB）超远程单文件上限不入库，clone 后执行 `src/ivg_pose_estimation/ivg_pose_estimation/models/fetch_u2net.sh`（或首次抠图时 rembg/pooch 下载）：

```bash
colcon test --packages-select ivg_interfaces ivg_pose_estimation ivg_graspnet
# GraspNet 纯核（torch 在 venv；须在包目录下跑，模块才可导入）
cd src/ivg_graspnet && ../../aubo_py3.12/bin/python -m pytest test/test_grasp_core.py && cd ../..
# 估姿 Web 回归（fastapi/httpx 在 venv；系统 python 下整文件 skip）
./aubo_py3.12/bin/python -m pytest src/ivg_pose_estimation/test/test_web_app.py
# 接触检测纯核（g++ 编译 contact_monitor.hpp，零 ROS）
python3 -m pytest src/peach_arm/test/test_contact_monitor.py
```

监控：`http://127.0.0.1:8090`。参数 `peach_observability/config/observability.yaml`（W11 起随包走）。过程页：作业票（发现→完成）、事件（含 details 展开）、落盘目录、阶段时序（调度 FSM / 技能周期两列，段时长服务器侧结算）、批次账本（per-target 结果/原因/失败码/阶段耗时，随 `ledger.json` 终局入账直播）、感知节拍（fps/检测/分割/几何耗时、掉锚/陈旧锚）、重建进度（机位/拒帧/TF 失败/基线/许可倒计时）、TCP 俯视（绕行比、Δz、对照预抓取/入口/弦）、本场目标、柜侧硬件（TCP xyz/rpy、六轴角/速度、电流 SDK 原单位、温度、跟随误差）、系统负载与参数镜像（折叠）。`/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor` / `robot`（`status` / `tcp` / `joints`）/ `metrics` / `record` / `params` / `pipeline` / `ledger` / `job` / `debug`。默认不上电、不派发运动、不打工具 IO、不自动开批。

### Web 单步调试（决策 0018）

Tab「调试」＝向**既有**动作/服务发请求的纯客户端，页面只留本管线：BeginScene、SurveyScene、Build/finalize、ExecuteTarget、去拍照位、RunHarvest、CANCEL_NOW。无令牌。`debug.enabled` 默认 true（false 时 POST 503）。运动类另需 `debug.motion_enabled=true`（false=423）。审计 `runs/debug_audit/<日期>.jsonl`。

动臂（真机手调预抓取等）：

```bash
# yaml：debug.motion_enabled: true
# （技能侧 execution/grasp/tool 仍须另行打开，ExecutionAuthority 照常复核）
# 浏览器 8090 → 调试 Tab
```

门控口径：无令牌、无 401。`motion_enabled=false` 时 Survey、ExecuteTarget 非 PREVIEW（含 OBSERVE_ONLY）、go_to_photo_pose、RunHarvest 全部档位（含 SURVEY_ONLY，09-18 收紧——其也会 Survey 移到拍照位）一律 `423`；PREVIEW/BeginScene/finalize 不拦。未知端点 `404`。Web **绕不过** `ExecutionAuthority`、调度使能和重建门。

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | mock | mock / real |
| `robot_ip` | 169.254.10.98 | 仅 real |
| `tool_profile` | adaptive_cylinder_v1 | 末端工具档案（URDF TCP、感知许可内径、消息标签统一随档案切换）；固定圆柱显式 `tool_profile:=hollow_cylinder_v1`。切换须整栈重启，RSP 与 move_group 同 arg |
| `camera_enabled` | false | 有相机时设 true |
| `camera_frontend` | percipio | `percipio` / `stereo`；stereo 时 harvest_system 直起 peach_stereo，压掉 bringup 内 percipio |
| `imu_enabled` | true | USB IMU；挂 tcp 并对齐。无设备时节点重试。关掉：`false` |
| `extrinsics_enabled` | true | wrist3 → camera_link |
| `moveit_enabled` | true | move_group + RViz |
| `hand_eye_enabled` | false | 标定流程 |
| `hand_eye_web_enabled` | false | 标定 Web `:8088` |
| `use_sim_time` | false | bag 回放须 `true` + `ros2 bag play --clock`；真机必须 false |
| 调度 `execute_pregrasp_only` | true | 接触段 `PREGRASP_ONLY`：停预抓取不回 stow；套入前改 false |
| `skip_reconstruction` | false | true 时调度不发 Build/补视，且同 arg 打开技能 `quality.allow_unrefined_geometry`。brain 用 `peach_supervisor` 键 ParameterFile overlay（unnamed Node 嵌套 dict 打不进）。起栈后核 `ros2 param get /peach_supervisor skip_reconstruction` 与 `ros2 param get /peach_arm quality.allow_unrefined_geometry`。真机 KEEP false |

过程录制不再有 launch 参数：observability 的 `record.enabled`（默认 true）随栈开合会话 bag，栈停自动出报告；`record.level`/`record.max_total_bag_gb` 见 `config/observability.yaml`。

完整列表：`--show-args`。只起手臂：`ros2 launch aubo_e5_bringup bringup.launch.py …`。

USB IMU（随整栈，不进 lifecycle）：手册 [`src/serial_imu/README.md`](../src/serial_imu/README.md)（udev、协议、TF、RViz 插件项、dialout/`newgrp`）。`harvest_system` 默认 `imu_enabled:=true`（mock / real 相同）：`tf_parent_frame:=tcp`、`align_to_parent:=true`、不起 IMU 自己的 RViz。画面在 MoveIt RViz **Peach → Imu**（订 `/imu/data`）。无 USB 时每 2 s 重试串口，不挡整栈。关掉：`imu_enabled:=false`。不要另起 `serial_imu.launch.py` 与整栈并行。摘要：

```bash
sudo usermod -aG dialout $USER && newgrp dialout
# 随 mock / real 整栈（默认已开）
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock camera_enabled:=false
ros2 topic echo /imu/data
# 只看 IMU、不起采摘：
ros2 launch serial_imu serial_imu.launch.py
```
`usermod` 后必须 `newgrp`（或重新登录）。缺插件：`sudo apt install ros-jazzy-imu-tools`。整栈里 Fixed Frame 用 `base_link`；姿态看 `/imu/data`（Rx(180°) + 对齐到 tcp）。单独 launch 时 Fixed Frame `world`。静置：`linear_acceleration.z` 为正（~+9.6）。再对齐：`ros2 service call /imu/align_to_parent std_srvs/srv/Trigger`。健康：`ros2 topic echo /diagnostics`。纯核：`PYTHONPATH=src/serial_imu pytest src/serial_imu/test/test_protocol.py src/serial_imu/test/test_frame.py`。协方差：未提供 `[0]=-1`，未知全 0。话题 QoS Reliable。

IMU 姿态跟随（`imu_follow`；**servo 主路径** 09-14 mock 全指标过：15°→目标 0.2618/关节 0.50、锥钳 0.3500、死区归零、回位 0.0002、位置零漂移、拒收告警 0）。自适应档案随 `harvest_system` 起；空心末端须独立 launch 才有节点（战役不走这条）：

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

**插入/回退（自适应圆柱套入，决策 0026）**——peach FULL 在预抓取验证后自动开窗：`enable` → `insert_start` → 剪切 → `insert_retract` → `disable`，回到预抓取才允许 MTC 回 stow。空心末端仍走 MTC LIN，永不调这些服务。`motion.enabled` 默认 false（peach 不自动开门）；mock 跟随另 `ros2 param set /imu_follow motion.enabled true`。禁止与 MTC 同时写控制器。

```bash
# 人工调试仍可用（自适应栈已随 harvest_system 起 imu_follow）：
ros2 service call /imu_follow/enable std_srvs/srv/Trigger
ros2 service call /imu_follow/insert_start std_srvs/srv/Trigger   # 沿工具开口 0.01 m/s 推进，行程钳 0.20 m
ros2 topic echo /imu_follow/target_pose --once
ros2 service call /imu_follow/insert_stop std_srvs/srv/Trigger
ros2 service call /imu_follow/insert_retract std_srvs/srv/Trigger  # 行程收回参考点
ros2 service call /imu_follow/disable std_srvs/srv/Trigger
```

验收口径：`insert_start` 后 `~/target_pose` 位置沿锁定方向匀速前移（0.01 m/s，到 0.20 m 自动停并告警）；`insert_retract` 后位置回到 enable 参考；姿态仍只跟 IMU 增量；`insert_stop`/`disable`/断流即停推进。

```bash
ros2 action send_goal /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'dev', scene_key: 'lab', profile_id: 'default'}"
ros2 topic echo /peach_supervisor/state
ros2 service call /peach_supervisor/control peach_interfaces/srv/ControlTask \
  "{command: 0, expected_state_seq: 0}"
# 预抓取看完方向/定位后 ACK（命令 6），才允许再 Survey
ros2 service call /peach_supervisor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0}"
```

打开真运动须同时改调度 `execution_enabled` 与技能 `execution.enabled`，并经人工授权。到预抓取还须 `grasp.enabled=true`、`tool.enabled=false`。调度侧用 `ros2 param set` 即可（开批与 `HarvestState` 会刷新快照）；技能侧 `ros2 param set` 空闲态全量生效、运行中拒改（execution→grasp→tool 依赖链由节点校验）。调参分层：Python 节点改默认值只改 `src/<pkg>/config/<节点>.yaml`；`peach_arm` 改 GPL `src/arm_parameters.yaml` 并重编。运行期临时改参用 `ros2 param set`（Python 节点原地刷新；感知逐帧读取键即时热生效，模型/管线等构造期捕获键仍需重启；技能空闲态全量生效、运行中拒改）。

---

## 2. 真机干跑（默认不运动、不 SetIO）

`auto_power_on` 必须为 false。柜侧用示教器；规划/FK/IK 用 MoveIt；停轨走透传取消 + 硬件 `RobotMoveStop`（应用停轨，**不是** ISO 13850 急停）。禁止调用 `aubo_dashboard`。真机授权前：示教器急停手能摸到；工作空间无人或已隔离；使能保持默认关。急停或保护停止之后：处理现场 → 示教器复位 → 取消并丢弃 ROS 在途 goal → 重新授权后再下发，**不要 resume 原轨迹**（UR ROS2 Driver 同类警告）。分层见 [AGENTS.md](../AGENTS.md) 第 2 章。

过程数据：新记录在工作区 `runs/`。**现行精华（2026-09-22 压缩）**：`testing-log.md` 为轮次权威；`runs/` 只留空心/自适应网格 jsonl（`sim_field_targets_20260922_155057` / `_161623`）、live SURVEY/PREGRASP 账本（`e2e_survey_20260922T162215` / `e2e_unrefined_20260922T162302`）、以及 `grid_hollow_stowfix_20260922` 的 RViz 关键帧。历史 idle/session/harvest 目录与 MCAP 已删（分析结论已进 testing-log）。08-20～08-24 旧根输出曾在 `_archive/runs/`，同轮清空。每次干跑把结论追加 [testing-log.md](testing-log.md)。过程录制仍为会话 bag（决策 0019）：observability 随栈开合 `runs/session_<时间戳>/bag/`，栈停出 `bag_report.md`；分析后 `python3 scripts/purge_analyzed_bags.py --all-reported runs`。bag 体积超 `record.max_total_bag_gb` 时停栈后自动回收最旧 bag。

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
# mock 无发布者（bringup 仅 real 起 aubo_io_controller）；真机须有
ros2 topic echo --once /aubo_io_controller/robot_status
ros2 topic hz /camera/color/image_raw
ros2 topic echo --once /peach_supervisor/state
ros2 topic echo --once /peach/perception/target_observations
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/state
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/trajectory
ros2 lifecycle get /peach_observability
ros2 lifecycle get /peach_scene_perception_node
ros2 lifecycle get /peach_target_reconstruction_node
ros2 lifecycle get /peach_arm
ros2 lifecycle get /peach_supervisor
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

关节名必须是：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。把 `/joint_states` 对照 SRDF `global_photo_pose`（`src/aubo_e5_moveit_config/config/aubo_e5.srdf` 的 `group_state`）。launch / lifecycle **不到**拍照位：mock xacro `initial_value` 是 `harvest_stow`（wrist2≈−0.50，拍照位 −0.28，Δ≈0.22 rad > `photo_pose_joint_tolerance_rad` 0.05）。开执行后第一次 `SurveyScene` 才 PTP 过去。`execution.enabled=false` 时 Survey **仍核当前**关节：停在袋口开批须 `termination_reason=survey_failed`，不得把袋口 FOV 收进本批锁定集。上一轮若停在 HoldPregrasp，当前多半还在袋口——差值大时先目视/示教器确认再开 `execution`。`harvest_system` mock **不起** `aubo_io_controller`（`/aubo_io_controller/robot_status` 发布者计数为 0）；同 launch 把技能 `execution.require_robot_status` 置 false，否则 Survey 入口 `robot_status_missing`、约数十毫秒 `survey_failed`。真机 KEEP 该门为 true，冒烟须 `drives_powered=1 motion_possible=1`。`ros2 topic hz /camera/color/image_raw` 的订阅 QoS 须与发布端一致。Percipio launch 默认 `color_qos:=default`（RELIABLE）；感知订户也是 RELIABLE depth 10。若改成 `SENSOR_DATA`（BEST_EFFORT），默认可靠的 `hz` 会误报未发布。帧率按前端计：percipio `fps ≈ 2.43` 与感知注册表；`camera_frontend:=stereo` 时 ~13.7 fps（组率由相机节拍限速，hh4+`temporal_k=3` 处理链仍小于相机 73 ms 帧间隔，见 `src/peach_stereo/README.md`）。驱动深度健康另有口径：头 30 帧 valid <10% 即异常（09-21 percipio 曾因 `parameters.xml` 残留值无条件下发崩到 ~5–12%，只看彩色 hz 冒烟发现不了）。本机 `ros2 topic hz` CLI 另有恒 0 帧的工具怪癖（`echo`/rclpy 订户正常）——帧率与健康一律以 30 帧探针为准。订户全线 0 帧而发布端进程健在时，先查挂死的 `ros2 bag record`（`-d N` 自停不可靠，录制一律 `timeout -s INT -k 10` 包裹；死订户会占住 `/camera` 大流）与 `/dev/shm/fastrtps_*` 残留，清杀后复验数据流。四节点须 Active。重建有时停在 inactive：`ros2 lifecycle set /peach_target_reconstruction_node activate`。固定座无导航动作（`NavigateToWorksite` 预留，调度 `_cmd_navigate` 直通 `NAV_OK`）。

显式只扫（SURVEY_ONLY：不选果、不接触；mock 开 execution 仍会 PTP 到拍照位）：

```bash
# mock：先开 execution（grasp/tool 保持关），再 SURVEY_ONLY。未授权不得对真机 SetEnables(execution=true)。
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: true, grasp: false, tool: false, reason: 'mock survey ptp'}"
ros2 action send_goal /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
```

`intent: 2` = SURVEY_ONLY。技能 `execution.enabled=false` 时 Survey **仍核当前关节**（只规划不够）。调度会先 Survey、再 Begin、再 WAIT_LOCK，然后结算（不选果）。默认 intent 0 且调度 `execution_enabled` 关时同样：拍照位失败则 `survey_failed`；成功则锁定后直接结算（不选果、不记 `SKIPPED_QUALITY`）。全流程到预抓取须两边 `execution=true` 且技能 `grasp.enabled=true`。mock 验收：Survey result `已到达全局拍照位姿`、`/joint_states` 过拍照位容差、`termination_reason=completed`、`scene_epoch≥1`、账本 `claimed` 仍为空。使能走操作台 `SetEnables`（广播 `/peach/batch/enables`）；不要只 `ros2 param set` 技能侧。测完 `SetEnables` 全 false。

### 实验室端到端（mock 臂 + 真立体相机）

物理相机不跟随 mock TF，重建凑不齐第二独立机位（`observe_build_view_race` / `min_views=2`）。**不要**放宽 `min_views` / `max_target_drift_m`。接触链验证走 `skip_reconstruction`（默认关；真机 KEEP false；不评套袋方向精度）。默认仍 `execute_pregrasp_only=true`（停预抓取、不 SetIO）。顺序：前清 → 起栈 → 自检 → 只扫 → 跳过重建 PICK_ALL → ACK → 关使能 → 停栈。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
# 实验室接触验证（只扫可去掉 skip_reconstruction）
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true autostart:=false
```

**自检（开批前，相机开时）：** §2 冒烟四节点 Active + `/joint_states` MUST 序；mock **不要**等 `/aubo_io_controller/robot_status`（无发布者，launch 已把 `execution.require_robot_status` 置 false）。另核：

```bash
ros2 param get /peach_supervisor skip_reconstruction          # 须 True（否则 brain overlay 没打进节点）
ros2 param get /peach_arm quality.allow_unrefined_geometry    # 须 True
ros2 param get /peach_scene_perception_node publish_debug_image  # 须 True
# RViz Debug Image 须有检/分割叠加；空图先核上一参再查相机流
timeout 5 ros2 topic hz /camera/color/image_raw
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

**只扫**（`intent: 2`，execution 开、grasp/tool 关）：命令见上节。通过：`photo_pose_reached` → `round_locked` → `completed`，账本 `claimed` 空。

**跳过重建到预抓取**（`intent: 0` = PICK_ALL，`view_policy: 0` = VIEW_FAST）：

```bash
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: true, grasp: true, tool: false, reason: 'mock skip_reconstruction pregrasp'}"
ros2 action send_goal -f /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'e2e_unrefined_YYYYMMDDTHHMMSS', scene_key: 'lab', profile_id: 'default', intent: 0, view_policy: 0}"
```

事件链：`photo_pose_reached` → `round_locked` → `target_dispatched`（调度 WARN `skip_reconstruction`，**无** `BuildTargetModel`）→ `ExecuteTarget PREGRASP_ONLY`（再确认 → 拍照位最短路径到预抓取 → 停稳）→ `target_succeeded`（`geometry_source=scene_observation`）→ `recovery_required`。`PREGRASP_ONLY` 后 `RunHarvest` **等 ACK 才收口**，须在结果返回前发命令 6（`expected_state_seq` 用当前 `HarvestState.state_seq`，ACK 路径允许 0）。不要在订阅回调里再 `spin_until_future_complete`（同 executor 会 `already spinning`）。ACK 后回拍照位再 Survey；已 claim 的果不再选，空巡达限 → `completed`。

```bash
ros2 service call /peach_supervisor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0, reason: 'unrefined pregrasp ack'}"
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: false, grasp: false, tool: false, reason: 'e2e done'}"
```

通过（接线，不评方向）：ledger `outcome=0`、`completion_level=2`、阶段 `reconfirm`+`approach_insert`；全程无 SetIO；`HarvestState.batch_state=completed`。重建节点仍在跑，可能 WARN「worker 队列已满」——skip 不关该节点，不挡接触。对照账本：`runs/e2e_unrefined_20260921T185527/`。

**跳过重建完整接触（套入干跑，不开刀）**：实验室有袋时同上起栈后，运行期把调度 `execute_pregrasp_only` 置 false（**不改仓库 yaml**）。`SetEnables(execution+grasp, tool=false)`。SELECT 在 FULL 下会预检套入终点 IK **和**沿轴笛卡尔（`sleeve_no_ik` / `sleeve_no_cartesian` 直接过滤，不再派到接触再 Cartesian 0/1）。FULL 走再确认→预抓取→沿轴套入→跳过 SetIO→原路撤离→`harvest_stow`。成功撤退后一般**不必 ACK**（recovery 在撤离/回 stow 时清）；套入已动再失败才 `recovery_required`。不评套袋方向；`harvest.grasped` 必须为 false。实验室袋若套入终点超出 E5 工作空间（|p|≳1.05 m），SELECT 会跳过该袋——软件接触链用 mock、无相机、`skip_reconstruction:=true` + `scripts/sim_field_targets.py --mode full --case 1757` 或 `--grid`（在达几何；goal 须 `PROFILE_FULL`，臂侧 `mode=FULL` 不被默认 `profile=0` 盖成 HOLD）。

```bash
ros2 param set /peach_supervisor execute_pregrasp_only false
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: true, grasp: true, tool: false, reason: 'mock skip_reconstruction FULL dry sleeve'}"
ros2 action send_goal -f /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'e2e_full_unrefined_YYYYMMDDTHHMMSS', scene_key: 'lab', profile_id: 'default', intent: 0, view_policy: 0}"
```

通过（实验室有袋）：ExecuteTarget `FULL`（无 Build）；阶段含 `approach_insert` 与套入/撤离；checkpoint 到 `CK_SLEEVED`/`CK_RETREATED`/`CK_STOWED`；日志无 SetIO；`tool.enabled=false` 终局不得宣称采摘成功。测完 `execute_pregrasp_only` 改回 true、使能全关。

通过（mock 无相机软件链）：`sim_field_targets --mode full --case 1757` outcome=0、`completion_level≥3`（套入到位；撤离后为 6）、`grasped=false`、无 SetIO。起栈须 `skip_reconstruction:=true`，核 `/peach_supervisor skip_reconstruction` 为 True（勿热设）。

### 感知稳定 + 两种末端（实验室战役，不开刀、不经 ROS 动真机）

物理臂用示教器停在 `global_photo_pose` 并保持。mock 先 Survey 到同位，锁定 + Reconfirm 才与真相机 3D 对齐；mock 离位后直播 3D 作废，接触只用钉住的场景几何（`skip_reconstruction:=true`）。重建多视本战役不评。两种末端必须**整栈隔离重启**切换 `tool_profile`，禁止混跑。launch 默认仍是 adaptive，空心必须显式 `tool_profile:=hollow_cylinder_v1`（忘传会起跟随）。

1. **空心网格（无相机）**：`hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true tool_profile:=hollow_cylinder_v1 imu_enabled:=false` + `python3 scripts/sim_field_targets.py --grid --mode full --velocity 1.0 --tool-profile hollow_cylinder_v1`。核 `/peach_arm tool.profile_id`、无 `imu_follow` 节点、`ros2 service list` 无 `/imu_follow/*`。在达 `succeed` 须 outcome=0、`completion_level≥3`、`grasped=false`；`lab_oos_20260922` 须 `sleeve_no_cartesian`；贴边/`tool_clearance_failed` 须 skip_select。
2. **真相机 stereo（空心）**：`camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true tool_profile:=hollow_cylinder_v1 imu_enabled:=false`。SURVEY_ONLY → 锁集稳定（确认、ID 不闪、debug 叠加、FPS≥2）→ PREGRASP（Hold 后才 `ControlTask` 命令 6；周期未结束 ACK 会被拒）→ 仅 cartesian 放行时 live FULL。
3. **自适应网格（无相机，与空心隔离）**：换栈 `tool_profile:=adaptive_cylinder_v1`（`imu_follow` 随档案 Include）。同一 `--grid --mode full --tool-profile adaptive_cylinder_v1`。套入走 IMU 接触窗；无 USB 时 `waitImuFollowTravel` 只是墙钟等待，不证明跟随位移。核 `tf2_echo tcp imu_link` 与 `/imu/data` 才算跟随链。FULL 接触窗 = 预抓取→insert→回预抓取。mock 下 `motion.enabled` 默认关，peach 不自动开门。
4. **Percipio**：同门换 `camera_frontend:=percipio`（量程 0.4–0.8 m）。不放宽 `min_views`。
5. **取证**：失败短抓 `record.level:=all`；RViz `scripts/record_rviz_harvest.sh <request_id>`；分析后 `python3 scripts/purge_analyzed_bags.py runs/session_*`。助手：`scripts/lab_perception_grasp_campaign.sh preflight`。

### 真机授权前检查清单（本战役不发 `hardware_mode:=real`）

同时满足才允许以后口头/书面授权 real：

- stereo 锁集稳定，Percipio 锁集稳定（FPS≥2、确认、无 ID 闪）。
- 约束网格 hollow 与 adaptive 各自隔离在达 FULL 干跑达标；贴边/过粗/超臂展以 SELECT skip 计正确。
- 至少一组 live 真袋 PREGRASP（hollow）；cartesian 放行则再一组 live FULL 干跑。
- adaptive IMU 跟随须 USB `/imu/data` + `tf2_echo tcp imu_link`；墙钟等待不算跟随证明。
- 全程无 SetIO、无 ROS 真机运动。

停栈（MUST，与启动前 pgrep 成对；整栈、探针、采集器、`ros2 bag record` 一律适用）：launch 终端 Ctrl+C；一次性命令用 `timeout` 包裹（`ros2 bag record -d N` 自停不可靠，用 `timeout -s INT -k 10`）。结束后复核：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
```

有残留按 PID `kill -TERM`，2 秒仍存活则 `kill -9`，再复核。不要宽泛 `pkill`。发现别人的残留先报告再清理；清后复验数据流（30 帧探针或 `ros2 topic echo --once`）。步骤见 [AGENTS.md](../AGENTS.md) 第 1、9 章。

### 透传冒烟（仅 real）

```bash
aubo_py3.12/bin/python _archive/parked_2026-08-24/tools/passthrough_traj_client.py wave_shoulder
```

### 标定（手眼外参 + 彩色内参）

日常采摘不跑标定，但必须有 `src/aubo_hand_eye_calibration/hand_eye/active.yaml`（入库随仓；改值或覆盖后重启 extrinsics_publisher 生效）。没有则名义 TF 平移 2 cm、单位四元数，光学系会偏约 10 cm。`_archive/runs/hand_eye/` 是历史归档，不被任何代码读取。平移应接近 `[0.045, 0.108, 0.002]`，不是 `[0, 0, 0.020]`。

手眼外参（real，需示教器上电与授权；求解核=OpenCV `calibrateHandEye` 五方法＋MAD＋Huber 精化，`per_frame_reprojection_rms_px` 为单帧门、`max_reprojection_rms_px` 为求解门）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
# 浏览器只开回环 http://127.0.0.1:8088
ros2 service call /hand_eye_extrinsics_publisher/reload std_srvs/srv/Trigger {}
```

彩色内参（棋盘格，与手眼同一块 11x8 内角点 @20mm 板；先起相机前端 percipio 或 stereo；需 X 显示）：

```bash
ros2 launch aubo_hand_eye_calibration intrinsics_calibration.launch.py
# GUI: 采满进度条 -> CALIBRATE -> SAVE  （写出 /tmp/calibrationdata.tar.gz；
# COMMIT 不可用——两前端均无 set_camera_info 服务）
ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz
# 校验（分辨率 640x480、焦距、主点、畸变系数个数）后原子写入
# src/percipio_camera/config/color_camera_info.yaml
colcon build --packages-select percipio_camera   # launch 读 install 副本, 必须重建
# 重启相机前端后验证:
ros2 topic echo /camera/color/camera_info --once   # 核对 K 与文件一致
```

**auto 档（自动视点 + 内外参联合求解，2026-09-17 新增）**：默认仍 poses 示教位姿；auto 需真机+相机+授权。前置：棋盘格固定摆放在臂可达处且当前画面可见；`hand_eye/active.yaml` 存在（初始外参定位板，缺失明确报错、**不做名义回退**）。几何量全部运行时取自活的 `camera_info`（相机无关；视点距离按「板宽画面占比」反推，适配任意焦距）。

```bash
# 与手眼同一 launch / Web 界面; 动作 goal 切档:
ros2 action send_goal /hand_eye_calibration_server/run \
  aubo_msgs/action/RunHandEyeCalibration \
  "{pose_source: auto, solve_target: joint, return_to_start: true}"
```

流程：当前帧定位板 → FOV 掩码过滤的环绕视点（极角 0–45°、方位 60° 步进、三档画面占比，余量 8%）→ 贪心选出旋转跨度 ≥30° 的视点队列 → plan-only 预检（不可达/碰撞视点跳过、冗余候选补位，有效 <12 拒跑）→ settle 后逐帧同步采集（每视点取帧内 RMS 最小前 2 帧**原始角点**，不做 SE3 均值）→ 联合求解（`calibrateCamera` 初始化 + 五方法 hand-eye 初值 + 20 参数 Huber 联合抛光）→ 产物落 `hand_eye/candidates/`（transforms + `intrinsics` 节 + `viewpoints` 明细 + joint 指标）。**FOV 保证是视点位的静态保证，视点间 transit 不承诺**（采集仅在 settle 后）。

验收门：joint 总重投影 RMS ≤0.8px（`joint_max_reprojection_rms_px`）+ 沿用平移/旋转一致性与跨度门；新外参与旧 active 差应在 mm 级。内参生效三步：`~/activate` 激活 → `ros2 run aubo_hand_eye_calibration apply_intrinsics src/aubo_hand_eye_calibration/hand_eye/active.yaml`（读 `intrinsics` 节）→ `colcon build --packages-select percipio_camera` 后重启前端。

离线复算（对已保存 tarball 重跑并打印 K/D/R/P）：
`ros2 run camera_calibration tarfile_calibration --mono -s 11x8 -q 0.020 /tmp/calibrationdata.tar.gz`。
内参变更后手动更新 `src/peach_harvester/config/scene_perception.yaml` 的 `calibration_version` 标签；感知/重建按新 K 重验。标定器是 vendored image_pipeline jazzy 包（`src/camera_calibration`，勿与 apt 同名包并装）。

### 授权后真运动（本页不写默认使能）

须同时打开调度与技能 `execution`。`RunHarvest` / `ExecuteTarget` / `SurveyScene` 动作入口会自动 arm；手动 Trigger 才 `~/set_execution_armed`。抓取再开 `grasp.enabled`；工具再开 `tool.enabled`。卸果站为预留（`DepositToStation` 已删，`DepositResult` 消息保留、W7 起不再随 `ExecuteTarget.Result` 携带，无 `deposit_pose_named_target` 参数）。使能顺序：先关 `grasp` 再关 `execution`（依赖链 execution→grasp→tool）。

---

## 3. RViz（`aubo_e5_moveit_config/rviz/moveit.rviz`）

Fixed Frame 用 **`base_link`**，不要用未接上的 `world`。改显示配置后须重装 `aubo_e5_moveit_config` 并重启 RViz。

现场优先看 **Debug Image**、**Detection Cloud** 与 **Perception Markers**。跳过重建时不要开 TSDF。不要同时开 Camera Points 和 Detection Cloud。TCP 只看 **TCP Trajectory**（已采样线段）；Path 显示与 Marker 双绘会像未走弦。

| 显示名 | 默认 | 话题 | 含义 |
|--------|------|------|------|
| Perception Markers | 开 | `/peach/perception/markers` | ns `scene_perception`。锁定目标 3D。绿 ACCEPT、黄 REOBSERVE、红 REJECT |
| Detection Cloud | 开 | `/peach/perception/single_cloud` | 检测框深度反投影（跳过重建时的单帧几何）；方块 8 mm |
| Camera Points | 关 | `/camera/depth_registered/points` | 整幅配准点云，很密；move_group octomap ③层同订（`sensors_3d.yaml`） |
| TSDF Cloud | 关 | `/peach/reconstruction/tsdf_cloud` | 绑定目标 TSDF 表面；重建开始后再开 |
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 未融体的拼接点 |
| Reconstruction Markers | 关 | `/peach/reconstruction/markers` | 主 ns `target_reconstruction`；精化时再开 |
| TCP Path | 关 | `/peach/observability/tcp_path` | 与 Marker 同源采样；默认关以免双线 |
| TCP Trajectory | 开 | `/peach/observability/markers` | 已采样线段 + 当前 TCP；套入轴只画预抓取→入口 |
| Planned Views | 开 | `/peach_arm/planned_views` | 已走到的拍照位；未执行候选不画 |
| Camera Color | 关 | `/camera/color/image_raw` | 原彩图 |
| Debug Image | 开（`publish_debug_image` 默认 true；关：`ros2 param set /peach_scene_perception_node publish_debug_image false`。可配 `debug_downscale` 降采样） | `/peach/perception/debug_image` | 检/分割叠加。灰框=未满 confirm_frames |
| Imu | 开 | `/imu/data` | `rviz_imu_plugin`：轴 6 cm、无盒子。姿态不写进 `imu_link` TF |
| TF | 开 | `/tf` | 轴长 0.15 m、不显示名字/箭头（避免 `tool_axis` 挡住筒口） |

---

## 4. 验收门（产品）

**适用范围：本节验收门针对现行代码**（阶段执行器 + ExecutionAuthority + CycleContext 重构后）。历史量化基线与现场轮次在 [testing-log.md](testing-log.md)，**仅供排障参考，不构成对现行代码的验证**；现行有效性以按本节就绪单完成的真机验证为唯一依据。

### M1 单果观察（不接触、不开工具）

档位：两边 `execution=true`，`grasp.enabled=false`，`tool.enabled=false`。确认 `motion_possible=1` `e_stop=0`。同一可见目标连续 3 次 PICK_ALL 或 OBSERVE_ONLY。每次须：独立机位 `view_count >= capture.min_views`（默认 2）；`captured_views` 是积分帧数，可大于机位数；TSDF 点数 > 0，refit `ok=True`；`ExecuteTarget.outcome=SUCCEEDED` 且 Build 成功；失败时 ledger 有 `failure_code`（不得计数器全 0 且无告警）。

### 单目标完整抓取（运动、接触，不开工具 IO）

档位：`execution=true`、`grasp.enabled=true`、`tool.enabled=false`。禁止为提速放宽质量门。

- Build 接收后 2 s 内反馈 COLLECTING/READY；未绑定时臂不得环绕。超时取消后须等该 Build 结束再派下一颗。
- 观察：覆盖达标或 `maximum_moves` 用尽才停（不做完位姿序列不收口）；不做墙钟预算/移动+等帧 EMA 预测收口（`time_budget_s` 键已随死分支删除，2026-09-20 W5）。拍照位 + 当前位采帧；下一视点沿当前相机直线截到 `max_camera_step_m`（默认 0.15 m，~0.7 m 处一跨过 8°），只 LIN，失败换候选（绕行看 4.0 rad / 单轴 1.5 rad，不按时长）。到位后等新机位再判覆盖，同机位连帧不算。时长随 ~2.5 FPS 等帧浮动。
- 机位数 `view_count >= capture.min_views`（默认 2），基线/深度/RMSE/内点率过门。`captured_views` 是积分帧数。
- 无精化不得宣称方向准确。
- MTC 接近、直线套入、同轴撤离均须 goal-hold。预抓取先回拍照位（有记录的接近则原路返程，否则 PTP 0.5 s / 失败 OMPL 3.0 s），再走接近主路径：**斜直线 + 沿轴垂直进入**（面内一跳到预抓取下方轴上，keep-roll 对轴；`plan()` 每趟至多两档（斜插 0.20/0.10，wrist1 超限再 0.10/0.04），降速重试是唯一兜底；staging PTP 候选扫描与 `staging.*` 参数已删，2026-09-23 用户定型；不满足即失败收口）（已齐 LIN 段加相对目标 20° 姿态约束）；执行路径 MTC `plan(1)`，不凑满 `mtc_max_solutions`。起点已在袋底侧、直连不穿囊的短修正走直连 LIN（未齐先 LIN 原地对齐工具 Z；keep-roll 自碰换滚转）。G/under 单弦档已删（2026-09-10：photo→G 弦 fraction 均值 0.77、同 seed 100 随机位姿基线 9/100；`sim_approach_probe.py` 复核）。解析覆盖不执臂：`python3 scripts/analyze_approach_envelope.py --n 10000 --seed 20260911`（感知包络 + TCP 测地线 + 果实胶囊；PTP 行程/弧绕行仍须规划）。不走 CIRC/STOMP/OMPL。失败 `skipped_unreachable`，不进 OMPL。再一段沿轴 LIN；反向同轨迹（含 staging 段）回预抓取后**先倒放回拍照位再短 PTP `harvest_stow`**。接近绕腕护栏 **累计 12 rad / 单轴 6.1 rad**；口侧/上方看果实胶囊，逐段审查（工具有限圆柱 vs 感知胶囊；反爬 s ≤ 本段起点 max(s,0)+2 cm，果平面折线各跳都查；PTP staging 兜底首段只查筒体接触）；笛卡尔绕行比 2.6 / 偏离 0.32 m / 回退 0.12 m（2026-09-18 标定：三门旧值 1.8/0.25/0.08 压在合法 staging 绕行簇边缘，同 seed 30 例从 26/30 崩至 1/30；无门实测合法簇 max 比 1.90/偏 0.258/退 0.075，游荡簇 min 比 4.37/偏 0.42/退 0.17，取分离带内余量；标定后 30/30）；TCP 姿态行程绝对 110°（相对起止余量 20°；0=不查）。不按时长；近果 LIN 0.05、自由空间 0.10。mock 回放：`python3 scripts/replay_field_pregrasp.py --case 1757`（现场坐标，不开批）；全链路逐目标回放 `python3 scripts/sim_field_targets.py --case all`（回拍照位走周期内 `goToPhotoPose`，与正式接触段同一函数）；包络内随机位姿 `python3 scripts/sim_field_targets.py --random 16 --seed 20260910`（默认 typical：`axis_z≥0.70` 且 `|entry|≤1.02`，与现场多数袋一致；感知算法允许水平，压测加 `--envelope algorithm`；轨迹形状对照 `--random 30 --seed 20260911 --velocity 1.0`，仅 mock；`--velocity > 0` 时用例间隔 0.05 s）；实时记录 `python3 scripts/trajectory_watchdog.py`（默认不按绕行比停轨）。接近轨迹形状以 mock 为准（下节）；方向/定位仍以真机目视。
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
| P2 袋模型 | 观测 `occlusion_class`；球 marker ns=`prior`；裸果不入 `next_target_id`（`enable_fruit=False`，`class_id=1` 不入管线）；`branch_blocked`/`neighbor_overlap`/`damaged_or_wet` 不得 `allowed` | 沿袋长轴半径剖面，窄头为口、宽头为底，箭头袋底→袋口；斜袋保持长轴不对成竖轴；袋底→袋口只许上半球（从下往上，左右最多水平，禁止朝下）；分割两端比沿轴朝外框边贴合，更贴边的一端为口（竖缝贴左边）；剪切参考在袋口/分割贴框极限，果距不足只否决 `allowed` 不挪刀；两端贴合差不够才用 3D 窄头/逆重力 |
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

### 回放塔（解析回归门；改接近/融合/护栏/包络必跑）

`colcon test --packages-select peach_system_tests` 的 `test_replay_approach`：零 ROS 纯几何，确定性语料三层（现场真袋 14 例（案册 `src/peach_arm/test/fixtures/field_pregrasp_cases.yaml`，W0 迁入并随包安装）；分层 200 例 seed 20260911；随机 100 例 seed 20260910）对照 `test/replay_baselines.json` 冻结基线（基线 a955cea，与 `scripts/analyze_approach_envelope.py --n 200 --seed 20260911` 输出逐数核对一致）。判定语义：`analytic_ok` 只许升不许降（全链路 66/100、26/30 属 mock 栈指标，本层是其下界——joint_travel/PTP 绕行/IK 自碰不在此层观测）；`lin_chord_fail` 双侧容差 1（降=护栏变松，升=变紧）；分母精确断言，采样器或案册漂移须显式重封基线。护栏数学为 `replay_oracle.py`（scripts 逐字移植）；阶段 2 起 `peach_arm` 以同一案册喂真实纯核 gtest 交叉对账。旧 bag 回放兼容：`peach_observability/bag_reader.py` 的 `LEGACY_TYPE_ALIASES`/`LEGACY_TOPIC_ALIASES`（重写轮每改名/删型登记一行）。

### 清洁重写轮功能清单（F1–F13，验收锚点）

重写轮（REFACTORING.md 2026-09-16 节）以功能等价验收，锚点如下；每阶段过门后才进下一段，最终逐条核销：

| # | 功能 | 门 |
|---|------|-----|
| F1 | mock/real 同管线起栈 + autostart + 托管拉起 + 进程死检 | 冒烟 + 杀节点试验 |
| F2 | 停走感知链（检测/分割/半径剖面/身份/锁定/TF 三态） | 纯核单测 + 相机语料回放 |
| F3 | 单目标建模（五门/精确 stamp TSDF/有界 ICP/Huber 融合/预算；融合失败不回滚体积） | 单测 + 回放 |
| F4 | 观察循环两档（fast 单视优先封顶 3 视 / conservative 现行多视原值） | 回放塔两档对照 |
| F5 | 接触周期 + 检查点（staging PTP+轴向 LIN+四层护栏；AT_STAGING→…→CUT_CONFIRMED→RETAINED→RETREATED→STOWED） | 回放塔 + mock 全链路基线（66/100、39/41、26/30、绕行比≤1.70） |
| F6 | 命令门单点强制（使能×clearance×robotReady×¬cancel；旁路=0） | gtest + 单测 + 冒烟 |
| F7 | 批次状态机 + 选果 + 账本 + 批次策略参数（采收率/单果时限/扇区时限/视点档）+ 补采清单 | 纯核单测 + 冒烟 |
| F8 | 操作台四栏（监控/排程/单步/回放分析+审计）+ 只读记录器闭环 | 冒烟闭环 |
| F9 | 工具档案单一事实源注入（双 profile 等价门） | 冒烟 + 等价门 |
| F10 | IMU 链路（udev→/imu/data→诊断） | 单测 + 冒烟 |
| F11 | imu_follow 跟随/插入推进（默认只算不发；disable 立停） | 纯核单测 + mock |
| F12 | 节拍分解计量与 KPI 快报（各段耗时进 ledger/操作台） | 冒烟输出节拍快报 |
| F13 | 单果档案与 yield 视图（视点数/各段耗时/结果/失败码） | 冒烟 + bag_report |

### 节拍基线与 KPI 换算链

每果周期分解（重写轮起全段计时进 ledger/metrics/操作台）：`粗扫帧 → 选果 → 观察循环[视点数×(移臂+停稳+等帧+积分)] → 接触[staging PTP+轴向 LIN+套入+剪切+撤退] → 后勤[回位+记账+放果]`。

- **现状基线**：观察 6 视 33.5 s（§8 缺口表）；接触干跑目标 45–60 s/果（上节）。
- **对标谱系**（完整采收机器人调研，2026-09-16）：人 3–6 s/果；机器人 2.78 s（猕猴桃 2020 大田 55.8% 可达 86%）/5.5 s/5.8–7 s（多臂苹果）/6 s（Panasonic，人 2–3 s，靠 10 h+ 连跑追平日产）/24 s（SWEEPER，其中放果 7.8 s+移车 4.7 s 后勤）/9.7 s·88%（Fu 2024 猕猴桃整簇，**AUBO E5 同臂**）；大田成功率带 51–88%。
- **fast 档方向目标**（方向不是门）：观察 ≤2 视 ≤12 s，单果 ≤20 s（可达果）。
- **KPI 换算链**：单工位节拍 s/果 → 3600/节拍 = 果/h → ÷果/箱 ≈ 箱/日（按有效作业 h）。节拍快报由冒烟/回放自动输出。
- **策略口径**：速度不够时长凑（可靠性优先，Abundant 教训）；跳过是调度参数不是失败（采收率/时限超即跳过入补采清单）。

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

收尾（轮次结束，MUST）：launch 终端 Ctrl+C，再按「停栈」节 `pgrep` 复核无残留（含 bag / collect / probe）。挂死按 PID 清。

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
colcon build --packages-select peach_harvester peach_arm \
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
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 等到 rosout：managed nodes Active（仍须显式 RunHarvest）

# 冒烟：五节点 Active；standoffs 入口 0 / 预抓取 0.03；
# /aubo_io_controller/robot_status drives_powered=1 motion_possible=1；
# tf2_echo wrist3_Link camera_link 平移 ≈ [0.045, 0.108, 0.002]；
# tf2_echo base_link tip 应失败；关节对照拍照位。
ros2 param set /peach_arm execution.enabled true
ros2 param set /peach_arm grasp.enabled true
ros2 param set /peach_supervisor execution_enabled true
# 确认 tool.enabled 仍为 false、execute_pregrasp_only 仍为 true。不要改仓库 yaml。

ros2 action send_goal -f /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_pregrasp_YYYYMMDD_HHMM', scene_key: 'lab', profile_id: 'default'}"
```

停预抓取后现场评方向/定位。看完：

```bash
ros2 service call /peach_supervisor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0}"
```

异常先命令 4（`CANCEL_NOW`）。过程数据在工作区 `runs/`：结构化文本（jsonl/json/csv/md/yaml/txt/log）入库随仓推送；bag/图像/点云二进制仍只留本地（`.gitignore` 白名单）。结论写进 [testing-log.md](testing-log.md) 与 `runs/field_test_<日期>/log.md`。

选果约束（有效深度 + TCP IK 可达，09-01 定稿）：

- **可达性权威 = TCP IK 预检**：SELECT 把各目标感知入口（`entry_pose`，Z=袋轴）与 `suggested_travel_m` 批量送技能 `CheckReachability`。服务端预抓取停位 `setFromIK`（位置沿袋轴后撤 `mtc_approach_along_axis_m`，现行 0.03 m；姿态=`alignFrameZ` 保留当前 TCP 滚转，不抄感知四元数；种子=当前关节，与 MTC 同一运动学）。`execute_pregrasp_only=false` 时 `require_sleeve`：再检套入终点（入口沿袋轴 + clamp(travel, 0.02, 0.20) m）。预抓取无解码 `ik_no_solution` / `no_ik`；套入终点无解码 `sleeve_no_ik`。仍不规划 LIN 路径。
- **有效深度窗**：相机距离 0.30–1.60 m（`selection_depth_min/max_m`）；逐帧掩膜有效深度仍由采集门 `min_mask_depth_ratio` 把关。
- 服务不可用（mock/技能未起）回退标定半径窗 0.88（成功 0.830–0.840 / MTC 0 解 ≥0.917）；FULL 时回退窗另卡套入终点半径。事件里带 `reach_check` 说明。
- 摆位建议：袋底距基座 0.5–0.8 m（参考成功标定区间）。

流程与判定：

1. `ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98`（先 moveit_enabled 默认 true）。
2. Active 后先对照拍照位，再 `ros2 param set` 开执行/抓取（`tool.enabled` 保持 false），再发 `RunHarvest`（launch 不自动开批）。完整命令见上「复现命令」。
3. 每颗期望链：首巡 Survey → Begin → WAIT_LOCK → SELECT → Build+OBSERVE 并行 → `PREGRASP_ONLY` 停在预抓取（作业票停在「靠近」）→ 现场目视评方向/定位（筒口对袋轴？侧向偏多少？剪切点落袋口？）→ `ControlTask` 命令 6 ACK → 回访 Survey（不 Begin）→ 下一颗。
4. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`（不得 `FAILED`）；全程无 SetIO；这些对错只在现场评，`allowed`/余量只作记录。Hold 等 ACK 期间不写 summary（结算只认批次终局）；若需当场核对，以技能 `[SUCCEEDED] PREGRASP_ONLY` 日志与现场停位为准。
5. 轮次结束：launch Ctrl+C，按「停栈」节 pgrep 复核无残留（MUST）。

中断与异常：

- 异常先 `ControlTask` 命令 4（CANCEL_NOW），臂停后人工撤离；恢复等待未 ACK 前调度不会 Survey/派下一颗。
- 停轨走透传取消 + 硬件 `RobotMoveStop`；避障绕行只看行程护栏（观察 4/1.5、接触 12/6.1、拍照 6/2.5）。
- 观察失败高发项（历史）：`selected_target_stale`/`selected_target_changed`（身份新鲜度）、`missing_mask`（有效深度不足）——summary 原因列现在直接给出，先看原因再调参。量化复算与归档数字见 [testing-log.md](testing-log.md)。
