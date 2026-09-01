# 测试

权威：源码 + 实机过程数据。与 [architecture.md](architecture.md)、[io.md](io.md) 构成仅有的三份活文档；**源码与本文互相更新，改启动/验收口径或改本文须同一轮改另一边**。约束：[AGENTS.md](../AGENTS.md)。

各包 `test/` **只保留 ROS 2 默认 lint**（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`）。不要写业务用例、gtest、DDS 假现场或 launch_testing。语法与流程由审查核对，对错以实机为准。`colcon test` 不等于采摘验收。套入剪切软件门看 flake8 / pep257 / uncrustify；文件头 BSD 版权块等项目结束再补，期间跳过 copyright lint。`ament_xmllint` 会拉 `package_format3.xsd`，网络卡住超时不阻塞本产品路径。

不要删 `_archive/runs/` 与现场 `runs/`。未授权不得真机运动或 SetIO。launch **不自动** `RunHarvest`。十四包职责见 [architecture.md](architecture.md) §3。`serial_imu` 不进整栈 launch。

---

## 1. 构建与开发机

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

有残留按 PID 补杀。Python：`aubo_py3.12`；numpy **1.26.4**。禁止 pip 装 opencv-python / scipy；scipy 用 apt 版（`python3-scipy`，感知身份分配与手眼标定共用，`peach_perception` 已声明 exec_depend）。感知 GPL 生成模块同时落在 `install/` 与源码包 `peach_perception/*_parameters.py`（gitignore）；不要把 `PYTHONPATH` 指到 `src/peach_perception` 却不带这两份生成文件，否则场景/重建节点会在 import 期退出，lifecycle 拉不齐 Active。本机若 venv 抢了 `PYTHONPATH`，launch 前先清再只留 Jazzy site-packages 并重新 `source` 两份 setup（见 §4 复现命令）。跨包轴向后撤只改 `src/peach_perception/config/grasp_standoffs.yaml` 两行；不要把它当 ROS `ParameterFile` 直接喂节点（rcl 不允许 `ros__parameters` 之前出现裸值）。能力 launch 读入后注入已声明参数。

```bash
# 开发机：无相机、不运动
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false

# 真机（须显式 real；示教器上电；bringup 不起 aubo_dashboard）
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。参数 `peach_executor/config/observability.yaml`。`/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor` / `robot` / `metrics` / `record` / `params` / `job`。`/api/trajectory` 为末端点列（对照预抓取/入口/弦）。首屏作业票须能看出当前果实停在哪一环、抓取档是否关闭、`GraspDecision.allowed` 与 base_link 坐标；其下三维能看出路径相对弦是否绕行（绕行比、Δz）。默认不上电、不派发运动、不打工具 IO、不自动开批。

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | mock | mock / real |
| `robot_ip` | 169.254.10.98 | 仅 real |
| `camera_enabled` | false | 有相机时设 true |
| `extrinsics_enabled` | true | wrist3 → camera_link |
| `moveit_enabled` | true | move_group + RViz |
| `hand_eye_enabled` | false | 标定流程 |
| `hand_eye_web_enabled` | false | 标定 Web `:8088` |
| `record_mcap` | false | `ros2 bag record -s mcap` |
| 调度 `execute_pregrasp_only` | true | 接触段 `PREGRASP_ONLY`：停预抓取不回 stow；套入前改 false |

完整列表：`--show-args`。只起手臂：`ros2 launch aubo_e5_bringup bringup.launch.py …`。

USB IMU（可选，不进整栈）：手册 [`src/serial_imu/README.md`](../src/serial_imu/README.md)（udev、协议、TF、RViz 插件项、dialout/`newgrp`）。摘要：

```bash
sudo usermod -aG dialout $USER && newgrp dialout
colcon build --packages-select serial_imu
source install/setup.bash
ros2 launch serial_imu serial_imu.launch.py
ros2 topic echo /imu/data --qos-reliability best_effort
```
`usermod` 后必须 `newgrp`（或重新登录）。缺插件：`sudo apt install ros-jazzy-imu-tools`。Fixed Frame `world`；拧模组看 `imu_attitude` / Imu 插件，不是 `imu_link`。

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

打开真运动须同时改调度 `execution_enabled` 与技能 `execution.enabled`，并经人工授权。到预抓取还须 `grasp.enabled=true`、`tool.enabled=false`。调度侧用 `ros2 param set` 即可（开批与 `HarvestState` 会刷新 ParamListener 快照）；不要改仓库 yaml 默认。

---

## 2. 真机干跑（默认不运动、不 SetIO）

`auto_power_on` 必须为 false。柜侧用示教器；规划/FK/IK 用 MoveIt；停轨走透传取消 + 硬件 `RobotMoveStop`。禁止调用 `aubo_dashboard`。

过程数据：新记录在工作区 `runs/`。08-20～08-24 在 `_archive/runs/root_2026-08-24/`。每次干跑把结论写进 `runs/field_test_<日期>/log.md`。批次结束自动生成 `summary.md`：头部含验收门对照（帧率 ≥2.0 / tf_failures=0 / 到预抓取停住 ≥1，口径见下）与配对账本路径（账本在 `runs/<request_id>/`，监控在 `runs/run_*/`，两树互引）；终局事件 `message` 带 `failure_code`，原因列直接可读。

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

关节名必须是：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。把 `/joint_states` 对照 SRDF `global_photo_pose`（`src/aubo_e5_moveit_config/config/aubo_e5.srdf` 的 `group_state`）。launch / lifecycle **不到**拍照位；开执行后第一次 `SurveyScene` 才 PTP 过去。上一轮若停在 HoldPregrasp，当前多半还在袋口——差值大时先目视/示教器确认再开 `execution`。`ros2 topic hz /camera/color/image_raw` 默认可靠 QoS，相机是 best_effort，可能误报未发布；以 Percipio `fps ≈ 2.43` 与感知注册表为准。四节点须 Active。重建有时停在 inactive：`ros2 lifecycle set /peach_target_reconstruction_node activate`。固定座无导航动作（`NavigateToWorksite` 预留，调度 `_cmd_navigate` 直通 `NAV_OK`）。

显式只扫（仍不运动）：

```bash
ros2 action send_goal /peach_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
```

`intent: 2` = SURVEY_ONLY。技能 `execution.enabled=false` 时 Survey 只规划拍照位。默认 intent 0 且调度 `execution_enabled` 关时，第一次 Survey 后直接结算（不选果、不记 `SKIPPED_QUALITY`）。全流程到预抓取须两边 `execution=true` 且技能 `grasp.enabled=true`。

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
| Camera Points | 关 | `/camera/depth_registered/points` | 整幅配准点云，很密 |
| TSDF Cloud | 开 | `/peach/reconstruction/tsdf_cloud` | 绑定目标 TSDF 表面 |
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 未融体的拼接点 |
| Reconstruction Markers | 开 | `/peach/reconstruction/markers` | 主 ns `target_reconstruction`；精化 `peach_reconstruction/refined`。相机轨迹与精化示意 |
| Planned Views | 开 | `/peach_manipulation_node/planned_views` | 候选拍照位；`execution.enabled=false` 时仍会出，不代表已走到 |
| Camera Color | 关 | `/camera/color/image_raw` | 原彩图 |
| Debug Image | 开 | `/peach/perception/debug_image` | 检/分割叠加。灰框=未满 confirm_frames |

---

## 4. 验收门（产品）

**适用范围：本节验收门针对现行代码**（阶段执行器 + ExecutionAuthority + CycleContext 重构后）。§5/§6 的历史量化基线与现场记录出自重构前代码，**仅供排障参考，不构成对现行代码的验证**；现行代码的有效性以按 §4 就绪单完成的真机验证为唯一依据。

### M1 单果观察（不接触、不开工具）

档位：两边 `execution=true`，`grasp.enabled=false`，`tool.enabled=false`。确认 `motion_possible=1` `e_stop=0`。同一可见目标连续 3 次 PICK_ALL 或 OBSERVE_ONLY。每次须：独立机位 `view_count >= capture.min_views`（默认 2）；`captured_views` 是积分帧数，可大于机位数；TSDF 点数 > 0，refit `ok=True`；`ExecuteTarget.outcome=SUCCEEDED` 且 Build 成功；失败时 ledger 有 `failure_code`（不得计数器全 0 且无告警）。

### 单目标完整抓取（运动、接触，不开工具 IO）

档位：`execution=true`、`grasp.enabled=true`、`tool.enabled=false`。禁止为提速放宽质量门。

- Build 接收后 2 s 内反馈 COLLECTING/READY；未绑定时臂不得环绕。超时取消后须等该 Build 结束再派下一颗。
- 观察：覆盖达标或 `maximum_moves` 用尽才停（不做完位姿序列不收口）；`time_budget_s` 只进日志，不按移动+等帧 EMA 预测收口。拍照位 + 当前位采帧；下一视点沿当前相机直线截到 `max_camera_step_m`（默认 0.15 m，~0.7 m 处一跨过 8°），先 LIN 失败才 PTP（绕行看 2.5 rad / 单轴 1.5 rad，不按时长）。到位后等新机位再判覆盖，同机位连帧不算。时长随 ~2.5 FPS 等帧浮动。
- 机位数 `view_count >= capture.min_views`（默认 2），基线/深度/RMSE/内点率过门。`captured_views` 是积分帧数。
- 无精化不得宣称方向准确。
- MTC 接近、直线套入、同轴撤离均须 goal-hold。预抓取先 PTP 回拍照位，再按最短路径选 LIN / CIRC / PTP（短程已齐且直线不穿预抓取球则 LIN；直线会穿球则 CIRC；短程未齐则 PTP 转 Z 再 LIN；远距或无 IK 则 PTP）。再一段沿轴 LIN；反向同轨迹回预抓取后 PTP `harvest_stow`。接近绕行护栏 **累计 12 rad / 单轴 6.1 rad**（URDF 满行程；不按时长；0.10 速度下直线可以超过 20 s）。
- 日志不得出现 SetIO。`harvest.grasped=false`（未开工具不得宣称采摘成功）。
- 单目标目标 45–60 s；失败必须有 `failure_code`，不得停在 `RUNNING + action_active=false`。

**2026-08-21 / 08-24：本节未通过。** 08-25 `field_full_20260825_1851`：`target_1` 接近/插入/撤离均 goal-hold，工具关、无 SetIO。`target_0` 因观察在同机位连帧上提前收口未接触。实验室两果均应走到接触干跑。方向准不准以现场目视为准。带工具采摘未做。此前同日许可后轨迹被过紧护栏拒掉，臂未走出接触。

树干/粗枝进 PlanningScene 是预留。无人工确认的无粗枝通道时不接触。

### 套袋套入剪切软件门（P0–P8；真机剪切未做）

全程默认 `execution/grasp/tool=false`。launch 不自动 `RunHarvest`。

| 门 | 怎么验 | 现行 |
|----|--------|------|
| P0 可构建 + 工具帧 | 干净 `build/install/log` 后 colcon；URDF 有 `tool_axis` / `sleeve_mouth` / `cutting_plane` / `tool_body_link` | TCP 在圆柱顶部 `(0, 47.90, 151.07) mm`，`Rx(-90°)`：Z=开口、XY=刀口；筒沿 −Z 200 mm |
| P1 几何基线 | `runs/` 写 `geometry.jsonl`；`ros2 run peach_perception peach_bag_baseline --runs runs` | 离线脚本已装 |
| P2 袋模型 | 观测 `occlusion_class`；球 marker ns=`prior`；裸果不入 `next_target_id`；`branch_blocked`/`neighbor_overlap`/`damaged_or_wet` 不得 `allowed` | 沿袋长轴半径剖面，窄头为口、宽头为底，箭头袋底→袋口；斜袋保持长轴不对成竖轴；袋底→袋口只许上半球（从下往上，左右最多水平，禁止朝下）；分割两端比沿轴朝外框边贴合，更贴边的一端为口（竖缝贴左边）；剪切参考在袋口/分割贴框极限，果距不足只否决 `allowed` 不挪刀；两端贴合差不够才用 3D 窄头/逆重力 |
| P3 重建权威 | `allowed` 须袋融合预算才套入；无 budget 不得接触；圆柱/TSDF 不定轴；包络轴只否决，扁袋不打 12°；35° 只诊断 | FULL 时 `allowed=false` → `SKIPPED_QUALITY`；`PREGRASP_ONLY` 不要求 `allowed` |
| P4 预抓取 | 默认 `execute_pregrasp_only=true`；停预抓取（现行 TCP 在拟合袋底，入口=预抓取）；无 SetIO；ACK 后再 Survey。**方向/定位是否可用与精度以到位后真机目视/测量为准**，不以预算或 2°/3 mm 残差代替 | 残差未过门也 Hold；08-28b 因 `allowed` 未到预抓取，源码已拆门。08-31 1554 拍照位 PTP 到达后接近 10.79 / 4.23 被当时 10 / 3.2 拒；护栏已改为 12 / 6.1。**09-01 1704 `target_1`**：入口外 70+100 mm，目视方向与位置良好。**09-01 1757 `target_1`（现行 0 后撤）**：TCP `[0.304, -0.614, 0.536]`，目视方向与定位**中上水平、只需微调** |
| P5 套入干跑 | `grasp=true` `tool=false`；套入与反向撤退均须先过 `PlanSleeve` 规划；到预抓取只走直线 | 软件路径已接线；直线失败不绕行 |
| P6 刀具 | `ToolActuator`：SetIO ACK ≠ `cut_confirmed`；无硬件反馈时不得 `CUT_CONFIRMED` | 已实现；真刀未接 |
| P7 成功语义 | `harvest.grasped` 仅 `cut_confirmed && retreat_confirmed`；tool 关干跑可 `outcome=SUCCEEDED` 但 grasped=false | 软件已钉死 |
| P8 导航适配（预留） | 导航包已归档（`_archive/parked_2026-09/`）；四个 IDL 名在 manifest `reserved_interfaces`；调度 `_cmd_navigate` 直通 `NAV_OK` | 固定座现行；清单脚本核对预留区 |

```bash
ros2 run peach_perception peach_bag_baseline --runs runs
python3 src/peach_interfaces/scripts/check_interface_manifest.py
```

### PREGRASP_ONLY 真机里程碑（运动、不开工具）

档位：两边 `execution=true`，`grasp.enabled=true`，`tool.enabled=false`，调度 `execute_pregrasp_only=true`（现行默认）。到预抓取后看筒口是否对袋轴、侧向是否偏、剪切紫点是否落在袋口（分割贴检测框极限）。**这些对错只在现场评**，监控里的 `allowed`/余量/RMSE 只作记录，不作为本里程碑通过条件。残差 2°/3 mm 未过也停住。任何路径无 SetIO。**结束后停在预抓取**，不回 `harvest_stow`。`ExecuteTarget` 终局须 `SUCCEEDED` 且 `recovery_required`（作业票停在靠近、等 ACK）；不得记 `FAILED` 或「须现场人工撤离」。看完后 `ControlTask` `ACKNOWLEDGE_RECOVERY`（命令 6）才允许再 Survey / 下一颗。`grasp.enabled=false` 到不了预抓取。判断/执行全图：`runs/field_test_20260828/pregrasp_only_flow.mmd`（现场结论同目录 `log.md`）。

**2026-08-28：** 轮次 A `field_pregrasp_20260828` 重建 ndarray `or` 崩溃（已修）。轮次 B `field_pregrasp_20260828b` 重建未崩：`target_0` 技能锁定集未跟上拒 OBSERVE；`target_4` 约 2 机位/4 帧，圆柱 RANSAC 与关键点轴冲突 → `allowed=false` / `refined_quality_not_allowed`，未进 `MovePregrasp`/`HoldPregrasp`。无 SetIO。随后改为：包络否决不拦预抓取、融合几何与接触许可拆开、入口侧向贴体积、独立剪切参考。轮次 C `field_pregrasp_20260828c`：六节点 Active 后开批，`target_0` 观察约 15 s 有效视点 0/1 → `observe_failed` / `insufficient_views`，仍未到预抓取，无 SetIO。轮次 D–D6 只看 Debug Image：上半球约束后 `target_1` 箭头朝左略上。轮次 E `field_pregrasp_20260828e`：开执行/抓取后 Survey 过，`target_0` `build_start_timeout`，`target_1` `build_rejected`（取消 Build 后未等结束就派下一颗），未进预抓取，无 SetIO。源码已改为取消后等待。轮次 F `field_pregrasp_20260828f`：`target_1` 观察约 17 s 有效视点 0/1 → `insufficient_views`，仍未到预抓取，无 SetIO。根因：袋融合后写 `geometry.jsonl` 对 `cut_pose` ndarray 用了 Python `or`，被当成 TSDF 积分失败并回滚体积，故 RViz 无 TSDF Cloud、技能有效视点 0。已修：写点不用 `or`；融合失败不回滚已积分体积。轮次 G `field_pregrasp_20260828g`：两颗均积分（各 2 视、TSDF ~2000 点、`refit ACCEPT`），观察门过；`PREGRASP_ONLY` 发了，`ptp to on-axis pregrasp` MTC 0/1，未到位、无 SetIO。批次结束后体积复位，RViz TSDF Cloud 会空，须在观察/重建进行中看。**方向定位精度待真机停预抓取后评。** 全图与逐门实测：`runs/field_test_20260828/`。

**2026-08-31：** 拍照位再最短路径。`field_pregrasp_20260831_1554`：发现 3、尝试 3、成功 0，无 SetIO、未 HoldPregrasp。`target_1` 观察 2 视 refit ACCEPT，PTP 回拍照位 goal-hold，随后单段 PTP 到预抓取 10.79 rad / 单轴 4.23 > 10 / 3.2 → `skipped_unreachable`。随后护栏改为 **12 / 6.1**。16:33 / 16:36 重测（`1633`/`1636`）三颗均 `observe_failed`（EMA 预测收口砍掉 8 cm 第二机位）。17:00 `field_pregrasp_20260831_1700`：观察停准则改为覆盖/次数用尽、步长 0.15 m。`target_1` 短移 0.117 m、基线 10.82°、PTP 预抓取 10.96 / 4.16 过 12 / 6.1，HoldPregrasp `SUCCEEDED`+`recovery_required`，无 SetIO。ACK 时调度因 `is_service_ready` 崩溃（已改为 `service_is_ready`）；另两颗未派。方向只在现场评。账本 `runs/field_pregrasp_20260831_1700/`；综述 `runs/field_test_20260831/log.md`。

**2026-09-01：** 全程 PREGRASP_ONLY，未 Hold。`field_pregrasp_20260901_0900`：两颗 `observe_failed`（锁定集竞态；0.15 m 观察 LIN 累计 **2.63–3.70 rad** 被默认 2.5 拒）。本会话运行时 `observe_max_total_joint_travel_rad=4.0`（yaml 默认仍 2.5）。`0907`：`target_1` 观察 LIN 约 8.3 s / 0.15 m 过 4.0，到位后 `selected_target_stale`；重建全程 `missing_mask`（有效深度 EMA≈0.27）。无 SetIO。过程 `runs/field_test_20260901/log.md`。

**09-01 下午（重构后代码首批真机）：** `1440`：`target_12/13` 首次走通 观察→重建→融合→再确认→MovePregrasp，卡 MTC `ptp to on-axis pregrasp` 0 解（预抓取半径 0.92/1.07 m 物理超程）；`radial_budget_negative`/`bag_d95_exceeds_tool` 双预算同拒。随后上线 **TCP IK 选果预检**（`CheckReachability`）+ 有效深度窗：`1500` 四颗全窗过滤零浪费；`1540` IK 过滤 0/1/3、选中 `target_2` 真做观察短移，卡 `neighbor_gap`（邻锚 58.5 mm，105 次拒帧）——近距双检。**现场定夺：近距双检先做检测框大的（小框多为叶遮残片/误检）**：选果次序改 priority+框面积降序、串扰门小框豁免（面积比 2.0）。同日随后：观察行程门固化 `observe_max_total_joint_travel_rad=4.0` 进 yaml；监控修三处——终局事件并入 `failure_code`、ACK/暂停/恢复审计事件、summary 加验收门对照与账本互引。

**09-01 17:04 `field_pregrasp_20260901_1704`（重构后首次 Hold）：** 验收门「到预抓取停住 ≥1」✓。`target_2` 观察 2 视过门后精化预抓取半径 ~0.93 m，MTC PTP 0 解 → `skipped_unreachable`（SELECT 用感知入口过 IK，精化入口仍超程）。`target_1` 观察 LIN 0.12 m / 8.8 s → 拍照位 PTP → 预抓取 PTP 5.71 s goal-hold → **HoldPregrasp `SUCCEEDED` + `recovery_required`**。实测 TCP `[0.298, -0.639, 0.367]` 与规划预抓取重合。`allowed=false`（`bag_d95_exceeds_tool`）未拦预抓取。无 SetIO、无 `neighbor_gap` 拒帧。**现场目视 `target_1`：方向与位置良好，轨迹可行**（该轮入口外 70 mm、再后撤 100 mm）。账本 `runs/field_pregrasp_20260901_1704/`。

**09-01 17:57 `field_pregrasp_20260901_1757`（现行 0 后撤、停拟合袋底）：** 命令与冒烟见下节就绪单。起栈前示教器手动回到 `global_photo_pose`（相对 SRDF 最大 |Δq|=0.0001 rad）。`target_2` SELECT `ik_no_solution`。`target_1` 观察 LIN 0.118 m / 8.28 s → 回拍照位 3.94 s → 预抓取 PTP 5.53 s goal-hold → **HoldPregrasp `SUCCEEDED` + `recovery_required`**。入口=预抓取=`[0.304, -0.614, 0.536]`（相对 1704 TCP 沿袋轴约 +17 cm，与去掉 70+100 mm 后撤一致）。`allowed=false`（`bag_d95_exceeds_tool`）未拦。无 SetIO。ACK 前 summary 计 `unfinished`（门「到预抓取停住」显示 0）——以 ExecuteTarget 终局与现场停位为准，不要等 ACK 才认 Hold。**现场目视 `target_1`：方向与定位中上水平，只需微调。** 过程 `runs/field_test_20260901/log.md`；账本 `runs/field_pregrasp_20260901_1757/`。

### 预抓取全程测试就绪单（真机前逐项核对）

> 历史过程记录（§5/§6）仅供排障参考；**本次测试是现行代码（阶段执行器重构后）的首次真机验收，以本单流程与现场实测为准。**

前置（上电/起栈前）：

1. `pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'` 无残留，有则按 PID 补杀。整栈 launch 预检同样拦 `extrinsics_publisher`（多代会叠发 `wrist3→camera_link`）。
2. 示教器上电、抱闸释放；`auto_power_on=false` 保持不动；不调 `aubo_dashboard`。
3. 手眼 TF 在位：`ros2 run tf2_ros tf2_echo wrist3_Link camera_link` 有输出（缺失则点云相对臂偏 ~10 cm 且轴不对，不得开批）。
4. TF 帧核查：`ros2 run tf2_ros tf2_echo base_link tip` 应报**不存在**——存在即有旧实例污染（link1/link2/tip 链）；launch 预检已拦，现场仍见则按 PID 清理后重启。
5. 相机出图且深度配准：`ros2 topic hz /camera/color/image_raw` ≈2.5 FPS。
6. 当前关节对照 SRDF `global_photo_pose`。launch 不自动到位；开执行后第一段运动是 Survey PTP 回拍照位。停在预抓取重启时这一段行程大，须现场确认再开批。

档位（预抓取全程）：

| 项 | 值 | 说明 |
|----|----|----|
| 调度 `execution_enabled` | true | 运行期可 `ros2 param set` |
| 技能 `execution.enabled` / `grasp.enabled` | true / true | 到不了预抓取先查这两档 |
| 技能 `tool.enabled` | **false** | 全程不得出现 SetIO |
| 调度 `execute_pregrasp_only` | true | 现行默认；到位后停住等 ACK |
| 观察 `observe_max_total_joint_travel_rad` | 4.0 | yaml 已固化，勿再运行时改 |
| 轴向后撤 | `grasp_standoffs.yaml` 现行 `0.0` / `0.0` | 入口=预抓取=拟合袋底 |

复现命令（09-01 1757 按此跑；`request_id` 每次换新，勿复用）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_perception peach_manipulation peach_executor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
# 有残留按 PID 补杀。停在预抓取的重启：示教器先回到拍照位
# （SRDF global_photo_pose ≈ 0.425, 0.195, 1.678, 1.462, -0.500, 0.039 rad）。
# go_to_photo_pose 行程门 6 / 2.5，从袋口回去可能被拒，故 1757 用示教器。

unset PYTHONPATH
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
source /opt/ros/jazzy/setup.bash
source /home/mu/Desktop/aubo_e5_jazzy_ws/install/setup.bash
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 等到 rosout：managed nodes Active（仍须显式 RunHarvest）

# 冒烟：五节点 Active；standoffs 三个节点均为 0；
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

异常先命令 4（`CANCEL_NOW`）。过程数据在工作区 `runs/`（gitignore，不入库）；结论写进本文与 `runs/field_test_<日期>/log.md`。

选果约束（有效深度 + TCP IK 可达，09-01 定稿）：

- **可达性权威 = TCP IK 预检**：SELECT 段把各目标感知入口（`entry_pose`，姿态=入口姿态）批量送技能 `CheckReachability`（`setFromIK`，种子=当前关节状态，与 MTC 同一运动学），无解目标过滤并留 `ik_no_solution` 归因。技能停位 = 入口 − `mtc_approach_along_axis_m`（`grasp_standoffs.yaml` 注入；现行 0 则停在入口/拟合袋底）。
- **有效深度窗**：相机距离 0.30–1.60 m（`selection_depth_min/max_m`）；逐帧掩膜有效深度仍由采集门 `min_mask_depth_ratio` 把关。
- 服务不可用（mock/技能未起）回退标定半径窗 0.88（成功 0.830–0.840 / MTC 0 解 ≥0.917），事件里带 `reach_check` 说明。
- 摆位建议：袋底距基座 0.5–0.8 m（参考成功标定区间）。

流程与判定：

1. `ros2 launch peach_executor harvest_system.launch.py hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98`（先 moveit_enabled 默认 true）。
2. Active 后先对照拍照位，再 `ros2 param set` 开执行/抓取（`tool.enabled` 保持 false），再发 `RunHarvest`（launch 不自动开批）。完整命令见上「复现命令」。
3. 每颗期望链：Survey → Build+OBSERVE 并行 → `PREGRASP_ONLY` 停在预抓取（作业票停在「靠近」）→ 现场目视评方向/定位（筒口对袋轴？侧向偏多少？剪切点落袋口？）→ `ControlTask` 命令 6 ACK → 下一颗。
4. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`（不得 `FAILED`）；全程无 SetIO；这些对错只在现场评，`allowed`/余量只作记录。ACK **前** 自动 `summary.md` 常把该目标记 `unfinished`、门「到预抓取停住」显示 0——以技能 `[SUCCEEDED] PREGRASP_ONLY` 与现场停位为准，不要等 ACK 才认 Hold。

中断与异常：

- 异常先 `ControlTask` 命令 4（CANCEL_NOW），臂停后人工撤离；恢复等待未 ACK 前调度不会 Survey/派下一颗。
- 停轨走透传取消 + 硬件 `RobotMoveStop`；避障绕行只看行程护栏（观察 4/1.5、接触 12/6.1、拍照 6/2.5）。
- 观察失败高发项（历史）：`selected_target_stale`/`selected_target_changed`（身份新鲜度）、`missing_mask`（有效深度不足）——summary 原因列现在直接给出，先看原因再调参。


---

## 5. 量化基线（归档实测；行为类基线出自重构前代码，仅作对照起点）

硬件事实类基线（相机 ~2.5 FPS 等）与机型无关，继续有效；行为类基线（护栏行程、视角数、新鲜度命中率、成功案例）记录的是重构前代码的表现，**只作排障对照起点**——现行代码改动后（阶段执行器、姿态约束、修正回路），这些数字必须由按就绪单完成的真机批次重新确立，不得拿旧数字当现行代码的合格证据。

仓库根、不依赖 ROS：

```bash
python3 scripts/replay_metrics.py
```

只读 `_archive/runs/` 与（若存在）`runs/`，不写、不删。改观察/MTC 后用同一脚本对比。

| 门 | 归档事实 | 变好长什么样 |
|----|----------|----------------|
| 相机速率 | ~2.4–2.5 FPS（log 2.43；中位间隔 0.4 s） | 新 live hz 覆盖前，设计仍按 2.5 |
| 有效视角 | 接触轮常 4–6；成功对照 15 | 停稳对齐后 skip 不以 `robot_not_static` 为主 |
| 静止跳过 | lin target_1：147 拒中 93 次（63%） | 有效视角/秒接近感知 FPS 的停走子集，而非 ~0.2 |
| 轴门 35° | 完全错轴诊断；套入许可改动态预算 | 不得为提速放过双表面；也不得把 35° 当套入角门 |
| TF | 接触轮 `tf_failures=0` | tf_failures 上升须停 |
| 新鲜度 | 08-24 观察失败 5/10，主因 `selected_target_stale` | 未测得 EMA 时门限 3.0 s；测得后只放宽 |
| MTC | 接近 25–29 s 撞 **12 s** 接触护栏；笛卡尔 0.967；`(0/1)` | 超护栏保持 `skipped_unreachable` |
| 会话 | 批次结束后 events 再写 ~65 min | 结束后不再追加空观测 |
| 工具 | 全程 `tool.enabled=false` | 日志无 SetIO |

数据根：`_archive/runs/root_2026-08-24/`。工作区 `runs/` 可缺失。

解读陷阱：批次 `summary.md` 重建终值常是 IDLE / `captured_views: 0`——用逐目标 `captured_views_max`。GraspDecision 心跳 `not_ready` 占多数 ≠ 精化从未 ACCEPT。感知单帧 ACCEPT 远少于 REOBSERVE，不能当批次成功率。

---

## 6. 现场记录（工具 IO 关；**重构前代码**，仅供排障参考）

本节全部记录（08-21～09-01）产生于重构前的代码（行为树执行器、旧护栏与旧监控）。保留价值：失败模式清单（身份新鲜度、缺掩膜、行程拒发、调度竞态）与对应排查线索。旧记录中的「已修/已改为」结论**不证明现行代码正确**——同类路径在重构中已重写，须按 §4 就绪单重新验证；「预抓取全程测试」即首次对现行代码的真机验收。

过程结论：`_archive/runs/root_2026-08-24/web_runs/field_test_20260821/log.md`；综述 `reports/process-data-analysis-2026-08-24.md`。

### 08-21 已落地

整栈 Active，相机 ~2.4 fps，手眼 TF。Survey → 并行 Build+OBSERVE → FULL(`skip_observation`)。静止采帧。35° 轴门拦住 ~49°。opt3：`target_0` 4 视角轴 ~11° `allowed=true`，再确认「未获得新鲜观测」但跟踪 OBSERVED → `skipped_quality`；`target_1` 5 视角轴 ~49°，MTC 预计 29 s > 12 s → `skipped_unreachable`。无 SetIO。

### 08-24

再确认改为 OBSERVED 用当前锚点 + `updated_s`。`grasp`：`target_1` 插入 0.967，当时 `min_fraction=1.0` → skip。`grasp4`：两目标 4 视角，接近预计 25 s / 27 s > 12 s → `skipped_unreachable`。未下发接触轨迹。

当日后源码已改为：当前位采帧 + 最多一次 12° 短 PTP；接触沿检测轴短程 LIN（未对轴则 PTP 到预抓取点），禁止接触段 OMPL；`min_views=2`；`min_fraction=0.95`；拍照位过 `transit_max_*`；不预填 EMA。**未再宣称抓取成功。**

### 08-25

开抓取关工具。许可后 9 s 与 12.6 s 的正常接近曾被 12 s / 4–8 rad 当成绕行拒掉。护栏改为 **20 s / 10 rad / 单轴 3.2**（仍拒 40 s 爬行与 4.5 rad 绕腕）后，18:53 `field_full_20260825_1851`：`target_1` 接近/插入/撤离均 goal-hold（12.4 s / 3.0 s / 4.2 s），工具关、无 SetIO；`target_0` 仍 `insufficient_angular_baseline`。方向准不准以现场目视为准。带工具采摘未做。

### 08-31

`execute_pregrasp_only=true`，开抓取关工具。时长门已关（`*_max_duration_s=0`）。同日早班：晨间 LIN 远移+slerp `NO_IK` / 工作空间；1347 观察硬可达过滤后无候选（随后去掉硬过滤）；1351 `target_0` LIN 规划过、IK 过、轴 ~11.7°，被当时 20 s 时长门拒（行程 8.2 rad，慢直线不是绕行）；1405 从观察 look-at 出发 LIN `NO_IK`。15:54 `field_pregrasp_20260831_1554` 先 PTP 拍照位再规划：`target_1` 拍照位到达，接近 PTP 10.79 rad / 单轴 4.23 被 10 / 3.2 拒；另两颗观察失败。护栏改为 **12 / 6.1** 后 16:33 `1633`、16:36 `1636` 均三颗 `observe_failed`，未进接近。17:00 `1700`：观察按位姿序列停、步长 0.15 m，`target_1` HoldPregrasp `SUCCEEDED`（PTP 10.96 / 4.16 过 12 / 6.1），无 SetIO；ACK 调度崩溃（`service_is_ready`），另两颗未派。方向只在现场评。过程 `runs/field_test_20260831/log.md`。

### 09-01 代码审查修复（P0 安全底线，未上真机）

全链路逻辑审查后先行修复四处高危 + 两处索引错位，行为回到文档既有口径，无需改验收门：

1. **身份分配死循环**：手写 Munkres 调整步方向写反（加错行集合，搜索区净变化为零，造不出新零点），密集果簇下感知 worker 线程静默挂死。改用 apt `scipy.optimize.linear_sum_assignment`（禁止边仍以大有限代价参与、结果按有限性过滤，语义同旧接口）；离线 500 密集门控用例无挂死、200 矩形用例与暴力枚举同最优。
2. **plan-only 泄漏真机运动**：plan-only 预览失败经 BT Fallback 落入 `AcquireReconstructionViews`（execute 硬编码 true）会真的移动机械臂。`btAcquireViews` 入口加 plan-only 安全门（只规划、终态 PLAN_READY）。P2 重写后由模式 switch 结构性保证。
3. **手动周期旧目标钉残留**：`start_cycle` 不清上一 action 周期的 `cycle_target_id_`，会按已 HARVESTED 目标的陈旧锚点执行。手动分支补清。
4. **切断假确认**：删除 `confirmFeedback(false)` 伪调用；反馈未接线前 `tool.enabled=true` 终局保持 FAILED/CUT_FEEDBACK_TIMEOUT（有意保守侧）。
5. **球内点索引错位**：果线球拟合 inliers 是「法线有效子集」下标，改用同一子集取点（原全量索引典型场景约 1/3 内点为离群）。
6. **圆柱抛光抽稀错样本**：内点 >800 时改为对内点下标等距抽稀（原对全点云采样，混入外点污染袋轴）。
