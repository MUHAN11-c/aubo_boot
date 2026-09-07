# 测试流程与命名

权威：源码。与 [architecture.md](architecture.md)、[io.md](io.md) 构成仅有的三份活文档；**源码与本文互相更新，改启动/验收口径或改本文须同一轮改另一边**。约束：[AGENTS.md](../AGENTS.md)。

真机轮次、量化基线、审查记录写在 [testing-log.md](testing-log.md)（过程记录，不驱动现行设计）。改行为只改本文 + 源码；补一条实测时追加 testing-log，不把轮次散文写回本文。

各包 `test/` **只保留 ROS 2 默认 lint**（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`）。不要写业务用例、gtest、DDS 假现场或 launch_testing。语法与流程由审查核对，对错以实机为准。`colcon test` 不等于采摘验收。套入剪切软件门看 flake8 / pep257 / uncrustify；文件头 BSD 版权块等项目结束再补，期间跳过 copyright lint。`ament_xmllint` 会拉 `package_format3.xsd`，网络卡住超时不阻塞本产品路径。

不要删 `_archive/runs/` 与现场 `runs/`。未授权不得真机运动或 SetIO。launch **不自动** `RunHarvest`。十四包职责见 [architecture.md](architecture.md) §3。`serial_imu` 不进整栈 launch。

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
| 监控会话 | `runs/run_<YYYYMMDD>_<HHMMSS>/`（观测自动） | 与账本互引 |

场景键 `scene_key` 现行实验室用 `lab`。`profile_id` 现行 `default`。

写记录：当场把结论写入 `runs/field_test_<日期>/log.md`，并追加 [testing-log.md](testing-log.md) 对应轮次。原始 jsonl 只在 `runs/`（gitignore），不入库。

---

## 1. 构建与开发机

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

有残留按 PID 补杀。clangd：上述 `CMAKE_EXPORT_COMPILE_COMMANDS` 让每个 CMake 包在 `build/<pkg>/compile_commands.json` 留下编译命令；工作区 `.clangd` 按包指向这些文件。驱动栈 CMakeLists 只读，不在那些包里写 `set(CMAKE_EXPORT_COMPILE_COMMANDS)`。改完 CMake 或新编一包后 **Clangd: Restart language server**。Python：`aubo_py3.12`。依赖分层（venv-first）：ROS 2 依赖走 Jazzy apt；其余第三方（numpy/scipy/opencv/PyYAML/open3d/torch 等）一律由工作区 `requirements.txt` 钉版本装进 venv（对同名 apt 包需 `pip install --ignore-installed -r requirements.txt` 才真正落入 venv）。**numpy 必须 ==1.26.4**（<2）：Jazzy 的 cv_bridge 二进制按 numpy 1.x 编译，numpy 2.x 会 `import cv2` 报错、`import cv_bridge` 段错误；该版本同时是 apt python3-numpy 的版本，双路径一致。感知身份分配与手眼标定共用 scipy（venv 内 1.11.4）；`peach_perception` 的 package.xml 只声明 ROS 键与 `python3-numpy`（ABI 边界），数值库不走 rosdep。感知 GPL 生成模块同时落在 `install/` 与源码包 `peach_perception/*_parameters.py`（gitignore）；不要把 `PYTHONPATH` 指到 `src/peach_perception` 却不带这两份生成文件，否则场景/重建节点会在 import 期退出，lifecycle 拉不齐 Active。本机若 venv 抢了 `PYTHONPATH`，launch 前先清再只留 Jazzy site-packages 并重新 `source` 两份 setup（见 §4 复现命令）。跨包轴向后撤只改 `src/peach_perception/config/grasp_standoffs.yaml` 两行；不要把它当 ROS `ParameterFile` 直接喂节点（rcl 不允许 `ros__parameters` 之前出现裸值）。能力 launch 读入后注入已声明参数。

```bash
# 开发机：无相机、不运动
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false

# 真机（须显式 real；示教器上电；bringup 不起 aubo_dashboard）
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。参数 `peach_executor/config/observability.yaml`。`/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor` / `robot` / `metrics` / `record` / `params` / `job` / `debug`。`/api/trajectory` 为末端点列（对照预抓取/入口/弦）。首屏作业票须能看出当前果实停在哪一环、抓取档是否关闭、`GraspDecision.allowed` 与 base_link 坐标；其下三维能看出路径相对弦是否绕行（绕行比、Δz）。默认不上电、不派发运动、不打工具 IO、不自动开批。

### Web 手动调试（决策 0013）

监控页 Tab「手动调试」＝向**既有**动作/服务发调试请求的纯客户端：生命周期（ManageNodes）、调度（RunHarvest/ControlTask）、感知（BeginScene）、重建（Build/finalize/save_session/query）、技能（SurveyScene/CheckReachability/go_to_photo_pose/preview/ack/arm/ExecuteTarget 各档位）。**三重门默认全关**：`debug.enabled=false`（POST 一律 503）→ `debug.token=""`（空=一律 401）→ 运动类另需 `debug.motion_enabled=true`（false=423）。每次操作（含被拒，含操作面未启用的 503）审计 `runs/debug_audit/<日期>.jsonl`（`audit_enabled` 默认开）。

启用（真机手调预抓取方向定位等场景）：

```bash
# 1) config/observability.yaml：debug.enabled: true、debug.token: <自定义>；
#    真要动臂再加 debug.motion_enabled: true（技能侧 execution/grasp/tool
#    三重使能仍须另行人工打开，ExecutionAuthority 照常复核）
# 2) 重新 launch（观测节点随整栈，或单起）：
ros2 run peach_executor peach_observability --ros-args \
  --params-file <含 debug 段的 yaml>
# 3) 浏览器 8090 → 手动调试 Tab → 填令牌 → 「测试令牌」应提示可用
```

门控验收口径（curl 矩阵，`X-Debug-Token` 头）：无/错令牌 `401`；令牌对 + `motion_enabled=false` 时 RunHarvest 非 SURVEY_ONLY、Survey、ExecuteTarget 非 PREVIEW（含 OBSERVE_ONLY）、go_to_photo_pose、arm、ControlTask 的 RESUME/EXIT_MAINTENANCE 一律 `423`；PREVIEW/BeginScene/save_session/生命周期/只规划 Trigger 不受运动门拦；未知端点 `404`；未知 mode/intent/command `400`；非调试路径 `404`。Web 只是又一客户端：**绕不过** `ExecutionAuthority`、调度使能与重建门；真机运动授权流程不变。

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

打开真运动须同时改调度 `execution_enabled` 与技能 `execution.enabled`，并经人工授权。到预抓取还须 `grasp.enabled=true`、`tool.enabled=false`。调度侧用 `ros2 param set` 即可（开批与 `HarvestState` 会刷新快照）；技能侧 `ros2 param set` 空闲态全量生效、运行中拒改（execution→grasp→tool 依赖链由节点校验）。调参分层：改默认值改 `config/*_parameters.yaml`（GPL 单一事实源，须重编）；固化部署覆盖写同名 `config/<节点>.yaml`（只写与默认不同的键）；运行期临时改参用 `ros2 param set`。

---

## 2. 真机干跑（默认不运动、不 SetIO）

`auto_power_on` 必须为 false。柜侧用示教器；规划/FK/IK 用 MoveIt；停轨走透传取消 + 硬件 `RobotMoveStop`。禁止调用 `aubo_dashboard`。

过程数据：新记录在工作区 `runs/`。08-20～08-24 在 `_archive/runs/root_2026-08-24/`。每次干跑把结论写进 `runs/field_test_<日期>/log.md`，并追加 [testing-log.md](testing-log.md)。批次结束自动生成 `summary.md`：头部含验收门对照（帧率 ≥2.0 / tf_failures=0 / 到预抓取停住 ≥1，口径见下）与配对账本路径（账本在 `runs/<request_id>/`，监控在 `runs/run_*/`，两树互引）；终局事件 `message` 带 `failure_code`，原因列直接可读。ACK 前自动 summary 常把 Hold 记成 `unfinished`——以技能终局与现场停位为准。

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

关节名必须是：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。把 `/joint_states` 对照 SRDF `global_photo_pose`（`src/aubo_e5_moveit_config/config/aubo_e5.srdf` 的 `group_state`）。launch / lifecycle **不到**拍照位；开执行后第一次 `SurveyScene` 才 PTP 过去。`execution.enabled=false` 时 Survey 仍核**当前**关节：停在袋口开批须 `termination_reason=survey_failed`，不得把袋口 FOV 收进本批锁定集。上一轮若停在 HoldPregrasp，当前多半还在袋口——差值大时先目视/示教器确认再开 `execution`。`ros2 topic hz /camera/color/image_raw` 默认可靠 QoS，相机是 best_effort，可能误报未发布；以 Percipio `fps ≈ 2.43` 与感知注册表为准。四节点须 Active。重建有时停在 inactive：`ros2 lifecycle set /peach_target_reconstruction_node activate`。固定座无导航动作（`NavigateToWorksite` 预留，调度 `_cmd_navigate` 直通 `NAV_OK`）。

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
| Camera Points | 关 | `/camera/depth_registered/points` | 整幅配准点云，很密 |
| TSDF Cloud | 开 | `/peach/reconstruction/tsdf_cloud` | 绑定目标 TSDF 表面 |
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 未融体的拼接点 |
| Reconstruction Markers | 开 | `/peach/reconstruction/markers` | 主 ns `target_reconstruction`；精化 `peach_reconstruction/refined`。相机轨迹与精化示意 |
| Planned Views | 开 | `/peach_manipulation_node/planned_views` | 候选拍照位；`execution.enabled=false` 时仍会出，不代表已走到 |
| Camera Color | 关 | `/camera/color/image_raw` | 原彩图 |
| Debug Image | 开 | `/peach/perception/debug_image` | 检/分割叠加。灰框=未满 confirm_frames |

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
- MTC 接近、直线套入、同轴撤离均须 goal-hold。预抓取先 PTP 回拍照位，再只走 LIN / CIRC（直线不穿预抓取球则 LIN；直线会穿球则 CIRC；未齐则先 LIN 原地对齐工具 Z 再 LIN）。已齐 LIN 段加相对目标 20° 姿态路径约束。LIN/CIRC 失败不改 PTP。再一段沿轴 LIN；反向同轨迹回预抓取后 PTP `harvest_stow`。接近绕行护栏 **累计 12 rad / 单轴 6.1 rad**，以及笛卡尔 **绕行比 2.2 / 弦偏离 0.25 m / 回退 0.08 m**（URDF 满行程；不按时长；0.10 速度下直线可以超过 20 s）。
- 日志不得出现 SetIO。`harvest.grasped=false`（未开工具不得宣称采摘成功）。
- 单目标目标 45–60 s；失败必须有 `failure_code`，不得停在 `RUNNING + action_active=false`。

接触干跑历史轮次见 [testing-log.md](testing-log.md)。实验室两果均应走到接触干跑。方向准不准以现场目视为准。带工具采摘未做。

树干/粗枝进 PlanningScene 是预留。无人工确认的无粗枝通道时不接触。

### 套袋套入剪切软件门（P0–P8；真机剪切未做）

全程默认 `execution/grasp/tool=false`。launch 不自动 `RunHarvest`。

| 门 | 怎么验 | 现行 |
|----|--------|------|
| P0 可构建 + 工具帧 | 干净 `build/install/log` 后 colcon；URDF 有 `tool_axis` / `sleeve_mouth` / `cutting_plane` / `tool_body_link` | TCP 在圆柱顶部 `(0, 47.90, 151.07) mm`，`Rx(-90°)`：Z=开口、XY=刀口；筒沿 −Z 200 mm |
| P1 几何基线 | `runs/` 写 `geometry.jsonl`；复算脚本已归档（需要时 `_archive/offline_2026-09/` 下以模块方式运行） | 离线脚本已归档 |
| P2 袋模型 | 观测 `occlusion_class`；球 marker ns=`prior`；裸果不入 `next_target_id`；`branch_blocked`/`neighbor_overlap`/`damaged_or_wet` 不得 `allowed` | 沿袋长轴半径剖面，窄头为口、宽头为底，箭头袋底→袋口；斜袋保持长轴不对成竖轴；袋底→袋口只许上半球（从下往上，左右最多水平，禁止朝下）；分割两端比沿轴朝外框边贴合，更贴边的一端为口（竖缝贴左边）；剪切参考在袋口/分割贴框极限，果距不足只否决 `allowed` 不挪刀；两端贴合差不够才用 3D 窄头/逆重力 |
| P3 重建权威 | `allowed` 须袋融合预算才套入；无 budget 不得接触；圆柱/TSDF 不定轴；包络轴只否决，扁袋不打 12°；35° 只诊断 | FULL 时 `allowed=false` → `SKIPPED_QUALITY`；`PREGRASP_ONLY` 不要求 `allowed` |
| P4 预抓取 | 默认 `execute_pregrasp_only=true`；停预抓取（入口在拟合袋底，预抓取沿 −axis 后撤 30 mm）；无 SetIO；ACK 后再 Survey。**方向/定位是否可用与精度以到位后真机目视/测量为准**，不以预算或 2°/3 mm 残差代替 | 残差未过门也 Hold；`allowed=false` 不拦预抓取。停袋底对照轮次见 [testing-log.md](testing-log.md) 1757 |
| P5 套入干跑 | `grasp=true` `tool=false`；套入与反向撤退均须先过 `PlanSleeve` 规划；到预抓取只走直线 | 软件路径已接线；直线失败不绕行 |
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
6. 当前关节对照 SRDF `global_photo_pose`。launch 不自动到位；开执行后第一段运动是 Survey PTP 回拍照位。停在预抓取重启时这一段行程大，须现场确认再开批。

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
4. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`（不得 `FAILED`）；全程无 SetIO；这些对错只在现场评，`allowed`/余量只作记录。ACK **前** 自动 `summary.md` 常把该目标记 `unfinished`、门「到预抓取停住」显示 0——以技能 `[SUCCEEDED] PREGRASP_ONLY` 与现场停位为准，不要等 ACK 才认 Hold。

中断与异常：

- 异常先 `ControlTask` 命令 4（CANCEL_NOW），臂停后人工撤离；恢复等待未 ACK 前调度不会 Survey/派下一颗。
- 停轨走透传取消 + 硬件 `RobotMoveStop`；避障绕行只看行程护栏（观察 4/1.5、接触 12/6.1、拍照 6/2.5）。
- 观察失败高发项（历史）：`selected_target_stale`/`selected_target_changed`（身份新鲜度）、`missing_mask`（有效深度不足）——summary 原因列现在直接给出，先看原因再调参。量化复算与归档数字见 [testing-log.md](testing-log.md)。
