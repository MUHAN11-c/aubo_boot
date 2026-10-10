# peach（v1）测试指导

> 本文档是 v1 采摘栈（`peach_*`，入口 `peach_bringup/harvest_system.launch.py`）的测试流程指导，2026-10-10 自仓库级 `docs/testing.md` 重写入包（用户裁定）。现行系统快照的活文档为 [docs/architecture.md](../../../docs/architecture.md) 与 [docs/io.md](../../../docs/io.md)；**改启动/验收口径或改本文须同一轮改源码**。历史过程文档（docs/testing-log.md、docs/REFACTORING.md）与工作区级战役脚本（scripts/）均已随 2026-10-10 转向 peach2 的清理退役：结论在 git 历史，脚本可 `git show <提交>:scripts/<名>` 取回。
>
> v2 栈的测试指导在 [src/peach2/peach2_system_tests/doc/testing.md](../../../src/peach2/peach2_system_tests/doc/testing.md)。

## 0. 红线（先读）

- 未授权不得真机运动或 SetIO（操作员在 real 上发起 launch/操作台指令 = 授权）。
- 硬件急停在示教器/柜，不经 ROS；`RobotMoveStop`/取消 action 是应用停轨，**不得称 e-stop**。
- 驱动九包只读；`auto_power_on=false`；bringup 不起 `aubo_dashboard`。
- **测完即停**：任何测试/程序结束后必须退出并 `pgrep` 清残留（见 §14）。
- mock 不证明安全；碰撞、电流、急停回路、刀、人在环只在真机 KEEP。

## 1. 构建与环境（开发机）

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

- Python：`aubo_py3.12` venv；**numpy 必须 ==1.26.4**（Jazzy cv_bridge 按 numpy 1.x 编译，升 2.x ABI 崩）。venv 抢 `PYTHONPATH` 时先清再依次 source Jazzy 与工作区两份 setup。
- ROS 2 依赖走 Jazzy apt；其余第三方钉在 `requirements.txt` 装 venv（scipy 1.11.4、open3d、torch 等）。
- 感知模型权重随仓：`peach_harvester/model/best.pt`（YOLO）与 `mobile_sam.pt`；空路径在参数层拒绝启动。
- 改完 C++ 后 **Clangd: Restart language server**；驱动栈 CMakeLists 只读。

## 2. 测试塔

| 层 | 工具 | 内容 |
|----|------|------|
| Lint | `ament_lint_auto`（flake8/pep257/uncrustify/cppcheck） | 各包 `test/` 默认保留；`peach_arm` 跳过 cpplint（版权头/include 序与本项目约定冲突，SNAPSHOT） |
| 纯核 | pytest（零 rclpy）/ gtest | `peach_common` 七文件、`peach_harvester` 纯核组、`peach_observability/test_bag_report`、`peach_arm` gtest 十二套、`serial_imu`/`imu_follow` 协议与核 |
| 集成 | isolated launch_testing | 本包（域 89–94），mock 硬件 |
| 仿真 | `peach_sim`（Gazebo Harmonic + Blender 离线门） | 见 §12 |
| 现场 | 真机 + `runs/` | 方向/接触对错的最终权威（KEEP） |

```bash
# 纯核+lint 门（原 r0_gate.sh 的等价命令；该脚本 2026-10-10 退役）
colcon test --packages-select peach_common peach_interfaces peach_harvester \
  peach_arm peach_bringup peach_observability peach_vegetation peach_system_tests peach_sim
colcon test-result --verbose
# 接口清单核对（改 IDL/接线必跑）
python3 src/peach_interfaces/scripts/check_interface_manifest.py
```

测试文件名必须匹配 pytest 默认收集（`test_*.py`）——2026-09-18 前曾因 `vision_test_*` 命名 24 个模块不被收集（假绿）。C++ 新测试允许 gtest；launch_testing 用 ENV 隔离域（Jazzy 无 `add_ros_isolated_launch_test` 宏，等效 `ROS_DOMAIN_ID=<N> ROS_LOCALHOST_ONLY=1`）。

## 3. 命名

批次 `request_id` 同时是账本目录名 `runs/<request_id>/`，**不得复用**，按当时本地时间填 `YYYYMMDD`/`HHMM`。

| 用途 | 格式 | 例 |
|------|------|-----|
| 预抓取真机 | `field_pregrasp_<YYYYMMDD>_<HHMM>` | `field_pregrasp_20260901_1757` |
| 接触干跑（不开刀） | `field_full_<YYYYMMDD>_<HHMM>` | `field_full_20260825_1851` |
| 只扫不运动 | `field_dry`；或预抓取名 + `intent: 2` | `intent: 2` = SURVEY_ONLY |
| 实验室只扫 | `e2e_survey_<YYYYMMDDTHHMMSS>` | 不评套袋方向 |
| 跳过重建接触验证 | `e2e_unrefined_<YYYYMMDDTHHMMSS>` | `intent: 0` + `skip_reconstruction:=true` |
| 开发机 mock | `dev` | `intent` 默认 0 |
| 监控会话 | `runs/session_<YYYYMMDD>_<HHMMSS>/bag/` | observability 随栈开合，停栈自动出 `bag_report.md` |

`scene_key` 现行实验室用 `lab`；`profile_id` 现行 `default`。`runs/` 结构化文本入库随仓、bag/图像等二进制只留本地（.gitignore 白名单）；分析完成后删除 `bag/` 目录回收（历史回收脚本已退役，政策不变）。

## 4. mock 起 + 冒烟

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false
```

冒烟（发 `RunHarvest` 之前）：

1. `ros2 control list_controllers`：JTC/透传控制器 active。
2. `ros2 topic echo /joint_states --once`：六关节名与 MUST 序一致（`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`；JSB 的 name 数组常为字母序，按下标当 MUST 序会拧腕）。
3. `ros2 run tf2_ros tf2_echo world tcp`（或现行 TCP 帧）：有变换、无跳变风暴；`tf2_echo base_link tip` 应报**不存在**（存在即旧实例污染）。
4. lifecycle 四节点 `active`（场景 → 重建 → 技能 → 调度）；`ros2 lifecycle get /peach_arm` 等。
5. mock 不起 `aubo_io_controller`（`robot_status` 发布者计数 0；launch 已把 `execution.require_robot_status` 置 false）。真机 KEEP 该门 true。
6. 相机开时：`ros2 topic hz /camera/color/image_raw`（订阅 QoS 须与发布端一致；本机 `hz` CLI 有恒 0 帧怪癖，帧率以 30 帧探针为准）；驱动深度健康口径=头 30 帧 valid <10% 即异常。
7. 启动自检（0037）：栈激活后 observability 自动首轮自检、每 30s 复检；`ros2 topic echo --once /peach/observability/selfcheck_passed`；软门语义=自检红只呈现不拦手动 RunHarvest，`autostart:=true` 的自动批硬等 `selfcheck_passed=true`。

launch 参数（常用）：`tool_profile`（默认 `adaptive_shear_v1`；`shear_v1`/`bite_shear_v1` 显式传，切换须整栈隔离重启）、`camera_frontend`（`stereo` 默认 / `percipio` / `replay`）、`imu_enabled`（默认 true）、`skip_reconstruction`（默认 false；真机 KEEP false，且真感知下不可用）、`brain_params_overlay`（冻结相机等测试 overlay）、`use_sim_time`（bag 回放 true + `ros2 bag play --clock`；真机必须 false）。完整列表 `--show-args`。

## 5. 本包 launch_testing（isolated 域）

| 测试 | 域 | 内容 |
|------|----|------|
| `test_mock_launch.py` | 89 | mock `harvest_system` 起栈接线：`/joint_states` MUST 序、lifecycle Active、**不发 RunHarvest**、ENV 隔离（`ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1`）、`AUBO_RUNS_DIR` 指临时目录、`QT_QPA_PLATFORM=offscreen` |
| `test_e1_supervisor_chain.py` | 90 | 测试进程真发 RunHarvest 走完 FSM（「不发 RunHarvest」仅指 mock_launch） |
| `test_tool_profile_smoke_{shear_v1,bite_shear_v1,adaptive_shear_v1}` | 91/92/93 | TF `wrist3_Link→tcp`==档案 ±2mm、`/peach_arm tool.profile_id` 对齐、imu_follow 接线按档案在/不在图 |
| `test_scene_obstacles.py` | 94 | 合成点云+假目标+假 apply 端：障碍成 box/目标邻域空洞/贴机器人滤除/超半径裁剪/REMOVE+ADD 原子替换 |
| `test_replay_approach.py` | — | 回放塔（见 §10），零 ROS 纯几何 |
| `test_frozen_camera_bag.py` | — | 零 GPU 零 ROS 图的 bag/overlay 断言，缺捕获目录自动 skip（进默认 colcon test） |
| `test_perf_baseline.py` / `test_baseline_inventory.py` | — | `perf_baseline.json` 性能对拍锚点 schema 守卫 |

`colcon test --packages-select peach_system_tests` 一次全跑。headless 下 `move_group`/`rviz2` 退出码不纳入 peach 进程门；本机**同域**栈残留时预检拒测（按 `ROS_DOMAIN_ID` 过滤；`frozen_camera_grasp.launch` 在白名单）。

## 6. 冻结相机一次感知 → 抓一颗（推荐仿真，域 81）

静止 RGB-D 来自 `runs/peach_camera_capture_20260930/stereo_10s`（`PEACH_FROZEN_BAG` 可改；该采集 mcap 已按回收政策清除，需重新采集或从备份恢复时先恢复 bag 再跑）。回放节点把 stamp 改成当前时刻，mock 臂 Survey 到拍照位后用当时腕 TF 反投影。**只扫一次、只抓一颗**；产品 yaml `min_views=2` 不动，测试 overlay `config/frozen_one_view.yaml` 才置 1（另：基线门 0°、`reconstruct_in_trajectory` 跳过补视 LIN、`tool.budget.d_inner=0.25` 过单视 sigma；驱动再开 `grasp.allow_stale_anchor`、关 `moveit.obstacle_guard_enabled`、`grasp.maximum_travel_m` 收到 0.02、快照 `workspace_radius_m` 收到 0.05）。不要走 `skip_reconstruction`（真感知未精化直径为 0）。第一颗终局后 action cancel。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
ros2 run peach_system_tests run_frozen_grasp.py        # v1 域 81；bite_shear_v1、tool=false 干跑套入
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|frozen_camera|move_group|ros2_control_node'
```

- 整段墙钟 **180 s**（launch+感知+接触+拆栈），超时即失败并排查，不要拉长。
- 2026-10-09 起 rviz2 不再强设 `QT_QPA_PLATFORM=offscreen`（会砸 Ogre GLX 窗口）；无头环境 rviz2 崩退不影响判定（退出码不计入），要彻底关 rviz 直起 `aubo_e5_moveit_config moveit.launch.py rviz:=false`。
- 成功口径：账本/结果至少一颗 `succeeded`（v1 `tool=false` 干跑套入+跳过 SetIO），不是"尝试过就算过"。
- FULL 抓取用例（需 bag+GPU）不进默认 colcon test；`test_frozen_camera_bag`（零 GPU）已注册进默认。

## 7. 实验室端到端（mock 臂 + 真立体相机）

物理相机不跟随 mock TF，重建凑不齐第二独立机位；**不要**改产品 `min_views`/`max_target_drift_m`。接触链验证走 `skip_reconstruction`（不评套袋方向）。顺序：前清 → 起栈 → 自检 → 只扫 → 跳过重建 PICK_ALL → ACK → 关使能 → 停栈。

```bash
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true autostart:=false
# 开批前核：
ros2 param get /peach_supervisor skip_reconstruction       # 须 True
ros2 param get /peach_arm quality.allow_unrefined_geometry # 须 True
timeout 5 ros2 topic hz /camera/color/image_raw
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

**只扫**（`intent: 2`，execution 开、grasp/tool 关）：`SetEnables(execution=true, grasp=false, tool=false)` → `RunHarvest{intent: 2}`。通过：`photo_pose_reached → round_locked → completed`，账本 `claimed` 空。

**跳过重建到预抓取**（`intent: 0`，view_policy 0）：

```bash
ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
  "{execution: true, grasp: true, tool: false, reason: 'mock skip_reconstruction pregrasp'}"
ros2 action send_goal -f /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'e2e_unrefined_YYYYMMDDTHHMMSS', scene_key: 'lab', profile_id: 'default', intent: 0, view_policy: 0}"
# PREGRASP_ONLY 后 RunHarvest 等 ACK 才收口（结果返回前发命令 6）：
ros2 service call /peach_supervisor/control peach_interfaces/srv/ControlTask \
  "{command: 6, expected_state_seq: 0, reason: 'unrefined pregrasp ack'}"
```

事件链：`photo_pose_reached → round_locked → target_dispatched`（无 BuildTargetModel）→ `ExecuteTarget PREGRASP_ONLY` → `target_succeeded(geometry_source=scene_observation)` → `recovery_required`。通过（接线）：ledger `outcome=0`、`completion_level=2`、阶段 `reconfirm`+`approach_insert`；全程无 SetIO。

**套入干跑（不开刀）**：运行期 `ros2 param set /peach_supervisor execute_pregrasp_only false`（不改仓库 yaml）+ `SetEnables(execution+grasp, tool=false)`。FULL 预检套入终点 IK 与沿轴笛卡尔（`sleeve_no_ik`/`sleeve_no_cartesian` 在 SELECT 过滤）。成功撤退后一般不必 ACK；`harvest.grasped` 必须 false。测完 `execute_pregrasp_only` 改回 true、使能全关。

> 原 mock 网格战役入口 `scripts/sim_field_targets.py`（`--grid`/`--random`/`--tool-profile`）与回放/探针脚本族已于 2026-10-10 退役（git 历史可取）。mock 接触链现走本节 supervisor goal 流（`drive_harvest.py`）+ 回放塔解析回归；三末端档位隔离与验收词表口径不变（§9）。

## 8. 真机纪律（授权前/中/后）

发真运动或 SetIO 之前：示教器急停手能摸到；工作空间无人或已隔离；使能保持默认关直到口头/书面授权；先 mock 再 real。`hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98`，示教器上电。

档位（干跑默认）：

| 项 | 值 |
|----|----|
| `hardware_mode` | real |
| `camera_enabled` | true |
| 调度 `execution_enabled` / 技能 `execution.enabled` / `grasp.enabled` / `tool.enabled` | 全 false |
| launch | 不自动 RunHarvest |

冒烟另加：`ros2 topic echo --once /aubo_io_controller/robot_status`（须 `drives_powered=1 motion_possible=1`）、相机 hz、四节点 Active、`tf2_echo wrist3_Link camera_link` 平移 ≈ `[0.045, 0.108, 0.002]`、当前关节对照 SRDF `global_photo_pose`。

急停/保护停止之后：处理现场 → 示教器复位 → 取消并丢弃 ROS 在途 goal → 重新授权后再下发，**不要 resume 原轨迹**。停轨 = 透传取消 + 硬件 `RobotMoveStop`。异常先 `ControlTask` 命令 4（CANCEL_NOW）。

### 预抓取全程就绪单（真机前逐项）

前置：pgrep 无残留；示教器上电、抱闸释放；手眼 TF 在位；`tf2_echo base_link tip` 报不存在；相机出图且深度配准（≈2.5 FPS percipio / ~13.7 stereo）；当前关节对照拍照位。复现命令链（含 venv PYTHONPATH 双 source 次序）与逐项检查见本节历史版（git 历史 `docs/testing.md`）；要点：

1. 起 real 栈，等 managed nodes Active。
2. `ros2 param set` 开 `/peach_arm execution.enabled`、`grasp.enabled` 与 `/peach_supervisor execution_enabled`；`tool.enabled` 保持 false、`execute_pregrasp_only` 保持 true。
3. `RunHarvest{request_id: 'field_pregrasp_YYYYMMDD_HHMM', scene_key: 'lab', profile_id: 'default'}`。
4. 每颗期望链：首巡 Survey → Begin → WAIT_LOCK → SELECT → Build+OBSERVE → `PREGRASP_ONLY` 停预抓取（作业票停「靠近」）→ 现场目视评方向/定位 → 命令 6 ACK → 回访 Survey → 下一颗。
5. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`；全程无 SetIO；对错只在现场评，`allowed`/余量只作记录。
6. 轮次结束 launch Ctrl+C + pgrep 复核（MUST）。

观察护栏参考：观察 `observe_max_total_joint_travel_rad` 4.0 / 单轴 1.5；接触 12 / 6.1；拍照位行程 6 / 2.5。轴向后撤 `grasp_standoffs.yaml` 现行入口 0.0 / 预抓取 0.03。

### 透传冒烟（仅 real）

```bash
aubo_py3.12/bin/python _archive/parked_2026-08-24/tools/passthrough_traj_client.py wave_shoulder
```

## 9. 验收门（产品）

**适用范围：现行代码**（阶段执行器 + ExecutionAuthority + CycleContext 重构后）。历史量化基线已随 testing-log 退役；现行有效性以按本节就绪单完成的真机验证为唯一依据。

### M1 单果观察（不接触、不开工具）

两边 `execution=true`，`grasp/tool=false`，确认 `motion_possible=1 e_stop=0`。同一可见目标连续 3 次 PICK_ALL/OBSERVE_ONLY。每次须：独立机位 `view_count >= capture.min_views`（默认 2）；TSDF 点数 >0，refit `ok=True`；`ExecuteTarget.outcome=SUCCEEDED` 且 Build 成功；失败时 ledger 有 `failure_code`。

### 单目标完整抓取（运动、接触，不开工具 IO）

- Build 接收后 2s 内反馈 COLLECTING/READY；未绑定时臂不得环绕。
- 观察：覆盖达标或 `maximum_moves` 用尽才停；下一视点沿当前相机直线截到 `max_camera_step_m`（0.15 m），只 LIN；同机位连帧不算新机位。
- 机位数、基线/深度/RMSE/内点率过门；无精化不得宣称方向准确。
- 接近主路径：**PTP 关节空间到中段点正下方（滚转梯子 {0,±30°,±60°}+随机重启 IK）→ 世界垂直 LIN 入冠 → 沿轴 LIN 对轴进入预抓取**；已齐 LIN 加相对目标 20° 姿态约束；MTC `plan(1)`。反向先倒放回拍照位再短 PTP `harvest_stow`。
- 护栏：接近绕腕累计 12 rad / 单轴 6.1 rad；笛卡尔绕行比 2.6 / 偏离 0.32 m / 回退 0.12 m；TCP 姿态行程绝对 110°（相对余量 20°）。近果 LIN 0.05、自由空间 0.10 m/s。
- 日志不得出现 SetIO；`harvest.grasped=false`；单目标 45–60 s；失败必须有 `failure_code`。

### 套袋套入剪切软件门（P0–P9）

| 门 | 怎么验 | 现行 |
|----|--------|------|
| P0 可构建+工具帧 | 干净 build 后 colcon；URDF 含 `tool_axis`/`sleeve_mouth`/`cutting_plane`/`tool_body_link`；三 profile xacro 展开含全部冻结帧+stl 引用；`test_tool_profile_smoke_*` 自动化 | TCP 按当前 `tool_profile`：shear `(0,47.90,151.07)mm Rx(−90°)` / bite `(0,30.24,165.5)mm` / adaptive `(0,47,168.66)mm Rx(−90°)` |
| P2 袋模型 | 观测 `occlusion_class`；裸果不入 `next_target_id`；`branch_blocked`/`neighbor_overlap`/`damaged_or_wet` 不得 `allowed` | 袋长轴半径剖面定口/底；袋底→袋口只许上半球；两端贴框差定口 |
| P3 重建权威 | `allowed` 须袋融合预算才套入；圆柱/TSDF 不定轴；包络轴只否决 | FULL `allowed=false` → `SKIPPED_QUALITY`；`PREGRASP_ONLY` 不要求 `allowed` |
| P4 预抓取 | 默认 `execute_pregrasp_only=true`；停预抓取（入口=拟合袋底，沿 −axis 后撤 30 mm）；无 SetIO；ACK 后再 Survey | 残差未过门也 Hold；方向/定位以现场目视为准 |
| P5 套入干跑 | `grasp=true tool=false`；套入与撤退先过 `PlanSleeve` | 软件路径已接线 |
| P6 刀具 | `ToolActuator`：SetIO ACK ≠ `cut_confirmed` | 已实现；真刀未接 |
| P7 成功语义 | `harvest.grasped` 仅 `cut_confirmed && retreat_confirmed` | 软件已钉死 |
| P8 导航适配 | 导航包已归档；四 IDL 名在 manifest `reserved_interfaces`；`_cmd_navigate` 直通 `NAV_OK` | 固定座现行 |
| P9 发现开窗 | 袋口开批须 `termination_reason=survey_failed` 不 Begin | `photo_pose_reached` 先于 `round_locked` |

### PREGRASP_ONLY 判定口径

档位：两边 execution、`grasp.enabled=true`、`tool.enabled=false`、`execute_pregrasp_only=true`。到预抓取后现场评：筒口是否对袋轴、侧向偏多少、剪切紫点是否落袋口。**这些对错只在现场评**；残差 2°/3mm 未过也停住。终局须 `SUCCEEDED` 且 `recovery_required`；结束后停在预抓取不回 stow；命令 6 ACK 后才允许再 Survey。

### 节拍基线

每果周期：`粗扫帧 → 选果 → 观察循环[视点×(移臂+停稳+等帧+积分)] → 接触[staging PTP+轴向 LIN+套入+剪切+撤退] → 后勤[回位+记账+放果]`。现状：观察 6 视 33.5 s；接触干跑目标 45–60 s/果。fast 档方向目标（不是门）：观察 ≤2 视 ≤12 s、单果 ≤20 s。对标谱系：人 3–6 s/果；机器人 2.78–24 s/果带；大田成功率 51–88%。策略：速度不够时长凑（可靠性优先）；跳过是调度参数不是失败。

## 10. 回放塔（解析回归门；改接近/融合/护栏/包络必跑）

`colcon test --packages-select peach_system_tests` 的 `test_replay_approach`：零 ROS 纯几何，确定性语料三层（现场真袋 14 例（`src/peach_arm/test/fixtures/field_pregrasp_cases.yaml`）；分层 200 例 seed 20260911；随机 100 例 seed 20260910）对照 `test/replay_baselines.json` 冻结基线（2026-09-23 v4 三路点重封；基线逐数权威是 json 自身 provenance/revised 链）。判定：`analytic_ok` 只许升不许降；`lin_chord_fail` 双侧容差 1；分母精确断言，采样器/案册漂移须显式重封。护栏数学 `test/replay_oracle.py`（v4 公式）。旧 bag 回放兼容走 `peach_observability/bag_reader.py` 的 LEGACY 别名。

## 11. 标定（手眼外参 + 彩色内参）

日常采摘不跑标定，但必须有 `src/aubo_hand_eye_calibration/hand_eye/active.yaml`（入库随仓；改值后重启 extrinsics_publisher 生效）。平移应接近 `[0.045, 0.108, 0.002]`。

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
# 浏览器只开回环 http://127.0.0.1:8088
ros2 service call /hand_eye_extrinsics_publisher/reload std_srvs/srv/Trigger {}
```

auto 档（自动视点+内外参联合求解）：Web 台「标定流程」卡选 位姿来源=auto、求解目标=joint；或 CLI `ros2 action send_goal /hand_eye_calibration_server/run aubo_msgs/action/RunHandEyeCalibration "{pose_source: auto, solve_target: joint, return_to_start: true}"`。流程：当前帧定位板 → FOV 掩码环绕视点（极角 0–45°、方位 60° 步进）→ 贪心选旋转跨度 ≥30° 队列 → plan-only 预检（有效 <12 拒跑）→ settle 后逐帧同步采集 → 联合求解（五方法初值+20 参数 Huber 抛光）→ 落 `hand_eye/candidates/`。验收门：joint 总重投影 RMS ≤0.8px；新旧外参差 mm 级。

彩色内参（棋盘 11x8 @20mm，先起相机前端，需 X 显示）：

```bash
ros2 launch aubo_hand_eye_calibration intrinsics_calibration.launch.py
ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz
colcon build --packages-select percipio_camera   # launch 读 install 副本，必须重建
ros2 topic echo /camera/color/camera_info --once # 核对 K 与文件一致
```

内参变更后手动更新 `peach_harvester/config/scene_perception.yaml` 的 `calibration_version`；感知/重建按新 K 重验。标定器是 vendored image_pipeline jazzy 包（`src/camera_calibration`）。

## 12. peach_sim 仿真（非验收门）

**A. Blender 重建 + 感知验证矩阵（离线门）**：`src/peach_sim/reconstruction/` 三道离线门 + 一道回归门：几何门（`validate_geometry.py`）、三锚定机位感知门（`validate_perception.py`）、感知验证矩阵门（`run_matrix.py` → `evaluate_matrix.py --gate` 对冻结基线）。GPU 渲染作业串行（双 Blender 并发抢 3090 OOM）；矩阵帧与 EXR 不入 git。命令与分层口径见 `peach_sim` README。

**A→gz 查看桥（2026-10-09 起，纯视觉）**：重建去草整园经 `blender/export_field_to_gz.py` 无头导出单 GLB（材质直连/摊平 + GN 叶 Realize + join 变换烘焙；约 220 MB），`GZ_SIM_RESOURCE_PATH=src:src/peach_sim gz sim -r src/peach_sim/worlds/field_no_grass.sdf` 人工查看。该世界无碰撞体/传感器/桥/TF，**不是**导航或感知 GT（仍以 `worlds/peach_orchard.sdf` 为准）；Blender 走 `_tools/` 自带 Python（勿混 aubo_py3.12/numpy）。看完关窗并 `pgrep -af 'gz sim'` 清残留（决策 0039）。

**B. 场景层（Gazebo Harmonic / gz-sim 8）**：改 `src/peach_sim/config/orchard.yaml` 后先重生成再起场景：

```bash
pgrep -af 'ros2 launch|gz sim|robot_state_publisher|parameter_bridge'
ros2 run peach_sim generate_orchard       # --check 只校验；同 seed 必同产物
ros2 run peach_sim scene_preview          # 无 GL 环境看正交预览
ros2 launch peach_sim orchard_sim.launch.py    # gui:=false 无头
pgrep -af 'ros2 launch|gz sim|robot_state_publisher|parameter_bridge'   # 测完必须空
```

袋具几何受工具余量与现场包络双重约束（`peach_sim/test/` 锚数据集）；入库 SDF 与生成器输出逐字节对账。默认档工位锚作业位、六关节钉拍照位；`gz_ros2_control` 未接（须授权动只读 xacro）。

**C. Nav2 巡园（2026-10-08 起）**：`orchard_sim.launch.py mobile:=true` 解锚换 gz DiffDrive；`orchard_nav.launch.py`（`slam:=True` 在线建图，否则 AMCL+确定性图）；`ros2 run peach_sim orchard_tour` 逐站 `NavigateToPose`。巡园只动底盘不动臂；不进 `harvest_system`。

## 13. IMU 与 imu_follow；RViz；IVG

**USB IMU（随整栈，不进 lifecycle）**：`imu_enabled` 默认 true；`tf_parent_frame:=tcp` 对齐；无 USB 每 2s 重试不挡栈。`usermod -aG dialout` 后必须 `newgrp`。画面在 MoveIt RViz **Peach → Imu**；静置 `linear_acceleration.z` 为正（~+9.6）；健康看 `/diagnostics`。不要另起 `serial_imu.launch.py` 与整栈并行。

**imu_follow（IMU 姿态跟随；仅 adaptive_shear_v1 随栈 Include）**：`motion.enabled` 默认 false 只算不发。servo 三坑：twist 输入必须 BEST_EFFORT；enable 自动 `switch_command_type(TWIST)`；参数名自带 `moveit_servo.` 前缀。`/imu/data` 勿与假发布器混流。插入/回退（决策 0026）：`enable → insert_start → 剪切 → insert_retract → disable`；`insert.max_travel_m` 部署 0.09 m；断流即停推进。shear/bite 两档走 MTC LIN 永不调这些服务；禁止与 MTC 同时写控制器。

**RViz（`aubo_e5_moveit_config/rviz/moveit.rviz`）**：Fixed Frame 用 `base_link`。现场优先 Debug Image、Detection Cloud、Perception Markers；跳过重建时不开 TSDF；TCP 只看 TCP Trajectory。灰框=未满 confirm_frames；Imu 轴 6 cm 不写 TF。

**IVG 演示栈（独立于采摘；默认独立 DDS 域 96）**：四包 `ivg_demo_services`/`latte_backend`/`tool_changer`/`aubo_ros2_web_dashboard`（Web 8095）。mock 冒烟：

```bash
colcon build --packages-select ivg_interfaces ivg_demo_services latte_backend tool_changer aubo_ros2_web_dashboard aubo_e5_moveit_config
colcon test  --packages-select ivg_demo_services latte_backend tool_changer aubo_ros2_web_dashboard
# 栈启动原 scripts/start_ivg_demo.sh（2026-10-10 退役；git 历史可取，内含 trap 清理+pgrep 复核）
ros2 service call /get_current_state ivg_interfaces/srv/GetCurrentState "{}"
```

与 peach 隔离（用户裁定）：勿与 peach 同域并行。真机：操作员起本栈=授权。

## 14. 停栈与清进程（MUST，与启动前 pgrep 成对）

launch 终端 Ctrl+C；一次性命令 `timeout` 包裹（`ros2 bag record -d N` 自停不可靠，用 `timeout -s INT -k 10`）。结束后复核：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
```

有残留按 PID `kill -TERM`，2 秒仍存活 `kill -9`，再复核。不要宽泛 `pkill`。别人的残留先报告再清理；清后复验数据流（30 帧探针或 `ros2 topic echo --once`）。
