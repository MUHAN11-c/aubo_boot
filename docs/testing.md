# 测试

权威：源码 + 实机过程数据。与 [architecture.md](architecture.md)、[io.md](io.md) 构成仅有的三份活文档；**源码与本文互相更新，改启动/验收口径或改本文须同一轮改另一边**。约束：[AGENTS.md](../AGENTS.md)。

各包 `test/` **只保留 ROS 2 默认 lint**（Python：`test_flake8.py` / `test_pep257.py`；CMake：`ament_lint_auto`）。不要写业务用例、gtest、DDS 假现场或 launch_testing。语法与流程由审查核对，对错以实机为准。`colcon test` 不等于采摘验收。

不要删 `_archive/runs/` 与现场 `runs/`。未授权不得真机运动或 SetIO。launch **不自动** `RunHarvest`。十四包职责见 [architecture.md](architecture.md) §3。

---

## 1. 构建与开发机

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

有残留按 PID 补杀。Python：`aubo_py3.12`；numpy **1.26.4**。禁止 pip 装 opencv-python / scipy。

```bash
# 开发机：无相机、不运动
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false

# 真机（须显式 real；示教器上电；bringup 不起 aubo_dashboard）
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。参数 `peach_task_executor/config/observability.yaml`。`/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor` / `robot` / `metrics` / `record` / `params` / `job`。首屏作业票须能看出当前果实停在哪一环、抓取档是否关闭、`GraspDecision.allowed` 与 base_link 坐标。默认不上电、不派发运动、不打工具 IO、不自动开批。

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
| `navigation_enabled` | false | true 时开批先 `NavigateToWorksite`（现行 stub） |

完整列表：`--show-args`。只起手臂：`ros2 launch aubo_e5_bringup bringup.launch.py …`。

```bash
ros2 action send_goal /peach_task_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'dev', scene_key: 'lab', profile_id: 'default'}"
ros2 topic echo /peach_task_executor/state
ros2 service call /peach_task_executor/control peach_interfaces/srv/ControlTask \
  "{command: 0, expected_state_seq: 0}"
```

打开真运动须同时改调度 `execution_enabled` 与技能 `execution.enabled`，并经人工授权。

---

## 2. 真机干跑（默认不运动、不 SetIO）

`auto_power_on` 必须为 false。柜侧用示教器；规划/FK/IK 用 MoveIt；停轨走透传取消 + 硬件 `RobotMoveStop`。禁止调用 `aubo_dashboard`。

过程数据：新记录在工作区 `runs/`。08-20～08-24 在 `_archive/runs/root_2026-08-24/`。每次干跑把结论写进 `runs/field_test_<日期>/log.md`。

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
ros2 topic echo --once /peach_task_executor/state
ros2 topic echo --once /peach/perception/target_observations
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/state
ros2 lifecycle get /peach_observability
ros2 lifecycle get /peach_scene_perception_node
ros2 lifecycle get /peach_target_reconstruction_node
ros2 lifecycle get /peach_manipulation_skills_node
ros2 lifecycle get /peach_navigation_node
ros2 lifecycle get /peach_task_executor
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

关节名必须是：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。五节点须 Active（含导航 stub）。重建有时停在 inactive：`ros2 lifecycle set /peach_target_reconstruction_node activate`。默认 `navigation_enabled=false`，开批不发 `NavigateToWorksite`。

显式只扫（仍不运动）：

```bash
ros2 action send_goal /peach_task_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
```

`intent: 2` = SURVEY_ONLY。技能 `execution.enabled=false` 时 Survey 只规划拍照位。默认 intent 0 且调度 `execution_enabled` 关时，会对第一个确认目标记 `SKIPPED_QUALITY` 后结束。

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

须同时打开调度与技能 `execution`。`RunHarvest` / `ExecuteTarget` / `SurveyScene` 动作入口会自动 arm；手动 Trigger 才 `~/set_execution_armed`。抓取再开 `grasp.enabled`；工具再开 `tool.enabled`。卸果须标定 `deposit_pose_named_target`（M8）。使能顺序：先关 `grasp` 再关 `execution`（依赖链 execution→grasp→tool）。

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
| Planned Views | 开 | `/peach_manipulation_skills_node/planned_views` | 候选拍照位；`execution.enabled=false` 时仍会出，不代表已走到 |
| Camera Color | 关 | `/camera/color/image_raw` | 原彩图 |
| Debug Image | 开 | `/peach/perception/debug_image` | 检/分割叠加。灰框=未满 confirm_frames |

---

## 4. 验收门（产品）

### M1 单果观察（不接触、不开工具）

档位：两边 `execution=true`，`grasp.enabled=false`，`tool.enabled=false`。确认 `motion_possible=1` `e_stop=0`。同一可见目标连续 3 次 PICK_ALL 或 OBSERVE_ONLY。每次须：`captured_views >= 4`；TSDF 点数 > 0，refit `ok=True`；`ExecuteTarget.outcome=SUCCEEDED` 且 Build 成功；失败时 ledger 有 `failure_code`（不得计数器全 0 且无告警）。

### 单目标完整抓取（运动、接触，不开工具 IO）

档位：`execution=true`、`grasp.enabled=true`、`tool.enabled=false`。禁止为提速放宽质量门。

- Build 接收后 2 s 内反馈 COLLECTING/READY；未绑定时臂不得环绕。
- 观察预算不超过 15 s：拍照位 + 当前位采帧；基线未过最多两次短 PTP（超 8 s / 2.5 rad 视为绕行拒发）。到位后等新机位再判覆盖，同机位连帧不算。
- `captured_views >= 4`，基线/深度/RMSE/内点率过门。
- 无精化不得宣称方向准确。
- MTC 接近、直线插入、同轴撤离均须 goal-hold。接触沿检测轴短程 LIN（预抓取后撤 0.10 m）；未对轴才 PTP 到预抓取点。接近护栏 **20 s / 累计 10 rad / 单轴 3.2 rad**（0.10 速度）只拦绕行，不把到预抓取的正常 PTP 拒掉。
- 日志不得出现 SetIO。
- 单目标目标 45–60 s；失败必须有 `failure_code`，不得停在 `RUNNING + action_active=false`。

**2026-08-21 / 08-24：本节未通过。** 08-25 `field_full_20260825_1851`：`target_1` 接近/插入/撤离均 goal-hold，工具关、无 SetIO。`target_0` 因观察在同机位连帧上提前收口未接触。实验室两果均应走到接触干跑。方向准不准以现场目视为准。带工具采摘未做。此前同日许可后轨迹被过紧护栏拒掉，臂未走出接触。

树干/粗枝进 PlanningScene 是预留。无人工确认的无粗枝通道时不接触。

---

## 5. 量化基线（归档实测，不是 launch 请求）

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
| 轴门 35° | ~11°/~26.7° 过；~49°/~88° 拦 | 不得为提速放过双表面 |
| TF | 接触轮 `tf_failures=0` | tf_failures 上升须停 |
| 新鲜度 | 08-24 观察失败 5/10，主因 `selected_target_stale` | 未测得 EMA 时门限 3.0 s；测得后只放宽 |
| MTC | 接近 25–29 s 撞 **12 s** 接触护栏；笛卡尔 0.967；`(0/1)` | 超护栏保持 `skipped_unreachable` |
| 会话 | 批次结束后 events 再写 ~65 min | 结束后不再追加空观测 |
| 工具 | 全程 `tool.enabled=false` | 日志无 SetIO |

数据根：`_archive/runs/root_2026-08-24/`。工作区 `runs/` 可缺失。

解读陷阱：批次 `summary.md` 重建终值常是 IDLE / `captured_views: 0`——用逐目标 `captured_views_max`。GraspDecision 心跳 `not_ready` 占多数 ≠ 精化从未 ACCEPT。感知单帧 ACCEPT 远少于 REOBSERVE，不能当批次成功率。

---

## 6. 现场记录（工具 IO 关，§4 接触验收未通过）

过程结论：`_archive/runs/root_2026-08-24/web_runs/field_test_20260821/log.md`；综述 `reports/process-data-analysis-2026-08-24.md`。

### 08-21 已落地

整栈 Active，相机 ~2.4 fps，手眼 TF。Survey → 并行 Build+OBSERVE → FULL(`skip_observation`)。静止采帧。35° 轴门拦住 ~49°。opt3：`target_0` 4 视角轴 ~11° `allowed=true`，再确认「未获得新鲜观测」但跟踪 OBSERVED → `skipped_quality`；`target_1` 5 视角轴 ~49°，MTC 预计 29 s > 12 s → `skipped_unreachable`。无 SetIO。

### 08-24

再确认改为 OBSERVED 用当前锚点 + `updated_s`。`grasp`：`target_1` 插入 0.967，当时 `min_fraction=1.0` → skip。`grasp4`：两目标 4 视角，接近预计 25 s / 27 s > 12 s → `skipped_unreachable`。未下发接触轨迹。

当日后源码已改为：当前位采帧 + 最多一次 12° 短 PTP；接触沿检测轴短程 LIN（未对轴则 PTP 到预抓取点），禁止接触段 OMPL；`min_views=2`；`min_fraction=0.95`；拍照位过 `transit_max_*`；不预填 EMA。**未再宣称抓取成功。**

### 08-25

开抓取关工具。许可后 9 s 与 12.6 s 的正常接近曾被 12 s / 4–8 rad 当成绕行拒掉。护栏改为 **20 s / 10 rad / 单轴 3.2**（仍拒 40 s 爬行与 4.5 rad 绕腕）后，18:53 `field_full_20260825_1851`：`target_1` 接近/插入/撤离均 goal-hold（12.4 s / 3.0 s / 4.2 s），工具关、无 SetIO；`target_0` 仍 `insufficient_angular_baseline`。方向准不准以现场目视为准。带工具采摘未做。
