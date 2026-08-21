# 真机测试怎么做

对象是套袋桃。对错以实机和过程数据为准。链路逻辑与源码阅读顺序见 [flow.md](flow.md)。本页只写**默认不运动、不 SetIO**的干跑。真接触须另开使能并经人工授权。

bringup **不起** `aubo_dashboard`，禁止调用该包。柜侧用示教器 / 控制柜，规划/FK/IK 用 MoveIt，停轨走透传取消 + 硬件 `RobotMoveStop`。`auto_power_on` 必须为 false。

## 0. 准备

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

有残留按 PID 补杀。本机网口须能 ping 控制器（默认 `169.254.10.98`）。Python 用 `aubo_py3.12`，numpy **1.26.4**。

过程数据：启动时的 CWD 下 `web_runs/`（观测 JSONL/图）、`harvest_runs/`（批次账本）。不要删 `_archive/runs/` 或现场 `web_runs/`。每次干跑把结论写进 `web_runs/field_test_<日期>/log.md`。

## 1. 档位（本次默认档）

| 项 | 值 | 含义 |
|----|----|------|
| `hardware_mode` | real | 真机驱动 |
| `camera_enabled` | true | Percipio |
| 执行器 `execution_enabled` | false | 不派 FULL 接触 |
| 技能 `execution.enabled` / `grasp.enabled` / `tool.enabled` | false | 只规划、不运动、不 IO |
| launch | 不自动 `RunHarvest` | 须显式发目标 |

## 2. 启动整栈

在工作区根目录：

```bash
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。可选 `record_mcap:=true`。

## 3. 冒烟（不运动）

另开终端，source 同上。

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /aubo_io_controller/robot_status
ros2 topic hz /camera/color/image_raw
ros2 topic echo --once /peach_task_executor/state
ros2 topic echo --once /peach/perception/target_observations
```

关节名须为：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。

Lifecycle 应为 Active：`peach_scene_perception_node`、`peach_target_reconstruction_node`、`peach_manipulation_skills_node`、`peach_task_executor`。

## RViz（`aubo_e5_moveit_config/rviz/moveit.rviz`）

整栈带的 RViz 里有分组 **Peach**。Displays 与 Views 的 Fixed Frame 都用 **`base_link`**，不要用未接上的 `world`（MoveIt 规划场景会报 `world`↔`camera_link` 断树，与 Peach 显示无关）。

改完本文件后须重装 `aubo_e5_moveit_config`（改了感知参数时还要重装 `peach_scene_perception`）并重启 RViz。

### Peach 分组（含义）

3D 项跟 `base_link`。图像项在 Displays 下方另开 Image 面板。Marker 可在该项 Namespaces 里按命名空间单独开关。

| 显示名 | 默认 | 话题 | 含义 |
|--------|------|------|------|
| Perception Markers | 开 | `/peach/perception/markers` | 当前帧每个锁定目标的 3D 几何（ns `peach_pose`）。绿=ACCEPT、黄=REOBSERVE、红=REJECT。线=袋底→袋颈；箭头=入袋行程；半透明圆柱=刀具内径×行程；半透明球=果半径（仅 fruit）；红/绿/蓝小箭头=抓取架 XYZ（5 cm）；文字=稳定 `target_id`。 |
| Detection Cloud | 关 | `/peach/perception/single_cloud` | 检测框内深度反投影的彩色点云（步长 `detection_cloud_stride`，默认每 2 像素取 1）。用来对 TF/深度，不是重建结果。 |
| Camera Points | 关 | `/camera/depth_registered/points` | Percipio 整幅配准深度点云，相机光学系。很密，现场默认关。 |
| TSDF Cloud | 开 | `/peach/reconstruction/tsdf_cloud` | 绑定目标的 TSDF 表面点（米、RGB，Transient Local）。多视角积分后的果/袋外形。 |
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 各已采视角点云拼在一起（未融成体）。不是作业用模型，默认关。 |
| Reconstruction Markers | 开 | `/peach/reconstruction/markers` | 重建相机轨迹与精化抓取示意（Transient Local）。绿点球/黄线/青箭头=已采相机位与光轴。ns `peach_reconstruction/refined` 与 Perception Markers 同款：袋轴、入袋行程、圆柱、果球（fruit）、入口 RGB 架、文字（`live` 随积分更新，`final` 为 finalize 定稿）。半透明青网格（ns `peach_reconstruction/tsdf_mesh`）=TSDF 三角面。换绑/reset 仍清屏。 |
| Planned Views | 开 | `/peach_manipulation_skills_node/planned_views` | 技能规划的候选拍照位（ns `candidate_views`，最多 24）。箭头从规划相机位指向目标中心；越绿分数越高、越红越低。`execution.enabled=false` 时仍会在 Survey/Observe 规划后出现，不代表已经走到该位。 |
| Camera Color | 关 | `/camera/color/image_raw` | 相机原彩图，无叠加。 |
| Debug Image | 开 | `/peach/perception/debug_image` | 感知叠加图。绿/橙框=已确认目标；灰框=未满 `confirm_frames` 的闪现（不进锁定/3D Marker）。紫色横线+空心圆=TCP 行程终点，同时也是物理刀刃剪切位置（袋颈前保留 `tool.margin_neck`）。文字=`target_id` + YOLO 置信度。`untracked_*` 无 3D/无 TF，不计入。 |

现场干跑优先看 **Debug Image**（2D 检/分割）和 **Perception Markers**（3D 是否落在 `base_link` 正确位置）。重建开始后看 **TSDF Cloud** 与 **Reconstruction Markers**。不要同时开 Camera Points 和 Detection Cloud，RViz 会卡。

## 4. 显式开批（仍不运动）

```bash
ros2 action send_goal /peach_task_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
```

`intent: 2` = `SURVEY_ONLY`。技能端 `execution.enabled=false` 时 `SurveyScene` 只规划拍照位姿、不执行。默认 intent 0 且 `execution_enabled=false` 时会对第一个确认目标记 `SKIPPED_QUALITY` 后结束。

看 `/peach_task_executor/events` 与 Web 事件时间线。账本：`harvest_runs/<request_id>/ledger.json`。

## 5. 停栈

launch 终端 Ctrl+C。再 `pgrep` 确认无残留。

## 现场命令备忘（随测更新）

另开终端都先 `source /opt/ros/jazzy/setup.bash`、`source install/setup.bash`。过程结论写进 `web_runs/field_test_<日期>/log.md`，不要只改本页。

```bash
# 残留
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'

# 真机整栈（工作区根目录，不自动开批）
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98

# 冒烟
ros2 topic echo --once /joint_states
ros2 topic echo --once /aubo_io_controller/robot_status
ros2 topic hz /camera/color/image_raw
ros2 topic echo --once /peach_task_executor/state
ros2 topic echo --once /peach/perception/target_observations
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/api/state

# 生命周期（重建有时 launch 应答超时停在 inactive）
ros2 lifecycle get /peach_scene_perception_node
ros2 lifecycle get /peach_target_reconstruction_node
ros2 lifecycle get /peach_manipulation_skills_node
ros2 lifecycle get /peach_task_executor
ros2 lifecycle set /peach_target_reconstruction_node activate

# 手眼
timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link

# 干跑只扫 / 完整意图（默认仍不运动，除非已开 execution）
ros2 action send_goal /peach_task_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_dry', scene_key: 'lab', profile_id: 'default', intent: 2}"
# intent 0 = PICK_ALL（须另开使能才真走臂）

# 授权后真运动（抓取/工具默认仍关）
# 动作入口（ExecuteTarget / SurveyScene）在 execution.enabled=true 时自动一次性 arm；
# 手动 ~/start_cycle 与 ~/go_to_photo_pose 仍须人工 arm。
ros2 param set /peach_task_executor execution_enabled true
ros2 param set /peach_manipulation_skills_node execution.enabled true
# 仅手动 Trigger 需要：
# ros2 service call /peach_manipulation_skills_node/set_execution_armed std_srvs/srv/SetBool "{data: true}"
ros2 service call /peach_manipulation_skills_node/go_to_photo_pose std_srvs/srv/Trigger "{}"
```

监控：`http://127.0.0.1:8090`。Debug Image 话题：`/peach/perception/debug_image`。账本：`harvest_runs/<request_id>/ledger.json`。

## 6. 以后若要真运动（须另授权）

同时打开执行器 `execution_enabled` 与技能 `execution.enabled`。`RunHarvest` / `ExecuteTarget` / `SurveyScene` 动作入口会自动 arm；手动 Trigger 才调用 `~/set_execution_armed`。抓取再开 `grasp.enabled` / `tool.enabled`。卸果须现场标定 `deposit_pose_named_target`（M8）。本页不写使能步骤的默认值。

## 7. M1 单果观察验收（不接触、不开工具）

档位：`execution_enabled=true`，技能 `execution.enabled=true`，`grasp.enabled=false`，`tool.enabled=false`。确认 `motion_possible=1` `e_stop=0`。

同一可见目标连续 3 次 `RunHarvest` intent=PICK_ALL（或单独 ExecuteTarget OBSERVE_ONLY）。每次须：

- `captured_views >= 4`
- TSDF 点数 > 0，refit `ok=True`
- `ExecuteTarget.outcome=SUCCEEDED` 且 `BuildTargetModel.success=True`
- 失败时 `web_runs/run_*/summary.md` 与 `harvest_runs/<id>/ledger.json` 能看到 `skip_reasons` / `failure_code`（不得再出现计数器全 0 且无告警）

结论写入 `web_runs/field_test_<日期>/log.md`。

## 8. 单目标完整抓取验收（运动、接触，不开工具 IO）

档位：执行器与技能 `execution=true`、`grasp.enabled=true`、`tool.enabled=false`。精度与安全优先于节拍，禁止为提速放宽质量门。

- Build action 接收后须在 2 秒内反馈 `COLLECTING`/`READY`；未确认绑定时机械臂不得开始环绕。
- 正常观察预算不超过 35 秒：初始拍照位 + 3 个有效移动视点，第 4 个只作质量补偿。
- `captured_views >= 4`，基线、深度覆盖、refit RMSE 与内点率全部通过。
- 记录初始与精化后 entry/neck 位移、axis 夹角和 TCP travel 差值；无精化结果不得宣称方向/定位准确。
- MTC 接近、直线插入与同轴撤离均须成功，控制器每段必须回报 goal-hold 成功。
- `tool.enabled=false` 全程保持，日志不得出现 SetIO 调用；工具阶段只允许明确记录“跳过末端 IO”。
- 单目标完整周期目标 45–60 秒；超时须按阶段拆分观察、重建、再确认和接触耗时。
- 失败必须产生明确 `failure_code`，批次可取消并收敛，不得停在 `RUNNING + action_active=false`。

**2026-08-21 结论：本节验收未通过。** 不得宣称接近、直线插入、同轴撤离成功。当日批次见 §10。

## 9. 树干/粗枝避障（下周调试项）

当前 URDF 已包含机械臂与相机碰撞几何，但环境中树干、粗枝尚未进入
MoveIt PlanningScene。本周真机抓取仅允许使用人工确认的无粗枝通道；相机或
机械臂可能接近木质结构时立即取消，不以末端工具“允许接触”替代环境避障。

计划在 2026-08-24 至 2026-08-30 调试：从配准深度点云提取跨帧稳定的树干/
粗枝体素或圆柱，转换到精确时间戳 `base_link`，按保守膨胀半径加入碰撞场景；
仅对末端工具链设置允许碰撞，相机和其余机械臂链保持禁止碰撞。验收须包含
静态场景稳定性、传感器盲区/陈旧数据停止门、规划失败安全收敛及近枝实机低速测试。

## 10. 2026-08-21 阶段性成果（真机，工具 IO 关）

过程结论另有 `web_runs/field_test_20260821/log.md`（目录 gitignore，勿删）。账本在 `harvest_runs/<request_id>/ledger.json`。

### 已通过 / 已落地

| 项 | 证据 |
|----|------|
| 整栈 Lifecycle Active，透传、相机 ~2.4 fps，手眼 TF | 冒烟；`drives_powered=1` `motion_possible=1` `e_stop=0` |
| Survey → 并行 Build + OBSERVE_ONLY → FULL（`skip_observation`） | 编排按设计走两段 `ExecuteTarget` |
| 静止采帧 `capture.require_robot_static=true` | 转移中 `robot_not_static` skip，不再把运动帧积进 TSDF |
| 检测轴 vs 精化轴 35° 门 | `1840_grasp_opt3` 的 `target_1` 约 49° → `perception_reconstruction_axis_mismatch` |
| 新机位静止时采集（不再当 `motion_jump` 丢掉） | 同批 `target_0` **4** 视角、`target_1` **5** 视角 |
| 派观察前等锁定集 | `1835` 起 ID 稳定为 `target_0`/`target_1`，不再观察 1.9 s 秒拒 |
| 账本 `failure_code` 分级 + 观察段耗时合并 | `skipped_quality` / `skipped_unreachable`；`elapsed_s` 含 observe |
| 工具 IO | 全程 `tool.enabled=false`，无 SetIO |

关键源码/参数：`reconstruction.yaml` 静止门；`frame_collector.auto_capture_decision` 新机位采集；重建/技能 `max_axis_angle_deg=35`；再确认精化路径不平移 TSDF；执行器锁定集等待与 OBSERVE 重试。

### 接触轮（均未形成成功抓取）

| 批次 | 目录 | 摘要 |
|------|------|------|
| `field_full_20260821_1743_mtc_guard_final` | `web_runs/run_20260821_174311` | `target_0` 插入 68.97% 接触前拒绝；`target_1` 22 帧轴偏 ~88°。运动中超采。 |
| `field_full_20260821_1827_grasp_opt` | `web_runs/run_20260821_182839` | 技能空锁定集秒拒 `target_0`。 |
| `field_full_20260821_1835_grasp_opt2` | `web_runs/run_20260821_183524` | 新机位 9 cm 被 80 mm `motion_jump` 丢掉，`captured_views=1`。 |
| **`field_full_20260821_1840_grasp_opt3`** | `web_runs/run_20260821_183951` | 观察恢复。见下行。 |

`1840_grasp_opt3`（71.2 s，使能已关回 false）：

- `target_0`：4 视角，轴夹角 ~11°，refit ACCEPT，`grasp_allowed=true`。FULL 再确认 3 窗（自适应约 2.6 s）报「未获得新鲜观测」，跟踪状态却是 OBSERVED → `skipped_quality`。未进 MTC。
- `target_1`：5 视角，轴夹角 ~49°，夹角门生效后降级锚点；MTC 接近预计 29 s > 12 s 短路径门 → `skipped_unreachable`。

### 实测节拍（opt3 账本）

| 目标 | observe | reconfirm | approach | 合计 |
|------|--------:|----------:|---------:|-----:|
| target_0 | 14.7 s | 8.1 s | — | 24.1 s |
| target_1 | 17.3 s | 0.2 s | 0.2 s | 17.9 s |

## 11. 下周继续（2026-08-24 起）

1. **再确认新鲜度（优先）**：锁定目标已是 OBSERVED 时应用当前锚点判漂移，不要只等 `received_s > after_s`。`target_0` 在 opt3 已具备接触几何。
2. **接触验收（§8）**：MTC 接近 + 直线插入 + 同轴撤离均 goal-hold；`tool.enabled` 仍默认 false。
3. **降级抓取**：轴夹角超限后不要用远距离 PTP 硬闯；短路径门拒绝应保持 `skipped_unreachable`。
4. **树干/粗枝进 PlanningScene**（§9），无通道不接触。
5. **M2 效率**（`docs/m2_efficiency.md`）须等 §8 通过后再下调 `min_views`。

使能顺序：先 `grasp.enabled=false` 再关 `execution.enabled`（依赖链 `execution→grasp→tool`）。launch 仍不自动 `RunHarvest`。
