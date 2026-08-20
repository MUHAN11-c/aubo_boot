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
| Local Cloud | 关 | `/peach/reconstruction/local_cloud` | 各已采视角点云拼在一起（未融成体）。比 TSDF 碎、更贴单帧。 |
| Reconstruction Markers | 开 | `/peach/reconstruction/markers` | 重建相机轨迹与精化抓取示意（Transient Local）。绿点球/黄线/青箭头=已采相机位与光轴。ns `peach_reconstruction/refined` 与 Perception Markers 同款：袋轴、入袋行程、圆柱、果球（fruit）、入口 RGB 架、文字（`live` 随积分更新，`final` 为 finalize 定稿）。半透明青网格（ns `peach_reconstruction/tsdf_mesh`）=TSDF 三角面。换绑/reset 仍清屏。 |
| Planned Views | 开 | `/peach_manipulation_skills_node/planned_views` | 技能规划的候选拍照位（ns `candidate_views`，最多 24）。箭头从规划相机位指向目标中心；越绿分数越高、越红越低。`execution.enabled=false` 时仍会在 Survey/Observe 规划后出现，不代表已经走到该位。 |
| Camera Color | 关 | `/camera/color/image_raw` | 相机原彩图，无叠加。 |
| Debug Image | 开 | `/peach/perception/debug_image` | 感知叠加图。绿/橙框=已确认目标；灰框=未满 `confirm_frames` 的闪现（不进锁定/3D Marker）。文字=`target_id` + YOLO 置信度。`untracked_*` 无 3D/无 TF，不计入。 |

现场干跑优先看 **Debug Image**（2D 检/分割）和 **Perception Markers**（3D 是否落在 `base_link` 正确位置）。重建开始后再看 **TSDF Cloud** 与 **Reconstruction Markers**。不要同时开 Camera Points 和 Detection Cloud，RViz 会卡。

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
ros2 param set /peach_task_executor execution_enabled true
ros2 param set /peach_manipulation_skills_node execution.enabled true
ros2 service call /peach_manipulation_skills_node/set_execution_armed std_srvs/srv/SetBool "{data: true}"
ros2 service call /peach_manipulation_skills_node/go_to_photo_pose std_srvs/srv/Trigger "{}"
```

监控：`http://127.0.0.1:8090`。Debug Image 话题：`/peach/perception/debug_image`。账本：`harvest_runs/<request_id>/ledger.json`。

## 6. 以后若要真运动（须另授权）

同时打开执行器 `execution_enabled` 与技能 `execution.enabled`，并调用 `~/set_execution_armed`。抓取再开 `grasp.enabled` / `tool.enabled`。卸果须现场标定 `deposit_pose_named_target`（M8）。本页不写使能步骤的默认值。
