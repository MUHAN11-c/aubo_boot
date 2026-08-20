# 使用方法

逻辑、调用关系、源码阅读顺序：[flow.md](flow.md)。真机干跑：[field_test.md](field_test.md)。

先：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

有残留按 PID 补杀。Python 用 `aubo_py3.12/bin/python`，numpy 保持 1.26.4。

## 构建

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`colcon test` 只跑 ROS 2 默认 lint。行为对错以实机和过程数据为准。

## 采摘整栈

```bash
# 开发机（无相机、不运动）
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=sim camera_enabled:=false

# 真机（须显式 real；示教器上电；bringup 不起 aubo_dashboard）
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。过程记录写在启动时的 CWD 下 `web_runs/`。账本在 `harvest_runs/<request_id>/ledger.json`。真机干跑步骤：[field_test.md](field_test.md)。

常用 launch 参数（完整列表：`--show-args`）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | sim | mock / sim / real |
| `robot_ip` | 169.254.10.98 | 仅 real |
| `camera_enabled` | false | 有相机时设 true |
| `extrinsics_enabled` | true | wrist3 → camera_link 静态 TF |
| `moveit_enabled` | true | move_group + RViz |
| `hand_eye_enabled` | false | 标定流程 |
| `hand_eye_web_enabled` | false | 标定 Web `http://127.0.0.1:8088` |
| `record_mcap` | false | 为 true 时 `ros2 bag record -s mcap` 录关键话题 |

无相机时保持默认 `camera_enabled:=false`。有设备时再打开。

只起手臂：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98
```

## 任务与技能（默认不运动、不自动开批）

```bash
ros2 action send_goal /peach_task_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'dev', scene_key: 'lab', profile_id: 'default'}"
ros2 action send_goal /peach_manipulation_skills_node/survey_scene peach_interfaces/action/SurveyScene \
  "{request_id: 'dev', scene_key: 'lab'}"
ros2 service call /peach_task_executor/control peach_interfaces/srv/ControlTask \
  "{command: 0, expected_state_seq: 0}"
ros2 topic echo /peach_task_executor/state
ros2 topic echo /peach_task_executor/events
```

打开真运动须同时改编排器 `execution_enabled` 与技能端 `execution.enabled` 等，并经人工授权。

## 手臂透传冒烟

关节名必须是权威六轴顺序。sim 下 `set_io` 失败属预期。

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /aubo_io_controller/robot_status
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder
```

## 手眼标定

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
```

浏览器只开回环 `http://127.0.0.1:8088`。

日常采摘不跑标定流程，但必须有激活外参：工作区 `hand_eye/active.yaml`（gitignore，不入库）。没有该文件时 `extrinsics_publisher` 会发名义 TF（`wrist3_Link→camera_link` 平移 2 cm、单位四元数），光学系相对法兰会偏约 10 cm 并转错方向。现场副本在 `_archive/runs/hand_eye/`。拷回后若节点已在跑：

```bash
ros2 service call /hand_eye_extrinsics_publisher/reload std_srvs/srv/Trigger {}
ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

平移应接近 `[0.045, 0.108, 0.002]`，而不是 `[0, 0, 0.020]`。
