# 使用方法

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

`colcon test` 只做 lint 和少量无 ROS 图的逻辑/参数检查。行为对错以实机和 `_archive/runs/`、`web_runs/` 为准。

## 采摘整栈

```bash
# 开发机（无相机、不运动）
ros2 launch peach_harvest_orchestrator harvest_system.launch.py \
  hardware_mode:=sim camera_enabled:=false

# 真机（默认 hardware_mode=real；先手动上电，禁止 /aubo_dashboard/startup）
ros2 launch peach_harvest_orchestrator harvest_system.launch.py \
  hardware_mode:=real robot_ip:=169.254.10.98
```

监控：`http://127.0.0.1:8090`。过程记录写在启动时的 CWD 下 `web_runs/`。

常用 launch 参数（完整列表：`--show-args`）：

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | real | mock / sim / real |
| `robot_ip` | 169.254.10.98 | 仅 real |
| `camera_enabled` | true | 无设备时设 false |
| `extrinsics_enabled` | true | wrist3 → camera_link 静态 TF |
| `moveit_enabled` | true | move_group + RViz |
| `hand_eye_enabled` | false | 标定流程 |
| `hand_eye_web_enabled` | false | 标定 Web `http://127.0.0.1:8088` |

无相机时务必 `camera_enabled:=false`，否则 Percipio 刷屏。

只起手臂：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=sim camera_enabled:=false
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=169.254.10.98
```

## 编排与技能（默认不运动）

```bash
ros2 action send_goal /peach_harvest_orchestrator/run_harvest peach_harvest_msgs/action/RunHarvest "{}"
ros2 service call /peach_harvest_orchestrator/control peach_harvest_msgs/srv/ControlHarvest \
  "{command: 0, expected_revision: 0}"   # 0=PAUSE 1=RESUME 4=CANCEL_NOW；revision 以 /state 为准
ros2 topic echo /peach_harvest_orchestrator/state
ros2 topic echo /peach_harvest_orchestrator/events
```

打开真运动必须同时改编排器与能力端 yaml（`execution_enabled` / `execution.enabled` 等），并经人工授权。未授权禁止 SetIO。

## 手臂透传冒烟

关节名必须是权威六轴顺序。sim 下 `set_io` 失败属预期。

```bash
ros2 topic echo --once /joint_states
ros2 topic echo --once /aubo_io_controller/robot_status
aubo_py3.12/bin/python tools/passthrough_traj_client.py wave_shoulder
```

`tools/motion_analyzer.py` 出图默认写 `test_results/`（已 gitignore）。历史分析数据在 `_archive/runs/`。

## 手眼标定

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real \
  robot_ip:=169.254.10.98 hand_eye_enabled:=true hand_eye_web_enabled:=true
```

浏览器只开回环 `http://127.0.0.1:8088`。

## 排障要点

| 现象 | 处理 |
|------|------|
| 改动像没生效 / 相机 -1014 | 旧进程占用设备；先 pgrep 再重启 |
| venv 缺 rcl_interfaces | 先 source `/opt/ros/jazzy` |
| cv2 / numpy 冲突 | `numpy==1.26.4`，不要 pip 装 opencv-python |
| 静态 TF 改了没变 | 重启 extrinsics_publisher |
| sim 里 set_io 失败 | 预期 |
