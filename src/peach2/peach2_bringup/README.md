# peach2_bringup

Peach v2 整栈入口（方案 §3.3 / §13）：预检 → 相机 → Include `aubo_e5_bringup`（驱动只读，
不复制 RSP）→ peach2 生命周期节点 → `nav2_lifecycle_manager`。本包只有 launch 与零 ROS 纯函数，
不含业务。

**launch 绝不发 `RunBatch`、`SetEnables`、任何运动或 SetIO。** 栈起来后 enables 全 false
（`peach2_task` on_activate 发布），批次永远由操作员发起。

**进度（2026-09-30）：** M1 骨架退出门已过（系统测 33/33；mock PREGRASP）。本 launch 不是生产
`harvest_system`。M0 台架标定与套袋+剪断产品验收未过。见方案
[`docs/peach_v2_重构终版方案.md`](../../../docs/peach_v2_重构终版方案.md) §14 现行进度。

## 公有 API

| 名字 | 说明 |
|------|------|
| `launch/peach2_system.launch.py` | 整栈入口（参数见下） |
| `peach2_bringup.preflight` | `matched_executable` / `domain_from_environ` / `running_stack_processes` / `refusal_message` |
| `peach2_bringup.stack` | `managed_node_names` / `start_stereo` / `aubo_camera_enabled` / `validate` / `as_bool` |

### launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | `mock` | `mock`=mock_components + JTC、臂侧 mock IO、`require_robot_status=false`；`real` 需示教器上电与授权 |
| `robot_ip` | `169.254.10.98` | 仅 real |
| `camera_enabled` | `false` | true 时起相机、`peach2_perception`、`peach2_scene` |
| `camera_frontend` | `stereo` | `stereo`=本 launch 起 `peach_stereo`（在 aubo include 之前）；`percipio`=aubo bringup 内 Percipio |
| `camera_ip` | `169.254.10.110` | stereo 前端 |
| `tool_id` | `adaptive_shear_v1` | 透传 aubo bringup `tool_profile`、臂侧 `tool_id`、任务 `default_tool_id` |
| `moveit_enabled` | `true` | `peach2_manipulation` 需要 move_group |
| `bond_timeout` | `4.0` | lifecycle_manager bond 超时 [s]；0 关 |
| `use_sim_time` | `false` | 全图跟 `/clock`；与 `hardware_mode:=real` 同开时拒启 |
| `runs_dir` | `''` | 任务账本根；空=`$PEACH_RUNS_DIR`，否则 `<cwd>/runs` |

Python 托管节点用 `from bondpy.bondpy import Bond`（Jazzy 顶层不导出 `Bond`）。本机无
`ros-jazzy-bondpy` 时节点 WARN 降级、不发心跳；要把默认 `bond_timeout:=4.0` 用起来需 apt 装上。
系统测默认 `bond_timeout:=0.0`（环境变量 `PEACH2_LT_BOND_TIMEOUT` 可覆盖），不依赖管理器侧 bond。

生命周期名单（顺序即 configure/activate 顺序，拆栈反向）：
`peach2_perception → peach2_target_model → peach2_scene → peach2_manipulation → peach2_task`，
未启动的节点（`camera_enabled:=false` 时 perception / scene）从名单中省略。管理器名
`peach2_lifecycle_manager`，`autostart=true`（只驱动 lifecycle 迁移，不发业务指令）。

每个 Include 都包在 `GroupAction(scoped=True)` 里：Jazzy 的 `IncludeLaunchDescription` 会把
launch_arguments 写进父上下文，后一个兄弟 Include 的 `DeclareLaunchArgument`（`config_file`、
`autostart`、`camera_enabled`）就会沿用泄漏值。

### 预检

启动前扫描 `/proc`，按 argv basename 匹配（C++ 看 argv[0]；Python 看 `…/lib/<pkg>/<exe>`），
只拒绝**同一 `ROS_DOMAIN_ID`** 的栈进程（旧 peach v1 节点、重复 RSP / extrinsics /
move_group / ros2_control_node / lifecycle_manager、残留 peach2 实例）；environ 读不到的进程
按冲突处理。其它域（例如隔离域的 launch_testing）互不阻塞。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
pgrep -af 'ros2 launch|component_container|ros2 run|move_group|ros2_control_node|robot_state_publisher|peach2_|lifecycle_manager'

# mock（默认，无相机）
ros2 launch peach2_bringup peach2_system.launch.py

# real（示教器上电、急停手可及、工作空间无人；仍然 enables 全 false）
ros2 launch peach2_bringup peach2_system.launch.py \
  hardware_mode:=real robot_ip:=169.254.10.98 camera_enabled:=true camera_frontend:=stereo
```

## 冒烟清单（发任何批次之前）

1. `ros2 control list_controllers`：`joint_state_broadcaster` 与轨迹控制器 active。lifecycle manager
   不等控制器，mock 下轨迹控制器可能比 peach2 节点 Active 晚几秒，过早发批次会 `EXEC_FAILED:execute_error:-4`
2. `ros2 topic echo /joint_states --once`：六关节名与冻结关节序一致
3. `ros2 lifecycle get /peach2_task`、`/peach2_manipulation`、`/peach2_target_model`（相机开时另加
   `/peach2_perception`、`/peach2_scene`）均 `active [3]`
4. `ros2 topic echo /peach/enables --once`：`execution/grasp/tool` 全 false
5. `ros2 topic echo /peach/end_effector/tool_state --once` 有值
6. `ros2 action list` 含 `/peach/task/run_batch`、`/peach/manipulation/harvest_target`、
   `/peach/manipulation/move_to`、`/peach/target_model/observe`
7. real 另查：相机 hz、`/aubo_io_controller/robot_status`、示教器状态。未授权到此为止。

## 停栈与清理（MUST）

launch 终端 Ctrl+C，然后复核：

```bash
pgrep -af 'ros2 launch|component_container|ros2 run|move_group|ros2_control_node|robot_state_publisher|peach2_|lifecycle_manager'
# 仍有本栈残留：按 PID 清，不要宽泛 pkill
kill -TERM <pid>; sleep 2; kill -0 <pid> 2>/dev/null && kill -9 <pid>
```

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
timeout 600 colcon build --base-paths src/peach2 --packages-select peach2_bringup \
  --packages-skip peach2_interfaces peach2_core \
  --build-base build/v2/peach2_bringup --install-base build/v2/peach2_bringup_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
timeout 600 colcon test --base-paths src/peach2 --packages-select peach2_bringup \
  --build-base build/v2/peach2_bringup --install-base build/v2/peach2_bringup_install
colcon test-result --test-result-base build/v2/peach2_bringup --verbose
```

纯 pytest（`test_preflight.py`）用假 `/proc` 目录，不 `rclpy.init`。整栈集成测见
`peach2_system_tests`。文档：本 README；许可 BSD-3-Clause。
