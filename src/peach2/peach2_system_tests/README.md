# peach2_system_tests

Peach v2 的隔离域 `launch_testing` 集成测：每个测试文件自己用
`peach2_bringup/peach2_system.launch.py`（`hardware_mode:=mock camera_enabled:=false`）起一整栈，
测完由 launch_testing 拆栈。只动 mock 关节（`mock_components` + JTC），**绝不连真机**。launch
本身不发任何批次/使能/运动/SetIO；所有 goal 都由测试进程显式发出。

## 测试

| 文件 | 域 | 内容 |
|------|----|------|
| `test_bringup_mock.py` | 95 | task / manipulation / target_model 均 Active（perception / scene 不起）；lifecycle manager `is_active`；`/peach/enables` 默认全 false；`tool_state` 有发布；全部 action / service server 就绪；launch 没触发 BeginScene / 快照 / SetIO、批次 IDLE；C++ 节点 bond 心跳；post_shutdown 退出码 |
| `test_plan_only_chain.py` | 96 | 测试进程发合成观测（两袋、已锁定、双视点、`swing_known`），等模型收敛与 `approach_allowed`；enables 全 false 发 `RunBatch(PREGRASP_ONLY)`：goal REJECT、`BatchState.message` 含 `execution_disabled`、关节不动、SetIO 0 次；直接发 `HarvestTarget(PREGRASP_ONLY)`：`plan_only=true`、`SKIPPED/NONE/planned`、`reached=NONE`、关节不动 |
| `test_pregrasp_mock_exec.py` | 97 | `SetEnables(execution=true, grasp=false, tool=false)` → `RunBatch(PREGRASP_ONLY)`：mock 关节真实运动、TCP 到达 `GetDecision.pregrasp_tcp`（<2 cm）、`reached=PREGRASP`；批次停在 `WAITING_ACK` 且不 ACK 不结束；`/peach/task/acknowledge_recovery` 后批次 `COMPLETED`、结果 `SUCCEEDED`；无目标批次（重勘，已在拍照位的 MoveTo 空运动成功）；SetIO 0 次 |

`test/peach2_harness.py` 是公共夹具（测试进程内节点，后台 `MultiThreadedExecutor`）：

- 代替 `camera_enabled:=false` 不起的节点：`/peach/scene/build_snapshot` 桩（success）、
  `/peach/perception/begin_scene` 桩（首次 epoch 1，之后每次 +1）、
  5 Hz `/peach/perception/observations`（reliable / volatile / 10）
- `/aubo_io_controller/set_io` 间谍服务：只计数、回 `success=false`；任一测试见到调用即失败
- 合成袋几何取 v1 网格夹具的可达案例（`typical_1757` / `right_lane_tilt`），袋长 0.07 m
  （`adaptive_shear_v1` 的 `L_insert=0.090`）
- `xfail(test, reason, check)`：已知上游失败 → xunit 记 skipped（`xfail: …`）；若意外通过 → 失败并提示删掉 xfail。
  `unittest.expectedFailure` 被 launch_testing 重绑测试方法后丢失，不能用。当前无 xfail。

测试栈用 `bond_timeout:=0.0`（环境变量 `PEACH2_LT_BOND_TIMEOUT` 可覆盖成 launch 默认 `4.0`）：
不依赖本机是否装了 `ros-jazzy-bondpy`。C++ 节点侧 `/bond` 心跳硬断言（test_5）；Python
`peach2_target_model` 未装 bondpy 时 skip（test_6）。

域 95–97 + `ROS_LOCALHOST_ONLY=1`（Jazzy 无 `add_ros_isolated_launch_test`，用 `add_launch_test` 的 ENV；
v1 系统测占 89–94）。preflight 只拒同域进程，所以开发机上 domain 0 的栈不挡测试，反之亦然。

## 构建与运行

`peach2_bringup` 已装进主 overlay 时，不必再 source 隔离目录：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select peach2_system_tests
colcon test-result --test-result-base build/peach2_system_tests --verbose
# 测完复核（MUST）：
pgrep -af 'ros2 launch|component_container|ros2 run|move_group|ros2_control_node|robot_state_publisher|peach2_|lifecycle_manager'
```

`peach2_bringup` 只在隔离目录时，先 source 它再按隔离 base 编测：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source build/v2/peach2_bringup_install/local_setup.bash
timeout 600 colcon build --base-paths src/peach2 --packages-select peach2_system_tests \
  --packages-skip peach2_interfaces peach2_core \
  --build-base build/v2/peach2_system_tests --install-base build/v2/peach2_system_tests_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
timeout 1800 colcon test --base-paths src/peach2 --packages-select peach2_system_tests \
  --build-base build/v2/peach2_system_tests --install-base build/v2/peach2_system_tests_install
colcon test-result --test-result-base build/v2/peach2_system_tests --verbose
# 测完复核（MUST）：
pgrep -af 'ros2 launch|component_container|ros2 run|move_group|ros2_control_node|robot_state_publisher|peach2_|lifecycle_manager'
```

单跑一个：`colcon test ... --ctest-args -R test_plan_only_chain`。各测完整输出在
`build/v2/peach2_system_tests/peach2_system_tests/launch_test/*.txt`，`[lt]` 开头的行是测试摘要。
rviz2（offscreen）与 move_group 在 SIGINT 时的退出码不计入断言。

## 2026-09-30 M1 验收（退出门）

`colcon test --packages-select peach2_system_tests`（主 overlay `install/`，域 95–97）**33 tests，0 error / 0 fail / 0 skip**。配套 `peach2_manipulation` gtest **191 tests，0 fail**（31 skip=cppcheck）。方案 §14 M1 退出门（launch_testing 全绿；mock PREGRASP 链）按此数字关闭。产品套袋+剪断、M0 台架、mock FULL 不在本包范围内。

`[lt]` 摘要（当晚）：

- `RunBatch rejected: execution_disabled: use CheckReachability or HarvestTarget for plan-only`
- `HarvestTarget plan-only: outcome=1 code=0 reason='planned' plan_only=True reached=0`
- `RunBatch done: termination='target_list_done' … reached=PREGRASP`，min_dist=0.0000
- `RunBatch no-target: termination='no_targets'`
- `test_6_bond_heartbeat_target_model` ok（须把 `peach2_target_model` 编进当前 overlay；源码 `from bondpy.bondpy import Bond`，过期 install 会静默无心跳）

改完源码后先 `colcon build --packages-select peach2_task peach2_manipulation peach2_target_model peach2_bringup` 再测；隔离 `build/v2/*_install` 与主 `install/` 混 source 会跑到旧二进制。`colcon test-result --packages-select` 本发行版不支持，用 `--test-result-base`。

## 已关闭的缺陷

2026-09-30 系统测曾 xfail 四项，均已在上游收口，现为硬断言：

1. `peach2_target_model` 用 `from bondpy.bondpy import Bond`；本机无 `ros-jazzy-bondpy` 时节点 WARN、test_6 skip。
2. `execution=false` 的 `RunBatch` 在 goal 阶段 REJECT（`execution_disabled`）；plan-only 走
   `HarvestTarget` / `CheckReachability`。
3. 已在拍照位的 MoveTo 塌缩为空运动（`at_goal`），重勘不再 `empty_trajectory`。
4. PREGRASP_ONLY 的 `reached` 停在 PREGRASP（撤退不抬到 RETREATED）。

### 注：lifecycle Active 不等于 JTC 就绪（测试侧等待）

`peach2_lifecycle_manager` 只管五个 peach2 节点，不看 `controller_manager`。mock 下
`spawner_joint_trajectory_controller` 可能比 task 进入 Active 晚约 3 s。这时如果马上发
`RunBatch`，`move_group` 会报 `Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory`，
批次以 `survey_failed:move_to global_photo_pose:EXEC_FAILED:execute_error:-4` 结束（2026-09-30 实测一次）。
测试在第一次运动前用 `/controller_manager/list_controllers` 等 `joint_state_broadcaster` 和
`joint_trajectory_controller` 都 active（`test_bringup_mock::test_0b`、`test_pregrasp_mock_exec::test_0`）。
操作员起栈后同样要先看 `ros2 control list_controllers`（见 `peach2_bringup` README 冒烟清单）。

许可 BSD-3-Clause。
