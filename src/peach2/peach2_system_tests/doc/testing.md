# peach2（v2）测试指导

> 本文是 v2 孵化栈（`src/peach2/` 十二包）的操作与验收指导，按 2026-10-12 的源码与测试注册核对。系统入口为 `peach2_bringup/peach2_system.launch.py`；各能力的现行行为见对应包 README，方案见 [v2 方案书](../../../../docs/peach_v2_重构终版方案.md)。v1 生产栈方法在 [v1 测试指导](../../../peach/peach_system_tests/doc/testing.md)。
>
> v2 的跨能力契约属于 `peach2_interfaces`，不进 v1 harvest_system/lifecycle。两栈共用驱动、模型、相机或仿真资产，不同时占有同一设备/运动端口；切换先取消并弃在途 goal、停栈、核无残留。历史设计书与脚本不作为运行入口。

日常回归按 §1 环境 → §3 测试与结果 → §10 清理；手工起栈读 §2，冻结数据验证读 §4，真值与产品门见 §5/§8。

## 0. 红线（先读）

与 v1 相同基线：未授权不得真机运动或 SetIO；硬件急停在示教器/柜不经 ROS；驱动九包只读；**测完即停 + pgrep 清残留**（§10）；mock 不证明安全。v2 当前所有系统测只动 mock 关节（`mock_components` + JTC），**绝不连真机**。

## 1. 构建与环境

从干净终端先构建，再按顺序装载 Jazzy、工作区 overlay 与 venv：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach2_system_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
python3 -c 'import sys, numpy, cv_bridge; print(sys.executable, numpy.__version__, cv_bridge.__file__); assert numpy.__version__ == "1.26.4"'
```

ROS 2 依赖使用 Jazzy apt，其余第三方按根 `requirements.txt` 固定版本；numpy 1.26.4 是 cv_bridge ABI 约束。若有旧 `PYTHONPATH`，先开干净终端或 `unset PYTHONPATH`，再重 source。已有 CTest 可能仍用 `/usr/bin/python3`；激活 venv 不会更改 CMake 已选解释器，显式的 site-packages 路径也要传给启动的节点。

无相机系统测由夹具注入观测，不需要真实相机/GPU。开相机或冻结 RGB-D 时，v2 感知使用 v1 `peach_harvester` share 内的 `best.pt` / `mobile_sam.pt`，因此还要确认该包已构建并安装权重，且 Python 能导入 torch/open3d；模型路径在 `peach2_perception/config/perception.yaml`。资产复用不代表可混用 v1/v2 IDL。

## 2. 手工 mock 入口与只读就绪核对

```bash
# 空闲手工域；不与本包 95–97 或 IVG 演示共域
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 launch peach2_bringup peach2_system.launch.py \
  hardware_mode:=mock camera_enabled:=false rviz:=false
```

launch 不发 RunBatch、SetEnables、运动 goal 或 SetIO。全部使能默认关，Nav2 管理器名为 `peach2_lifecycle_manager`。`rviz:=false` 使用 MoveIt include 现有下行参数，只关显示端，保留 move_group；不另起驱动或第二个 MoveIt。完整声明参数可 `ros2 launch peach2_bringup peach2_system.launch.py --show-args` 查看。

| 档位/参数 | 当前行为 |
|-----------|----------|
| `camera_enabled=false`（默认） | 不起 perception/scene；托管名单为 target_model → manipulation → task |
| `camera_enabled=true` | 托管名单为 perception → target_model → scene → manipulation → task；replay 档不连接相机驱动 |
| `tool_id=adaptive_shear_v1`（默认） | 三工具之一；冻结 runner 默认 bite，与系统默认不同 |
| `bond_timeout=4.0`（launch 默认） | 开进程死亡检测；系统测试夹具默认置 0.0，不是同一验收档 |
| `use_sim_time=false`（默认） | real 禁止 true；普通 bag play --clock 才需全图仿真时钟 |
| 使能默认全 false | RunBatch 在 execution 关时拒绝；直接 HarvestTarget 可走 plan_only，不能写成两者都会规划 |

另一终端 source 同一环境与域，先核只读状态：

```bash
timeout -s INT -k 3 10 ros2 control list_controllers
timeout -s INT -k 3 10 ros2 lifecycle get /peach2_target_model
timeout -s INT -k 3 10 ros2 lifecycle get /peach2_manipulation
timeout -s INT -k 3 10 ros2 lifecycle get /peach2_task
ros2 service type /peach2_lifecycle_manager/manage_nodes
timeout -s INT -k 3 10 ros2 service call /peach2_lifecycle_manager/is_active std_srvs/srv/Trigger '{}'
timeout -s INT -k 3 10 ros2 topic echo /peach/enables --once \
  --qos-durability transient_local --qos-reliability reliable
ros2 action list -t
```

应为标准 JSB/JTC active、三个能力节点 active、manager `is_active.success=true`、enables 全 false，`/peach/task/run_batch` 和 `/peach/manipulation/harvest_target` 在场。关节名按六关节集合检查，不能把 JSB 输出数组的排序当成控制器命令序。

生命周期服务类型为 `nav2_msgs/srv/ManageLifecycleNodes`，响应只有 `success`。管理器 PAUSE/RESUME 不是批次暂停/恢复 API；v2 当前批次停用 action cancel。不要用生命周期转换接续故障前目标；SHUTDOWN 后需整栈重启。v1 的 8090 五命令桥不作为 v2 专属控制台使用。

## 3. 分层自动化测试

方法参考 [ROS 2 测试指导](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html)、[launch_testing](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html) 和 [Colcon test 参数](https://colcon.readthedocs.io/en/released/reference/verb/test.html)。本包 [CMakeLists.txt](../CMakeLists.txt) 注册三个 launch 组、冻结数据 pytest 和 flake8/pep257/xmllint；当前源码有 29 个测试函数（10/7/9/3），不是 29 个 CTest 项，也不是当前全部通过数。

### 3.1 测试组、预算与夹具

| CTest 名称 | 域 / CTest 上限 | 覆盖与限制 |
|------------|-----------------|------------|
| `test_test_bringup_mock.py` | 95 / 420 s | 三节点 Active、JSB/JTC与关节、server 就绪、默认使能关、launch 无 goal/SetIO 副作用、bond 心跳、退出 |
| `test_test_plan_only_chain.py` | 96 / 720 s | 合成观测建模；RunBatch 执行关闭时 REJECT；直接 HarvestTarget 返回 plan_only、reached=NONE；关节不动、SetIO 0 次 |
| `test_test_pregrasp_mock_exec.py` | 97 / 900 s | execution 开、grasp/tool 关；mock 关节执行至决策预抓取（误差 <2 cm）、reached=PREGRASP、WAITING_ACK；ACK 后完成；无目标再巡、关门、SetIO 0 次 |
| `test_frozen_camera_bag` | 无 ROS 图 / 默认 60 s | 冻结 overlay、optical 静态 TF与一帧 bag 解码；缺 bag 的数据项 skip，不能当成冻结抓取通过 |
| `flake8` / `pep257` / `xmllint` | 无 ROS 图 / 各 60 s | 包内 Python 风格、文档字符串和 package XML；不删除检查换取全绿 |

[peach2_harness.py](../test/peach2_harness.py) 提供 BeginScene/BuildSceneSnapshot 桩、5 Hz 合成观测及 SetIO 间谍服务；无真实感知。预抓取测试允许 mock 关节运动，断言 SetIO 调用计数为 0。它验证任务/建模/规划/执行接线，不验证相机多视质量和真刀剪切。bringup 中缺 bondpy 可导致对应心跳测试 skip，应单独记。

域 95–97 固定 localhost-only；夹具 `bond_timeout` 默认 0.0，可用 `PEACH2_LT_BOND_TIMEOUT` 覆盖。心跳在场不等于进程死检通过；开启 bond 后须另验托管节点异常退出时的降级，不能把关 bond 的结果迁为生产验收。夹具现行仍设 `QT_QPA_PLATFORM=offscreen` 并豁免 RViz/move_group 退出码；无头显示问题要记录，但不能豁免桃采摘业务节点异常。

### 3.2 日常回归与单组复现

```bash
# 先跑能力包的核/gtest与各包已注册的 lint
colcon test --packages-select peach2_interfaces peach2_core peach2_perception \
  peach2_target_model peach2_scene peach2_end_effector peach2_manipulation \
  peach2_task peach2_bringup peach2_calibration peach2_observability \
  --return-code-on-test-failure
colcon test-result --verbose

# 查真实名称、静态/冻结组；此组不启动整栈或 GPU
ctest --test-dir build/peach2_system_tests -N
colcon test --packages-select peach2_system_tests --return-code-on-test-failure \
  --ctest-args -R '^test_frozen_camera_bag$'
# 单独复现不运动的图/plan-only链；先 -N 确认正则确实命中
ctest --test-dir build/peach2_system_tests -N \
  -R '^test_test_(bringup_mock|plan_only_chain)[.]py$'
colcon test --packages-select peach2_system_tests --return-code-on-test-failure \
  --ctest-args -R '^test_test_(bringup_mock|plan_only_chain)[.]py$' -j 1
# mock 执行到预抓取组；不是零运动组
colcon test --packages-select peach2_system_tests --return-code-on-test-failure \
  --ctest-args -R '^test_test_pregrasp_mock_exec[.]py$'
# 完整本包门，包括三个 launch 组和三项 lint
colcon test --packages-select peach2_system_tests --return-code-on-test-failure \
  --ctest-args -j 1
colcon test-result --test-result-base build/peach2_system_tests --verbose
```

`No tests were found` / 只收集未执行 / 显式 skip 都不算相应行为通过。筛选只更新命中的 XML；查看 `build/peach2_system_tests/test_results/peach2_system_tests/` 与具体结果时间，再查 `build/peach2_system_tests/Testing/Temporary/LastTest.log` 或本次 `log/latest_test/peach2_system_tests/stdout_stderr.log`。`colcon test` 默认返回 0 可能包含失败，必须用上述失败返回选项及 test-result。

### 3.3 当前未关闭的门

历史 M1 的 33/33 只代表当时骨架验收，不代替当前版本回归。本地既有结果中预抓取组有三项失败，首个依赖失败为 `DEPENDENCY_UNAVAILABLE:check_reachability:timeout`，随后 ACK/结果断言受影响，并有 Python/pep257 lint 债；其中一些 XML 仍引用已删除的 photo_pose 流，属于旧结果，必须按本节单组命令复跑后再判断。文档更新不宣称 v2 整栈已复测全绿。

遇到 reachability 超时，先核 `/peach/manipulation/check_reachability` server、move_group、节点日志、executor/QoS及请求耗时；缺依赖和超时归类基础设施/依赖失败，不能记为目标不可达。模型未收敛查 fixture 发布、epoch、模型/decision topic 与对应失败码；WAITING_ACK 是预抓取预期停驻，不把它当死锁，也不无条件自动 ACK。

测试报告至少写清：提交/工作区版本、工具档案、ROS 域、bond档、相机来源、测试/overlay配置、执行/跳过/失败组与首条失败码、结果目录、退出与残留检查。不得通过拉长 180 s 冻结预算、关 lint 或复用旧 XML 来改写通过结论。

## 4. 冻结相机一次感知 → 抓一颗（推荐仿真，域 82）

静止 RGB-D 来自 `runs/peach_camera_capture_20260930/stereo_10s`（`--bag` 或 `PEACH_FROZEN_BAG` 可指定；先核目录含 metadata.yaml 与实际 bag 存储文件，缺数据需重新采集或恢复）。`camera_frontend:=replay` 不起驱动，由 `frozen_camera_replay.py` 重发 `/camera/*`（stamp 改当前时刻）。**只扫一次、只抓一颗**、`max_targets=1`、mock IO。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
# runner 只在 ROS_DOMAIN_ID 未设置时默认取 82；显式设置避免继承上一测试域
export ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1
ros2 run peach2_system_tests run_frozen_grasp.py --intent PREGRASP_ONLY   # 本数据的历史通过档
ros2 run peach2_system_tests run_frozen_grasp.py                          # FULL：本数据下预期 no_targets（插入出可达球）
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|frozen_camera|move_group|ros2_control_node'
```

**终态（2026-10-09 操作员三轮裁定）**：仿真经 `mock_initial_positions`（授权参数化）**出生即真拍照位 `global_photo_pose`**——无 home→photo 过渡，Survey 原地锚定，仅抓取首运动冻结回放（`watch_motion_freeze`，phase≥OBSERVING 门控 + 2 mrad 关节角偏差触发）。photo-move 备用流已彻底删除（drive/run 脚本、`photo_pose.py`、SRDF `frozen_photo_pose` 位——操作员裁定该流是错的）。frozen 流默认工具 `bite_shear_v1`（`--tool-id` 可改；系统栈 `tool_id` 默认 `adaptive_shear_v1`，两者不同）。

**本数据（stereo_10s，静止场景）数据边界**：主袋 ~1.04 m、沿外向轴插入终点在 E5 可达球外（三轮确定性复现）；**PREGRASP_ONLY 是本数据通过档**；FULL 需更近采集或底盘前移。

**v2 测试 overlay 旋钮**（均只在本包 config，产品 yaml 不动）：

- `frozen_target_model.yaml`：`min_views=1`（与产品同值，非差异项）、关 `cut_requires_remeasure`/`require_known_swing`、sigma 放到参数上限 0.05、`observe_max_views:=1`。
- 测试标定 `status: calibrated` + 测试工具档案 `frozen_tools/<tool_id>.yaml` 放宽插入/径向门过单视检查（`--tool-id bite_shear_v1`→`L_insert` 0.150 + `D_inner` 0.250，产品 0.030/0.104；`adaptive_shear_v1`→`L_insert` 0.200，产品 0.090；产品标定状态仍 `design_reference`，在 `peach2_calibration/results/`）。
- `frozen_scene.yaml`：硬障碍抬到飞点（v2 L2 对全臂受查，冻结云否则挡套入）。
- `frozen_perception.yaml`：锁定 3 帧 / 0.5 s、`confirm_hits=1`、传感 QoS RELIABLE、`lock_wait_s:=20`、轴向分箱调粗（`n_bins_3d` 12→6、`taper_ratio` 0.85→0.60——**测试专用几何旋钮**：stereo_10s 拍照位融合颈部在可达球外 ~46 mm，粗分箱把颈部估计拉回可达域；产品保持 12/0.85；`max_depth_m` 保持 1.5）。

成功口径：结果至少一颗 `succeeded`（mock IO；PREGRASP_ONLY 档以预抓取到位终局计），不是"尝试过就算过"。整段墙钟 **180 s**，超时即失败并排查。FULL 抓取用例（需 bag+GPU）不进默认 colcon test；`test_frozen_camera_bag`（零 GPU）已注册进默认。冻结 runner 不强设 offscreen，也没有 --rviz 参数；其显示端豁免不证明业务退出正常。先恢复数据与运行环境，再复核实际终态。冻结回放将 stamp 改当前时刻，默认 use_sim_time=false，不另外启动 ros2 bag play --clock 形成第二时钟/发布源。

## 5. 分期门现状（方案 §14）

| 阶段 | 门 | 现状 |
|------|----|------|
| M0 台架实测 | 剪切标定 `status: calibrated`（现 `design_reference` → `cut_allowed` 恒 false） | **未过**——需台架标定（`peach2_calibration/results/` 落地真值） |
| M1 骨架 | 十二包 + lifecycle+bond + 命令门 + 三插件 mock IO + BT 主树 + launch_testing | **历史已过**（2026-09-30）；当前是否通过以 §3 本轮测试为准 |
| M2 感知 / M3 建模预算 / M4 运动避障 / M5 三末端 / M6 果园 / M7 上底盘 | 见方案 | 未过；产品套袋+剪断与 M0 不是本门 |

## 6. 迁移与对照（方案 §18.13）

- 同 bag 离线对照先行（v1/v2 回放同一冻结 bag，结果对拍）。
- v1/v2 不同时占有同一驱动/相机/运动端口。
- 回退：停作业、弃在途 goal，再切回 v1 生产入口。
- 可复用资产：URDF/meshes、手眼标定、立体前端 C++、TSDF/ICP、`peach_sim` 场景、`harvest_fsm` 测试用例、账本格式。

## 7. 命名与过程数据

与 v1 同一规范：`request_id` = 账本目录 `runs/<request_id>/` 不得复用；`runs/session_*/bag/` 会话袋随栈开合（**bag_report 为 v1 专属——v2 `peach2_observability` 未迁离线报告 CLI**）；结构化文本入库随仓、bag 二进制只留本地、分析后回收。v2 测试轮次结论随提交说明记录（仓库级 testing-log 与 REFACTORING 均已于 2026-10-10 退役，历史在 git）。

## 8. 标定（工作区级，v1/v2 共用基线）

- **手眼外参**：真值源 `src/aubo_hand_eye_calibration/hand_eye/active.yaml`（入库随仓）；v2 感知/建模消费同一 TF 链。标定流程见 v1 指导 §12（Web 8088 / auto 档）。
- **剪切工具标定**：v2 专属 `peach2_calibration`（真值在 `results/{shear_v1,bite_shear_v1,adaptive_shear_v1}.yaml`，现行 `design_reference`；M0 门要求台架标定后置 `calibrated`，会话产物也落 `results/`）。
- **彩色内参**：同 v1（棋盘 + `apply_intrinsics` + 重建 `percipio_camera`）；v2 感知读同一 `camera_info`。

## 9. 共享设施现状

- **IMU / imu_follow**：`serial_imu` 随 v1 `harvest_system` 起（`imu_enabled` 默认 true）；`imu_follow` 接触窗是 v1 `adaptive_shear_v1` 档专属，v2 接触走 BT+MTC，不经 imu_follow。
- **RViz**：v1 面板在 `aubo_e5_moveit_config/rviz/moveit.rviz`（Fixed Frame `base_link`）；v2 无专属 RViz 配置，看 TF/话题用通用 rviz2；手工无头整栈传 rviz:=false，测试夹具的显示设置以 §3 为准。
- **peach_sim**：Gazebo 果园世界/目标清单 GT 供两栈共用（导航/感知 GT 以 `worlds/peach_orchard.sdf` 为准；`field_no_grass` 仅视觉查看非 GT）。命令见 `peach_sim` README 与 v1 指导 §13。
- **IVG 演示栈**：独立 DDS 域 96（`ivg_sim` 另用域 98）——与 v2 plan-only 域 96、v1 Nav2 调试测试域 98 撞号：跑演示时勿同时跑对应系统测，反之亦然；见 v1 指导 §14。

## 10. 取消、停栈与清进程（MUST）

1. 有在途 RunBatch/HarvestTarget 时，由持有 goal handle 的测试/客户端发 action cancel，等待批次和子动作终态；v2 无批次 PAUSED 实现，不用 Nav2 PAUSE 当停批。人工 `send_goal` 的客户端取消行为以当前 CLI/服务端反馈为准，不能只退出窗口就假定动作已结束。
2. 开过使能的 mock 批次，结束前统一关闭并核对 `/peach/enables`。ACK 只在现场或测试已检查预抓取后用于解除预期停驻，不能当取消。

```bash
# 对仍 active 的 v2 task 关门；不发运动或工具请求
ros2 service call /peach/task/set_enables peach2_interfaces/srv/SetEnables \
  '{execution: false, grasp: false, tool: false}'
```

3. 退出回放、注入器和探针，再 Ctrl+C 整栈 launch，等 CTest post_shutdown/runner 拆栈结束。
4. 核对残留的具体 PID、命令及域，再清本轮拥有的进程；结果文件成功不能替代此步骤。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|ros2 bag|move_group|ros2_control_node|robot_state_publisher|peach2_|lifecycle_manager|frozen_camera|parameter_bridge|gz sim'
# 12345 替换成确认属于本轮的真实 PID
ps -p 12345 -o pid,ppid,args
tr '\0' '\n' < /proc/12345/environ | rg '^ROS_DOMAIN_ID='
```

对本轮残留先 SIGINT/TERM并等待；确认运动和写盘已停、仍无法退出才 KILL，随后重查。不要宽泛 pkill，不清别人任务。一次性探针用有限时长 `timeout -s INT -k 3 5 ...`；外层整栈 timeout 要留拆栈余量，超时之后仍查子进程。

`run_frozen_grasp.py` 只清自己的域/marker匹配进程（TERM→等待→KILL），其 domain 取现有 ROS_DOMAIN_ID，未设置才是 82；本域必须专用。v1/v2/IVG 的 DDS 域不同也不能解决相机、驱动、运动端口或 Web 端口争用，切栈必须先停干净。真机任何运动/SetIO仍需独立授权，本页自动测试不连接真机。
