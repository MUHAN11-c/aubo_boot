# peach（v1）测试指导

> 本文是 v1 生产栈的操作与验收指导，按 2026-10-12 的源码、CMake 和实际入口核对。整栈入口只有 `peach_bringup/harvest_system.launch.py`；观测入口属于 `peach_observability`。系统现状见 [bringup 架构](../../peach_bringup/doc/architecture.md) 与 [接口](../../peach_bringup/doc/io.md)，v2 方法见 [v2 测试指导](../../../peach2/peach2_system_tests/doc/testing.md)。文档记录当前实现，改行为时同步对应包文档。
>
> 历史方案、一次性脚本及退役实现已归档，不能作为活动测试入口。运行方法以本页、包内 `test/` / `scripts/` 和实际 `--help` 为准；轮次结果写提交说明，不另建仓库级测试日志。

日常回归按 §2 环境 → §3 包测试 → §6 集成与结果 → §15 清理；冻结数据验证读 §7，真机验收另读 §9/§10。

## 0. 红线（先读）

- 未授权不得真机运动或 SetIO（操作员在 real 上发起 launch/操作台指令 = 授权）。
- 硬件急停在示教器/柜，不经 ROS；`RobotMoveStop`/取消 action 是应用停轨，**不得称 e-stop**。
- 驱动九包只读；`auto_power_on=false`；bringup 不起 `aubo_dashboard`。
- **测完即停**：任何测试/程序结束后必须退出并 `pgrep` 清残留（见 §15）。
- mock 不证明安全；碰撞、电流、急停回路、刀、人在环只在真机 KEEP。

## 1. 默认配置（先读：不带参数启动你得到什么）

```bash
ros2 launch peach_bringup harvest_system.launch.py
```

等价于 `hardware_mode:=mock camera_enabled:=false`：**mock 臂（GenericSystem+标准 JTC）、无相机、MoveIt+RViz2 起、手眼 TF 起、IMU 起、会话 bag 随栈开、全部执行使能关**。默认不发 RunHarvest；手工发批次仍受运行期使能门约束。SURVEY_ONLY 会请求 Survey，允许执行时会移动到拍照位，不能把“无接触派发”当成“无运动”。进程拓扑与数据流见 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。

### launch 参数默认值（`harvest_system.launch.py`；完整列表 `--show-args`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `hardware_mode` | `mock` | 真机须显式 `real`（另需示教器上电与 `robot_ip`） |
| `robot_ip` | `169.254.10.98` | AUBO 控制器 IP；mock 不使用 |
| `tool_profile` | `adaptive_shear_v1` | 末端工具档案；`shear_v1`/`bite_shear_v1` 显式传，切换须整栈隔离重启 |
| `autostart` | `false` | true 时托管栈就绪自动发 RunHarvest（授权语义=操作员在 real 上发起本 launch） |
| `moveit_enabled` | `true` | move_group + RViz2；无头测试另传下行参数 `rviz:=false`，保留 move_group |
| `camera_enabled` | `false` | 相机总开关；前端由 `camera_frontend` 决定 |
| `camera_frontend` | `stereo` | stereo=peach_stereo 主机单图案立体（~13.5 FPS，话题同构）；percipio=设备端 18 图案（2.43 FPS、额定量程 0.4–0.8 m）显式备用；replay=不起驱动、冻结 bag 重发 `/camera/*`（§7）。stereo/percipio 互斥（相机连接独占） |
| `camera_ip` | `169.254.10.110` | peach_stereo 前端使用 |
| `extrinsics_enabled` | `true` | 手眼外参静态 TF（extrinsics_publisher） |
| `hand_eye_enabled` / `hand_eye_web_enabled` | `false` / `false` | 标定采集流程与标定 Web（8088 回环）；标定轮才开（§12） |
| `imu_enabled` | `true` | USB 串口 IMU；无设备每 2 s 重试不挡栈；不进 lifecycle |
| `use_sim_time` | `false` | bag 回放 true + `ros2 bag play --clock`；真机禁止误开 |
| `bond_timeout` | `0.0` | nav2_lm 进程死检窗口 [s]；0=关（现行） |
| `skip_reconstruction` | `false` | 真机 KEEP false；测试档语义见 §8 |
| `brain_params_overlay` | `''` | 冻结相机等测试 overlay；空=不装（§7） |

### 运行期使能默认（软件门全关）

| 门 | 默认 | 语义 |
|----|------|------|
| `/peach_supervisor execution_enabled` | false | false 时 WAIT_LOCK 后直接结算，不选目标、不派 ExecuteTarget |
| `/peach_arm execution.enabled` | false | 运动输出总门；当前批次优先用 supervisor `SetEnables` 广播统一设置，人工调试按臂包授权规则执行 |
| `/peach_arm grasp.enabled` | false | 默认停在 READY_FOR_GRASP；依赖链 execution→grasp→tool 由参数校验钩子强制 |
| `/peach_arm tool.enabled` | false | 工具 IO 默认关；开启要求 grasp.enabled=true |
| `/peach_supervisor execute_pregrasp_only` | true | 接触段发 PREGRASP_ONLY：到预抓取停住、不回 stow、不 SetIO、ACK 后再 Survey |
| RunHarvest `intent` | 0 | PICK_ALL=0 / PICK_SELECTED=1 / SURVEY_ONLY=2（只扫、不派接触目标；Survey 仍可能运动） |

使能是运行时开关（`ros2 param set`，或 `/peach_supervisor/set_enables` 服务——意图源 supervisor、广播 `/peach/batch/enables` 并写审计）；mock/real 同一套门。mock 差异：不起 `aubo_io_controller`，launch 把 `/peach_arm execution.require_robot_status` 置 false（真机 KEEP true，`robot_status` 门生效）。会话 bag 默认随栈录制（`record.level: std`——相机大流限 1 Hz；总量预算 100 GB 最旧先删，报告/账本永不删）。

**仿真下相机 TF 是静态的（重要语义）**：相机为腕载（eye-in-hand，外参 `wrist3_Link → camera_link` 静态 × 腕 TF 动态复合，真机上相机随臂动）。mock 下这一复合**不成立**——真相机一旦打开就物理固定在机位上、回放 bag 也是固定机位采集，都跟不上 mock 臂的 TF 运动。因此仿真下相机相关 TF 必须按**静态锚定**理解：相机位姿锚在拍照/采集位，移动 mock 臂**造不出**新相机机位（视点增量只在真机上存在）。具体形态见 §7（回放窗内静态锚定+锚点允许过期）与 §8（物理相机+mock 臂凑不出第二独立机位）。

环境默认（venv / numpy / 权重）见 §2；真机档位与授权纪律见 §9。

## 2. 构建与环境（所有命令的前置）

从干净终端进入工作区，构建后再装载 overlay：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_system_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
python3 -c 'import sys, numpy, cv_bridge, open3d; print(sys.executable, numpy.__version__, cv_bridge.__file__, open3d.__version__); assert numpy.__version__ == "1.26.4"'
```

- `python3` 应指向本工作区 `aubo_py3.12/bin/python3`；`cv_bridge` 应来自 Jazzy。ROS 依赖使用 apt，其余依赖按 `requirements.txt` 固定版本安装。不要升级 numpy。
- 若旧终端的 `PYTHONPATH` 污染 ROS 包，开新终端，或先 `unset PYTHONPATH`，再依次 source Jazzy、工作区和 venv；不要把 ROS 同名包装一份到 pip。
- source venv 不会改变已生成的 CTest 解释器。首次配置测试包后，核对 `build/peach_system_tests/CTestTestfile.cmake` 的 Python 路径；本节显式设置 site-packages，使系统 Python 启动的节点也能找到 open3d/torch。
- 从零构建整个工作区可用根 `AGENTS.md` 的全量命令；只改某包时构建依赖及受影响包即可。IDL 变更先重建 `peach_interfaces` 和消费包。驱动源文件只读。
- 权重事实源是 `peach_harvester/model/best.pt` 与 `mobile_sam.pt`；缺依赖/权重属于环境失败，不是规划失败。

## 3. 分层测试与结果判读

按 ROS 2 的 [测试指导](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html) 与 [launch_testing 集成方法](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html)，先验证纯核，再验证 ROS 图与退出，最后才做现场验收。

| 层 | 当前入口 | 验证边界 |
|----|----------|----------|
| 包内核与 lint | `peach_common`、`peach_harvester`、`peach_arm`、`peach_bringup`、`peach_observability` 等包的 pytest/gtest/ament lint | 参数规则、状态机、预算、几何、协议、报告；具体收集以各包测试配置为准 |
| 零整栈测试 | 本包 CTest 的 preflight / replay / baseline / frozen overlay | 不起驱动；冻结 bag 缺失时会 skip，不能当成回放验收 |
| ROS 集成 | 本包 `add_launch_test`（域 89–94、98） | mock 接线、生命周期、服务/QoS、取消和退出 |
| 场景仿真 | `peach_sim` 纯核、Blender 离线门、Gazebo 场景 | 见 §13；场景可运行不等于已接入物理臂控制 |
| 现场 | 独立授权的 real 栈与 `runs/` | 方向、接触、刀具、电流与安全回路的最终验收 |

```bash
# 能力包测试：核与该包已注册的 lint；不启动整套采摘系统
colcon test --packages-select peach_common peach_interfaces peach_harvester \
  peach_arm peach_bringup peach_observability peach_vegetation peach_stereo \
  --return-code-on-test-failure
colcon test-result --verbose

# 列出本包 CTest 的实际名称，不运行节点
ctest --test-dir build/peach_system_tests -N
# 只跑静态/回放/基线组；frozen 测试可读 bag，但不会起 ROS 图
colcon test --packages-select peach_system_tests --return-code-on-test-failure \
  --ctest-args -R '^test_(preflight|replay_approach|perf_baseline|baseline_inventory|frozen_camera_bag)$'
colcon test-result --test-result-base build/peach_system_tests --verbose
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

[Colcon 官方说明](https://colcon.readthedocs.io/en/released/reference/verb/test.html)明确区分测试失败与构建/执行错误：使用 `--return-code-on-test-failure`，再核对失败、错误与 skip；只看到 `Finished` 或进程返回 0 不够。`No tests were found` 也不算通过。CTest 的 `test_test_*.py` 名称和 pytest 文件名不同，筛选前先 `ctest -N -R '<正则>'` 确认命中。

CMake 测试结果在 `build/<包>/test_results/<包>/`，详细输出在 `build/<包>/Testing/Temporary/LastTest.log` 与 `log/latest_test/<包>/stdout_stderr.log`。筛选运行只更新被选中的 XML，其他 XML 可能是旧轮次；核对具体文件和时间，不把全目录汇总误认为本轮全测。Python 包结果位置以本次 colcon 输出为准。

本包 CMake 当前没有注册整包 `ament_lint_auto`；能力包的 lint 和本包测试组分别核对。`peach_arm` 跳过 cpplint 是既存设置，不扩展这个忽略面。所有测试保留正常收集：Python 文件用 `test_*.py`，新增接线用 launch_testing，不在普通 pytest 中抢默认 DDS 域。

## 4. 命名

批次 `request_id` 同时是账本目录名 `runs/<request_id>/`，**不得复用**，按当时本地时间填 `YYYYMMDD`/`HHMM`。

| 用途 | 格式 | 例 |
|------|------|-----|
| 预抓取真机 | `field_pregrasp_<YYYYMMDD>_<HHMM>` | `field_pregrasp_20260901_1757` |
| 接触干跑（不开刀） | `field_full_<YYYYMMDD>_<HHMM>` | `field_full_20260825_1851` |
| 只扫、不派接触目标 | `survey_<YYYYMMDD>_<HHMMSS>` | `intent: 2` = SURVEY_ONLY；允许执行时会移动拍照位 |
| 实验室只扫 | `e2e_survey_<YYYYMMDDTHHMMSS>` | 不评套袋方向 |
| 跳过重建接触验证 | `e2e_unrefined_<YYYYMMDDTHHMMSS>` | `intent: 0` + `skip_reconstruction:=true` |
| 开发机 mock | `dev_<YYYYMMDD>_<HHMMSS>` | 每批唯一；`intent` 默认 0 |
| 监控会话 | `runs/session_<YYYYMMDD>_<HHMMSS>/bag/` | observability 随栈开合，停栈自动出 `bag_report.md` |

`scene_key` 现行实验室用 `lab`；`profile_id` 现行 `default`。`runs/` 结构化文本入库随仓、bag/图像等二进制只留本地（.gitignore 白名单）；分析完成后删除 `bag/` 目录回收——现行入口是 `peach_observability.retention.purge_analyzed_sessions` Python API（纯核选择+I/O 执行，逐条写 `runs/retention_audit.jsonl` 审计；原 scripts/ 回收脚本已随 2026-10-10 清理退役）。

## 5. mock 起 + 冒烟

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false imu_enabled:=false rviz:=false
```

`rviz` 是现行 MoveIt include 的下行参数；无头开发保留 move_group，关闭显示端。需要可视化时传 `rviz:=true`。测试前分配空闲域（例如 `export ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=1`），前后执行 §15 进程检查；localhost-only 不隔离 HTTP 8090、相机连接或控制器资源。

冒烟（发 `RunHarvest` 之前）：

1. `timeout -s INT -k 3 10 ros2 control list_controllers`：mock 的 `joint_state_broadcaster` 与标准 `joint_trajectory_controller` active；真机透传另验。
2. `timeout -s INT -k 3 10 ros2 topic echo /joint_states --once --qos-reliability best_effort`：六关节名与 MUST 序一致（`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`；JSB 的 name 数组常为字母序，按下标当 MUST 序会拧腕）。
3. `timeout -s INT -k 3 5 ros2 run tf2_ros tf2_echo world tcp`（或现行 TCP 帧）：有变换、无跳变风暴；`tf2_echo base_link tip` 应报**不存在**（存在即旧实例污染）。
4. lifecycle 四节点 `active`（场景 → 重建 → 技能 → 调度）；`ros2 lifecycle get /peach_arm` 等。
5. mock 不起 `aubo_io_controller`（`robot_status` 发布者计数 0；launch 已把 `execution.require_robot_status` 置 false）。真机 KEEP 该门 true。
6. 相机开时：`ros2 topic hz /camera/color/image_raw`（订阅 QoS 须与发布端一致；本机 `hz` CLI 有恒 0 帧怪癖，帧率以 30 帧探针为准）；驱动深度健康口径=头 30 帧 valid <10% 即异常。
7. 启动自检（0037）：栈激活后 observability 自动首轮自检、每 30s 复检；`timeout -s INT -k 3 100 ros2 topic echo /peach/observability/selfcheck_passed --once --qos-durability transient_local --qos-reliability reliable`；软门语义=自检红只呈现不拦手动 RunHarvest，`autostart:=true` 的自动批硬等 `selfcheck_passed=true`。

launch 参数默认值全表见 §1；测试常用覆盖：`camera_enabled:=true`（前端默认 stereo）、`tool_profile`（切换须整栈隔离重启）、`skip_reconstruction`（真机 KEEP false，且真感知下不可用，语义见 §8）、`brain_params_overlay`（冻结相机 overlay，§7）、`use_sim_time`（bag 回放 true + `ros2 bag play --clock`；真机必须 false）。完整列表 `--show-args`。

## 6. 本包 ROS 集成与 Nav2 生命周期调试

### 6.1 已注册的集成组

本发行版安装的 `launch_testing_ament_cmake` 使用 `add_launch_test(... ENV ...)` 固定域；以 [Jazzy 宏源码](https://github.com/ros2/launch/blob/jazzy/launch_testing_ament_cmake/cmake/add_launch_test.cmake) 和本包 [CMakeLists.txt](../CMakeLists.txt) 为准，不照搬其他发行版的 isolated 宏。

| CTest 名称 | 域 / CTest 上限 | 覆盖 |
|------------|-----------------|------|
| `test_test_mock_launch.py` | 89 / 420 s | 无相机 mock 起栈；关节在场、四托管节点 Active、bond 心跳、自检通过及工件；不发 RunHarvest；显式 `rviz:=false` |
| `test_test_e1_supervisor_chain.py` | 90 / 600 s | 注入观测并发批次：C01 SURVEY_ONLY 无接触派发，C03 PREGRASP_ONLY 停驻待 ACK；C02 仍显式 skip，不能计入已通过 |
| `test_tool_profile_smoke_shear_v1` | 91 / 420 s | 工具档案与 TF 一致、节点按工具档位出现 |
| `test_tool_profile_smoke_bite_shear_v1` | 92 / 420 s | 同上 |
| `test_tool_profile_smoke_adaptive_shear_v1` | 93 / 420 s | 同上，含 adaptive 的 imu_follow 接线 |
| `test_test_scene_obstacles.py` | 94 / 300 s | 合成云与假 PlanningScene 服务；目标邻域空洞、滤除、裁剪及原子替换 |
| `test_test_nav2_lifecycle_debug.py` | 98 / 60 s | 假 Nav2 服务 + 实际 DebugBridge：五条命令的标准类型、success 真/假、无 message 返回、服务不可用 503、退出码；不证明实际四节点能完成所有转换 |

```bash
# 先确认两个名称确实命中，再跑常用冒烟
ctest --test-dir build/peach_system_tests -N \
  -R '^test_test_(mock_launch|nav2_lifecycle_debug)[.]py$'
colcon test --packages-select peach_system_tests --return-code-on-test-failure \
  --ctest-args -R '^test_test_(mock_launch|nav2_lifecycle_debug)[.]py$'
colcon test-result --test-result-base build/peach_system_tests --verbose

# 完整本包测试：包括会移动 mock 关节的 E1；不启动真机
colcon test --packages-select peach_system_tests --return-code-on-test-failure \
  --ctest-args -j 1
```

域 89–94、98 均 `ROS_LOCALHOST_ONLY=1`。IVG 默认域 96、ivg_sim 域 98，与 v2/本包有冲突；不要在这些域已有演示时测试。即使域不同，两套整栈也可能争 HTTP 8090、相机和设备，手工整栈验证按顺序运行。`test_scene_obstacles` / Nav2 假服务测试不需要真实相机；整栈测试仍会导入感知库/权重，因此环境需完整。

### 6.2 8090 与真实管理器分两步验

生命周期已完全由 Nav2 管理。本仓旧 `peach_interfaces/srv/ManageLifecycleNodes`、自研管理器 launch 与大脑转发入口均已退役，不提供兼容别名。标准服务是 [Nav2 Jazzy ManageLifecycleNodes](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/srv/ManageLifecycleNodes.srv)：STARTUP=0、PAUSE=1、RESUME=2、RESET=3、SHUTDOWN=4，响应只有 `success`；本调试面不增加 CONFIGURE/CLEANUP 命令。

```bash
# 已按 §5 起好独立 mock 栈后，只读核对
ros2 service type /peach_lifecycle_manager/manage_nodes
ros2 interface show nav2_msgs/srv/ManageLifecycleNodes
curl --max-time 15 --fail --silent http://127.0.0.1:8090/api/state
# 对空闲 mock 栈取消技能；不发运动 goal
curl --max-time 15 --fail --silent -H 'Content-Type: application/json' \
  -d '{}' http://127.0.0.1:8090/api/debug/skill_cancel_service
# PAUSE 后核对四节点 inactive，RESUME 后再核对 active
curl --max-time 15 --fail --silent -H 'Content-Type: application/json' \
  -d '{"command":"PAUSE"}' http://127.0.0.1:8090/api/debug/manage_nodes_service
timeout -s INT -k 3 10 ros2 lifecycle get /peach_arm
curl --max-time 15 --fail --silent -H 'Content-Type: application/json' \
  -d '{"command":"RESUME"}' http://127.0.0.1:8090/api/debug/manage_nodes_service
timeout -s INT -k 3 10 ros2 lifecycle get /peach_arm
```

必须同时看 HTTP 返回与 `result.success`。HTTP `accepted=true` 只是请求已送达，`success=false` 仍是转换失败；不依赖结果中的 `message`。服务不存在时应返回 503 且 `accepted=false`。取消调用成功只证明取消请求被接收；有在途 goal 时还要等 action 终态和运动退出，再拆栈。

仅空闲 mock 栈进行管理转换：PAUSE→RESUME；RESET 应从 active 发起并观察四节点 unconfigured；STARTUP 只从 unconfigured 发起，不能对 active 栈重复启动。SHUTDOWN 从新起且空闲的 active 栈单独验，之后重启 launch，不做自动恢复旧 goal。真机故障后不得用 RESUME 接续原运动。

**现行限制（2026-10-12 复核）**：实际 8090 mock 的取消、PAUSE、RESUME、RESET，以及新起栈 SHUTDOWN 成功；RESET 后再次 STARTUP 在 `peach_scene_perception_node` 重新 configure 时返回 false。该缺口未在清理轮修改；记录失败状态/日志后停栈重启，不能把假服务五命令通过写成真实栈复启通过。`bond_timeout=0` 的测试也不能证明进程死亡检测生效；开 bond 的验证须另跑。

### 6.3 失败分层

| 现象 | 先检查 | 结果怎么记 |
|------|--------|------------|
| 无 joint_states / lifecycle 未 Active | 预检残留、controllers、节点异常、域与 QoS | 栈/环境失败；保留首次异常与 launch 日志 |
| `open3d` / `torch` ImportError | §2 Python 路径、venv、权重 | 依赖失败，不归类 IK 失败 |
| 无 selfcheck 判定 | observability Active、`use_sim_time=false`、启动事实、自检开关、HTTP/日志 | 未得到判定；不能按“没有 false”通过 |
| RViz Ogre/GLX 崩溃 | 无头用已有 `rviz:=false`，不要另起 MoveIt | 显示端问题；保留豁免退出码，不掩盖业务节点异常 |
| stereo C++ lint / sim lint 红 | 分别看该包报告；清理轮 sim 全量 flake8 尚有 194 条既存问题 | 与本轮新增问题分开；不删检查、不靠重复重试宣称全绿 |
| xmllint 下载 schema 失败 | 网络与 package XML 本地解析 | 记录基础设施失败；不能直接认定 XML 正确或格式错误 |
| bag 用例 skip / E1 C02 skip | 实际跳过原因与数据可用性 | 未验证对应行为；数目不能并入通过 |

## 7. 冻结相机一次感知 → 抓一颗（推荐仿真，域 81）

静止 RGB-D 来自 `runs/peach_camera_capture_20260930/stereo_10s`（`PEACH_FROZEN_BAG` 可改；先核 metadata.yaml 与实际 bag 存储文件；缺数据需重新采集或从备份恢复后再跑）。回放节点把 stamp 改成当前时刻，mock 臂 Survey 到拍照位后用当时腕 TF 反投影。**相机侧在整个回放窗内是静态的**（§1：bag 为固定机位采集）——锚定发生在拍照位停驻时刻，之后 mock 臂的运动不代表相机动；overlay 开 `grasp.allow_stale_anchor` 正是为此（锚点允许过期不重算）。**只扫一次、只抓一颗**；产品 yaml `min_views=2` 不动，测试 overlay `config/frozen_one_view.yaml` 才置 1（另：基线门 0°、`reconstruct_in_trajectory` 跳过补视 LIN、`tool.budget.d_inner=0.25` 过单视 sigma；驱动再开 `grasp.allow_stale_anchor`、关 `moveit.obstacle_guard_enabled`、`grasp.maximum_travel_m` 收到 0.02、快照 `workspace_radius_m` 收到 0.05）。不要走 `skip_reconstruction`（真感知未精化直径为 0）。第一颗终局后 action cancel。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
ros2 run peach_system_tests run_frozen_grasp.py        # v1 域 81；bite_shear_v1、tool=false 干跑套入
# 常用参数：--bag <路径>（默认 PEACH_FROZEN_BAG 或 runs/peach_camera_capture_20260930/stereo_10s）、
#           --index <n>（抓第几颗）、--hz、--request-id（默认 frozen_v1_<unix>）、--skip-launch
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|frozen_camera|move_group|ros2_control_node'
```

- 整段墙钟 **180 s**（launch+感知+接触+拆栈），超时即失败并排查，不要拉长。
- 2026-10-09 起 rviz2 不再强设 `QT_QPA_PLATFORM=offscreen`（会砸 Ogre GLX 窗口）；现行冻结 runner 未提供 `--rviz` 参数，退出判定仍豁免 RViz；无头跑这条路径时先核对其实际 launch。不要为关闭显示另起第二份 MoveIt/驱动。
- 成功口径：账本/结果至少一颗 `succeeded`（v1 `tool=false` 干跑套入+跳过 SetIO），不是"尝试过就算过"。
- FULL 抓取用例（需 bag+GPU）不进默认 colcon test；`test_frozen_camera_bag`（零 GPU）已注册进默认。

## 8. 实验室端到端：区分真相机与注入观测

mock 臂配真相机时，相机物理机位固定，移动 mock 关节不会得到新 RGB-D 机位。此档用于相机出图、感知、TF 与场景观测验证，不证明多视重建或套袋方向；不要改产品 `min_views` / `max_target_drift_m` 来凑通过。

```bash
# 相机验证：会连接相机设备，但臂仍是 mock；先确认设备未被其他栈占有
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo imu_enabled:=false \
  rviz:=false autostart:=false
# 在另一同域终端做有限时间的只读探针
timeout -s INT -k 3 5 ros2 topic hz /camera/color/image_raw
timeout -s INT -k 3 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link
```

**接触链接线回归优先跑 §6 的 E1**：无相机、`skip_reconstruction:=true`、夹具注入带尺寸的合成 `target_observations`，在 mock 中开 execution/grasp，tool 保持 false。真感知的未精化直径为 0，不能把“真相机 + skip_reconstruction”当成可靠预抓取或套入入口。需要真实 RGB-D 抓一颗时走 §7 的冻结 overlay，并把结果限于该数据与测试配置。

E1 C03 的判据：`PREGRASP_ONLY` 到位后停住，不 ACK 不结束；ACK 后批次收口；无 SetIO。需要单独人工批次时，使用本包现行 `drive_harvest.py --help` / `perception_sim.py` 与实际 IDL；每批唯一 request_id，不复活退役的网格/回放战役脚本。

SURVEY_ONLY 只禁止接触目标派发，Survey 可移到拍照位。FULL 干跑也不是“零运动”：只有 mock、当前工具档案及完整质量门成立才验证，`tool=false` 全程保持；验收 `harvest.grasped=false`。轮末取消在途 goal、关闭全部使能，再按 §15 拆栈。

## 9. 真机纪律（授权前/中/后）

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

前置：pgrep 无残留；示教器上电、抱闸释放；手眼 TF 在位；`tf2_echo base_link tip` 报不存在；相机出图且深度配准（≈2.5 FPS percipio / ~13.5–13.7 stereo）；当前关节对照拍照位。复现命令链（含 venv PYTHONPATH 双 source 次序）与逐项检查见本节历史版（git 历史 `docs/testing.md`）；要点：

1. 起 real 栈，等 managed nodes Active。
2. 通过 `/peach_supervisor/set_enables` 统一设置 execution/grasp，tool 保持 false；`execute_pregrasp_only` 保持 true。逐项核对广播与臂侧实际门，避免只改调度参数而臂侧不同步。
3. `RunHarvest{request_id: 'field_pregrasp_YYYYMMDD_HHMM', scene_key: 'lab', profile_id: 'default'}`。
4. 每颗期望链：首巡 Survey → Begin → WAIT_LOCK → SELECT → Build+OBSERVE → `PREGRASP_ONLY` 停预抓取（作业票停「靠近」）→ 现场目视评方向/定位 → 命令 6 ACK → 回访 Survey → 下一颗。
5. 通过判据：`ExecuteTarget` 终局 `SUCCEEDED` 且 `recovery_required=true`；全程无 SetIO；对错只在现场评，`allowed`/余量只作记录。
6. 轮次结束 launch Ctrl+C + pgrep 复核（MUST）。

观察护栏参考：观察 `observe_max_total_joint_travel_rad` 4.0 / 单轴 1.5；接触 12 / 6.1；拍照位行程 6 / 2.5。轴向后撤 `grasp_standoffs.yaml` 现行入口 0.0 / 预抓取 0.03。

### 透传与标定的边界

不从 `_archive/` 运行历史透传运动脚本。透传验收由操作员另行授权并依照驱动/控制器现行接口实施；本页默认自动化路径只有 mock。标定命令见 §12，可能移动真机的 auto 档必须单独确认授权、人员隔离和示教器状态。

## 10. 验收门（产品）

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

## 11. 回放塔（解析回归门；改接近/融合/护栏/包络必跑）

`colcon test --packages-select peach_system_tests` 的 `test_replay_approach`：零 ROS 纯几何，确定性语料三层（现场真袋 14 例（`src/peach/peach_arm/test/fixtures/field_pregrasp_cases.yaml`）；分层 200 例 seed 20260911；随机 100 例 seed 20260910）对照 `test/replay_baselines.json` 冻结基线（2026-09-23 v4 三路点重封；基线逐数权威是 json 自身 provenance/revised 链）。判定：`analytic_ok` 只许升不许降；`lin_chord_fail` 双侧容差 1；分母精确断言，采样器/案册漂移须显式重封。护栏数学 `test/replay_oracle.py`（v4 公式）。旧 bag 回放兼容走 `peach_observability/bag_reader.py` 的 LEGACY 别名。

## 12. 标定（手眼外参 + 彩色内参）

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

内参变更后手动更新 `peach_harvester/config/scene_perception.yaml` 的 `calibration_version`；感知/重建按新 K 重验。标定器原为 vendored image_pipeline jazzy 包 `src/camera_calibration`（2026-10-10 已随清理删除，可 `git show <提交>:src/camera_calibration/...` 取回）。

## 13. peach_sim 仿真（非验收门）

**A. Blender 重建 + 感知验证矩阵（离线门）**：`src/peach_sim/reconstruction/` 三道离线门 + 一道回归门：几何门（`validate_geometry.py`）、三锚定机位感知门（`validate_perception.py`）、感知验证矩阵门（`run_matrix.py` → `evaluate_matrix.py --gate` 对冻结基线）。GPU 渲染作业串行（双 Blender 并发抢 3090 OOM）；矩阵帧与 EXR 不入 git。命令与分层口径见 `peach_sim` README。

**A→gz 查看桥（2026-10-09/10）**：重建去草整园经 `blender/export_field_to_gz.py` 无头导出单 GLB（材质 Base Color 重接贴图/回退 diffuse + GN 叶 Realize **两阶段物化**（先全部物化再删原对象，边删边物化会静默丢叶）+ 逐对象 `Mesh.transform` 烘焙世界变换、多对象不 join + `export_yup=False`——本机 gz-common 5.9/Assimp 按 Z-up 直读顶点，yup=True 会被竖成 90° 墙；`field_no_grass.glb` 的大小与顶点数随导出版本变化，首次加载预算按实际产物核对；轻量版可用 `--decimate 0.25`），`GZ_SIM_RESOURCE_PATH=src:src/peach_sim gz sim -r src/peach_sim/worlds/field_no_grass.sdf` 人工查看。该世界无碰撞体/传感器/桥/TF，**不是**导航或感知 GT（仍以 `worlds/peach_orchard.sdf` 为准）；Blender 走 `_tools/` 自带 Python（勿混 aubo_py3.12/numpy）。看完关窗并 `pgrep -af 'gz sim'` 清残留（决策 0039）。

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

## 14. IMU 与 imu_follow；RViz；IVG

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

与 peach 隔离：IVG 默认域 96 与 v2 plan-only 测试撞号，ivg_sim 域 98 与 v1 Nav2 调试测试撞号；不在对应域演示时同时测试，也不同时占用驱动或相机。真机另按授权纪律执行。

## 15. 停栈与清进程（MUST）

1. 有在途批次先取消，等终态；v1 用 `ControlTask.CANCEL_NOW=4`，不要把 ACK 或 RESUME 当成取消。默认无批次的冒烟无需发运动类请求。
2. 关闭 execution/grasp/tool；终止相机探针、bag、回放与注入器。
3. launch 终端 Ctrl+C，等退出和报告写盘。observability 的报告/录制收尾可能超过普通节点 5 s 窗口，不在正在写盘时立刻强杀。
4. 复核本轮拥有的 PID、命令与 DDS 域，再处理残留；不要按宽泛名字 `pkill`。本包 post_shutdown 允许部分信号退出码且豁免 MoveIt/RViz，这不能替代残留检查。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|ros2 bag|move_group|ros2_control_node|robot_state_publisher|peach_|lifecycle_manager|frozen_camera|parameter_bridge|gz sim'
# 查看一个确认属于本轮测试的进程及其域；将 12345 换成实际 PID
ps -p 12345 -o pid,ppid,args
tr '\0' '\n' < /proc/12345/environ | rg '^ROS_DOMAIN_ID='
```

残留先对本轮 PID 发 SIGINT/TERM并等待；确认不再运动、写盘已收尾且仍不能退出时才 KILL。重查进程、订阅和 TF；别人的栈只报告，不在清理自己的测试时终止。超时使用 `timeout -s INT -k <拆栈余量> <预算>`，并在超时后再次检查子进程；外层超时不保证已完成所有子进程的退出。一次性只读探针可用 `timeout -s INT -k 3 5 ...`，到时间停止采样不是“收到足够数据”的通过证据。

冻结 runner 会清理本域 marker 匹配进程，因此选空闲域，勿和同域其他作业共用。v1/v2/IVG 切换必须先取消并弃在途 goal、停当前栈、核无残留，再起下一栈；不能依赖 DDS 不同域解决设备和端口争用。
