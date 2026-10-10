# peach2（v2）测试指导

> 本文档是 v2 重构栈（`src/peach2/` 十二包）的测试流程指导，2026-10-10 自仓库级 `docs/testing.md` 拆写入包（用户裁定）。现行系统快照的活文档为 [docs/architecture.md](../../../../docs/architecture.md) 与 [docs/io.md](../../../../docs/io.md)；v2 设计与分期落地的权威是 [docs/peach_v2_重构终版方案.md](../../../../docs/peach_v2_重构终版方案.md)。v1 栈指导在 [src/peach_system_tests/doc/testing.md](../../../peach_system_tests/doc/testing.md)。
>
> **边界（与 v1 并存）**：v2 不进 `harvest_system`/现行 lifecycle、不订 `peach_interfaces`（契约单源 `peach2_interfaces`）、入口 `peach2_bringup/peach2_system.launch.py`。生产入口仍是 v1 `peach_bringup`；v1/v2 不同时占有同一驱动/相机/运动端口；回退以停作业弃在途 goal 为前提（方案 §18.13）。

## 0. 红线（先读）

与 v1 相同基线：未授权不得真机运动或 SetIO；硬件急停在示教器/柜不经 ROS；驱动九包只读；**测完即停 + pgrep 清残留**（§10）；mock 不证明安全。v2 当前所有系统测只动 mock 关节（`mock_components` + JTC），**绝不连真机**。

## 1. 构建与环境（开发机）

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

- Python：`aubo_py3.12` venv；**numpy 必须 ==1.26.4**（Jazzy cv_bridge 按 numpy 1.x 编译，升 2.x ABI 崩）。venv 抢 `PYTHONPATH` 时先清再依次 source Jazzy 与工作区 setup。
- ROS 2 依赖走 Jazzy apt；其余第三方钉在 `requirements.txt` 装 venv。
- v2 感知共用 v1 权重：`$(find-pkg-share peach_harvester)/model/best.pt`（YOLO）与 `mobile_sam.pt`（`peach2_perception/config/perception.yaml` 指同一 share 路径）。

## 2. 入口与档位

```bash
ros2 launch peach2_bringup peach2_system.launch.py     # 默认 mock、camera_enabled:=false
```

launch 不发任何 `RunBatch` / `SetEnables` / 运动 / SetIO；所有 goal 由测试进程或人工显式发出。托管顺序 perception → target_model → scene → manipulation → task（lifecycle + bond）。

## 3. 系统测（isolated 域）

```bash
colcon test --packages-select peach2_system_tests
colcon test-result --verbose        # 现行 33/33 绿
```

| 文件 | 域 | 内容 |
|------|----|------|
| `test_bringup_mock.py` | 95 | task/manipulation/target_model Active；`is_active`；`/peach/enables` 默认全 false；action/service server 就绪；launch 无副作用；C++ 节点 bond 心跳；post_shutdown 退出码 |
| `test_plan_only_chain.py` | 96 | 合成观测 → 模型收敛；enables 全 false 发 `RunBatch(PREGRASP_ONLY)`：goal REJECT（`execution_disabled`）、关节不动、SetIO 0 次；`HarvestTarget(PREGRASP_ONLY)`：`plan_only=true`、`reached=NONE` |
| `test_pregrasp_mock_exec.py` | 97 | `SetEnables(execution=true)` → `RunBatch(PREGRASP_ONLY)`：mock 关节真实运动、TCP 到 `GetDecision.pregrasp_tcp`（<2 cm）、`reached=PREGRASP`、停 `WAITING_ACK` 不 ACK 不结束；`/peach/task/acknowledge_recovery` 后 `COMPLETED/SUCCEEDED`；SetIO 0 次 |
| `test_frozen_camera_bag.py` | — | 零 ROS 图核 frozen overlay 三 yaml；有 bag 再核一帧 RGB-D；缺 bag skip |

域 95–97 均 `ROS_LOCALHOST_ONLY=1`（ENV 隔离，Jazzy 无 `add_ros_isolated_launch_test` 宏的等效实现）。`test/peach2_harness.py` 是公共夹具（begin_scene/build_snapshot 桩、5 Hz 合成观测、SetIO 间谍服务——任何测试见到调用即失败、xfail 机制）。xfail 收口史（bond import、RunBatch REJECT、空运动 MoveTo、PREGRASP reached）见包 README 与 git 历史（原 REFACTORING 2026-09-30 条）。

## 4. 冻结相机一次感知 → 抓一颗（推荐仿真，域 82）

静止 RGB-D 来自 `runs/peach_camera_capture_20260930/stereo_10s`（`PEACH_FROZEN_BAG` 可改；采集 mcap 已按回收政策清除，需重新采集或恢复 bag 后再跑）。`camera_frontend:=replay` 不起驱动，由 `frozen_camera_replay.py` 重发 `/camera/*`（stamp 改当前时刻）。**只扫一次、只抓一颗**、`max_targets=1`、mock IO。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
ros2 run peach2_system_tests run_frozen_grasp.py --intent PREGRASP_ONLY   # 通过档（cycle ~21 s）
ros2 run peach2_system_tests run_frozen_grasp.py                          # FULL：本数据下预期 no_targets（插入出可达球）
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|frozen_camera|move_group|ros2_control_node'
```

**终态（2026-10-09 操作员三轮裁定）**：仿真经 `mock_initial_positions`（授权参数化）**出生即真拍照位 `global_photo_pose`**——无 home→photo 过渡，Survey 原地锚定，仅抓取首运动冻结回放（`watch_motion_freeze`，phase≥OBSERVING 门控+2 mm 关节偏差触发）。photo-move 备用流已彻底删除（drive/run 脚本、`photo_pose.py`、SRDF `frozen_photo_pose` 位——操作员裁定该流是错的）。

**本数据（stereo_10s，静止场景）数据边界**：主袋 ~1.04 m、沿外向轴插入终点在 E5 可达球外（三轮确定性复现）；**PREGRASP_ONLY 是本数据通过档**；FULL 需更近采集或底盘前移。

**v2 测试 overlay 旋钮**（均只在本包 config，产品 yaml 不动）：

- `frozen_target_model.yaml`：`min_views=1`、关 `cut_requires_remeasure`/`require_known_swing`、sigma 放到参数上限 0.05、`observe_max_views:=1`。
- 测试标定 `status: calibrated` + 测试工具档案 `frozen_tools/<tool_id>.yaml` 放宽 `L_insert` 过单视 `bag_longer_than_L_insert` 门（`--tool-id bite_shear_v1`→0.150，产品 0.030；`adaptive_shear_v1`→0.200，产品 0.090；产品标定状态仍 `design_reference`）。
- `frozen_scene.yaml`：硬障碍抬到飞点（v2 L2 对全臂受查，冻结云否则挡套入）。
- `frozen_perception.yaml`：锁定 3 帧 / 0.5 s、`confirm_hits=1`、传感 QoS RELIABLE、`lock_wait_s:=20`、轴向分箱调粗（`n_bins_3d` 12→6、`taper_ratio` 0.85→0.60——**测试专用几何旋钮**：stereo_10s 拍照位融合颈部在可达球外 ~46 mm，粗分箱把颈部估计拉回可达域；产品保持 12/0.85；`max_depth_m` 保持 1.5）。

成功口径：结果至少一颗 `succeeded`（mock IO；PREGRASP_ONLY 档以预抓取到位终局计），不是"尝试过就算过"。整段墙钟 **180 s**，超时即失败并排查。FULL 抓取用例（需 bag+GPU）不进默认 colcon test；`test_frozen_camera_bag`（零 GPU）已注册进默认。rviz2 同 v1 口径：不强设 offscreen，无头崩退不计退出码。

## 5. 分期门现状（方案 §14）

| 阶段 | 门 | 现状 |
|------|----|------|
| M0 台架实测 | 剪切标定 `status: calibrated`（现 `design_reference` → `cut_allowed` 恒 false） | **未过**——需台架标定（`peach2_calibration/results/` 落地真值） |
| M1 骨架 | 十二包 + lifecycle+bond + 命令门 + 三插件 mock IO + BT 主树 + launch_testing 全绿 | **已过**（2026-09-30，33/33） |
| M2 感知 / M3 建模预算 / M4 运动避障 / M5 三末端 / M6 果园 / M7 上底盘 | 见方案 | 未过；产品套袋+剪断与 M0 不是本门 |

## 6. 迁移与对照（方案 §18.13）

- 同 bag 离线对照先行（v1/v2 回放同一冻结 bag，结果对拍）。
- v1/v2 不同时占有同一驱动/相机/运动端口。
- 回退：停作业、弃在途 goal，再切回 v1 生产入口。
- 可复用资产：URDF/meshes、手眼标定、立体前端 C++、TSDF/ICP、`peach_sim` 场景、`harvest_fsm` 测试用例、账本格式。

## 7. 命名与过程数据

与 v1 同一规范：`request_id` = 账本目录 `runs/<request_id>/` 不得复用；`runs/session_*/bag/` 会话袋随栈开合、停栈出 `bag_report.md`；结构化文本入库随仓、bag 二进制只留本地、分析后回收。v2 测试轮次结论随提交说明记录（仓库级 testing-log 与 REFACTORING 均已于 2026-10-10 退役，历史在 git）。

## 8. 标定（工作区级，v1/v2 共用基线）

- **手眼外参**：真值源 `src/aubo_hand_eye_calibration/hand_eye/active.yaml`（入库随仓）；v2 感知/建模消费同一 TF 链。标定流程见 v1 指导 §11（Web 8088 / auto 档）。
- **剪切工具标定**：v2 专属 `peach2_calibration`（`config/` 现为 `design_reference`；M0 门要求台架标定后置 `calibrated`，`results/` 存会话产物 shear_v1/bite_shear_v1/adaptive_shear_v1）。
- **彩色内参**：同 v1（棋盘 + `apply_intrinsics` + 重建 `percipio_camera`）；v2 感知读同一 `camera_info`。

## 9. 共享设施现状

- **IMU / imu_follow**：`serial_imu` 随 v1 `harvest_system` 起（`imu_enabled` 默认 true）；`imu_follow` 接触窗是 v1 `adaptive_shear_v1` 档专属，v2 接触走 BT+MTC，不经 imu_follow。
- **RViz**：v1 面板在 `aubo_e5_moveit_config/rviz/moveit.rviz`（Fixed Frame `base_link`）；v2 无专属 RViz 配置，看 TF/话题用通用 rviz2。
- **peach_sim**：Gazebo 果园世界/目标清单 GT 供两栈共用（导航/感知 GT 以 `worlds/peach_orchard.sdf` 为准；`field_no_grass` 仅视觉查看非 GT）。命令见 `peach_sim` README 与 v1 指导 §12。
- **IVG 演示栈**：独立 DDS 域 96，与 peach/v2 均隔离；见 v1 指导 §13。

## 10. 停栈与清进程（MUST，与启动前 pgrep 成对）

launch 终端 Ctrl+C；一次性命令 `timeout` 包裹。结束后复核：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
```

有残留按 PID `kill -TERM`，2 秒仍存活 `kill -9`，再复核。不要宽泛 `pkill`。
