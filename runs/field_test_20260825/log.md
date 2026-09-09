# 真机测试 2026-08-25 — 方向 / 定位 / 抓取过程

目的：检验算法给出的抓取方向与定位是否准确。对照 [docs/testing.md](../../docs/testing.md)、[docs/io.md](../../docs/io.md)、[docs/architecture.md](../../docs/architecture.md)。

## 架构审查（开测前）

流程无设计断裂：调度显式 `RunHarvest` → `BeginScene` → `SurveyScene` → 并行 `BuildTargetModel` + `ExecuteTarget` OBSERVE_ONLY → 过门后 FULL。作业目标只认调度 `~/state.target_id`。几何权威是 `/peach/reconstruction/grasp_decision` 的 `GraspDecision.allowed`；感知 `BagGraspCandidate.status` ACCEPT 只当初值/可视化。重建积分只用精确 stamp TF。默认 `execution/grasp/tool=false`，launch 不自动开批。接触验收 08-21/08-24 未通过，本轮不开 `grasp` / `tool`。

现场约束：

- 柜 `169.254.10.98`、相机 `169.254.10.110` ping 通。
- 手眼 `hand_eye/active.yaml` 平移约 `[0.045, 0.108, 0.002]`。
- 本机 ROS 域 0 已被另一套 UR Gazebo 占用（`/tf` `/joint_states` `/robot_description`）。采摘栈用 **`ROS_DOMAIN_ID=1`**，避免坐标混叠。
- 档位：`hardware_mode:=real` `camera_enabled:=true`；调度/技能 `execution*` 先关；`grasp.enabled=false` `tool.enabled=false`。

看方向/定位时认这些，不认感知单帧绿框：

| 查什么 | 话题 / 显示 | 通过长什么样 |
|--------|-------------|--------------|
| 身份/初值定位 | `/peach/perception/target_observations`、Debug Image、Perception Markers ns `scene_perception` | 锁定 `target_id`，框与 3D 标记对上实物 |
| 局部模型 | `/peach/reconstruction/tsdf_cloud`、Reconstruction Markers ns `target_reconstruction` | TSDF 贴在袋/果表面 |
| 方向权威 | `/peach/reconstruction/grasp_decision`、`refined_pose`、ns `peach_reconstruction/refined` | `allowed=true` 且 `reason=refined_geometry_accept` 才信 `entry`/`axis`；轴夹角 ≤35° |
| 过程 | 监控 `http://127.0.0.1:8090`、`runs/run_*` jsonl、`ledger.json` | 每目标有 `failure_code`；无 SetIO |

## 启动

```
ROS_DOMAIN_ID=1
hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

## 点云 TF（16:21）

`wrist3_Link→camera_link` 实测名义值 `[0, 0, 0.020]` + 单位四元数。日志：`No active hand-eye result`。

原因：当时 **install** 里 `default_storage_directory()` 把标定目录解析成工作区根 `hand_eye/`（无 `active.yaml`），不是 `src/aubo_hand_eye_calibration/hand_eye/`。源码已是后者，未重装。感知仍出 `base_link` 观测，但光学系相对腕部转错，点云/标记相对臂会偏。

处理：重装 `aubo_hand_eye_calibration`；确认目录指向包内 `hand_eye/` 且 `active.yaml` 可读。重启整栈后 TF 应接近 `[0.045, 0.108, 0.002]`。RViz Fixed Frame 用 `base_link`。

## 全流程重跑（16:38，不开抓取）

档位：`hardware_mode:=real` `camera_enabled:=true`；`ROS_DOMAIN_ID=1`。调度/技能 `execution*=true`；**`grasp.enabled=false` `tool.enabled=false`**。intent=`PICK_ALL` `field_full_20260825_1640`。

冒烟：五能力节点 Active；`motion_possible=1` `e_stop=0`。监控当时经常停在 Unconfigured（后续已改为节点自行 Active）。TCP2CAN `errorCode:10023`，观察 PTP 仍能走。

结果：`success=false` `no_targets_succeeded`；发现 2，成功 0，质量跳过 2。全程无靠近。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 22.5 s | skipped_quality | `observe_failed` 有效视点 0/1，`reconstruction_data_stale` |
| target_1 | 180 s | skipped_quality | `build_target_model rejected`（重建进程 `latest_reconstruction.json.tmp` 并发 replace 崩掉） |

`target_0` 重建中心 `base_link` ≈ `[0.476, -0.663, 0.595]` m；`allowed=false` `reconstruction_not_ready`。观察仍走 0.40 m 球面 orbit（`orbit_a0` 护栏拒、`a1_e0_r0` 63 点、`a1_e0_r1` 32 点），不是产品规定的「当前位 + 一次 ~12° 短 PTP」。无 `GraspHypothesis`。无 SetIO。

## 接触干跑（17:19，开抓取、关工具 IO）

档位：同上；调度 `execution_enabled=true`；技能 `execution.enabled=true` **`grasp.enabled=true` `tool.enabled=false`**。intent=`PICK_ALL` `field_full_20260825_1720`。监控 `http://127.0.0.1:8090`，记录 `runs/run_20260825_172104/`。未起 `aubo_dashboard`，未开 `tool.enabled`。

冒烟：lifecycle 六节点 Active（含 observability）；`robot_status` `mode=2` `e_stopped=0` `drives_powered=1` `motion_possible=1`；`/api/state` 含 `job`；相机约 2.43 FPS。手眼 `active.yaml` 在包内。TCP2CAN `errorCode:10023` 再现，观察运动仍下发。

结果：40.6 s 结束，`success=false` `no_targets_succeeded`；发现 2，成功 0；质量跳过 1，不可达跳过 1。日志无 SetIO。接触验收仍未通过。

### target_0（进到许可与 MTC 规划，臂未走出插入）

阶段：OBSERVING 13.74 s → VALIDATING 0.4 s → APPROACHING 0.6 s。技能：prepare / observe / finalize / reconfirm / approach_insert。Build：3 视角，29932 点，TSDF 1488，refit ACCEPT cylinder，rmse 1.7 mm，inlier 0.47，轴夹角 20.27°（≤35°）。

| 量 | `base_link`（m） |
|----|------------------|
| 感知入口 | `[0.377, -0.651, 0.542]` |
| 重建中心 | `[0.475, -0.664, 0.595]` |
| 抓取进入 / 假设进入 | `[0.379, -0.727, 0.563]` |
| 精化轴 | `[0.972, 0.099, 0.215]` |
| 直径 / 行程 | 99.3 mm / 0.146 m |

`GraspDecision.allowed=true` `reason=refined_geometry_accept`。再确认通过。预规划笛卡尔到入口只到 0.24，改 PTP；内联 MTC **guarded linear insertion** 只到 **0.50**（`min_fraction` 未满），**未下发接触轨迹**。outcome `skipped_unreachable`。监控作业票本目标曾到 **靠近 active、工具 gated**。

观察仍执行 `orbit_a0_e0_r0`、`orbit_a-1_e0_r1`（不是短 PTP 产品口径）。跳过原因里 `robot_not_static` 36 次占主因。

### target_1（停在观察）

OBSERVING 17 s。有效视点 2/1，`insufficient_angular_baseline`。重建停在 COLLECTING。outcome `skipped_quality` `observe_failed`。未到许可/靠近。

摘要表「逐目标作业票」把第二颗的观察失败写进了两行（落盘按最后一帧折叠）。以 ledger 与 `job.jsonl` 过程线为准。

## 再执行（17:36，档位同接触干跑）

`field_full_20260825_1735`，约 51 s。发现 3，成功 0；质量跳过 2，不可达 1。无 SetIO。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 2.8 s | skipped_quality | `selected_target_changed` |
| target_3 | 15.4 s | skipped_unreachable | MTC 短路径护栏：预计 40.9 s > 12 s |
| target_4 | 16.2 s | skipped_quality | 有效视点 0/1，`insufficient_views` |

## 绕行修补后完整重测（18:07 重启，18:09 PICK_ALL）

停 17:19 旧 launch，重新 `harvest_system`（新 `peach_manipulation_skills`）。域 1；`hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`。调度 `execution_enabled=true`；技能 `execution.enabled=true` **`grasp.enabled=true` `tool.enabled=false`**。intent=`PICK_ALL` `field_full_20260825_1808`。记录 `runs/run_20260825_180916/`。未起 `aubo_dashboard`。

冒烟：六节点 Active；`mode=2` `e_stopped=0` `drives_powered=1` `motion_possible=1`；`moveit.mtc_approach_max_duration_s=12` `cartesian_max=0.12` `observe_max_duration_s=8` `scan.maximum_moves=1`；HTTP 8090=200。

结果：22.2 s 结束，`success=false` `no_targets_succeeded`；发现 2，成功 0，质量跳过 2，不可达 0。全程无 SetIO。未进 MTC。接触验收仍未通过。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 6.3 s | skipped_quality | `observe_failed` 有效视点 1/1，`insufficient_angular_baseline` |
| target_1 | 4.0 s | skipped_quality | 同上 |

观察口径这次对上：当前位采帧后一次 `see_a*` 短 PTP（不是 `orbit_a*` 贴球面）。`target_0` 的 `see_a-1_e0_r0` 规划 0.4 ms，透传 38 点，goal-hold **3.94 s** 成功，未被 8 s 观察护栏拒。重建各目标最多 3 视角，`max_baseline_deg≈9.5`（门限 12°）。到位后 0.12 s 就因 `maximum_moves=1` 收口，第三视角记在失败之后。无 40 s 笛卡尔爬行、无 `skipped_unreachable`。

监控作业票停在 lock；靠近未开始；工具 gated。不得宣称接近/插入成功。

## 8° 覆盖门重启后接触干跑（18:24 重启，18:26 PICK_ALL）

停 18:07 旧 launch（重建仍是 12°）。重新 `harvest_system` 加载 8°。域 1；`hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`。调度 `execution_enabled=true`；技能 `execution.enabled=true` **`grasp.enabled=true` `tool.enabled=false`**。intent=`PICK_ALL` `field_full_20260825_1826`。记录 `runs/run_20260825_182652/`、账本 `runs/field_full_20260825_1826/ledger.json`。未起 `aubo_dashboard`。

冒烟：六节点 Active；`mode=2` `e_stopped=0` `drives_powered=1` `motion_possible=1`；`quality.minimum_baseline_deg=8` `capture.minimum_baseline_deg=8`；相机 ~2.43 FPS；HTTP 8090=200。

结果：13.0 s 结束，`success=false` `no_targets_succeeded`；发现 2，成功 0，质量跳过 1，不可达 1。全程无 SetIO。接触轨迹未下发。接触验收仍未通过。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 2.1 s | skipped_quality | `observe_failed: observe_only rejected (skills locked set)`（锁定集空、无锚点；4 次 OBSERVE_ONLY 拒） |
| target_1 | 5.0 s | skipped_unreachable | MTC 到入口 PTP 预计 **18.37 s > 12 s**（累计 10.63 rad，单轴 4.50 rad） |

8° 门对 target_1 生效：当前位采帧 + `see_a1_e0_r0` 短 PTP（goal-hold **3.80 s**），4 视角 READY，`max_baseline_deg≈8.80`，`GraspDecision.allowed=true`（cylinder rmse 1.9 mm，inlier 0.43，轴角 11.3°）。再确认通过。入口距末端 **0.492 m**，日志「跳过多段笛卡尔爬行」。接触几何：entry `[0.262, -0.687, 0.532]` axis `[0.247, -0.359, 0.900]` travel 0.153 m；重建中心 `[0.281, -0.713, 0.642]`。

预规划 PTP 8.95 s / 6.55 rad / 单轴 2.05 rad，被 **4 rad 累计**拒（时长未超 12 s）。内联重规划更差（18 s / 单轴 4.5 rad，典型绕关节），被 12 s 拒。未抬护栏、未执行接触。监控作业票把 target_1 的 MTC 失败叠到两行，以 ledger 与 `job.jsonl` 为准。不得宣称接近/插入成功。

## 12 s 仍卡正常接近（18:44 重启，18:46 PICK_ALL）

累计已改 8 rad、笛卡尔上限 0.60 m，时长仍 12 s。`field_full_20260825_1846`，32 s，成功 0。无 SetIO。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 15.2 s | skipped_quality | `reconstruction_data_stale`（有效视点 0/1） |
| target_1 | 5.4 s | skipped_unreachable | 笛卡尔 0.55 m 只到 79%；PTP **12.6 s / 9.2 rad / 单轴 3.0** 被 12 s 拒 0.6 s；降级锚点 |

## 护栏按绕行重标定后接触干跑（18:51 在线改参，PICK_ALL）

空闲时 `ros2 param set`：接近 **20 s / 10 rad / 单轴 3.2**，`quality.maximum_data_age_s=3`。档位同前（抓取开、工具关）。`field_full_20260825_1851`，44 s，`success=true`，发现 2，成功 1。账本 `runs/field_full_20260825_1851/ledger.json`。全程无 SetIO。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_0 | 4.6 s | skipped_quality | `insufficient_angular_baseline`（有效视点 1/1） |
| target_1 | 24.4 s | succeeded | 观察 + MTC 接近/插入/撤离；卸果 `pending_m8_unload_pose` |

`target_1` 接触几何：entry `[0.298, -0.699, 0.531]` axis `[-0.029, -0.292, 0.956]` travel 0.153 m，轴角 26.0°。预规划 PTP **11.98 s / 9.14 rad / 单轴 1.95** 过门并复用。透传 goal-hold：接近 **12.38 s**（121 点）、插入 **3.04 s**（29 点）、撤离 **4.15 s**（29 点）。`tool.enabled=false` 跳过末端 IO。重建 2 视角，refit ACCEPT（rmse 1.8 mm，inlier 0.39）。

方向和定位准不准以现场目视为准。这是开抓取关工具的接触干跑，不是带工具采摘。

## 沿轴短程接近重跑（19:26 重启，19:28 PICK_ALL）

停 19:13 栈，加载沿检测轴 LIN（预抓取后撤 0.10 m）。域 1；`hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`。调度 `execution_enabled=true`；技能 `execution.enabled=true` **`grasp.enabled=true` `tool.enabled=false`**。intent=`PICK_ALL` `field_full_20260825_1928`。账本 `runs/field_full_20260825_1928/ledger.json`。未起 `aubo_dashboard`。

结果：46 s，`success=false` `no_targets_succeeded`；发现 3，成功 0，质量跳过 3。全程无接触、无 SetIO。沿轴 LIN 未执行。

| 目标 | 耗时 | outcome | 原因 |
|------|------|---------|------|
| target_1 | 16.7 s | skipped_quality | 两次 `see_a*` 到位但 `waitForNewStation` 超时（未形成新机位），有效视点 0/1，`insufficient_views` |
| target_2 | 11.8 s | skipped_quality | `selected_target_stale`（获取性 PTP 后仍无新鲜观测） |
| target_0 | 0.4 s | skipped_quality | 观察候选均超 `observe_max_*`（2.54–3.51 rad 或 12.3 s > 8 s） |

接触沿轴短程接近本轮没有走到。

