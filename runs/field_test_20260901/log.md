# 真机测试 2026-09-01 — PREGRASP_ONLY 停预抓取

目的：观察后走到预抓取停住，目视袋轴方向与定位；顺带采真实 TCP jsonl。不开套入、不 SetIO。对照 [docs/testing.md](../../docs/testing.md)。

## 档位（各轮相同，除非另注）

- `hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`
- 调度 `execution_enabled=true`，`execute_pregrasp_only=true`
- 技能 `execution.enabled=true` `grasp.enabled=true` `tool.enabled=false`
- launch 不自动 `RunHarvest`；不起 `aubo_dashboard`
- 接触护栏 12 / 6.1；观察行程默认 2.5 / 1.5
- 相机约 2.43 FPS
- 监控 `http://127.0.0.1:8090`（含 TCP 3D）

冒烟：lifecycle Active；`drives_powered=1` `motion_possible=1` `e_stopped=0`；`/api/state` 与 `/api/trajectory` 200；`wrist3_Link←camera_link` 有外参；彩色/深度各收到一帧。

---

## 轮次 09:00 — `field_pregrasp_20260901_0900`

批次约 8.7 s。`termination_reason=no_targets_succeeded`。`discovered=2` `attempted=2` `succeeded=0` `skipped_quality=2`。无 SetIO。未 HoldPregrasp，无需 ACK。账本 `runs/field_pregrasp_20260901_0900/ledger.json`；综述 `runs/run_20260901_090109/summary.md`。TCP 几乎未动（路径 0.001 m，Δz 0.7 mm）。

| 目标 | 观察 | 终局 |
|------|------|------|
| `target_0` | Survey 后立刻派 OBSERVE_ONLY，技能锁定集尚未跟上（4 次 REJECT） | `observe_failed: observe_only rejected (skills locked set)` |
| `target_1` | 当前位采帧未完成（重建 `missing_mask`）；短移候选 `travel=0.150 m` 的 LIN 累计关节 **2.63–3.70 rad**，全部 > 观察门 2.5；一路 LIN 失败后 PTP 9.76 rad 亦拒 | `observe_failed: 剩余候选视点均不可达或规划失败` |

光照 WARN：掩膜内有效深度 EMA≈0.27 < 0.35（yaml 不阻断）。感知入口：`target_0` `[0.273, -0.648, 0.505]`，`target_1` `[0.626, -0.731, 0.451]`。

接触 12 / 6.1 未用到。

---

## 轮次 09:07 — `field_pregrasp_20260901_0907`

本会话运行时 `moveit.observe_max_total_joint_travel_rad=4.0`（仓库 yaml 默认仍 2.5；单轴仍 1.5）。接触仍 12 / 6.1。`tool.enabled=false`。

批次约 30 s。`termination_reason=no_targets_succeeded`。`discovered=2` `attempted=2` `succeeded=0` `skipped_quality=2`。无 SetIO。未 HoldPregrasp，无需 ACK。账本 `runs/field_pregrasp_20260901_0907/ledger.json`；综述 `runs/run_20260901_090727/summary.md`。

| 目标 | 观察 | 终局 |
|------|------|------|
| `target_0` | 锁定集已跟上；当前位采帧未完成（`missing_mask`）；短移前身份门 `selected_target_changed` | `observe_failed: selected_target_changed` |
| `target_1` | 当前位采帧未完成；`see_a-1_e0_r1` LIN `travel=0.15 m` **已下发**（82 点，goal-hold **8.31 s**，过观察行程 4.0）。到位后等新鲜观测超时，再 `selected_target_stale`。重建 `missing_mask` 41 次、有效视点 0 | `observe_failed: selected_target_stale` |

TCP jsonl：198 点，路径 0.376 m，偏弦最大 0.185 m，Z 0.65–0.71 m。起止几乎同点（观察段往返/回拍照位）。光照 WARN：有效深度 EMA≈0.26–0.28 < 0.35。感知入口：`target_0` `[0.272, -0.647, 0.505]`，`target_1` `[0.622, -0.724, 0.467]`。

接触 12 / 6.1 仍未用到。整栈仍在跑；观察行程本会话保持 4.0。

---

## 轮次 17:04 — `field_pregrasp_20260901_1704`

重构后代码首次 HoldPregrasp。档位：两边 `execution=true`，`grasp.enabled=true`，`tool.enabled=false`，`execute_pregrasp_only=true`；观察行程 yaml 4.0；手眼 `[0.045, 0.108, 0.002]`；`motion_possible=1` `e_stop=0`。launch 预检补杀 10 个残留 `extrinsics_publisher` 后起栈。

批次 41 s。终局 `RECOVERY_REQUIRED`（停预抓取等 ACK）。`succeeded=1` `skipped=1`。无 SetIO。验收门：FPS 2.5 ✓、tf_failures=0 ✓、到预抓取停住 1 ✓。账本/综述同根 `runs/field_pregrasp_20260901_1704/`。

| 目标 | 观察 / 重建 | 终局 |
|------|-------------|------|
| `target_2` | 2 机位、TSDF 812、refit ACCEPT；无 `neighbor_gap` | 拍照位到达后 PTP 预抓取 0 解（精化预抓取 `[0.533, -0.444, 0.614]` 半径 ~0.93 m）。`skipped_unreachable`。SELECT 感知入口过 IK，精化入口仍超程 |
| `target_1` | 短移 LIN 0.117 m / 8.8 s，基线 10.2°；2 机位、TSDF 1651、refit ACCEPT；夹角 8.0° | 回拍照位 3.9 s → 预抓取 PTP **5.71 s goal-hold** → HoldPregrasp `SUCCEEDED` + `recovery_required`。TCP `[0.298, -0.639, 0.367]` = 规划预抓取。`allowed=false`（`bag_d95_exceeds_tool`）未拦。**现场目视：方向与位置良好，轨迹可行** |

TCP jsonl：路径 1.09 m，起止弦 0.53 m，Z 0.37–0.71 m。作业票停在「靠近」。现场目视已评：`target_1` 筒口对袋轴、定位可用、拍照位→预抓取 PTP 轨迹可行。看完后 `ControlTask` 命令 6 ACK 才再 Survey。

---

## 轮次 17:57 — `field_pregrasp_20260901_1757`

现行几何：`peach_perception/config/grasp_standoffs.yaml` `entry_standoff_m=0`、`pregrasp_standoff_m=0`（launch 字典注入，不是 ParameterFile）。档位同表头；观察行程 yaml 4.0；手眼 `[0.045, 0.108, 0.002]`；`drives_powered=1` `motion_possible=1`。上一轮 1704 停在袋口，本轮起栈前**示教器手动回到** SRDF `global_photo_pose`（六轴 |Δq|≤0.0001 rad）。未走 `go_to_photo_pose`（行程门 6/2.5，从袋口回去可能拒）。

### 命令（按执行顺序）

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_perception peach_manipulation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
# pgrep 无采摘残留；ping 169.254.10.98 / 169.254.10.110 通

unset PYTHONPATH
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
source /opt/ros/jazzy/setup.bash
source /home/mu/Desktop/aubo_e5_jazzy_ws/install/setup.bash
ros2 launch peach_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
# 17:55:14 rosout：managed nodes Active（仍须显式 RunHarvest）
# Percipio fps ≈ 2.43；感知注册 3 目标

ros2 param set /peach_manipulation_node execution.enabled true
ros2 param set /peach_manipulation_node grasp.enabled true
ros2 param set /peach_executor execution_enabled true
# 确认 execute_pregrasp_only=true、tool.enabled=false

ros2 action send_goal -f /peach_executor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: 'field_pregrasp_20260901_1757', scene_key: 'lab', profile_id: 'default'}"
```

冒烟（launch 后、开批前）：五节点 Active；`tool.entry_d_tool` / `refit.*_standoff_m` / `moveit.mtc_approach_along_axis_m` 均为 0.0；`base_link tip` 不存在。`ros2 topic hz /camera/color/image_raw` 因 QoS 可能误报未发布，以设备 fps 为准。

### 过程

批次 17:58:04–17:58:30（墙钟 26.4 s）。终局 `RECOVERY_REQUIRED`（停预抓取等 ACK，本文记录时尚未命令 6）。无 SetIO。`target_2` SELECT `ik_no_solution`。透传 goal-hold：Survey 0.27 s（已在拍照位）→ 观察 LIN 8.28 s → 回拍照位 3.94 s → 预抓取 PTP 5.53 s。

| 目标 | 观察 / 重建 | 终局 |
|------|-------------|------|
| `target_2` | 未派观察 | SELECT `ik_no_solution` |
| `target_1` | 短移 LIN 0.118 m / 8.28 s；2 机位、TSDF 1582、基线 11.9°、refit ACCEPT | 回拍照位 → 预抓取 PTP **5.53 s goal-hold** → HoldPregrasp `SUCCEEDED` + `recovery_required`。入口=预抓取=`[0.304, -0.614, 0.536]`（感知入口 `[0.328, -0.598, 0.569]`，重建中心 `[0.307, -0.626, 0.601]`）。`allowed=false`（`bag_d95_exceeds_tool`）未拦 |

相对 1704 TCP `[0.298, -0.639, 0.367]`：本轮约沿袋轴 **+17 cm**（去掉 70+100 mm 后撤，停在拟合袋底）。TCP jsonl：227 点，路径 0.657 m，起止弦 0.419 m，绕行比 1.57，Z 0.536–0.712 m。自动 `summary.md` 在 ACK 前计 `unfinished`、门「到预抓取停住」显示 0；技能日志 `[SUCCEEDED] PREGRASP_ONLY：停在预抓取，未套入未 SetIO`。

**现场目视 `target_1`：方向与定位中上水平，只需微调。** 未 ACK、未派下一颗。账本 `runs/field_pregrasp_20260901_1757/`。
