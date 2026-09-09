# 真机测试 2026-09-03 — PREGRASP_ONLY（先拍照再开窗）

目的：Survey 到位并核关节后再 BeginScene；观察后走到预抓取停住。不开套入、不 SetIO。对照 [docs/testing.md](../../docs/testing.md)。

## 档位

- `hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`
- 调度 `execution_enabled=true`，`execute_pregrasp_only=true`
- 技能 `execution.enabled=true` `grasp.enabled=true` `tool.enabled=false`
- launch 不自动 `RunHarvest`；不起 `aubo_dashboard`
- 轴向后撤 `mtc_approach_along_axis_m=0`

冒烟：五节点 Active；`drives_powered=1` `motion_possible=1`；手眼 `[0.045, 0.108, 0.002]`；开批前关节对照 `global_photo_pose` |Δq|=0.0001 rad。

---

## 轮次 16:04 — `field_pregrasp_20260903_1604`

批次 116 s。`termination_reason=no_targets_succeeded`。`discovered=3` `attempted=3` `succeeded=0` `skipped_quality=3`。无 SetIO。未 HoldPregrasp，无需 ACK。验收门「到预抓取停住 ≥1」✗。账本 `runs/field_pregrasp_20260903_1604/`。

开窗：`surveying` → `photo_pose_reached` → `collecting`（Begin 后 WAIT_LOCK）→ `round_locked`，`scene_epoch=1`。首轮三颗 `ik_no_solution`；回访 Survey（不 Begin）后再锁，仅 `target_1` 仍无 IK。

| 目标 | 观察 | 终局 |
|------|------|------|
| `target_0` | 短移约 0.12 m；有效视点 0/1；拒帧 `neighbor_gap` 79、`missing_mask` 13 | `observe_failed: insufficient_views` |
| `target_2` | 重建全程 `missing_mask` 44 | `observe_failed: selected_target_stale` |
| `target_1` | 重建全程 `missing_mask` 43 | `observe_failed: selected_target_stale` |

感知入口：`target_1` `[0.328, -0.589, 0.568]`，`target_0` `[0.624, -0.607, 0.672]`，`target_2` `[0.612, -0.658, 0.569]`。结束 TCP `[0.302, -0.232, 0.708]`（回到拍照位）。路径长 1.03 m。

---

## 轮次 17:09 — `field_pregrasp_20260903_1709`

SIGINT 16:35 栈，用当日 16:57 编的 `peach_manipulation` 重起（`CheckReachability` 入口→停位几何后再 IK）。冒烟：五节点 Active；`drives_powered=1`；手眼 `[0.045, 0.108, 0.002]`；关节对照拍照位；`tool.enabled=false`；后撤 0.03 m。Percipio 日志 ~2.43 fps。

批次 51 s。`discovered=2` `attempted=1` `succeeded=0` `skipped_unreachable=1`。`termination_reason=no_targets_succeeded`。无 SetIO。未 Hold，无需 ACK。验收门「到预抓取停住 ≥1」✗。账本 `runs/field_pregrasp_20260903_1709/`。

| 目标 | 选果 | 观察 | 终局 |
|------|------|------|------|
| `target_0` | SELECT 过，派出 | 2 视 / 14133 点，refit ACCEPT | `skipped_unreachable`：MTC `ptp to on-axis pregrasp (0/1)`，`ValidateSolution` 路径姿态误差 ~0.48 rad > 20° |
| `target_1` | 回访过滤 | — | `out_of_depth_window:2.20m;ik_no_solution:no_ik` |

感知入口 `target_0` `[0.380, -0.585, 0.563]`。TCP 路径长 0.16 m（观察短移），Z 0.663–0.708 m，结束回拍照位。

---

## 轮次 17:16 — `field_pregrasp_20260903_1716`

同栈连开。46.8 s。`discovered=1` `attempted=1` `skipped_unreachable=1`。`target_0` SELECT 过、2 视 refit ACCEPT，MTC `ptp to on-axis pregrasp (0/1)`。无 SetIO。回拍照位。账本 `runs/field_pregrasp_20260903_1716/`。

---

## 轮次 17:18 — `field_pregrasp_20260903_1717`（Hold，停在预抓取）

36.6 s。`target_0` SELECT 过 → 观察 → **HoldPregrasp SUCCEEDED + `recovery_required`**。到位 TCP `[0.388, -0.598, 0.510]`，作业票预抓取同值；入口 `[0.409, -0.587, 0.586]`。`allowed=false`（`bag_d95_exceeds_tool`）未拦。无 SetIO。ACK 前 summary 记 `unfinished`。未再开下一批。请现场评方向/定位后再 ACK。账本 `runs/field_pregrasp_20260903_1717/`。
