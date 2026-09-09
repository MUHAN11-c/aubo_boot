# 真机测试 2026-08-31 — PREGRASP_ONLY 停预抓取

目的：观察后走到预抓取停住，目视袋轴方向与定位。不开套入、不 SetIO。对照 [docs/testing.md](../../docs/testing.md)。

## 档位（各轮相同）

- `hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`
- 调度 `execution_enabled=true`，`execute_pregrasp_only=true`
- 技能 `execution.enabled=true` `grasp.enabled=true` `tool.enabled=false`
- launch 不自动 `RunHarvest`；不起 `aubo_dashboard`
- 相机约 2.43 FPS

---

## 早班（优化前/中）

| request_id | 结果 |
|------------|------|
| `field_pregrasp_20260831` | 两颗 `skipped_unreachable`：LIN 远移且姿态 slerp → `lin to on-axis pregrasp (0/1)`。果距约 0.88 m 相对 E5 约 0.80 m 工作空间也紧。 |
| `field_pregrasp_20260831_1347` | 两颗 `observe_failed`：没有生成可用观察视点（当时硬可达过滤，随后去掉）。 |
| `field_pregrasp_20260831_1351` | `target_0` LIN 规划过、IK 过、轴约 11.7°，护栏拒 **时长 62 s > 20 s**（行程 8.2 rad，慢直线不是绕行）。`target_1` `insufficient_angular_baseline`。随后关时长门（`*_max_duration_s=0`）。 |
| `field_pregrasp_20260831_1405` | 两颗 `skipped_unreachable`：从观察 look-at 出发 LIN `NO_IK` / `lin align tool z (0/1)`。 |

根因：调度 OBSERVE_ONLY 再 PREGRASP_ONLY（`skip_observation=true`），接触从最后观察姿态出发，相机 look-at ≠ 袋轴工具 Z。08-28 G 从观察位直接 PTP 也是 MTC 0/1。

---

## 轮次 15:54 — `field_pregrasp_20260831_1554`（拍照位再最短路径）

源码：`btMovePregrasp` 先 `goToPhotoPose`，再 `classifyApproach` 选 LIN / CIRC / PTP-align+LIN / PTP。接触护栏累计 10 rad / 单轴 3.2 rad，不按时长。

批次约 63 s。`termination_reason=no_targets_succeeded`。`discovered=3` `attempted=3` `succeeded=0` `skipped_quality=2` `skipped_unreachable=1`。无 SetIO。未 HoldPregrasp，无需 ACK。账本：`runs/field_pregrasp_20260831_1554/ledger.json`。

| 目标 | 观察 | 接近 | 终局 |
|------|------|------|------|
| `target_1` | 2 视、14585 点、TSDF 1670、refit ACCEPT；轴 `[0.315, 0.472, 0.823]`，`axis_angle_deg=23.18`，入口 `[0.228, -0.670, 0.493]` | 预规划（观察位）2 段 11.52 rad / 单轴 4.70 已拒。PTP 回拍照位 goal-hold（约 3.3 s）。拍照位出发单段 PTP：175 点、预计 17.3 s、**10.79 rad / 单轴 4.23**。规划过程 FCL：`table_link`–`foreArm`/`upperArm`、`camera_body_link`–`wrist1`。未下发。 | `skipped_unreachable` |
| `target_0` | 8.5 s / 预算 15 s，移动成本 EMA 8.7 s，有效视点 1/1 | 未进接近 | `observe_failed` / `insufficient_angular_baseline` |
| `target_2` | 约 19 s | 未进接近 | `observe_failed` / `selected_target_stale` |

### 逐门

| 门 | 结果 |
|----|------|
| 真机 launch、dashboard 未起 | 过 |
| `e_stopped=0` `motion_possible=1` | 过 |
| execution→grasp、tool=false、pregrasp_only | 过 |
| Survey / 绑定 / Build | `target_1` 过（2 视 ACCEPT） |
| 预抓取先 PTP 拍照位 | 过（`已到达全局拍照位姿`） |
| 接触行程门 10 / 3.2 | **拒** 10.79 / 4.23（绕腕类，与 08-25「仍拒 4.5 rad 绕腕」同类） |
| HoldPregrasp / 方向目视 | **未到** |
| SetIO | 无 |

护栏随后改为 **12 / 6.1**（URDF 满行程），见下节重测。

整栈随后于 15:58 中止。16:32 重新 launch。

---

## 轮次 16:33 / 16:36 — 护栏 12 / 6.1 重测

档位同前；参数已确认 `mtc_approach_max_total_joint_travel_rad=12.0`、`single=6.1`。`tool.enabled=false`。无 SetIO。未 HoldPregrasp，12/6.1 **未被接近段用到**。

### `field_pregrasp_20260831_1633`（约 57 s）

`discovered=3` `attempted=3` `succeeded=0` `skipped_quality=3`。账本 `runs/field_pregrasp_20260831_1633/ledger.json`。

| 目标 | 终局 |
|------|------|
| `target_0` | `observe_failed` / `insufficient_angular_baseline`（8.5 s，有效视点 1/1，移动 EMA 8.2 s，剩余预算不够再走 8 cm） |
| `target_1` | 同上（9.0 s，有效视点 1/1） |
| `target_2` | `observe_failed` / `selected_target_stale`（约 20 s） |

重建仍采了同机位 2–3 帧；技能不算第二视点（基线 < 8°）。

### `field_pregrasp_20260831_1636`（约 43 s）

| 目标 | 终局 |
|------|------|
| `target_0` | `observe_failed` / `selected_target_changed` |
| `target_1` | `insufficient_angular_baseline`（8.7 s，有效视点 1/1） |
| `target_2` | `selected_target_stale` |

结束后已关 `grasp` / `execution`。整栈仍在跑。要进预抓取须先让观察走出第二机位（预算被 EMA 吃掉，或基线门），不是再抬接触护栏。

---

## 轮次 17:00 — `field_pregrasp_20260831_1700`（观察门按位姿序列，步长 0.15 m）

停准则改为覆盖达标或 `maximum_moves` 用尽，不用移动+等帧 EMA 预测收口。`max_camera_step_m=0.15`。接触护栏仍 12 / 6.1。`tool.enabled=false`。

`target_1` 走完观察→拍照位→PTP 预抓取并 Hold。账本 `runs/field_pregrasp_20260831_1700/ledger.json`；综述 `runs/run_20260831_170201/summary.md`。批次停在 `RECOVERY_REQUIRED`（其余两颗未派）。

| 段 | 实测 |
|----|------|
| 观察 | 当前位采帧 + 短移 `travel=0.117 m`；约 9.8 s。重建 2 机位 / 2 帧，基线 **10.82°**（过 8°），TSDF 1620，refit ACCEPT。无「观察预算收口」。 |
| 接近 | 先 PTP 拍照位 goal-hold。原语 PTP。审查 **10.96 rad / 单轴 4.16**（12 / 6.1 过；1554 同路径曾被 10 / 3.2 拒）。172 点、约 17 s，goal-hold。 |
| Hold | `ExecuteTarget` `SUCCEEDED`（outcome=0），`recovery_required`。入口 `[0.238, -0.649, 0.481]`，轴 `[0.243, 0.305, 0.921]`，`axis_angle_deg=10.08`。夹角 23.2° / 侧向 0.325 m（规划分档，不是目视验收）。`GraspDecision.allowed=false`（`bag_d95_exceeds_tool`）不拦 PREGRASP。 |
| SetIO | 无 |

ControlTask ACK 时调度崩溃：`Client.is_service_ready` 在 Jazzy 不存在（应为 `service_is_ready`）。技能侧已 `acknowledge_recovery`（Trigger 成功）。随后 RViz 另发过一条 1441 点轨迹（约 14.8 s），Hold 姿态可能已离开。技能 `grasp`/`execution` 已关；调度节点已退出。

**方向/定位对错只在现场评。** 未宣称采摘成功。另两颗未观察。

