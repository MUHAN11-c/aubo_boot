# 采摘会话分析 `session_20261021_152438`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_152438/bag`
- 开始时间：2026-10-21 15:24:38
- 结束时间：2026-10-21 15:31:56
- 总时长：437.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_152522_r01 | COMPLETED | 2026-10-21 15:25:26 | 2026-10-21 15:27:22 | skipped=2，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_152522_r01/ledger.json` |
| flow10_152722_r02 | COMPLETED | 2026-10-21 15:27:26 | 2026-10-21 15:29:18 | skipped=2，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_152722_r02/ledger.json` |
| flow10_152918_r03 | COMPLETED | 2026-10-21 15:29:22 | 2026-10-21 15:30:52 | skipped=1，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_152918_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.68 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 6 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_152522_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | skipped | 8.46 | build_failed |
| target_3 | 2 | succeeded | 21.47 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.179, 0.214, 9.881, 0.0, 8.881], "harvest_confirmed": false, "completion_level": 6} |
| target_1 | 3 | succeeded | 21.26 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.037, 0.309, 10.068, 0.0, 9.671], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 4 | skipped | 8.11 | skipped_quality |

### flow10_152722_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 18.47 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.105, 0.442, 8.86, 0.0, 7.899], "harvest_confirmed": false, "completion_level": 6} |
| target_7 | 2 | succeeded | 23.48 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [4.879, 0.284, 9.692, 0.0, 8.498], "harvest_confirmed": false, "completion_level": 6} |
| target_5 | 3 | skipped | 8.11 | skipped_quality |
| target_6 | 4 | skipped | 5.91 | build_failed |

### flow10_152918_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 1 | skipped | 5.04 | build_failed |
| target_10 | 2 | succeeded | 20.58 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.224, 0.319, 9.092, 0.0, 8.606], "harvest_confirmed": false, "completion_level": 6} |
| target_11 | 3 | succeeded | 21.67 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.219, 0.278, 10.581, 0.0, 9.452], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_152522_r01 | — | 1.41 | — | — | — | — | — | — | — | 1.41 |
| flow10_152522_r01:target_2 | target_2 | — | — | 6.81 | 0.25 | — | — | — | — | 7.05 |
| flow10_152522_r01:target_2 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| flow10_152522_r01:target_3 | target_3 | — | — | 1.98 | 0.47 | 9.98 | — | 8.81 | — | 21.24 |
| flow10_152522_r01:target_3 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_152522_r01:target_1 | target_1 | — | — | 0.99 | 0.19 | 10.2 | — | 9.57 | — | 20.94 |
| flow10_152522_r01:target_1 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_152522_r01:target_0 | target_0 | — | — | 7.86 | — | — | — | — | — | 7.86 |
| flow10_152522_r01:target_0 | — | 7.51 | — | — | — | — | — | — | — | 7.51 |
| flow10_152722_r02 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_152722_r02:target_8 | target_8 | — | — | 0.99 | 0.4 | 8.79 | — | 8.02 | — | 18.2 |
| flow10_152722_r02:target_8 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_152722_r02:target_7 | target_7 | — | — | 4.83 | 0.2 | 9.77 | — | 8.4 | — | 23.21 |
| flow10_152722_r02:target_7 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_152722_r02:target_5 | target_5 | — | — | 7.84 | — | — | — | — | — | 7.84 |
| flow10_152722_r02:target_5 | — | 0.3 | — | — | — | — | — | — | — | 0.3 |
| flow10_152722_r02:target_6 | target_6 | — | — | 5.61 | — | — | — | — | — | 5.61 |
| flow10_152722_r02:target_6 | — | 10.15 | — | — | — | — | — | — | — | 10.15 |
| flow10_152918_r03 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_152918_r03:target_9 | target_9 | — | — | 4.82 | — | — | — | — | — | 4.82 |
| flow10_152918_r03:target_9 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| flow10_152918_r03:target_10 | target_10 | — | — | 2.2 | 0.47 | 9.17 | — | 8.62 | — | 20.47 |
| flow10_152918_r03:target_10 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_152918_r03:target_11 | target_11 | — | — | 1.2 | 0.2 | 10.59 | — | 9.37 | — | 21.37 |
| flow10_152918_r03:target_11 | — | 6.04 | — | — | — | — | — | — | — | 6.04 |

## 事件统计

- 事件总数：140
- severity AUDIT：28
- severity INFO：112
- `enables_changed`：4
- `photo_pose_reached`：34
- `recovery_acknowledged`：12
- `recovery_required`：12
- `round_locked`：34
- `target_dispatched`：22
- `target_skipped`：10
- `target_succeeded`：12

## 感知统计

- 记录帧数：39
- 帧间隔中位数：1.468 s（≈0.68 FPS）
- 目标数范围：3 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.16021728515625
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_2 | 1 | 0 | COLLECTING | — |
| target_3 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_0 | 0 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | READY | — |
| target_7 | 2 | 0 | READY | — |
| target_5 | 0 | 0 | COLLECTING | — |
| target_6 | 2 | 0 | COLLECTING | — |
| target_9 | 1 | 0 | COLLECTING | — |
| target_10 | 1 | 0 | READY | — |
| target_11 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_2 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=8.000065s timeout=8.000000s | 0.314,-0.596,0.619 | — | — | — |
| target_3 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.265,-0.670,0.650 | 0.467,-0.588,0.722 | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.467,-0.583,0.667 | 0.598,-0.556,0.735 | — | — |
| target_0 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.590,-0.548,0.676 | 0.597,-0.554,0.731 | — | — |
| target_8 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.262,-0.670,0.661 | — | — | — |
| target_7 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.467,-0.581,0.667 | 0.597,-0.554,0.732 | — | — |
| target_5 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.589,-0.549,0.676 | 0.336,-0.679,0.674 | — | — |
| target_6 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.332,-0.685,0.614 | 0.336,-0.679,0.674 | — | — |
| target_9 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.553,-0.578,0.671 | — | — | — |
| target_10 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.381,-0.618,0.664 | 0.185,-0.703,0.685 | — | — |
| target_11 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.192,-0.703,0.649 | 0.185,-0.703,0.685 | 0.189,-0.680,0.644 | 0.183,-0.697,0.667 |

## 运行性能统计

- 性能采样条数：406
- CPU %：均值 46.6 / 峰值 84.1
- 内存 %：均值 54.66 / 峰值 56.2
- GPU 利用率 %：均值 31.23 / 峰值 66.0
- GPU 显存 MB：均值 2518.91 / 峰值 2831.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：858
- 路径长：8.2207 m
- 起止弦：0.0004 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.5273 m
- Z 范围：0.5493 ~ 0.7644 m（Δz=0.0）
