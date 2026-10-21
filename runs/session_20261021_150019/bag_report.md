# 采摘会话分析 `session_20261021_150019`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_150019/bag`
- 开始时间：2026-10-21 15:00:19
- 结束时间：2026-10-21 15:09:20
- 总时长：541.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_150138_r01 | COMPLETED | 2026-10-21 15:01:42 | 2026-10-21 15:04:09 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_150138_r01/ledger.json` |
| flow10_150409_r02 | COMPLETED | 2026-10-21 15:04:14 | 2026-10-21 15:06:38 | skipped=3，succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_150409_r02/ledger.json` |
| flow10_150638_r03 | COMPLETED | 2026-10-21 15:06:42 | 2026-10-21 15:08:56 | skipped=2，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_150638_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.59 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 6 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_150138_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 18.58 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.382, 0.523, 7.274, 0.0, 7.321], "harvest_confirmed": false, "completion_level": 6} |
| target_3 | 2 | succeeded | 18.49 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.765, 0.364, 8.346, 0.0, 7.851], "harvest_confirmed": false, "completion_level": 6} |
| target_1 | 3 | succeeded | 20.48 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.839, 0.321, 9.992, 0.0, 9.134], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 4 | skipped | 30.2 | skipped_quality |

### flow10_150409_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_6 | 1 | skipped | 5.69 | build_failed |
| target_7 | 2 | skipped | 30.26 | skipped_quality |
| target_5 | 3 | succeeded | 23.21 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.667, 0.254, 10.631, 0.0, 10.428], "harvest_confirmed": false, "completion_level": 6} |
| target_4 | 4 | skipped | 30.14 | skipped_quality |

### flow10_150638_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_10 | 1 | skipped | 2.87 | build_failed |
| target_11 | 2 | succeeded | 19.77 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.47, 0.427, 9.101, 0.0, 8.536], "harvest_confirmed": false, "completion_level": 6} |
| target_9 | 3 | succeeded | 21.09 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.065, 0.317, 10.489, 0.0, 9.088], "harvest_confirmed": false, "completion_level": 6} |
| target_8 | 4 | skipped | 30.17 | skipped_quality |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_150138_r01 | — | 0.91 | — | — | — | — | — | — | — | 0.91 |
| flow10_150138_r01:target_2 | target_2 | — | — | 2.41 | 0.65 | 7.23 | — | 7.42 | — | 17.69 |
| flow10_150138_r01:target_2 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_150138_r01:target_3 | target_3 | — | — | 1.61 | 0.39 | 8.44 | — | 7.82 | — | 18.26 |
| flow10_150138_r01:target_3 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_150138_r01:target_1 | target_1 | — | — | 0.78 | 0.22 | 9.95 | — | 9.2 | — | 20.15 |
| flow10_150138_r01:target_1 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_150138_r01:target_0 | target_0 | — | — | 29.88 | — | — | — | — | — | 29.88 |
| flow10_150138_r01:target_0 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |
| flow10_150409_r02 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_150409_r02:target_6 | target_6 | — | — | 5.43 | — | — | — | — | — | 5.43 |
| flow10_150409_r02:target_6 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| flow10_150409_r02:target_7 | target_7 | — | — | 29.7 | 0.25 | — | — | — | — | 29.95 |
| flow10_150409_r02:target_7 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_150409_r02:target_5 | target_5 | — | — | 1.58 | 0.47 | 10.63 | — | 10.4 | — | 23.06 |
| flow10_150409_r02:target_5 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| flow10_150409_r02:target_4 | target_4 | — | — | 29.88 | 0.18 | — | — | — | — | 30.08 |
| flow10_150409_r02:target_4 | — | 7.28 | — | — | — | — | — | — | — | 7.28 |
| flow10_150638_r03 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_150638_r03:target_10 | target_10 | — | — | 2.61 | — | — | — | — | — | 2.61 |
| flow10_150638_r03:target_10 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_150638_r03:target_11 | target_11 | — | — | 1.4 | 0.65 | 9.0 | — | 8.63 | — | 19.67 |
| flow10_150638_r03:target_11 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_150638_r03:target_9 | target_9 | — | — | 0.99 | 0.2 | 10.63 | — | 9.01 | — | 20.82 |
| flow10_150638_r03:target_9 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_150638_r03:target_8 | target_8 | — | — | 29.91 | 0.2 | — | — | — | — | 30.11 |
| flow10_150638_r03:target_8 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |

## 事件统计

- 事件总数：148
- severity AUDIT：28
- severity INFO：120
- `enables_changed`：4
- `photo_pose_reached`：36
- `recovery_acknowledged`：12
- `recovery_required`：12
- `round_locked`：36
- `target_dispatched`：24
- `target_skipped`：12
- `target_succeeded`：12

## 感知统计

- 记录帧数：43
- 帧间隔中位数：1.701 s（≈0.59 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.07915496826171875
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_2 | 1 | 0 | READY | — |
| target_3 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_0 | 0 | 0 | COLLECTING | — |
| target_6 | 1 | 0 | COLLECTING | — |
| target_7 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | READY | — |
| target_4 | 0 | 0 | COLLECTING | — |
| target_10 | 1 | 0 | COLLECTING | — |
| target_11 | 1 | 0 | READY | — |
| target_9 | 1 | 0 | READY | — |
| target_8 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_2 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000065s timeout=30.000000s | 0.319,-0.665,0.629 | — | — | — |
| target_3 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.265,-0.667,0.649 | 0.467,-0.589,0.722 | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.465,-0.584,0.668 | 0.598,-0.552,0.736 | — | — |
| target_0 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.589,-0.551,0.681 | 0.598,-0.552,0.735 | — | — |
| target_6 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000284s timeout=30.000000s | 0.320,-0.665,0.628 | — | — | — |
| target_7 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.265,-0.667,0.649 | 0.252,-0.679,0.688 | — | — |
| target_5 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.465,-0.584,0.667 | 0.598,-0.553,0.736 | — | — |
| target_4 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.590,-0.551,0.685 | 0.598,-0.552,0.737 | — | — |
| target_10 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.320,-0.665,0.628 | — | — | — |
| target_11 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.266,-0.667,0.649 | 0.466,-0.587,0.723 | — | — |
| target_9 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.465,-0.584,0.668 | 0.598,-0.553,0.737 | — | — |
| target_8 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.590,-0.550,0.685 | 0.598,-0.555,0.737 | — | — |

## 运行性能统计

- 性能采样条数：503
- CPU %：均值 44.78 / 峰值 84.6
- 内存 %：均值 48.46 / 峰值 50.2
- GPU 利用率 %：均值 27.61 / 峰值 57.0
- GPU 显存 MB：均值 1845.07 / 峰值 2277.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：866
- 路径长：8.1818 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.4691 m
- Z 范围：0.5091 ~ 0.766 m（Δz=0.0）
