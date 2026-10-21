# 采摘会话分析 `session_20261021_154930`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_154930/bag`
- 开始时间：2026-10-21 15:49:30
- 结束时间：2026-10-21 15:57:10
- 总时长：460.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_155004_r01 | COMPLETED | 2026-10-21 15:50:08 | 2026-10-21 15:51:51 | skipped=2，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_155004_r01/ledger.json` |
| flow10_155151_r02 | COMPLETED | 2026-10-21 15:51:55 | 2026-10-21 15:52:52 | skipped=1，succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_155151_r02/ledger.json` |
| flow10_155252_r03 | COMPLETED | 2026-10-21 15:52:56 | 2026-10-21 15:54:40 | skipped=2，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_155252_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.64 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_155004_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 21.48 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.84, 0.453, 8.936, 0.0, 8.14], "harvest_confirmed": false, "completion_level": 6} |
| target_3 | 2 | succeeded | 23.51 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.448, 0.245, 11.105, 0.0, 10.52], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 3 | skipped | 3.35 | build_failed |
| target_0 | 4 | skipped | 2.87 | build_failed |

### flow10_155151_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 18.66 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.266, 0.617, 8.833, 0.0, 7.855], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 2 | skipped | 2.88 | build_failed |

### flow10_155252_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 1 | succeeded | 20.51 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [3.277, 0.377, 8.924, 0.0, 7.808], "harvest_confirmed": false, "completion_level": 6} |
| target_10 | 2 | succeeded | 19.33 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.749, 0.291, 9.219, 0.0, 7.813], "harvest_confirmed": false, "completion_level": 6} |
| target_11 | 3 | skipped | 5.06 | skipped_quality |
| target_8 | 4 | skipped | 4.57 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_155004_r01 | — | 1.06 | — | — | — | — | — | — | — | 1.06 |
| flow10_155004_r01:target_1 | target_1 | — | — | 2.79 | 0.65 | 8.98 | — | 8.0 | — | 20.43 |
| flow10_155004_r01:target_1 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_155004_r01:target_3 | target_3 | — | — | 1.4 | 0.2 | 10.98 | — | 10.62 | — | 23.22 |
| flow10_155004_r01:target_3 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_155004_r01:target_2 | target_2 | — | — | 3.05 | — | — | — | — | — | 3.05 |
| flow10_155004_r01:target_2 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| flow10_155004_r01:target_0 | target_0 | — | — | 2.44 | 0.27 | — | — | — | — | 2.71 |
| flow10_155004_r01:target_0 | — | 10.09 | — | — | — | — | — | — | — | 10.09 |
| flow10_155151_r02 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_155151_r02:target_5 | target_5 | — | — | 1.28 | 0.53 | 8.79 | — | 7.83 | — | 18.43 |
| flow10_155151_r02:target_5 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_155151_r02:target_6 | target_6 | — | — | 2.63 | — | — | — | — | — | 2.63 |
| flow10_155151_r02:target_6 | — | 10.18 | — | — | — | — | — | — | — | 10.18 |
| flow10_155252_r03 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| flow10_155252_r03:target_9 | target_9 | — | — | 3.19 | 0.6 | 8.77 | — | 7.78 | — | 20.37 |
| flow10_155252_r03:target_9 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_155252_r03:target_10 | target_10 | — | — | 1.61 | 0.39 | 9.21 | — | 7.82 | — | 19.02 |
| flow10_155252_r03:target_10 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_155252_r03:target_11 | target_11 | — | — | 4.82 | — | — | — | — | — | 4.82 |
| flow10_155252_r03:target_11 | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| flow10_155252_r03:target_8 | target_8 | — | — | 4.22 | — | — | — | — | — | 4.22 |
| flow10_155252_r03:target_8 | — | 10.15 | — | — | — | — | — | — | — | 10.15 |

## 事件统计

- 事件总数：128
- severity AUDIT：24
- severity INFO：104
- `enables_changed`：4
- `photo_pose_reached`：32
- `recovery_acknowledged`：10
- `recovery_required`：10
- `round_locked`：32
- `target_dispatched`：20
- `target_skipped`：10
- `target_succeeded`：10

## 感知统计

- 记录帧数：31
- 帧间隔中位数：1.568 s（≈0.64 FPS）
- 目标数范围：3 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09083747863769531
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 1 | 0 | READY | — |
| target_3 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | COLLECTING | — |
| target_0 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | READY | — |
| target_6 | 2 | 0 | COLLECTING | — |
| target_9 | 2 | 0 | READY | — |
| target_10 | 1 | 0 | READY | — |
| target_11 | 0 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.385,-0.611,0.682 | — | — | — |
| target_3 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.184,-0.714,0.643 | 0.275,-0.701,0.674 | — | — |
| target_2 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.271,-0.689,0.627 | — | — | — |
| target_0 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.545,-0.586,0.674 | 0.546,-0.583,0.721 | — | — |
| target_5 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.386,-0.610,0.682 | — | — | — |
| target_6 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.272,-0.688,0.628 | 0.274,-0.702,0.674 | — | — |
| target_9 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.515,-0.567,0.683 | — | — | — |
| target_10 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.208,-0.742,0.646 | 0.170,-0.722,0.683 | — | — |
| target_11 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.515,-0.567,0.683 | 0.552,-0.592,0.721 | — | — |
| target_8 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.569,-0.608,0.674 | 0.552,-0.592,0.721 | — | — |

## 运行性能统计

- 性能采样条数：426
- CPU %：均值 46.64 / 峰值 88.0
- 内存 %：均值 50.89 / 峰值 52.8
- GPU 利用率 %：均值 30.35 / 峰值 61.0
- GPU 显存 MB：均值 2528.53 / 峰值 2769.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：731
- 路径长：6.9337 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.5442 m
- Z 范围：0.5444 ~ 0.7583 m（Δz=-0.0）
