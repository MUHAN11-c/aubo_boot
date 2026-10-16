# 采摘会话分析 `session_20261016_183844`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_183844/bag`
- 开始时间：2026-10-16 18:38:44
- 结束时间：2026-10-16 18:43:00
- 总时长：256.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_f804887d79d1 | COMPLETED | 2026-10-16 18:39:13 | 2026-10-16 18:39:48 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f804887d79d1/ledger.json` |
| round_e79b72440d7c | COMPLETED | 2026-10-16 18:40:06 | 2026-10-16 18:40:37 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_e79b72440d7c/ledger.json` |
| round_7069db0f60e0 | COMPLETED | 2026-10-16 18:40:53 | 2026-10-16 18:41:25 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_7069db0f60e0/ledger.json` |
| round_a8bdf2655842 | COMPLETED | 2026-10-16 18:41:42 | 2026-10-16 18:42:12 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_a8bdf2655842/ledger.json` |
| round_6b0a684bb8b8 | COMPLETED | 2026-10-16 18:42:30 | 2026-10-16 18:42:59 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_6b0a684bb8b8/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.19 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_f804887d79d1

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 29.01 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.803, 10.404, 0.0, 8.866], "harvest_confirmed": false, "completion_level": 6} |

### round_e79b72440d7c

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 24.5 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.62, 10.415, 0.0, 8.939], "harvest_confirmed": false, "completion_level": 6} |

### round_7069db0f60e0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 26.7 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.64, 11.497, 0.001, 10.73], "harvest_confirmed": false, "completion_level": 6} |

### round_a8bdf2655842

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | succeeded | 24.89 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.248, 9.767, 0.0, 8.775], "harvest_confirmed": false, "completion_level": 6} |

### round_6b0a684bb8b8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | succeeded | 23.89 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.704, 10.557, 0.0, 8.818], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_f804887d79d1 | — | 1.91 | — | — | — | — | — | — | — | 1.91 |
| round_f804887d79d1:target_0 | target_0 | — | — | 5.61 | 2.05 | 10.41 | — | 8.96 | — | 27.02 |
| round_e79b72440d7c | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_e79b72440d7c:target_1 | target_1 | — | — | 3.19 | 1.6 | 10.42 | — | 8.99 | — | 24.19 |
| round_7069db0f60e0 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_7069db0f60e0:target_2 | target_2 | — | — | 2.58 | 1.61 | 11.37 | — | 10.77 | — | 26.36 |
| round_a8bdf2655842 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| round_a8bdf2655842:target_3 | target_3 | — | — | 4.03 | 2.0 | 9.78 | — | 8.8 | — | 24.61 |
| round_6b0a684bb8b8 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_6b0a684bb8b8:target_4 | target_4 | — | — | 2.61 | 1.64 | 10.56 | — | 8.76 | — | 23.59 |

## 事件统计

- 事件总数：68
- severity AUDIT：28
- severity INFO：40
- `enables_changed`：8
- `photo_pose_reached`：10
- `recovery_acknowledged`：10
- `recovery_required`：10
- `round_locked`：10
- `target_dispatched`：10
- `target_succeeded`：10

## 感知统计

- 记录帧数：11
- 帧间隔中位数：5.204 s（≈0.19 FPS）
- 目标数范围：1 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_4
- captured_views：1
- rejected_views：20
- tf_failures：0
- tf_latency_ms：0.11563301086425781
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：bag_mouth_geometry

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |
| target_3 | 1 | 0 | READY | — |
| target_4 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.405,-0.538,0.688 | 0.417,-0.545,0.724 | 0.410,-0.525,0.656 | 0.412,-0.534,0.684 |
| target_1 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.405,-0.538,0.688 | 0.417,-0.545,0.724 | 0.410,-0.525,0.656 | 0.412,-0.534,0.684 |
| target_2 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.405,-0.538,0.688 | 0.417,-0.545,0.724 | 0.410,-0.525,0.656 | 0.412,-0.534,0.685 |
| target_3 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.405,-0.538,0.688 | — | — | — |
| target_4 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.405,-0.538,0.688 | 0.417,-0.545,0.723 | 0.410,-0.525,0.656 | 0.412,-0.534,0.685 |

## 运行性能统计

- 性能采样条数：221
- CPU %：均值 52.72 / 峰值 78.3
- 内存 %：均值 43.92 / 峰值 47.2
- GPU 利用率 %：均值 29.03 / 峰值 62.0
- GPU 显存 MB：均值 2874.97 / 峰值 3210.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：745
- 路径长：6.0117 m
- 起止弦：0.0568 m
- 绕行比（路径/弦）：105.805（直线≈1）
- 相对弦最大偏离：0.3367 m
- Z 范围：0.5582 ~ 0.7792 m（Δz=0.0019）
