# 采摘会话分析 `session_20261016_164923`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_164923/bag`
- 开始时间：2026-10-16 16:49:23
- 结束时间：2026-10-16 17:10:42
- 总时长：1278.7 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_383f214ddcc8 | COMPLETED | 2026-10-16 16:52:41 | 2026-10-16 16:53:58 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_383f214ddcc8/ledger.json` |
| round_2661656bceda | COMPLETED | 2026-10-16 16:54:19 | 2026-10-16 16:55:39 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_2661656bceda/ledger.json` |
| round_321866cf73d8 | COMPLETED | 2026-10-16 16:56:00 | 2026-10-16 16:57:26 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_321866cf73d8/ledger.json` |
| round_0558b556c213 | COMPLETED | 2026-10-16 16:57:48 | 2026-10-16 16:59:09 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_0558b556c213/ledger.json` |
| round_689c797fe0d8 | COMPLETED | 2026-10-16 16:59:30 | 2026-10-16 17:00:47 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_689c797fe0d8/ledger.json` |
| round_ea83bc4c7973 | COMPLETED | 2026-10-16 17:01:09 | 2026-10-16 17:02:31 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_ea83bc4c7973/ledger.json` |
| round_cb74fb024880 | COMPLETED | 2026-10-16 17:02:53 | 2026-10-16 17:04:15 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_cb74fb024880/ledger.json` |
| round_f69d06620af6 | COMPLETED | 2026-10-16 17:04:38 | 2026-10-16 17:05:58 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f69d06620af6/ledger.json` |
| round_141b74e39d39 | COMPLETED | 2026-10-16 17:06:20 | 2026-10-16 17:07:27 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_141b74e39d39/ledger.json` |
| round_8e29707df2c0 | COMPLETED | 2026-10-16 17:07:49 | 2026-10-16 17:09:00 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_8e29707df2c0/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.7 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 8 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_383f214ddcc8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 42.6 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.437, 15.442, 0.0, 15.633], "harvest_confirmed": false, "completion_level": 6} |

### round_2661656bceda

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 44.56 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.278, 20.204, 0.0, 19.731], "harvest_confirmed": false, "completion_level": 6} |

### round_321866cf73d8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | succeeded | 51.0 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.242, 19.784, 0.0, 19.716], "harvest_confirmed": false, "completion_level": 6} |

### round_0558b556c213

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | succeeded | 47.97 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.477, 19.617, 0.0, 17.859], "harvest_confirmed": false, "completion_level": 6} |

### round_689c797fe0d8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 41.74 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.351, 15.578, 0.0, 15.607], "harvest_confirmed": false, "completion_level": 6} |

### round_ea83bc4c7973

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_6 | 1 | succeeded | 45.77 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.34, 19.718, 0.0, 19.491], "harvest_confirmed": false, "completion_level": 6} |

### round_cb74fb024880

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | succeeded | 46.35 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.318, 15.661, 0.0, 15.395], "harvest_confirmed": false, "completion_level": 6} |

### round_f69d06620af6

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | skipped | 42.6 | skipped_quality |

### round_141b74e39d39

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 1 | skipped | 34.21 | skipped_quality |

### round_8e29707df2c0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_10 | 1 | succeeded | 36.56 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.501, 15.93, 0.0, 15.598], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_383f214ddcc8 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_383f214ddcc8:target_1 | target_1 | — | — | 9.87 | 1.53 | 15.38 | — | 15.6 | — | 42.41 |
| round_383f214ddcc8:target_1 | — | 10.05 | — | — | — | — | — | — | — | 10.05 |
| round_2661656bceda | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_2661656bceda:target_2 | target_2 | — | — | 3.04 | 1.59 | 20.04 | — | 19.78 | — | 44.46 |
| round_2661656bceda:target_2 | — | 10.18 | — | — | — | — | — | — | — | 10.18 |
| round_321866cf73d8 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_321866cf73d8:target_3 | target_3 | — | — | 9.99 | 1.38 | 19.8 | — | 19.59 | — | 50.77 |
| round_321866cf73d8:target_3 | — | 10.14 | — | — | — | — | — | — | — | 10.14 |
| round_0558b556c213 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_0558b556c213:target_4 | target_4 | — | — | 8.71 | 1.46 | 19.57 | — | 17.81 | — | 47.57 |
| round_0558b556c213:target_4 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |
| round_689c797fe0d8 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_689c797fe0d8:target_5 | target_5 | — | — | 9.01 | 1.41 | 15.41 | — | 15.61 | — | 41.4 |
| round_689c797fe0d8:target_5 | — | 10.07 | — | — | — | — | — | — | — | 10.07 |
| round_ea83bc4c7973 | — | 0.09 | — | — | — | — | — | — | — | 0.09 |
| round_ea83bc4c7973:target_6 | target_6 | — | — | 4.77 | 1.6 | 19.58 | — | 19.6 | — | 45.55 |
| round_ea83bc4c7973:target_6 | — | 10.07 | — | — | — | — | — | — | — | 10.07 |
| round_cb74fb024880 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_cb74fb024880:target_7 | target_7 | — | — | 13.61 | 1.59 | 15.59 | — | 15.4 | — | 46.19 |
| round_cb74fb024880:target_7 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |
| round_f69d06620af6 | — | 2.63 | — | — | — | — | — | — | — | 2.63 |
| round_f69d06620af6:target_8 | target_8 | — | — | 42.24 | — | — | — | — | — | 42.24 |
| round_f69d06620af6:target_8 | — | 10.12 | — | — | — | — | — | — | — | 10.12 |
| round_141b74e39d39 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_141b74e39d39:target_9 | target_9 | — | — | 33.79 | — | — | — | — | — | 33.79 |
| round_141b74e39d39:target_9 | — | 10.18 | — | — | — | — | — | — | — | 10.18 |
| round_8e29707df2c0 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_8e29707df2c0:target_10 | target_10 | — | — | 3.44 | 1.36 | 16.04 | — | 15.43 | — | 36.25 |
| round_8e29707df2c0:target_10 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |

## 事件统计

- 事件总数：204
- severity AUDIT：40
- severity INFO：164
- `enables_changed`：8
- `photo_pose_reached`：62
- `recovery_acknowledged`：16
- `recovery_required`：16
- `round_locked`：62
- `target_dispatched`：20
- `target_skipped`：4
- `target_succeeded`：16

## 感知统计

- 记录帧数：112
- 帧间隔中位数：1.429 s（≈0.7 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.11467933654785156
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |
| target_3 | 1 | 0 | READY | — |
| target_4 | 1 | 0 | READY | — |
| target_5 | 1 | 0 | READY | — |
| target_6 | 1 | 0 | READY | — |
| target_7 | 1 | 0 | READY | — |
| target_8 | 1 | 0 | READY | — |
| target_9 | 1 | 0 | READY | — |
| target_10 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.404,-0.532,0.670 | 0.417,-0.545,0.726 | 0.413,-0.526,0.658 | 0.414,-0.535,0.686 |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.538,0.691 | — | — | — |
| target_2 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.408,-0.537,0.691 | — | — | — |
| target_3 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.408,-0.537,0.691 | — | — | — |
| target_4 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.537,0.690 | — | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.537,0.690 | — | — | — |
| target_6 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.537,0.689 | — | — | — |
| target_7 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.538,0.689 | — | — | — |
| target_8 | observe | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.407,-0.538,0.689 | — | — | — |
| target_9 | observe | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.407,-0.538,0.690 | — | — | — |
| target_10 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.537,0.690 | — | — | — |

## 运行性能统计

- 性能采样条数：1044
- CPU %：均值 65.09 / 峰值 84.4
- 内存 %：均值 49.73 / 峰值 57.0
- GPU 利用率 %：均值 29.16 / 峰值 63.0
- GPU 显存 MB：均值 1988.19 / 峰值 2924.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：2235
- 路径长：11.9714 m
- 起止弦：0.0021 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.3373 m
- Z 范围：0.5579 ~ 0.7798 m（Δz=0.0001）
