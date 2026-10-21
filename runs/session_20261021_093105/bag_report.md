# 采摘会话分析 `session_20261021_093105`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_093105/bag`
- 开始时间：2026-10-21 09:31:06
- 结束时间：2026-10-21 09:37:00
- 总时长：354.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_093134_r01 | COMPLETED | 2026-10-21 09:31:39 | 2026-10-21 09:33:23 | skipped=1，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_093134_r01/ledger.json` |
| flow10_093323_r02 | COMPLETED | 2026-10-21 09:33:27 | 2026-10-21 09:35:05 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_093323_r02/ledger.json` |
| flow10_093505_r03 | COMPLETED | 2026-10-21 09:35:09 | 2026-10-21 09:36:47 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_093505_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.7 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 8 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_093134_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 16.52 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.419, 6.947, 0.0, 7.053], "harvest_confirmed": false, "completion_level": 6} |
| target_1 | 2 | succeeded | 19.46 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.309, 9.086, 0.0, 8.032], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 3 | skipped | 30.16 | skipped_quality |

### flow10_093323_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 15.45 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.322, 6.825, 0.0, 7.002], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 2 | succeeded | 16.65 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.456, 7.652, 0.0, 7.448], "harvest_confirmed": false, "completion_level": 6} |
| target_4 | 3 | succeeded | 17.92 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.304, 8.35, 0.0, 8.0], "harvest_confirmed": false, "completion_level": 6} |

### flow10_093505_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 17.08 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.367, 7.173, 0.0, 7.164], "harvest_confirmed": false, "completion_level": 6} |
| target_9 | 2 | succeeded | 17.3 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.216, 7.771, 0.0, 7.441], "harvest_confirmed": false, "completion_level": 6} |
| target_7 | 3 | succeeded | 18.26 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.326, 8.439, 0.0, 8.022], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_093134_r01 | — | 1.49 | — | — | — | — | — | — | — | 1.49 |
| flow10_093134_r01:target_2 | target_2 | — | — | 0.4 | 0.63 | 7.01 | — | 6.99 | — | 15.02 |
| flow10_093134_r01:target_2 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_093134_r01:target_1 | target_1 | — | — | 1.83 | 0.39 | 8.98 | — | 8.0 | — | 19.19 |
| flow10_093134_r01:target_1 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_093134_r01:target_0 | target_0 | — | — | 29.87 | — | — | — | — | — | 29.87 |
| flow10_093134_r01:target_0 | — | 7.64 | — | — | — | — | — | — | — | 7.64 |
| flow10_093323_r02 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_093323_r02:target_5 | target_5 | — | — | 1.03 | 0.37 | 6.78 | — | 7.02 | — | 15.2 |
| flow10_093323_r02:target_5 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_093323_r02:target_6 | target_6 | — | — | 0.99 | 0.4 | 7.59 | — | 7.41 | — | 16.4 |
| flow10_093323_r02:target_6 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| flow10_093323_r02:target_4 | target_4 | — | — | 1.0 | 0.6 | 8.17 | — | 7.99 | — | 17.76 |
| flow10_093323_r02:target_4 | — | 6.48 | — | — | — | — | — | — | — | 6.48 |
| flow10_093505_r03 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_093505_r03:target_8 | target_8 | — | — | 2.17 | 0.2 | 7.2 | — | 7.24 | — | 16.81 |
| flow10_093505_r03:target_8 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_093505_r03:target_9 | target_9 | — | — | 1.59 | 0.21 | 7.81 | — | 7.45 | — | 17.06 |
| flow10_093505_r03:target_9 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_093505_r03:target_7 | target_7 | — | — | 1.21 | 0.41 | 8.37 | — | 8.0 | — | 17.98 |
| flow10_093505_r03:target_7 | — | 6.09 | — | — | — | — | — | — | — | 6.09 |

## 事件统计

- 事件总数：132
- severity AUDIT：36
- severity INFO：96
- `enables_changed`：4
- `photo_pose_reached`：30
- `recovery_acknowledged`：16
- `recovery_required`：16
- `round_locked`：30
- `target_dispatched`：18
- `target_skipped`：2
- `target_succeeded`：16

## 感知统计

- 记录帧数：39
- 帧间隔中位数：1.434 s（≈0.7 FPS）
- 目标数范围：3 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.07772445678710938
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_2 | 2 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_0 | 0 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | READY | — |
| target_6 | 1 | 0 | READY | — |
| target_4 | 1 | 0 | READY | — |
| target_8 | 1 | 0 | READY | — |
| target_9 | 1 | 0 | READY | — |
| target_7 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_2 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000233s timeout=30.000000s | 0.304,-0.615,0.628 | — | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.453,-0.543,0.670 | 0.587,-0.532,0.731 | — | — |
| target_0 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.580,-0.529,0.681 | 0.587,-0.531,0.734 | — | — |
| target_5 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.312,-0.621,0.626 | — | — | — |
| target_6 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.258,-0.623,0.650 | 0.456,-0.550,0.724 | — | — |
| target_4 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.462,-0.546,0.668 | 0.456,-0.550,0.724 | 0.449,-0.530,0.647 | 0.452,-0.536,0.677 |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.311,-0.621,0.625 | — | — | — |
| target_9 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.259,-0.623,0.650 | 0.463,-0.555,0.722 | — | — |
| target_7 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.461,-0.547,0.667 | 0.463,-0.555,0.722 | 0.449,-0.530,0.648 | 0.452,-0.536,0.677 |

## 运行性能统计

- 性能采样条数：332
- CPU %：均值 54.02 / 峰值 87.3
- 内存 %：均值 37.31 / 峰值 38.5
- GPU 利用率 %：均值 22.3 / 峰值 41.0
- GPU 显存 MB：均值 1585.13 / 峰值 1672.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1097
- 路径长：10.5105 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.4284 m
- Z 范围：0.5045 ~ 0.7628 m（Δz=-0.0）
