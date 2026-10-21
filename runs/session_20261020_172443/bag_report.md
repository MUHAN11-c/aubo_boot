# 采摘会话分析 `session_20261020_172443`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261020_172443/bag`
- 开始时间：2026-10-20 17:24:43
- 结束时间：2026-10-20 17:38:58
- 总时长：855.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_172502_r01 | COMPLETED | 2026-10-20 17:25:02 | 2026-10-20 17:26:45 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_172502_r01/ledger.json` |
| flow10_172645_r02 | COMPLETED | 2026-10-20 17:26:49 | 2026-10-20 17:28:34 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_172645_r02/ledger.json` |
| flow10_172834_r03 | COMPLETED | 2026-10-20 17:28:39 | 2026-10-20 17:30:52 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_172834_r03/ledger.json` |
| flow10_173052_r04 | COMPLETED | 2026-10-20 17:30:56 | 2026-10-20 17:32:43 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_173052_r04/ledger.json` |
| flow10_173243_r05 | COMPLETED | 2026-10-20 17:32:48 | 2026-10-20 17:34:25 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_173243_r05/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.79 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 15 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_172502_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 16.45 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.369, 6.807, 0.0, 7.063], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 2 | succeeded | 17.87 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.364, 8.184, 0.0, 7.662], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 3 | succeeded | 17.27 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.392, 8.029, 0.0, 7.559], "harvest_confirmed": false, "completion_level": 6} |
| target_3 | 4 | skipped | 3.24 | skipped_unreachable |

### flow10_172645_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 15.67 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.239, 6.897, 0.0, 7.073], "harvest_confirmed": false, "completion_level": 6} |
| target_4 | 2 | succeeded | 17.49 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.312, 8.223, 0.0, 7.894], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 3 | succeeded | 16.05 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.22, 7.431, 0.0, 7.214], "harvest_confirmed": false, "completion_level": 6} |
| target_7 | 4 | skipped | 2.06 | skipped_unreachable |

### flow10_172834_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 1 | succeeded | 16.26 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.593, 6.721, 0.0, 7.059], "harvest_confirmed": false, "completion_level": 6} |
| target_10 | 2 | succeeded | 19.26 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.436, 9.801, 0.0, 7.602], "harvest_confirmed": false, "completion_level": 6} |
| target_8 | 3 | succeeded | 17.11 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.346, 7.866, 0.0, 7.679], "harvest_confirmed": false, "completion_level": 6} |
| target_11 | 4 | skipped | 30.13 | skipped_quality |

### flow10_173052_r04

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_13 | 1 | succeeded | 15.1 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.341, 6.848, 0.0, 7.085], "harvest_confirmed": false, "completion_level": 6} |
| target_12 | 2 | succeeded | 17.29 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.366, 8.193, 0.0, 7.734], "harvest_confirmed": false, "completion_level": 6} |
| target_14 | 3 | succeeded | 17.07 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.418, 8.347, 0.0, 7.409], "harvest_confirmed": false, "completion_level": 6} |

### flow10_173243_r05

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_16 | 1 | succeeded | 16.06 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.459, 6.686, 0.0, 7.149], "harvest_confirmed": false, "completion_level": 6} |
| target_15 | 2 | succeeded | 18.09 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.473, 8.14, 0.0, 7.776], "harvest_confirmed": false, "completion_level": 6} |
| target_17 | 3 | succeeded | 16.79 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.598, 7.557, 0.0, 7.283], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_172502_r01 | — | 0.76 | — | — | — | — | — | — | — | 0.76 |
| flow10_172502_r01:target_1 | target_1 | — | 0.93 | 0.96 | 0.71 | 6.8 | — | 7.02 | — | 16.37 |
| flow10_172502_r01:target_1 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_172502_r01:target_0 | target_0 | — | — | 1.38 | 0.41 | 8.17 | — | 7.6 | — | 17.56 |
| flow10_172502_r01:target_0 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_172502_r01:target_2 | target_2 | — | — | 1.01 | 0.4 | 7.99 | — | 7.59 | — | 16.99 |
| flow10_172502_r01:target_2 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_172502_r01:target_3 | target_3 | — | — | 3.01 | — | — | — | — | — | 3.01 |
| flow10_172502_r01:target_3 | — | 6.08 | — | — | — | — | — | — | — | 6.08 |
| flow10_172645_r02 | — | 0.72 | — | — | — | — | — | — | — | 0.72 |
| flow10_172645_r02:target_5 | target_5 | — | 0.25 | 1.21 | 0.2 | 6.8 | — | 7.19 | — | 15.65 |
| flow10_172645_r02:target_5 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_172645_r02:target_4 | target_4 | — | — | 0.81 | 0.2 | 8.19 | — | 8.02 | — | 17.21 |
| flow10_172645_r02:target_4 | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| flow10_172645_r02:target_6 | target_6 | — | — | 1.0 | 0.2 | 7.39 | — | 7.2 | — | 15.8 |
| flow10_172645_r02:target_6 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_172645_r02:target_7 | target_7 | — | — | 1.83 | — | — | — | — | — | 1.83 |
| flow10_172645_r02:target_7 | — | 6.08 | — | — | — | — | — | — | — | 6.08 |
| flow10_172834_r03 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_172834_r03:target_9 | target_9 | — | — | 1.75 | 0.46 | 6.78 | — | 7.01 | — | 16.0 |
| flow10_172834_r03:target_9 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_172834_r03:target_10 | target_10 | — | — | 1.19 | 0.41 | 9.8 | — | 7.63 | — | 19.03 |
| flow10_172834_r03:target_10 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_172834_r03:target_8 | target_8 | — | — | 1.0 | 0.41 | 7.77 | — | 7.62 | — | 16.8 |
| flow10_172834_r03:target_8 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_172834_r03:target_11 | target_11 | — | — | 29.84 | — | — | — | — | — | 29.84 |
| flow10_172834_r03:target_11 | — | 6.05 | — | — | — | — | — | — | — | 6.05 |
| flow10_173052_r04 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_173052_r04:target_13 | target_13 | — | — | 0.6 | 0.41 | 6.8 | — | 7.02 | — | 14.83 |
| flow10_173052_r04:target_13 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_173052_r04:target_12 | target_12 | — | — | 0.79 | 0.41 | 8.18 | — | 7.6 | — | 16.96 |
| flow10_173052_r04:target_12 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_173052_r04:target_14 | target_14 | — | — | 0.8 | 0.39 | 8.2 | — | 7.4 | — | 16.77 |
| flow10_173052_r04:target_14 | — | 6.73 | — | — | — | — | — | — | — | 6.73 |
| flow10_173243_r05 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_173243_r05:target_16 | target_16 | — | — | 1.59 | 0.4 | 6.78 | — | 7.0 | — | 15.77 |
| flow10_173243_r05:target_16 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_173243_r05:target_15 | target_15 | — | — | 1.6 | 0.4 | 8.18 | — | 7.61 | — | 17.78 |
| flow10_173243_r05:target_15 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_173243_r05:target_17 | target_17 | — | — | 0.81 | 0.86 | 7.55 | — | 7.44 | — | 16.65 |
| flow10_173243_r05:target_17 | — | 6.1 | — | — | — | — | — | — | — | 6.1 |

## 事件统计

- 事件总数：248
- severity AUDIT：64
- severity INFO：184
- `enables_changed`：4
- `photo_pose_reached`：56
- `recovery_acknowledged`：30
- `recovery_required`：30
- `round_locked`：56
- `target_dispatched`：36
- `target_skipped`：6
- `target_succeeded`：30

## 感知统计

- 记录帧数：74
- 帧间隔中位数：1.268 s（≈0.79 FPS）
- 目标数范围：3 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.11873245239257812
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 1 | 0 | READY | — |
| target_0 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |
| target_3 | 1 | 0 | READY | — |
| target_5 | 1 | 0 | READY | — |
| target_4 | 1 | 0 | READY | — |
| target_6 | 1 | 0 | READY | — |
| target_7 | 1 | 0 | READY | — |
| target_9 | 1 | 0 | READY | — |
| target_10 | 1 | 0 | READY | — |
| target_8 | 1 | 0 | READY | — |
| target_11 | 0 | 0 | COLLECTING | — |
| target_13 | 1 | 0 | READY | — |
| target_12 | 1 | 0 | READY | — |
| target_14 | 1 | 0 | READY | — |
| target_16 | 1 | 0 | READY | — |
| target_15 | 1 | 0 | READY | — |
| target_17 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；精化几何可达性未通过: no_ik | 0.291,-0.572,0.628 | — | — | — |
| target_0 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.509,0.667 | 0.224,-0.572,0.687 | — | — |
| target_2 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.238,-0.565,0.653 | 0.683,-0.852,0.537 | — | — |
| target_3 | lock | 是 | 工具档关；尚未到达预抓取 · SKIPPED_UNREACHABLE | 0.705,-0.854,0.509 | 0.683,-0.852,0.537 | 0.742,-0.844,0.530 | 0.713,-0.848,0.530 |
| target_5 | lock | 否 | 工具档关；精化几何可达性未通过: no_ik | 0.291,-0.572,0.628 | — | — | — |
| target_4 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.509,0.667 | 0.223,-0.572,0.687 | — | — |
| target_6 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.239,-0.563,0.653 | 0.683,-0.853,0.533 | — | — |
| target_7 | lock | 是 | 工具档关；尚未到达预抓取 · SKIPPED_UNREACHABLE | 0.692,-0.853,0.517 | 0.683,-0.853,0.533 | 0.740,-0.845,0.526 | 0.711,-0.849,0.529 |
| target_9 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000066s timeout=30.000000s | 0.295,-0.584,0.629 | — | — | — |
| target_10 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.240,-0.580,0.651 | 0.442,-0.515,0.722 | — | — |
| target_8 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.444,-0.520,0.667 | 0.442,-0.515,0.722 | 0.434,-0.501,0.649 | 0.436,-0.507,0.679 |
| target_11 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.295,-0.584,0.629 | — | — | — |
| target_13 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.294,-0.584,0.629 | — | — | — |
| target_12 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.444,-0.520,0.667 | 0.225,-0.586,0.688 | — | — |
| target_14 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.240,-0.580,0.652 | 0.225,-0.586,0.688 | 0.242,-0.561,0.642 | 0.234,-0.576,0.667 |
| target_16 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.294,-0.584,0.629 | — | — | — |
| target_15 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.444,-0.520,0.667 | — | — | — |
| target_17 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.240,-0.580,0.652 | 0.225,-0.585,0.688 | 0.242,-0.561,0.644 | 0.234,-0.577,0.669 |

## 运行性能统计

- 性能采样条数：796
- CPU %：均值 49.12 / 峰值 92.9
- 内存 %：均值 51.53 / 峰值 52.9
- GPU 利用率 %：均值 23.69 / 峰值 46.0
- GPU 显存 MB：均值 1668.17 / 峰值 1755.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：2072
- 路径长：18.795 m
- 起止弦：0.0023 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.3937 m
- Z 范围：0.5009 ~ 0.7622 m（Δz=0.0001）
