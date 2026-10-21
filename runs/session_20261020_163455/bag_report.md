# 采摘会话分析 `session_20261020_163455`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261020_163455/bag`
- 开始时间：2026-10-20 16:34:55
- 结束时间：2026-10-20 17:00:04
- 总时长：1509.4 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow_20261020_163524 | COMPLETED | 2026-10-20 16:35:31 | 2026-10-20 16:37:04 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow_20261020_163524/ledger.json` |
| flow10_163943_r01 | COMPLETED | 2026-10-20 16:39:47 | 2026-10-20 16:41:25 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_163943_r01/ledger.json` |
| flow10_164125_r02 | COMPLETED | 2026-10-20 16:41:29 | 2026-10-20 16:42:59 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_164125_r02/ledger.json` |
| flow10_164259_r03 | COMPLETED | 2026-10-20 16:43:03 | 2026-10-20 16:45:21 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_164259_r03/ledger.json` |
| flow10_164521_r04 | COMPLETED | 2026-10-20 16:45:26 | 2026-10-20 16:47:04 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_164521_r04/ledger.json` |
| flow10_164704_r05 | COMPLETED | 2026-10-20 16:47:08 | 2026-10-20 16:48:58 | skipped=1，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_164704_r05/ledger.json` |
| flow10_164858_r06 | COMPLETED | 2026-10-20 16:49:02 | 2026-10-20 16:50:40 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_164858_r06/ledger.json` |
| flow10_165040_r07 | COMPLETED | 2026-10-20 16:50:45 | 2026-10-20 16:52:33 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_165040_r07/ledger.json` |
| flow10_165233_r08 | COMPLETED | 2026-10-20 16:52:37 | 2026-10-20 16:54:06 | succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_165233_r08/ledger.json` |
| flow10_165406_r09 | COMPLETED | 2026-10-20 16:54:10 | 2026-10-20 16:56:32 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_165406_r09/ledger.json` |
| flow10_165632_r10 | COMPLETED | 2026-10-20 16:56:36 | 2026-10-20 16:58:14 | failed=1，succeeded=2 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_165632_r10/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.73 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 31 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow_20261020_163524

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 17.72 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.325, 6.991, 0.0, 7.108], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 2 | succeeded | 17.06 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.39, 8.112, 0.0, 7.176], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 3 | succeeded | 19.67 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.226, 9.479, 0.0, 8.062], "harvest_confirmed": false, "completion_level": 6} |

### flow10_163943_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | succeeded | 16.49 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.287, 6.77, 0.0, 7.032], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 2 | succeeded | 18.48 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.294, 8.737, 0.0, 7.932], "harvest_confirmed": false, "completion_level": 6} |
| target_8 | 3 | succeeded | 16.48 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.382, 8.002, 0.0, 7.105], "harvest_confirmed": false, "completion_level": 6} |

### flow10_164125_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_11 | 1 | succeeded | 15.27 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.349, 6.645, 0.0, 6.94], "harvest_confirmed": false, "completion_level": 6} |
| target_10 | 2 | succeeded | 19.9 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.335, 9.561, 0.0, 9.042], "harvest_confirmed": false, "completion_level": 6} |
| target_12 | 3 | succeeded | 17.08 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.421, 8.417, 0.0, 7.234], "harvest_confirmed": false, "completion_level": 6} |

### flow10_164259_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_15 | 1 | succeeded | 16.86 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.339, 6.746, 0.0, 6.949], "harvest_confirmed": false, "completion_level": 6} |
| target_14 | 2 | succeeded | 18.28 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.328, 8.233, 0.0, 8.056], "harvest_confirmed": false, "completion_level": 6} |
| target_17 | 3 | skipped | 30.14 | skipped_quality |
| target_16 | 4 | succeeded | 16.08 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.37, 7.265, 0.0, 6.998], "harvest_confirmed": false, "completion_level": 6} |

### flow10_164521_r04

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_20 | 1 | succeeded | 15.06 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.312, 6.603, 0.0, 6.954], "harvest_confirmed": false, "completion_level": 6} |
| target_19 | 2 | succeeded | 18.69 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.333, 9.091, 0.0, 8.144], "harvest_confirmed": false, "completion_level": 6} |
| target_21 | 3 | succeeded | 15.3 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.412, 7.244, 0.001, 6.981], "harvest_confirmed": false, "completion_level": 6} |
| target_22 | 4 | skipped | 2.86 | skipped_unreachable |

### flow10_164704_r05

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_25 | 1 | succeeded | 15.67 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.347, 6.816, 0.001, 7.0], "harvest_confirmed": false, "completion_level": 6} |
| target_24 | 2 | succeeded | 18.46 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.32, 8.271, 0.0, 7.955], "harvest_confirmed": false, "completion_level": 6} |
| target_28 | 3 | skipped | 30.15 | skipped_quality |

### flow10_164858_r06

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_30 | 1 | succeeded | 15.26 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.307, 6.711, 0.0, 7.081], "harvest_confirmed": false, "completion_level": 6} |
| target_29 | 2 | succeeded | 18.08 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.277, 8.35, 0.0, 7.971], "harvest_confirmed": false, "completion_level": 6} |
| target_31 | 3 | succeeded | 18.47 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.37, 9.42, 0.0, 7.576], "harvest_confirmed": false, "completion_level": 6} |

### flow10_165040_r07

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_33 | 1 | succeeded | 18.71 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.344, 6.72, 0.0, 7.007], "harvest_confirmed": false, "completion_level": 6} |
| target_32 | 2 | succeeded | 18.85 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.372, 8.274, 0.0, 7.906], "harvest_confirmed": false, "completion_level": 6} |
| target_34 | 3 | succeeded | 16.68 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.43, 7.536, 0.0, 7.114], "harvest_confirmed": false, "completion_level": 6} |
| target_35 | 4 | skipped | 1.11 | build_failed |

### flow10_165233_r08

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_38 | 1 | succeeded | 15.26 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.264, 6.832, 0.0, 7.002], "harvest_confirmed": false, "completion_level": 6} |
| target_37 | 2 | succeeded | 18.08 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.475, 8.227, 0.0, 7.975], "harvest_confirmed": false, "completion_level": 6} |
| target_39 | 3 | succeeded | 16.86 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.405, 8.29, 0.0, 7.203], "harvest_confirmed": false, "completion_level": 6} |

### flow10_165406_r09

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_41 | 1 | succeeded | 16.07 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.444, 6.76, 0.0, 7.01], "harvest_confirmed": false, "completion_level": 6} |
| target_40 | 2 | succeeded | 18.3 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.314, 8.277, 0.0, 8.135], "harvest_confirmed": false, "completion_level": 6} |
| target_42 | 3 | succeeded | 17.31 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.477, 8.389, 0.0, 7.348], "harvest_confirmed": false, "completion_level": 6} |
| target_43 | 4 | skipped | 30.11 | skipped_quality |

### flow10_165632_r10

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_45 | 1 | succeeded | 18.67 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.646, 6.823, 0.0, 7.071], "harvest_confirmed": false, "completion_level": 6} |
| target_46 | 2 | failed | 16.87 | full_failed |
| target_44 | 3 | succeeded | 17.66 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.307, 8.024, 0.0, 7.868], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow_20261020_163524 | — | 0.71 | — | — | — | — | — | — | — | 0.71 |
| flow_20261020_163524:target_1 | target_1 | — | 1.04 | 2.0 | 0.67 | 6.99 | — | 7.01 | — | 17.71 |
| flow_20261020_163524:target_1 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow_20261020_163524:target_2 | target_2 | — | — | 1.2 | 0.39 | 8.18 | — | 6.96 | — | 16.71 |
| flow_20261020_163524:target_2 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow_20261020_163524:target_0 | target_0 | — | — | 1.6 | 0.2 | 9.39 | — | 8.14 | — | 19.35 |
| flow_20261020_163524:target_0 | — | 6.07 | — | — | — | — | — | — | — | 6.07 |
| flow10_163943_r01 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_163943_r01:target_7 | target_7 | — | — | 2.18 | 0.2 | 6.79 | — | 7.0 | — | 16.17 |
| flow10_163943_r01:target_7 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_163943_r01:target_6 | target_6 | — | — | 1.4 | 0.2 | 8.78 | — | 7.79 | — | 18.18 |
| flow10_163943_r01:target_6 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_163943_r01:target_8 | target_8 | — | — | 0.81 | 0.39 | 8.01 | — | 7.04 | — | 16.22 |
| flow10_163943_r01:target_8 | — | 6.47 | — | — | — | — | — | — | — | 6.47 |
| flow10_164125_r02 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_164125_r02:target_11 | target_11 | — | — | 1.18 | 0.41 | 6.58 | — | 6.8 | — | 14.98 |
| flow10_164125_r02:target_11 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164125_r02:target_10 | target_10 | — | — | 0.62 | 0.39 | 9.59 | — | 9.01 | — | 19.61 |
| flow10_164125_r02:target_10 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164125_r02:target_12 | target_12 | — | — | 0.79 | 0.4 | 8.39 | — | 7.2 | — | 16.8 |
| flow10_164125_r02:target_12 | — | 6.12 | — | — | — | — | — | — | — | 6.12 |
| flow10_164259_r03 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_164259_r03:target_15 | target_15 | — | — | 2.59 | 0.2 | 6.8 | — | 6.98 | — | 16.57 |
| flow10_164259_r03:target_15 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164259_r03:target_14 | target_14 | — | — | 1.4 | 0.2 | 8.19 | — | 8.2 | — | 17.99 |
| flow10_164259_r03:target_14 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| flow10_164259_r03:target_17 | target_17 | — | — | 29.76 | 0.2 | — | — | — | — | 29.99 |
| flow10_164259_r03:target_17 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_164259_r03:target_16 | target_16 | — | — | 1.21 | 0.4 | 7.2 | — | 7.03 | — | 15.84 |
| flow10_164259_r03:target_16 | — | 6.61 | — | — | — | — | — | — | — | 6.61 |
| flow10_164521_r04 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_164521_r04:target_20 | target_20 | — | — | 0.99 | 0.41 | 6.59 | — | 6.83 | — | 14.81 |
| flow10_164521_r04:target_20 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164521_r04:target_19 | target_19 | — | — | 0.81 | 0.39 | 9.0 | — | 8.23 | — | 18.41 |
| flow10_164521_r04:target_19 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow10_164521_r04:target_21 | target_21 | — | — | 0.4 | 0.4 | 7.39 | — | 6.8 | — | 14.98 |
| flow10_164521_r04:target_21 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164521_r04:target_22 | target_22 | — | — | 2.6 | — | — | — | — | — | 2.6 |
| flow10_164521_r04:target_22 | — | 6.08 | — | — | — | — | — | — | — | 6.08 |
| flow10_164704_r05 | — | 0.74 | — | — | — | — | — | — | — | 0.74 |
| flow10_164704_r05:target_25 | target_25 | — | 0.24 | 1.38 | 0.2 | 6.81 | — | 7.01 | — | 15.63 |
| flow10_164704_r05:target_25 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_164704_r05:target_24 | target_24 | — | — | 1.81 | 0.2 | 8.37 | — | 7.82 | — | 18.2 |
| flow10_164704_r05:target_24 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164704_r05:target_28 | target_28 | — | — | 29.85 | — | — | — | — | — | 29.85 |
| flow10_164704_r05:target_28 | — | 6.41 | — | — | — | — | — | — | — | 6.41 |
| flow10_164858_r06 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_164858_r06:target_30 | target_30 | — | — | 0.98 | 0.2 | 6.8 | — | 7.03 | — | 15.02 |
| flow10_164858_r06:target_30 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_164858_r06:target_29 | target_29 | — | — | 1.18 | 0.2 | 8.39 | — | 7.98 | — | 17.75 |
| flow10_164858_r06:target_29 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_164858_r06:target_31 | target_31 | — | — | 1.0 | 0.4 | 9.39 | — | 7.41 | — | 18.22 |
| flow10_164858_r06:target_31 | — | 6.13 | — | — | — | — | — | — | — | 6.13 |
| flow10_165040_r07 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_165040_r07:target_33 | target_33 | — | — | 4.39 | 0.4 | 6.61 | — | 7.02 | — | 18.4 |
| flow10_165040_r07:target_33 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_165040_r07:target_32 | target_32 | — | — | 1.99 | 0.41 | 8.17 | — | 7.99 | — | 18.57 |
| flow10_165040_r07:target_32 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_165040_r07:target_34 | target_34 | — | — | 1.38 | 0.39 | 7.59 | — | 6.99 | — | 16.35 |
| flow10_165040_r07:target_34 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_165040_r07:target_35 | target_35 | — | — | 0.84 | — | — | — | — | — | 0.84 |
| flow10_165040_r07:target_35 | — | 10.15 | — | — | — | — | — | — | — | 10.15 |
| flow10_165233_r08 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_165233_r08:target_38 | target_38 | — | — | 0.86 | 0.37 | 6.76 | — | 6.99 | — | 14.98 |
| flow10_165233_r08:target_38 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_165233_r08:target_37 | target_37 | — | — | 1.2 | 0.38 | 8.23 | — | 8.01 | — | 17.82 |
| flow10_165233_r08:target_37 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_165233_r08:target_39 | target_39 | — | — | 0.6 | 0.6 | 8.22 | — | 7.21 | — | 16.63 |
| flow10_165233_r08:target_39 | — | 6.21 | — | — | — | — | — | — | — | 6.21 |
| flow10_165406_r09 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_165406_r09:target_41 | target_41 | — | — | 1.6 | 0.59 | 6.58 | — | 6.97 | — | 15.73 |
| flow10_165406_r09:target_41 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_165406_r09:target_40 | target_40 | — | — | 1.19 | 0.39 | 8.2 | — | 8.18 | — | 17.96 |
| flow10_165406_r09:target_40 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| flow10_165406_r09:target_42 | target_42 | — | — | 0.81 | 0.82 | 8.37 | — | 7.19 | — | 17.19 |
| flow10_165406_r09:target_42 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_165406_r09:target_43 | target_43 | — | — | 29.85 | — | — | — | — | — | 29.85 |
| flow10_165406_r09:target_43 | — | 6.15 | — | — | — | — | — | — | — | 6.15 |
| flow10_165632_r10 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| flow10_165632_r10:target_45 | target_45 | — | — | 4.0 | 0.59 | 6.78 | — | 7.03 | — | 18.4 |
| flow10_165632_r10:target_45 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_165632_r10:target_46 | target_46 | — | — | 1.0 | 0.39 | 7.81 | — | 7.41 | — | 16.61 |
| flow10_165632_r10:target_46 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_165632_r10:target_44 | target_44 | — | — | 1.2 | 0.2 | 8.01 | — | 7.96 | — | 17.38 |
| flow10_165632_r10:target_44 | — | 6.11 | — | — | — | — | — | — | — | 6.11 |

## 事件统计

- 事件总数：518
- severity AUDIT：134
- severity ERROR：2
- severity INFO：382
- `enables_changed`：8
- `photo_pose_reached`：118
- `recovery_acknowledged`：64
- `recovery_required`：62
- `round_locked`：118
- `target_dispatched`：74
- `target_failed`：2
- `target_skipped`：10
- `target_succeeded`：62

## 感知统计

- 记录帧数：147
- 帧间隔中位数：1.368 s（≈0.73 FPS）
- 目标数范围：3 ~ 5

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08106231689453125
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 2 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |
| target_0 | 1 | 0 | READY | — |
| target_7 | 1 | 0 | READY | — |
| target_6 | 1 | 0 | READY | — |
| target_8 | 1 | 0 | READY | — |
| target_11 | 1 | 0 | READY | — |
| target_10 | 1 | 0 | READY | — |
| target_12 | 1 | 0 | READY | — |
| target_15 | 1 | 0 | READY | — |
| target_14 | 1 | 0 | READY | — |
| target_17 | 0 | 0 | COLLECTING | — |
| target_16 | 1 | 0 | READY | — |
| target_20 | 1 | 0 | READY | — |
| target_19 | 1 | 0 | READY | — |
| target_21 | 1 | 0 | READY | — |
| target_22 | 1 | 0 | READY | — |
| target_25 | 1 | 0 | READY | — |
| target_24 | 1 | 0 | READY | — |
| target_28 | 0 | 0 | COLLECTING | — |
| target_30 | 1 | 0 | READY | — |
| target_29 | 1 | 0 | READY | — |
| target_31 | 1 | 0 | READY | — |
| target_33 | 1 | 0 | READY | — |
| target_32 | 1 | 0 | READY | — |
| target_34 | 1 | 0 | READY | — |
| target_35 | 1 | 0 | COLLECTING | — |
| target_38 | 1 | 0 | READY | — |
| target_37 | 1 | 0 | READY | — |
| target_39 | 1 | 0 | READY | — |
| target_41 | 1 | 0 | READY | — |
| target_40 | 2 | 0 | READY | — |
| target_42 | 1 | 0 | READY | — |
| target_43 | 0 | 0 | COLLECTING | — |
| target_45 | 1 | 0 | READY | — |
| target_46 | 1 | 0 | READY | — |
| target_44 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.291,-0.569,0.627 | — | — | — |
| target_2 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.238,-0.562,0.652 | 0.441,-0.511,0.722 | — | — |
| target_0 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.443,-0.504,0.668 | 0.441,-0.511,0.722 | 0.431,-0.485,0.648 | 0.433,-0.492,0.677 |
| target_7 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.569,0.627 | — | — | — |
| target_6 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.222,-0.568,0.688 | — | — |
| target_8 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.562,0.652 | 0.222,-0.568,0.688 | 0.240,-0.543,0.641 | 0.232,-0.558,0.666 |
| target_11 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.569,0.627 | — | — | — |
| target_10 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.222,-0.568,0.687 | — | — |
| target_12 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.562,0.652 | 0.222,-0.568,0.687 | 0.240,-0.543,0.641 | 0.232,-0.557,0.666 |
| target_15 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.569,0.627 | — | — | — |
| target_14 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.573,-0.505,0.724 | — | — |
| target_17 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.571,-0.506,0.707 | 0.222,-0.568,0.688 | — | — |
| target_16 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.562,0.652 | 0.222,-0.568,0.688 | 0.240,-0.542,0.642 | 0.232,-0.557,0.667 |
| target_20 | lock | 否 | 工具档关；精化几何可达性未通过: no_ik | 0.290,-0.569,0.627 | — | — | — |
| target_19 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.222,-0.568,0.688 | — | — |
| target_21 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.238,-0.562,0.653 | 0.684,-0.851,0.531 | — | — |
| target_22 | lock | 是 | 工具档关；尚未到达预抓取 · SKIPPED_UNREACHABLE | 0.700,-0.849,0.530 | 0.684,-0.851,0.531 | 0.743,-0.843,0.525 | 0.713,-0.847,0.528 |
| target_25 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000074s timeout=30.000000s | 0.290,-0.569,0.627 | — | — | — |
| target_24 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.573,-0.505,0.722 | — | — |
| target_28 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.571,-0.506,0.708 | 0.573,-0.504,0.724 | — | — |
| target_30 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.569,0.627 | — | — | — |
| target_29 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.504,0.668 | 0.222,-0.568,0.687 | — | — |
| target_31 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.562,0.652 | 0.222,-0.568,0.687 | 0.240,-0.542,0.642 | 0.232,-0.557,0.667 |
| target_33 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.569,0.626 | — | — | — |
| target_32 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.504,0.668 | 0.222,-0.568,0.688 | — | — |
| target_34 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.562,0.653 | 0.685,-0.851,0.531 | — | — |
| target_35 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.720,-0.847,0.530 | 0.685,-0.851,0.531 | — | — |
| target_38 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.290,-0.568,0.626 | — | — | — |
| target_37 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.504,0.668 | 0.222,-0.568,0.688 | — | — |
| target_39 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.239,-0.564,0.653 | 0.222,-0.568,0.688 | 0.240,-0.542,0.642 | 0.232,-0.557,0.667 |
| target_41 | lock | 否 | 工具档关；decision_snapshot_unavailable: 同目标模型与许可未在时限内到齐或已过期 elapsed=30.000307s timeout=30.000000s | 0.290,-0.571,0.628 | — | — | — |
| target_40 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.442,-0.508,0.668 | 0.224,-0.571,0.687 | — | — |
| target_42 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.238,-0.565,0.653 | 0.224,-0.571,0.687 | 0.240,-0.544,0.648 | 0.232,-0.561,0.671 |
| target_43 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.682,-0.853,0.540 | — | — | — |
| target_45 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.291,-0.571,0.628 | — | — | — |
| target_46 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.238,-0.565,0.652 | 0.441,-0.516,0.721 | — | — |
| target_44 | lock | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.442,-0.509,0.667 | 0.441,-0.516,0.721 | 0.432,-0.488,0.648 | 0.434,-0.495,0.677 |

## 运行性能统计

- 性能采样条数：1406
- CPU %：均值 50.59 / 峰值 91.8
- 内存 %：均值 51.43 / 峰值 53.3
- GPU 利用率 %：均值 24.7 / 峰值 49.0
- GPU 显存 MB：均值 1688.24 / 峰值 2010.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：4458
- 路径长：39.5965 m
- 起止弦：0.0018 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.383 m
- Z 范围：0.5011 ~ 0.7664 m（Δz=0.0001）
