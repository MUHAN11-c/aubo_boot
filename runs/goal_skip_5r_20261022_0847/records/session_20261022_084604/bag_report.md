# 采摘会话分析 `session_20261022_084604`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_084604/bag`
- 开始时间：2026-10-22 08:46:05
- 结束时间：2026-10-22 08:49:53
- 总时长：228.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_8199a93b9ff3 | COMPLETED | 2026-10-22 08:46:32 | 2026-10-22 08:46:58 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_8199a93b9ff3/ledger.json` |
| round_225b29e9c2bc | COMPLETED | 2026-10-22 08:47:14 | 2026-10-22 08:47:41 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_225b29e9c2bc/ledger.json` |
| round_affb0717d3e5 | COMPLETED | 2026-10-22 08:47:55 | 2026-10-22 08:48:23 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_affb0717d3e5/ledger.json` |
| round_7f8976838f10 | COMPLETED | 2026-10-22 08:48:39 | 2026-10-22 08:49:02 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_7f8976838f10/ledger.json` |
| round_84da47c99eae | COMPLETED | 2026-10-22 08:49:22 | 2026-10-22 08:49:51 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_84da47c99eae/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.48 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_8199a93b9ff3

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | succeeded | 19.5 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 0.594, 9.605, 0.0, 9.056], "harvest_confirmed": false, "completion_level": 6} |

### round_225b29e9c2bc

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 20.48 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.225, 0.405, 10.38, 0.0, 9.42], "harvest_confirmed": false, "completion_level": 6} |

### round_affb0717d3e5

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 21.89 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.259, 0.558, 10.997, 0.0, 10.015], "harvest_confirmed": false, "completion_level": 6} |

### round_7f8976838f10

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_15 | 4 | succeeded | 16.85 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.006, 0.625, 8.268, 0.0, 7.805], "harvest_confirmed": false, "completion_level": 6} |

### round_84da47c99eae

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_16 | 1 | succeeded | 21.9 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.258, 0.4, 10.982, 0.0, 9.999], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_8199a93b9ff3 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| round_8199a93b9ff3:target_0 | target_0 | — | 0.24 | — | 0.42 | 9.59 | — | 9.24 | — | 19.48 |
| round_225b29e9c2bc | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| round_225b29e9c2bc:target_5 | target_5 | — | 0.22 | 0.2 | 0.58 | 10.04 | — | 9.4 | — | 20.42 |
| round_affb0717d3e5 | — | 0.17 | — | — | — | — | — | — | — | 0.17 |
| round_affb0717d3e5:target_8 | target_8 | — | 0.23 | 0.35 | 0.44 | 10.81 | — | 9.99 | — | 21.82 |
| round_7f8976838f10 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_7f8976838f10:target_15 | target_15 | — | 0.23 | — | 0.58 | 8.21 | — | 7.81 | — | 16.83 |
| round_84da47c99eae | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_84da47c99eae:target_16 | target_16 | — | 0.22 | 0.2 | 0.39 | 11.01 | — | 10.05 | — | 21.87 |

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

- 记录帧数：10
- 帧间隔中位数：2.069 s（≈0.48 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_16
- captured_views：1
- rejected_views：14
- tf_failures：0
- tf_latency_ms：0.12326240539550781
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：bag_axis_envelope_ill_conditioned

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_5 | 2 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_15 | 1 | 0 | COLLECTING | — |
| target_16 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_2 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.569,-0.608,0.673 | — | — | — |
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.657,0.663 | 0.404,-0.666,0.717 | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.673 | 0.569,-0.608,0.730 | — | — |
| target_11 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | — | — | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.683 | — | — | — |
| target_12 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.683 | 0.192,-0.759,0.683 | — | — |
| target_15 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.208,-0.747,0.645 | 0.192,-0.759,0.683 | — | — |
| target_16 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.672 | 0.570,-0.608,0.729 | — | — |

## 运行性能统计

- 性能采样条数：216
- CPU %：均值 26.51 / 峰值 43.1
- 内存 %：均值 30.27 / 峰值 31.7
- GPU 利用率 %：均值 20.27 / 峰值 49.0
- GPU 显存 MB：均值 1499.76 / 峰值 1539.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：803
- 路径长：7.6032 m
- 起止弦：0.0575 m
- 绕行比（路径/弦）：132.342（直线≈1）
- 相对弦最大偏离：0.5335 m
- Z 范围：0.5245 ~ 0.7698 m（Δz=0.0019）
