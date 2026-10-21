# 采摘会话分析 `session_20261021_172018`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_172018/bag`
- 开始时间：2026-10-21 17:20:18
- 结束时间：2026-10-21 17:25:13
- 总时长：294.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_f976c8261825 | COMPLETED | 2026-10-21 17:20:51 | 2026-10-21 17:21:25 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f976c8261825/ledger.json` |
| round_d8b5ee8e4fae | COMPLETED | 2026-10-21 17:21:59 | 2026-10-21 17:22:34 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_d8b5ee8e4fae/ledger.json` |
| round_5a98485b58f6 | COMPLETED | 2026-10-21 17:23:08 | 2026-10-21 17:23:48 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_5a98485b58f6/ledger.json` |
| round_bd8232ff4a6c | COMPLETED | 2026-10-21 17:24:05 | 2026-10-21 17:24:31 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_bd8232ff4a6c/ledger.json` |
| round_1b822712df9e | COMPLETED | 2026-10-21 17:24:49 | 2026-10-21 17:25:12 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_1b822712df9e/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.02 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_f976c8261825

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 16.67 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 1.131, 8.469, 0.0, 7.002], "harvest_confirmed": false, "completion_level": 6} |

### round_d8b5ee8e4fae

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 2 | succeeded | 15.47 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 1.775, 6.998, 0.0, 6.501], "harvest_confirmed": false, "completion_level": 6} |

### round_5a98485b58f6

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 17.27 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 1.79, 8.324, 0.0, 6.988], "harvest_confirmed": false, "completion_level": 6} |

### round_bd8232ff4a6c

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_17 | 4 | succeeded | 16.43 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 2.005, 7.569, 0.0, 6.681], "harvest_confirmed": false, "completion_level": 6} |

### round_1b822712df9e

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_18 | 4 | succeeded | 16.24 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 1.777, 7.379, 0.0, 6.915], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_f976c8261825 | — | 2.68 | — | — | — | — | — | — | — | 2.68 |
| round_f976c8261825:target_0 | target_0 | — | — | — | 0.99 | 8.39 | — | 6.98 | — | 16.36 |
| round_d8b5ee8e4fae | — | 4.49 | — | — | — | — | — | — | — | 4.49 |
| round_d8b5ee8e4fae:target_5 | target_5 | — | — | — | 1.6 | 6.99 | — | 6.61 | — | 15.22 |
| round_5a98485b58f6 | — | 7.21 | — | — | — | — | — | — | — | 7.21 |
| round_5a98485b58f6:target_8 | target_8 | — | — | — | 1.6 | 8.41 | — | 7.06 | — | 17.07 |
| round_bd8232ff4a6c | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| round_bd8232ff4a6c:target_17 | target_17 | — | — | — | 2.0 | 7.36 | — | 6.78 | — | 16.15 |
| round_1b822712df9e | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| round_1b822712df9e:target_18 | target_18 | — | — | — | 1.61 | 7.37 | — | 6.99 | — | 15.97 |

## 事件统计

- 事件总数：79
- severity AUDIT：27
- severity INFO：52
- `enables_changed`：8
- `photo_pose_reached`：16
- `recovery_acknowledged`：10
- `recovery_required`：9
- `round_locked`：16
- `target_dispatched`：10
- `target_succeeded`：10

## 感知统计

- 记录帧数：7
- 帧间隔中位数：42.762 s（≈0.02 FPS）
- 目标数范围：4 ~ 5

## 重建关键指标终值

- state：COLLECTING
- target_id：target_18
- captured_views：1
- rejected_views：16
- tf_failures：0
- tf_latency_ms：0.13399124145507812
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_17 | 1 | 0 | COLLECTING | — |
| target_18 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.673 | 0.567,-0.609,0.723 | — | — |
| target_4 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.679 | 0.403,-0.666,0.718 | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.406,-0.656,0.664 | 0.403,-0.666,0.718 | — | — |
| target_11 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.402,-0.655,0.664 | — | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.673 | 0.568,-0.609,0.725 | — | — |
| target_14 | permit | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.673 | — | — | — |
| target_17 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.743,0.646 | 0.193,-0.755,0.682 | — | — |
| target_19 | permit | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.571,-0.608,0.672 | — | — | — |
| target_18 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.743,0.646 | 0.192,-0.755,0.681 | — | — |

## 运行性能统计

- 性能采样条数：273
- CPU %：均值 42.23 / 峰值 67.2
- 内存 %：均值 41.91 / 峰值 44.1

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：672
- 路径长：8.087 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：140.119（直线≈1）
- 相对弦最大偏离：0.5166 m
- Z 范围：0.5261 ~ 0.7017 m（Δz=0.0019）
