# 采摘会话分析 `session_20261022_092817`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_092817/bag`
- 开始时间：2026-10-22 09:28:17
- 结束时间：2026-10-22 09:35:34
- 总时长：437.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_24ca9205628e | COMPLETED | 2026-10-22 09:28:45 | 2026-10-22 09:29:12 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_24ca9205628e/ledger.json` |
| round_881e9333b2ca | COMPLETED | 2026-10-22 09:29:27 | 2026-10-22 09:29:53 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_881e9333b2ca/ledger.json` |
| round_4a3b430c4735 | COMPLETED | 2026-10-22 09:30:10 | 2026-10-22 09:30:36 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_4a3b430c4735/ledger.json` |
| round_95d2c7ce9608 | COMPLETED | 2026-10-22 09:30:52 | 2026-10-22 09:31:15 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_95d2c7ce9608/ledger.json` |
| round_bb7d9e364648 | COMPLETED | 2026-10-22 09:31:36 | 2026-10-22 09:32:02 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_bb7d9e364648/ledger.json` |
| round_c589974622a5 | COMPLETED | 2026-10-22 09:32:18 | 2026-10-22 09:32:44 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_c589974622a5/ledger.json` |
| round_58b46411a8ea | COMPLETED | 2026-10-22 09:33:01 | 2026-10-22 09:33:24 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_58b46411a8ea/ledger.json` |
| round_148aa819c127 | COMPLETED | 2026-10-22 09:33:42 | 2026-10-22 09:34:05 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_148aa819c127/ledger.json` |
| round_28ae3f8c677e | COMPLETED | 2026-10-22 09:34:21 | 2026-10-22 09:34:46 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_28ae3f8c677e/ledger.json` |
| round_d3f068d13645 | COMPLETED | 2026-10-22 09:35:06 | 2026-10-22 09:35:32 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_d3f068d13645/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.02 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 4 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_24ca9205628e

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 20.85 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.254, 0.363, 10.568, 0.0, 9.547], "harvest_confirmed": false, "completion_level": 6} |

### round_881e9333b2ca

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | succeeded | 19.49 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 0.575, 9.59, 0.0, 9.148], "harvest_confirmed": false, "completion_level": 6} |

### round_4a3b430c4735

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 3 | succeeded | 19.07 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.596, 9.348, 0.0, 9.038], "harvest_confirmed": false, "completion_level": 6} |

### round_95d2c7ce9608

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 3 | succeeded | 16.46 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 0.761, 7.909, 0.0, 7.74], "harvest_confirmed": false, "completion_level": 6} |

### round_bb7d9e364648

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 19.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.253, 0.406, 10.113, 0.0, 8.739], "harvest_confirmed": false, "completion_level": 6} |

### round_c589974622a5

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | succeeded | 19.48 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.665, 9.525, 0.0, 9.098], "harvest_confirmed": false, "completion_level": 6} |

### round_58b46411a8ea

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 3 | succeeded | 17.05 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 0.596, 8.495, 0.0, 7.915], "harvest_confirmed": false, "completion_level": 6} |

### round_148aa819c127

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 4 | succeeded | 16.26 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.552, 7.91, 0.0, 7.745], "harvest_confirmed": false, "completion_level": 6} |

### round_28ae3f8c677e

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 17.86 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.704, 8.948, 0.0, 8.024], "harvest_confirmed": false, "completion_level": 6} |

### round_d3f068d13645

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | succeeded | 19.87 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.6, 9.705, 0.0, 9.333], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_24ca9205628e | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_24ca9205628e:target_0 | target_0 | — | 0.21 | 0.2 | 0.48 | 10.32 | — | 9.58 | — | 20.78 |
| round_881e9333b2ca | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_881e9333b2ca:target_1 | target_1 | — | 0.23 | — | 0.41 | 9.58 | — | 9.19 | — | 19.41 |
| round_4a3b430c4735 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_4a3b430c4735:target_2 | target_2 | — | 0.22 | — | 0.59 | 9.2 | — | 9.02 | — | 19.04 |
| round_95d2c7ce9608 | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| round_95d2c7ce9608:target_2 | target_2 | — | 0.21 | — | 0.59 | 8.02 | — | 7.6 | — | 16.42 |
| round_bb7d9e364648 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_bb7d9e364648:target_0 | target_0 | — | 0.23 | 0.2 | 0.39 | 10.01 | — | 8.77 | — | 19.61 |
| round_c589974622a5 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_c589974622a5:target_1 | target_1 | — | 0.24 | — | 0.58 | 9.39 | — | 9.21 | — | 19.42 |
| round_58b46411a8ea | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_58b46411a8ea:target_2 | target_2 | — | 0.22 | — | 0.38 | 8.61 | — | 7.8 | — | 17.01 |
| round_148aa819c127 | — | 0.17 | — | — | — | — | — | — | — | 0.17 |
| round_148aa819c127:target_3 | target_3 | — | 0.22 | — | 0.42 | 7.99 | — | 7.61 | — | 16.23 |
| round_28ae3f8c677e | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_28ae3f8c677e:target_0 | target_0 | — | 0.24 | — | 0.6 | 9.01 | — | 7.98 | — | 17.84 |
| round_d3f068d13645 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_d3f068d13645:target_1 | target_1 | — | 0.22 | — | 0.6 | 9.59 | — | 9.38 | — | 19.81 |

## 事件统计

- 事件总数：128
- severity AUDIT：48
- severity INFO：80
- `enables_changed`：8
- `photo_pose_reached`：20
- `recovery_acknowledged`：20
- `recovery_required`：20
- `round_locked`：20
- `target_dispatched`：20
- `target_succeeded`：20

## 感知统计

- 记录帧数：16
- 帧间隔中位数：40.726 s（≈0.02 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：1
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08630752563476562
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |
| target_2 | 1 | 0 | COLLECTING | — |
| target_3 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.674 | 0.404,-0.665,0.718 | — | — |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.406,-0.655,0.664 | 0.404,-0.665,0.718 | — | — |
| target_2 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.208,-0.746,0.646 | 0.193,-0.756,0.683 | — | — |
| target_3 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.209,-0.744,0.646 | 0.194,-0.755,0.682 | — | — |

## 运行性能统计

- 性能采样条数：409
- CPU %：均值 30.42 / 峰值 45.1
- 内存 %：均值 36.98 / 峰值 41.0
- GPU 利用率 %：均值 21.14 / 峰值 46.0
- GPU 显存 MB：均值 1857.18 / 峰值 2066.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1612
- 路径长：15.4063 m
- 起止弦：0.0554 m
- 绕行比（路径/弦）：277.958（直线≈1）
- 相对弦最大偏离：0.5301 m
- Z 范围：0.5011 ~ 0.765 m（Δz=0.0018）
