# 采摘会话分析 `session_20261022_101640`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_101640/bag`
- 开始时间：2026-10-22 10:16:40
- 结束时间：2026-10-22 10:19:12
- 总时长：151.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_448e26425b6d | COMPLETED | 2026-10-22 10:17:07 | 2026-10-22 10:17:35 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_448e26425b6d/ledger.json` |
| round_9c9636f49fbd | COMPLETED | 2026-10-22 10:17:56 | 2026-10-22 10:18:26 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_9c9636f49fbd/ledger.json` |
| round_7b20d87a6066 | COMPLETED | 2026-10-22 10:18:42 | 2026-10-22 10:19:10 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_7b20d87a6066/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.04 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 2 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_448e26425b6d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | succeeded | 21.29 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.256, 0.414, 10.965, 0.0, 9.402], "harvest_confirmed": false, "completion_level": 6} |

### round_9c9636f49fbd

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | succeeded | 21.68 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.256, 0.471, 11.201, 0.0, 9.542], "harvest_confirmed": false, "completion_level": 6} |

### round_7b20d87a6066

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | succeeded | 20.69 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.212, 0.798, 10.56, 0.0, 8.982], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_448e26425b6d | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| round_448e26425b6d:target_0 | target_0 | — | 0.22 | 0.21 | 0.6 | 10.82 | — | 9.41 | — | 21.23 |
| round_9c9636f49fbd | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_9c9636f49fbd:target_3 | target_3 | — | 0.22 | 0.2 | 0.4 | 11.21 | — | 9.58 | — | 21.6 |
| round_7b20d87a6066 | — | 0.15 | — | — | — | — | — | — | — | 0.15 |
| round_7b20d87a6066:target_3 | target_3 | — | 0.23 | 0.2 | 0.81 | 10.41 | — | 8.95 | — | 20.61 |

## 事件统计

- 事件总数：44
- severity AUDIT：20
- severity INFO：24
- `enables_changed`：8
- `photo_pose_reached`：6
- `recovery_acknowledged`：6
- `recovery_required`：6
- `round_locked`：6
- `target_dispatched`：6
- `target_succeeded`：6

## 感知统计

- 记录帧数：5
- 帧间隔中位数：23.333 s（≈0.04 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_3
- captured_views：1
- rejected_views：15
- tf_failures：0
- tf_latency_ms：0.08535385131835938
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：bag_axis_envelope_ill_conditioned

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_3 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.568,-0.609,0.674 | 0.568,-0.610,0.723 | — | — |
| target_3 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.568,-0.608,0.674 | 0.569,-0.610,0.723 | — | — |

## 运行性能统计

- 性能采样条数：140
- CPU %：均值 33.78 / 峰值 50.2
- 内存 %：均值 39.92 / 峰值 42.0
- GPU 利用率 %：均值 27.58 / 峰值 57.0
- GPU 显存 MB：均值 2445.7 / 峰值 2608.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：482
- 路径长：4.4079 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：76.373（直线≈1）
- 相对弦最大偏离：0.4476 m
- Z 范围：0.5421 ~ 0.7606 m（Δz=0.0019）
