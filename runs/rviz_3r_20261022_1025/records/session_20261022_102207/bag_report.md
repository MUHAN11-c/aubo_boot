# 采摘会话分析 `session_20261022_102207`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_102207/bag`
- 开始时间：2026-10-22 10:22:07
- 结束时间：2026-10-22 10:24:39
- 总时长：151.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_0e883620d225 | COMPLETED | 2026-10-22 10:22:35 | 2026-10-22 10:23:03 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_0e883620d225/ledger.json` |
| round_d3cf7df2fa26 | COMPLETED | 2026-10-22 10:23:26 | 2026-10-22 10:23:53 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_d3cf7df2fa26/ledger.json` |
| round_388059070fdc | COMPLETED | 2026-10-22 10:24:10 | 2026-10-22 10:24:37 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_388059070fdc/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.65 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 2 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_0e883620d225

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 21.08 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.26, 0.413, 10.995, 0.0, 9.242], "harvest_confirmed": false, "completion_level": 6} |

### round_d3cf7df2fa26

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 3 | succeeded | 19.65 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.675, 9.626, 0.0, 9.158], "harvest_confirmed": false, "completion_level": 6} |

### round_388059070fdc

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 3 | succeeded | 20.68 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.744, 10.411, 0.0, 9.404], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_0e883620d225 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_0e883620d225:target_0 | target_0 | — | 0.24 | 0.2 | 0.61 | 10.76 | — | 9.24 | — | 21.03 |
| round_d3cf7df2fa26 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_d3cf7df2fa26:target_1 | target_1 | — | 0.22 | — | 0.59 | 9.61 | — | 9.2 | — | 19.63 |
| round_388059070fdc | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| round_388059070fdc:target_1 | target_1 | — | 0.25 | — | 0.62 | 10.41 | — | 9.43 | — | 20.73 |

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

- 记录帧数：6
- 帧间隔中位数：1.535 s（≈0.65 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：1
- rejected_views：26
- tf_failures：0
- tf_latency_ms：0.08487701416015625
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：bag_axis_silhouette_unobservable

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.677 | 0.301,-0.731,0.672 | — | — |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.299,-0.715,0.627 | 0.301,-0.731,0.672 | — | — |

## 运行性能统计

- 性能采样条数：140
- CPU %：均值 34.87 / 峰值 48.7
- 内存 %：均值 40.73 / 峰值 42.6
- GPU 利用率 %：均值 28.65 / 峰值 55.0
- GPU 显存 MB：均值 2611.31 / 峰值 2814.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：480
- 路径长：4.4752 m
- 起止弦：0.0552 m
- 绕行比（路径/弦）：81.107（直线≈1）
- 相对弦最大偏离：0.4954 m
- Z 范围：0.5029 ~ 0.7557 m（Δz=0.0018）
