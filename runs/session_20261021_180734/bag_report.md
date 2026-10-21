# 采摘会话分析 `session_20261021_180734`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_180734/bag`
- 开始时间：2026-10-21 18:07:34
- 结束时间：2026-10-21 18:09:56
- 总时长：141.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_3adda5300eca | COMPLETED | 2026-10-21 18:07:56 | 2026-10-21 18:08:23 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_3adda5300eca/ledger.json` |
| round_1263d2cf2af0 | COMPLETED | 2026-10-21 18:08:41 | 2026-10-21 18:09:07 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_1263d2cf2af0/ledger.json` |
| round_684064110ef8 | COMPLETED | 2026-10-21 18:09:23 | 2026-10-21 18:09:53 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_684064110ef8/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.05 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 3 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_3adda5300eca

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 19.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.255, 0.269, 10.414, 0.0, 8.503], "harvest_confirmed": false, "completion_level": 6} |

### round_1263d2cf2af0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 3 | succeeded | 19.89 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.008, 0.571, 10.03, 0.0, 9.075], "harvest_confirmed": false, "completion_level": 6} |

### round_684064110ef8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 22.86 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.253, 0.353, 11.838, 0.0, 10.308], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_3adda5300eca | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_3adda5300eca:target_0 | target_0 | — | 0.22 | 0.43 | 0.2 | 10.14 | — | 8.57 | — | 19.57 |
| round_1263d2cf2af0 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_1263d2cf2af0:target_5 | target_5 | — | 0.22 | — | 0.42 | 10.0 | — | 9.22 | — | 19.86 |
| round_684064110ef8 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| round_684064110ef8:target_8 | target_8 | — | 0.22 | 0.2 | 0.39 | 11.8 | — | 10.19 | — | 22.79 |

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
- 帧间隔中位数：21.498 s（≈0.05 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_8
- captured_views：1
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.0782012939453125
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：bag_axis_envelope_ill_conditioned

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.607,0.673 | 0.568,-0.608,0.720 | — | — |
| target_6 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.209,-0.743,0.646 | — | — | — |
| target_7 | done | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.679 | — | — | — |
| target_5 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.298,-0.717,0.627 | 0.299,-0.731,0.672 | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.571,-0.608,0.672 | 0.569,-0.609,0.729 | — | — |

## 运行性能统计

- 性能采样条数：132
- CPU %：均值 28.42 / 峰值 45.0
- 内存 %：均值 41.25 / 峰值 42.9
- GPU 利用率 %：均值 19.67 / 峰值 58.0
- GPU 显存 MB：均值 2619.5 / 峰值 2709.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：479
- 路径长：4.5187 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：78.293（直线≈1）
- 相对弦最大偏离：0.4922 m
- Z 范围：0.4997 ~ 0.7696 m（Δz=0.0019）
