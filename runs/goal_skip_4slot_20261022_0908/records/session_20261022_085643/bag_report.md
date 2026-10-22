# 采摘会话分析 `session_20261022_085643`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_085643/bag`
- 开始时间：2026-10-22 08:56:43
- 结束时间：2026-10-22 08:59:54
- 总时长：190.9 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_483368387f17 | COMPLETED | 2026-10-22 08:57:10 | 2026-10-22 08:57:38 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_483368387f17/ledger.json` |
| round_e1203b951219 | COMPLETED | 2026-10-22 08:57:57 | 2026-10-22 08:58:22 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_e1203b951219/ledger.json` |
| round_476b6222ece5 | COMPLETED | 2026-10-22 08:58:42 | 2026-10-22 08:59:11 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_476b6222ece5/ledger.json` |
| round_a9a0b65b66e7 | COMPLETED | 2026-10-22 08:59:28 | 2026-10-22 08:59:52 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_a9a0b65b66e7/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.04 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 4 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_483368387f17

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 21.87 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.255, 0.275, 11.052, 0.0, 10.146], "harvest_confirmed": false, "completion_level": 6} |

### round_e1203b951219

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 2 | succeeded | 19.48 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 0.533, 9.738, 0.0, 9.108], "harvest_confirmed": false, "completion_level": 6} |

### round_476b6222ece5

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 3 | succeeded | 22.26 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.62, 11.086, 0.0, 10.391], "harvest_confirmed": false, "completion_level": 6} |

### round_a9a0b65b66e7

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_14 | 4 | succeeded | 16.86 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.006, 0.695, 8.063, 0.0, 7.977], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_483368387f17 | — | 0.17 | — | — | — | — | — | — | — | 0.17 |
| round_483368387f17:target_0 | target_0 | — | 0.23 | 0.23 | 0.2 | 10.99 | — | 10.2 | — | 21.82 |
| round_e1203b951219 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| round_e1203b951219:target_4 | target_4 | — | 0.22 | — | 0.4 | 9.84 | — | 9.01 | — | 19.48 |
| round_476b6222ece5 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_476b6222ece5:target_9 | target_9 | — | 0.23 | — | 0.59 | 11.03 | — | 10.4 | — | 22.25 |
| round_a9a0b65b66e7 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_a9a0b65b66e7:target_14 | target_14 | — | 0.22 | — | 0.6 | 8.01 | — | 7.99 | — | 16.82 |

## 事件统计

- 事件总数：56
- severity AUDIT：24
- severity INFO：32
- `enables_changed`：8
- `photo_pose_reached`：8
- `recovery_acknowledged`：8
- `recovery_required`：8
- `round_locked`：8
- `target_dispatched`：8
- `target_succeeded`：8

## 感知统计

- 记录帧数：7
- 帧间隔中位数：23.301 s（≈0.04 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_14
- captured_views：1
- rejected_views：6
- tf_failures：0
- tf_latency_ms：0.19478797912597656
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_4 | 1 | 0 | COLLECTING | — |
| target_9 | 1 | 0 | COLLECTING | — |
| target_14 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.568,-0.610,0.720 | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.676 | 0.404,-0.666,0.717 | — | — |
| target_4 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.657,0.663 | 0.404,-0.666,0.717 | — | — |
| target_8 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.676 | 0.300,-0.734,0.671 | — | — |
| target_9 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.299,-0.716,0.627 | 0.300,-0.734,0.671 | — | — |
| target_12 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.674 | 0.192,-0.758,0.683 | — | — |
| target_14 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.746,0.645 | 0.192,-0.758,0.683 | — | — |

## 运行性能统计

- 性能采样条数：181
- CPU %：均值 24.54 / 峰值 41.3
- 内存 %：均值 30.75 / 峰值 32.2
- GPU 利用率 %：均值 19.78 / 峰值 48.0
- GPU 显存 MB：均值 1436.33 / 峰值 1480.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：641
- 路径长：6.1485 m
- 起止弦：0.0575 m
- 绕行比（路径/弦）：106.861（直线≈1）
- 相对弦最大偏离：0.5323 m
- Z 范围：0.5028 ~ 0.7689 m（Δz=0.0019）
