# 采摘会话分析 `session_20261022_090028`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_090028/bag`
- 开始时间：2026-10-22 09:00:28
- 结束时间：2026-10-22 09:03:30
- 总时长：181.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_21bd4e3c6f69 | COMPLETED | 2026-10-22 09:00:56 | 2026-10-22 09:01:21 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_21bd4e3c6f69/ledger.json` |
| round_b1a584c35010 | COMPLETED | 2026-10-22 09:01:40 | 2026-10-22 09:02:07 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_b1a584c35010/ledger.json` |
| round_df7a8413f74b | COMPLETED | 2026-10-22 09:02:23 | 2026-10-22 09:02:49 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_df7a8413f74b/ledger.json` |
| round_78470d8f8775 | COMPLETED | 2026-10-22 09:03:05 | 2026-10-22 09:03:28 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_78470d8f8775/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.68 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 4 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_21bd4e3c6f69

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 18.45 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.209, 0.371, 9.408, 0.0, 8.334], "harvest_confirmed": false, "completion_level": 6} |

### round_b1a584c35010

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 2 | succeeded | 20.07 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.006, 0.552, 10.076, 0.0, 9.268], "harvest_confirmed": false, "completion_level": 6} |

### round_df7a8413f74b

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 3 | succeeded | 19.46 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.011, 0.697, 9.554, 0.0, 8.998], "harvest_confirmed": false, "completion_level": 6} |

### round_78470d8f8775

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_14 | 4 | succeeded | 16.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.013, 0.543, 8.171, 0.0, 7.847], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_21bd4e3c6f69 | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| round_21bd4e3c6f69:target_0 | target_0 | — | 0.22 | 0.21 | 0.2 | 9.4 | — | 8.36 | — | 18.39 |
| round_b1a584c35010 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_b1a584c35010:target_4 | target_4 | — | 0.22 | — | 0.4 | 10.19 | — | 9.24 | — | 20.04 |
| round_df7a8413f74b | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_df7a8413f74b:target_9 | target_9 | — | 0.23 | — | 0.61 | 9.61 | — | 9.0 | — | 19.45 |
| round_78470d8f8775 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_78470d8f8775:target_14 | target_14 | — | 0.22 | — | 0.41 | 8.19 | — | 7.77 | — | 16.59 |

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

- 记录帧数：8
- 帧间隔中位数：1.469 s（≈0.68 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_14
- captured_views：1
- rejected_views：10
- tf_failures：0
- tf_latency_ms：0.07581710815429688
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
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.673 | 0.569,-0.609,0.719 | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.674 | 0.404,-0.665,0.717 | — | — |
| target_4 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.655,0.663 | 0.404,-0.665,0.717 | — | — |
| target_8 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.673 | 0.299,-0.733,0.672 | — | — |
| target_9 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.299,-0.717,0.627 | 0.299,-0.733,0.672 | — | — |
| target_12 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.672 | — | — | — |
| target_14 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.746,0.645 | 0.193,-0.758,0.683 | — | — |

## 运行性能统计

- 性能采样条数：172
- CPU %：均值 25.96 / 峰值 44.5
- 内存 %：均值 31.04 / 峰值 32.4
- GPU 利用率 %：均值 20.34 / 峰值 46.0
- GPU 显存 MB：均值 1415.51 / 峰值 1456.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：635
- 路径长：6.1172 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：106.001（直线≈1）
- 相对弦最大偏离：0.532 m
- Z 范围：0.5017 ~ 0.7543 m（Δz=0.0019）
