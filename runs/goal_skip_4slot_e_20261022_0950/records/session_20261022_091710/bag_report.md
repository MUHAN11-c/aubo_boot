# 采摘会话分析 `session_20261022_091710`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_091710/bag`
- 开始时间：2026-10-22 09:17:10
- 结束时间：2026-10-22 09:20:09
- 总时长：178.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_09f0c21a864d | COMPLETED | 2026-10-22 09:17:38 | 2026-10-22 09:18:04 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_09f0c21a864d/ledger.json` |
| round_8ffb285f4b20 | COMPLETED | 2026-10-22 09:18:19 | 2026-10-22 09:18:45 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_8ffb285f4b20/ledger.json` |
| round_4bdff0b594df | COMPLETED | 2026-10-22 09:19:01 | 2026-10-22 09:19:26 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_4bdff0b594df/ledger.json` |
| round_3107b51186ef | COMPLETED | 2026-10-22 09:19:43 | 2026-10-22 09:20:07 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_3107b51186ef/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.05 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 3 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_09f0c21a864d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 19.47 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.26, 0.266, 10.24, 0.0, 8.646], "harvest_confirmed": false, "completion_level": 6} |

### round_8ffb285f4b20

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | succeeded | 19.86 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 0.705, 9.787, 0.0, 9.305], "harvest_confirmed": false, "completion_level": 6} |

### round_4bdff0b594df

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | succeeded | 18.87 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 0.553, 9.135, 0.0, 9.082], "harvest_confirmed": false, "completion_level": 6} |

### round_3107b51186ef

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 4 | succeeded | 17.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.011, 0.577, 8.755, 0.0, 8.112], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_09f0c21a864d | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_09f0c21a864d:target_0 | target_0 | — | 0.22 | 0.41 | 0.21 | 9.99 | — | 8.59 | — | 19.41 |
| round_8ffb285f4b20 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_8ffb285f4b20:target_1 | target_1 | — | 0.22 | — | 0.58 | 9.82 | — | 9.16 | — | 19.76 |
| round_4bdff0b594df | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_4bdff0b594df:target_1 | target_1 | — | 0.22 | — | 0.4 | 9.19 | — | 9.03 | — | 18.85 |
| round_3107b51186ef | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_3107b51186ef:target_2 | target_2 | — | 0.23 | — | 0.42 | 8.79 | — | 8.2 | — | 17.61 |

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
- 帧间隔中位数：20.23 s（≈0.05 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_2
- captured_views：1
- rejected_views：26
- tf_failures：0
- tf_latency_ms：0.08249282836914062
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

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.673 | 0.193,-0.756,0.683 | — | — |
| target_1 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.298,-0.717,0.626 | 0.299,-0.732,0.672 | — | — |
| target_2 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.209,-0.744,0.646 | 0.193,-0.756,0.683 | — | — |

## 运行性能统计

- 性能采样条数：169
- CPU %：均值 25.95 / 峰值 43.9
- 内存 %：均值 33.69 / 峰值 35.7
- GPU 利用率 %：均值 19.57 / 峰值 46.0
- GPU 显存 MB：均值 1443.64 / 峰值 1488.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：634
- 路径长：6.1287 m
- 起止弦：0.0575 m
- 绕行比（路径/弦）：106.515（直线≈1）
- 相对弦最大偏离：0.5306 m
- Z 范围：0.5007 ~ 0.7587 m（Δz=0.0019）
