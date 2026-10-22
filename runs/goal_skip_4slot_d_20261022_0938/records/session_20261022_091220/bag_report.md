# 采摘会话分析 `session_20261022_091220`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_091220/bag`
- 开始时间：2026-10-22 09:12:20
- 结束时间：2026-10-22 09:15:24
- 总时长：183.9 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_426b3ffee040 | COMPLETED | 2026-10-22 09:12:48 | 2026-10-22 09:13:15 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_426b3ffee040/ledger.json` |
| round_20e385aa018e | COMPLETED | 2026-10-22 09:13:36 | 2026-10-22 09:14:01 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_20e385aa018e/ledger.json` |
| round_15e7c6cd2b0d | COMPLETED | 2026-10-22 09:14:17 | 2026-10-22 09:14:42 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_15e7c6cd2b0d/ledger.json` |
| round_8283f09b7e6d | COMPLETED | 2026-10-22 09:14:59 | 2026-10-22 09:15:23 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_8283f09b7e6d/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.05 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 4 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_426b3ffee040

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 19.87 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.26, 0.386, 10.351, 0.0, 8.662], "harvest_confirmed": false, "completion_level": 6} |

### round_20e385aa018e

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 2 | succeeded | 19.67 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.621, 9.698, 0.0, 9.212], "harvest_confirmed": false, "completion_level": 6} |

### round_15e7c6cd2b0d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 3 | succeeded | 19.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.009, 0.528, 9.809, 0.0, 9.259], "harvest_confirmed": false, "completion_level": 6} |

### round_8283f09b7e6d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_14 | 4 | succeeded | 17.47 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 0.646, 8.492, 0.0, 8.099], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_426b3ffee040 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_426b3ffee040:target_0 | target_0 | — | 0.22 | 0.2 | 0.59 | 9.98 | — | 8.82 | — | 19.82 |
| round_20e385aa018e | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_20e385aa018e:target_4 | target_4 | — | 0.23 | — | 0.59 | 9.6 | — | 9.21 | — | 19.62 |
| round_15e7c6cd2b0d | — | 0.17 | — | — | — | — | — | — | — | 0.17 |
| round_15e7c6cd2b0d:target_9 | target_9 | — | 0.22 | — | 0.42 | 9.78 | — | 9.19 | — | 19.62 |
| round_8283f09b7e6d | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_8283f09b7e6d:target_14 | target_14 | — | 0.24 | — | 0.6 | 8.39 | — | 8.18 | — | 17.41 |

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
- 帧间隔中位数：20.231 s（≈0.05 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_14
- captured_views：1
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.06031990051269531
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
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.568,-0.609,0.719 | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.674 | 0.404,-0.665,0.718 | — | — |
| target_4 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.406,-0.655,0.663 | 0.404,-0.665,0.718 | — | — |
| target_8 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.674 | 0.299,-0.732,0.672 | — | — |
| target_9 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.299,-0.718,0.627 | 0.299,-0.732,0.672 | — | — |
| target_12 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.673 | 0.194,-0.756,0.683 | — | — |
| target_14 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.209,-0.745,0.646 | 0.194,-0.756,0.683 | — | — |

## 运行性能统计

- 性能采样条数：173
- CPU %：均值 26.77 / 峰值 44.2
- 内存 %：均值 33.92 / 峰值 35.7
- GPU 利用率 %：均值 20.22 / 峰值 71.0
- GPU 显存 MB：均值 1459.84 / 峰值 1518.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：625
- 路径长：6.1031 m
- 起止弦：0.0549 m
- 绕行比（路径/弦）：111.142（直线≈1）
- 相对弦最大偏离：0.5309 m
- Z 范围：0.5006 ~ 0.7563 m（Δz=0.0018）
