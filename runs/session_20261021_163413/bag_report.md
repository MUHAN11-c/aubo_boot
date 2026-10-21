# 采摘会话分析 `session_20261021_163413`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_163413/bag`
- 开始时间：2026-10-21 16:34:13
- 结束时间：2026-10-21 16:42:51
- 总时长：517.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_163452_r01 | COMPLETED | 2026-10-21 16:34:58 | 2026-10-21 16:37:14 | failed=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_163452_r01/ledger.json` |
| flow10_163714_r02 | COMPLETED | 2026-10-21 16:37:18 | 2026-10-21 16:39:28 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_163714_r02/ledger.json` |
| flow10_163928_r03 | COMPLETED | 2026-10-21 16:39:32 | 2026-10-21 16:41:50 | skipped=1，succeeded=3 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_163928_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.65 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 9 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_163452_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | failed | 14.84 | full_failed |
| target_1 | 2 | succeeded | 20.16 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 4.365, 7.933, 0.0, 7.649], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 3 | succeeded | 17.7 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 1.699, 8.272, 0.0, 7.528], "harvest_confirmed": false, "completion_level": 6} |
| target_3 | 4 | succeeded | 16.13 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 1.956, 7.001, 0.0, 6.941], "harvest_confirmed": false, "completion_level": 6} |

### flow10_163714_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | skipped | 5.64 | skipped_quality |
| target_5 | 2 | succeeded | 19.51 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 3.689, 7.973, 0.0, 7.58], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 3 | succeeded | 18.56 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 4.299, 7.178, 0.0, 6.837], "harvest_confirmed": false, "completion_level": 6} |
| target_7 | 4 | succeeded | 15.09 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.009, 1.427, 6.883, 0.0, 6.654], "harvest_confirmed": false, "completion_level": 6} |

### flow10_163928_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | skipped | 5.43 | skipped_quality |
| target_9 | 2 | succeeded | 19.55 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.006, 3.696, 8.12, 0.0, 7.617], "harvest_confirmed": false, "completion_level": 6} |
| target_10 | 3 | succeeded | 19.27 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 4.923, 7.233, 0.0, 6.988], "harvest_confirmed": false, "completion_level": 6} |
| target_11 | 4 | succeeded | 16.11 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 2.433, 6.836, 0.0, 6.66], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_163452_r01 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| flow10_163452_r01:target_0 | target_0 | — | — | — | 0.67 | 7.51 | — | 6.39 | — | 14.55 |
| flow10_163452_r01:target_0 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| flow10_163452_r01:target_1 | target_1 | — | — | — | 4.43 | 7.95 | — | 7.64 | — | 20.02 |
| flow10_163452_r01:target_1 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| flow10_163452_r01:target_2 | target_2 | — | — | — | 1.79 | 8.21 | — | 7.59 | — | 17.58 |
| flow10_163452_r01:target_2 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| flow10_163452_r01:target_3 | target_3 | — | — | — | 2.02 | 7.02 | — | 6.98 | — | 16.02 |
| flow10_163452_r01:target_3 | — | 10.2 | — | — | — | — | — | — | — | 10.2 |
| flow10_163714_r02 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_163714_r02:target_4 | target_4 | — | — | — | 0.81 | 4.63 | — | — | — | 5.44 |
| flow10_163714_r02:target_4 | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| flow10_163714_r02:target_5 | target_5 | — | — | — | 3.79 | 8.04 | — | 7.58 | — | 19.4 |
| flow10_163714_r02:target_5 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| flow10_163714_r02:target_6 | target_6 | — | — | — | 4.39 | 7.15 | — | 6.79 | — | 18.35 |
| flow10_163714_r02:target_6 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_163714_r02:target_7 | target_7 | — | — | — | 1.61 | 6.8 | — | 6.62 | — | 15.01 |
| flow10_163714_r02:target_7 | — | 10.18 | — | — | — | — | — | — | — | 10.18 |
| flow10_163928_r03 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_163928_r03:target_8 | target_8 | — | — | — | 0.61 | 4.61 | — | — | — | 5.24 |
| flow10_163928_r03:target_8 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| flow10_163928_r03:target_9 | target_9 | — | — | — | 3.81 | 7.95 | — | 7.62 | — | 19.39 |
| flow10_163928_r03:target_9 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| flow10_163928_r03:target_10 | target_10 | — | — | — | 4.99 | 7.18 | — | 6.95 | — | 19.12 |
| flow10_163928_r03:target_10 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| flow10_163928_r03:target_11 | target_11 | — | — | — | 2.6 | 6.77 | — | 6.57 | — | 15.94 |
| flow10_163928_r03:target_11 | — | 10.1 | — | — | — | — | — | — | — | 10.1 |

## 事件统计

- 事件总数：170
- severity AUDIT：50
- severity ERROR：2
- severity INFO：118
- `enables_changed`：4
- `photo_pose_reached`：36
- `recovery_acknowledged`：24
- `recovery_required`：22
- `round_locked`：36
- `target_dispatched`：24
- `target_failed`：2
- `target_skipped`：4
- `target_succeeded`：18

## 感知统计

- 记录帧数：10
- 帧间隔中位数：1.535 s（≈0.65 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.07367134094238281
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |
| target_2 | 1 | 0 | COLLECTING | — |
| target_3 | 1 | 0 | COLLECTING | — |
| target_4 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | COLLECTING | — |
| target_6 | 1 | 0 | COLLECTING | — |
| target_7 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_9 | 1 | 0 | COLLECTING | — |
| target_10 | 1 | 0 | COLLECTING | — |
| target_11 | 2 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_3 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.742,0.645 | 0.192,-0.755,0.682 | — | — |
| target_0 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.673 | — | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.405,-0.657,0.663 | 0.404,-0.666,0.718 | — | — |
| target_2 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.298,-0.718,0.627 | 0.300,-0.730,0.672 | — | — |
| target_4 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.674 | — | — | — |
| target_5 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.405,-0.658,0.663 | 0.403,-0.666,0.718 | — | — |
| target_6 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.296,-0.718,0.627 | 0.299,-0.729,0.672 | — | — |
| target_7 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.208,-0.743,0.645 | 0.192,-0.755,0.682 | — | — |
| target_8 | done | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.570,-0.608,0.673 | — | — | — |
| target_9 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.403,-0.657,0.663 | 0.403,-0.666,0.718 | — | — |
| target_10 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.297,-0.718,0.626 | 0.300,-0.728,0.672 | — | — |
| target_11 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.207,-0.743,0.645 | 0.193,-0.755,0.682 | — | — |

## 运行性能统计

- 性能采样条数：477
- CPU %：均值 41.85 / 峰值 82.7
- 内存 %：均值 49.35 / 峰值 51.5
- GPU 利用率 %：均值 23.45 / 峰值 62.0
- GPU 显存 MB：均值 2722.97 / 峰值 3008.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1448
- 路径长：17.5374 m
- 起止弦：0.0021 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.5415 m
- Z 范围：0.497 ~ 0.7407 m（Δz=0.0001）
