# 采摘会话分析 `session_20261021_162547`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_162547/bag`
- 开始时间：2026-10-21 16:25:47
- 结束时间：2026-10-21 16:33:54
- 总时长：486.9 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_162640_r01 | COMPLETED | 2026-10-21 16:26:48 | 2026-10-21 16:28:40 | skipped=3，succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_162640_r01/ledger.json` |
| flow10_162840_r02 | COMPLETED | 2026-10-21 16:28:44 | 2026-10-21 16:30:32 | skipped=4 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_162840_r02/ledger.json` |
| flow10_163032_r03 | COMPLETED | 2026-10-21 16:30:36 | 2026-10-21 16:32:24 | skipped=4 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_163032_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.08 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### flow10_162640_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | succeeded | 13.25 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.826, 6.429, 0.0, 5.895], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 2 | skipped | 7.5 | skipped_quality |
| target_1 | 3 | skipped | 5.44 | skipped_quality |
| target_2 | 4 | skipped | 5.21 | skipped_quality |

### flow10_162840_r02

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | skipped | 4.83 | skipped_quality |
| target_4 | 2 | skipped | 9.69 | skipped_quality |
| target_5 | 3 | skipped | 8.87 | skipped_quality |
| target_6 | 4 | skipped | 7.35 | skipped_quality |

### flow10_163032_r03

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_11 | 1 | skipped | 4.43 | skipped_quality |
| target_8 | 2 | skipped | 9.85 | skipped_quality |
| target_9 | 3 | skipped | 6.51 | skipped_quality |
| target_10 | 4 | skipped | 6.27 | skipped_quality |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_162640_r01 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_162640_r01:target_3 | target_3 | — | — | — | 0.87 | 6.34 | — | 5.81 | — | 13.01 |
| flow10_162640_r01:target_3 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_162640_r01:target_0 | target_0 | — | — | 0.21 | 2.18 | 5.02 | — | — | — | 7.41 |
| flow10_162640_r01:target_0 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| flow10_162640_r01:target_1 | target_1 | — | — | — | 1.43 | 3.75 | — | — | — | 5.17 |
| flow10_162640_r01:target_1 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_162640_r01:target_2 | target_2 | — | — | — | 1.59 | 3.39 | — | — | — | 4.98 |
| flow10_162640_r01:target_2 | — | 10.16 | — | — | — | — | — | — | — | 10.16 |
| flow10_162840_r02 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| flow10_162840_r02:target_7 | target_7 | — | — | — | 0.6 | 4.0 | — | — | — | 4.6 |
| flow10_162840_r02:target_7 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| flow10_162840_r02:target_4 | target_4 | — | — | 0.19 | 4.41 | 5.01 | — | — | — | 9.6 |
| flow10_162840_r02:target_4 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_162840_r02:target_5 | target_5 | — | — | — | 5.26 | 3.61 | — | — | — | 8.87 |
| flow10_162840_r02:target_5 | — | 0.09 | — | — | — | — | — | — | — | 0.09 |
| flow10_162840_r02:target_6 | target_6 | — | — | — | 3.85 | 3.39 | — | — | — | 7.24 |
| flow10_162840_r02:target_6 | — | 10.18 | — | — | — | — | — | — | — | 10.18 |
| flow10_163032_r03 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow10_163032_r03:target_11 | target_11 | — | — | — | 0.62 | 3.6 | — | — | — | 4.22 |
| flow10_163032_r03:target_11 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_163032_r03:target_8 | target_8 | — | — | 0.19 | 4.59 | 4.79 | — | — | — | 9.6 |
| flow10_163032_r03:target_8 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| flow10_163032_r03:target_9 | target_9 | — | — | — | 2.96 | 3.4 | — | — | — | 6.37 |
| flow10_163032_r03:target_9 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| flow10_163032_r03:target_10 | target_10 | — | — | — | 2.41 | 3.59 | — | — | — | 6.0 |
| flow10_163032_r03:target_10 | — | 10.09 | — | — | — | — | — | — | — | 10.09 |

## 事件统计

- 事件总数：172
- severity AUDIT：52
- severity INFO：120
- `enables_changed`：4
- `photo_pose_reached`：36
- `recovery_acknowledged`：24
- `recovery_required`：24
- `round_locked`：36
- `target_dispatched`：24
- `target_skipped`：22
- `target_succeeded`：2

## 感知统计

- 记录帧数：13
- 帧间隔中位数：12.091 s（≈0.08 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.06103515625
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_3 | 1 | 0 | COLLECTING | — |
| target_0 | 2 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |
| target_2 | 1 | 0 | COLLECTING | — |
| target_7 | 1 | 0 | COLLECTING | — |
| target_4 | 2 | 0 | COLLECTING | — |
| target_5 | 2 | 0 | COLLECTING | — |
| target_6 | 2 | 0 | COLLECTING | — |
| target_11 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_9 | 1 | 0 | COLLECTING | — |
| target_10 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.672 | 0.567,-0.609,0.720 | — | — |
| target_3 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.203,-0.746,0.658 | — | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.405,-0.659,0.663 | 0.404,-0.667,0.718 | — | — |
| target_2 | lock | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.300,-0.717,0.627 | 0.299,-0.729,0.672 | — | — |
| target_7 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.203,-0.746,0.658 | — | — | — |
| target_4 | lock | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.569,-0.608,0.672 | 0.567,-0.609,0.723 | — | — |
| target_5 | lock | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.405,-0.657,0.663 | 0.404,-0.666,0.718 | — | — |
| target_6 | lock | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.299,-0.716,0.627 | 0.299,-0.730,0.672 | — | — |
| target_11 | done | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.203,-0.747,0.658 | — | — | — |
| target_8 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.607,0.672 | 0.567,-0.609,0.723 | — | — |
| target_9 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.405,-0.658,0.663 | 0.403,-0.666,0.718 | — | — |
| target_10 | lock | 否 | 工具档关；预抓取已验证 · SKIPPED_QUALITY | 0.298,-0.716,0.627 | 0.300,-0.728,0.672 | — | — |

## 运行性能统计

- 性能采样条数：452
- CPU %：均值 42.29 / 峰值 84.4
- 内存 %：均值 48.56 / 峰值 51.6
- GPU 利用率 %：均值 26.21 / 峰值 58.0
- GPU 显存 MB：均值 2878.65 / 峰值 2956.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：881
- 路径长：14.7062 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.5328 m
- Z 范围：0.4983 ~ 0.6907 m（Δz=-0.0）
