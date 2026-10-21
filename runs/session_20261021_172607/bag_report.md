# 采摘会话分析 `session_20261021_172607`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_172607/bag`
- 开始时间：2026-10-21 17:26:07
- 结束时间：2026-10-21 17:29:03
- 总时长：175.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_5382b44d8dfc | COMPLETED | 2026-10-21 17:26:34 | 2026-10-21 17:26:54 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_5382b44d8dfc/ledger.json` |
| round_f6d06739601b | COMPLETED | 2026-10-21 17:27:10 | 2026-10-21 17:27:27 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f6d06739601b/ledger.json` |
| round_ea0b7e2bf3c5 | COMPLETED | 2026-10-21 17:27:42 | 2026-10-21 17:28:00 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_ea0b7e2bf3c5/ledger.json` |
| round_b0e3c195db12 | COMPLETED | 2026-10-21 17:28:12 | 2026-10-21 17:28:30 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_b0e3c195db12/ledger.json` |
| round_c32bb33d6e0b | COMPLETED | 2026-10-21 17:28:44 | 2026-10-21 17:28:59 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_c32bb33d6e0b/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.03 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_5382b44d8dfc

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 16.25 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.002, 0.762, 8.425, 0.0, 6.987], "harvest_confirmed": false, "completion_level": 6} |

### round_f6d06739601b

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 3 | succeeded | 14.03 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.801, 6.637, 0.0, 6.549], "harvest_confirmed": false, "completion_level": 6} |

### round_ea0b7e2bf3c5

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 15.65 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.747, 7.766, 0.0, 7.091], "harvest_confirmed": false, "completion_level": 6} |

### round_b0e3c195db12

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_15 | 1 | succeeded | 15.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.007, 0.752, 7.732, 0.0, 6.988], "harvest_confirmed": false, "completion_level": 6} |

### round_c32bb33d6e0b

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_16 | 1 | succeeded | 13.03 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.775, 6.207, 0.0, 6.002], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_5382b44d8dfc | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| round_5382b44d8dfc:target_0 | target_0 | — | — | — | 0.64 | 8.36 | — | 6.99 | — | 15.97 |
| round_f6d06739601b | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| round_f6d06739601b:target_5 | target_5 | — | — | — | 0.81 | 6.65 | — | 6.38 | — | 13.83 |
| round_ea0b7e2bf3c5 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| round_ea0b7e2bf3c5:target_8 | target_8 | — | — | — | 0.63 | 7.75 | — | 7.0 | — | 15.4 |
| round_b0e3c195db12 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_b0e3c195db12:target_15 | target_15 | — | — | — | 0.6 | 7.83 | — | 6.98 | — | 15.4 |
| round_c32bb33d6e0b | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| round_c32bb33d6e0b:target_16 | target_16 | — | — | — | 0.62 | 6.2 | — | 6.01 | — | 12.84 |

## 事件统计

- 事件总数：68
- severity AUDIT：28
- severity INFO：40
- `enables_changed`：8
- `photo_pose_reached`：10
- `recovery_acknowledged`：10
- `recovery_required`：10
- `round_locked`：10
- `target_dispatched`：10
- `target_succeeded`：10

## 感知统计

- 记录帧数：8
- 帧间隔中位数：30.317 s（≈0.03 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_16
- captured_views：1
- rejected_views：3
- tf_failures：0
- tf_latency_ms：0.08654594421386719
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_15 | 1 | 0 | COLLECTING | — |
| target_16 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.568,-0.610,0.721 | — | — |
| target_7 | permit | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.571,-0.608,0.672 | — | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.297,-0.717,0.627 | 0.299,-0.727,0.672 | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.673 | 0.570,-0.608,0.725 | — | — |
| target_12 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.414,-0.638,0.666 | — | — | — |
| target_15 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.569,-0.608,0.729 | — | — |
| target_16 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.408,-0.647,0.681 | 0.400,-0.647,0.718 | — | — |

## 运行性能统计

- 性能采样条数：164
- CPU %：均值 28.65 / 峰值 41.5
- 内存 %：均值 43.05 / 峰值 44.3
- GPU 利用率 %：均值 24.04 / 峰值 48.0
- GPU 显存 MB：均值 2767.18 / 峰值 2823.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：629
- 路径长：6.8826 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：119.302（直线≈1）
- 相对弦最大偏离：0.4929 m
- Z 范围：0.4998 ~ 0.7024 m（Δz=0.0019）
