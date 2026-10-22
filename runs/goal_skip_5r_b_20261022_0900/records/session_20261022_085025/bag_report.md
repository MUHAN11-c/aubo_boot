# 采摘会话分析 `session_20261022_085025`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_085025/bag`
- 开始时间：2026-10-22 08:50:26
- 结束时间：2026-10-22 08:54:22
- 总时长：236.7 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_ff38284826f0 | COMPLETED | 2026-10-22 08:50:53 | 2026-10-22 08:51:22 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_ff38284826f0/ledger.json` |
| round_8fcd2ff7d85b | COMPLETED | 2026-10-22 08:51:41 | 2026-10-22 08:52:07 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_8fcd2ff7d85b/ledger.json` |
| round_d8d20c0c23ce | COMPLETED | 2026-10-22 08:52:22 | 2026-10-22 08:52:49 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_d8d20c0c23ce/ledger.json` |
| round_82274d510ccd | COMPLETED | 2026-10-22 08:53:04 | 2026-10-22 08:53:32 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_82274d510ccd/ledger.json` |
| round_40a47f5d6d41 | COMPLETED | 2026-10-22 08:53:53 | 2026-10-22 08:54:20 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_40a47f5d6d41/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.65 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 5 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_ff38284826f0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 22.28 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["observe", "finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.001, 0.253, 0.305, 11.296, 0.0, 10.207], "harvest_confirmed": false, "completion_level": 6} |

### round_8fcd2ff7d85b

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 3 | succeeded | 20.07 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.584, 9.801, 0.0, 9.453], "harvest_confirmed": false, "completion_level": 6} |

### round_d8d20c0c23ce

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 20.45 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.256, 0.332, 10.592, 0.0, 9.067], "harvest_confirmed": false, "completion_level": 6} |

### round_82274d510ccd

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_15 | 1 | succeeded | 21.47 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.255, 0.335, 10.974, 0.001, 9.849], "harvest_confirmed": false, "completion_level": 6} |

### round_40a47f5d6d41

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_16 | 1 | succeeded | 20.47 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.257, 0.396, 10.469, 0.0, 9.142], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_ff38284826f0 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_ff38284826f0:target_0 | target_0 | — | 0.23 | 0.23 | 0.2 | 11.35 | — | 10.18 | — | 22.18 |
| round_8fcd2ff7d85b | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_8fcd2ff7d85b:target_5 | target_5 | — | 0.23 | — | 0.42 | 9.77 | — | 9.59 | — | 20.02 |
| round_d8d20c0c23ce | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_d8d20c0c23ce:target_8 | target_8 | — | 0.22 | 0.2 | 0.2 | 10.61 | — | 9.18 | — | 20.39 |
| round_82274d510ccd | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_82274d510ccd:target_15 | target_15 | — | 0.22 | 0.21 | 0.2 | 11.02 | — | 9.78 | — | 21.43 |
| round_40a47f5d6d41 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| round_40a47f5d6d41:target_16 | target_16 | — | 0.23 | 0.19 | 0.4 | 10.38 | — | 9.23 | — | 20.45 |

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

- 记录帧数：10
- 帧间隔中位数：1.535 s（≈0.65 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_16
- captured_views：1
- rejected_views：11
- tf_failures：0
- tf_latency_ms：0.05936622619628906
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
| target_15 | 1 | 0 | COLLECTING | — |
| target_16 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.673 | 0.569,-0.608,0.730 | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.681 | 0.300,-0.735,0.671 | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.298,-0.718,0.626 | 0.300,-0.735,0.671 | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.672 | 0.570,-0.607,0.729 | — | — |
| target_15 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.672 | 0.570,-0.607,0.729 | — | — |
| target_16 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.568,-0.608,0.720 | — | — |

## 运行性能统计

- 性能采样条数：223
- CPU %：均值 25.41 / 峰值 39.3
- 内存 %：均值 30.54 / 峰值 32.1
- GPU 利用率 %：均值 19.86 / 峰值 46.0
- GPU 显存 MB：均值 1461.33 / 峰值 1528.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：804
- 路径长：7.5305 m
- 起止弦：0.0577 m
- 绕行比（路径/弦）：130.492（直线≈1）
- 相对弦最大偏离：0.5 m
- Z 范围：0.5019 ~ 0.7694 m（Δz=0.0019）
