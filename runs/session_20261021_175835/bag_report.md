# 采摘会话分析 `session_20261021_175835`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_175835/bag`
- 开始时间：2026-10-21 17:58:35
- 结束时间：2026-10-21 18:00:51
- 总时长：136.2 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_95bf563eebcc | COMPLETED | 2026-10-21 17:58:57 | 2026-10-21 17:59:22 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_95bf563eebcc/ledger.json` |
| round_b941fff34556 | COMPLETED | 2026-10-21 17:59:42 | 2026-10-21 18:00:08 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_b941fff34556/ledger.json` |
| round_138b58bc3adb | COMPLETED | 2026-10-21 18:00:23 | 2026-10-21 18:00:50 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_138b58bc3adb/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.68 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 3 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_95bf563eebcc

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | succeeded | 19.25 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.211, 0.498, 10.016, 0.0, 8.34], "harvest_confirmed": false, "completion_level": 6} |

### round_b941fff34556

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 2 | succeeded | 19.24 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 0.7, 9.499, 0.0, 8.954], "harvest_confirmed": false, "completion_level": 6} |

### round_138b58bc3adb

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 2 | succeeded | 19.64 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.638, 9.641, 0.0, 9.248], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_95bf563eebcc | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| round_95bf563eebcc:target_0 | target_0 | — | — | 0.2 | 0.56 | 9.87 | — | 8.41 | — | 19.04 |
| round_b941fff34556 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_b941fff34556:target_5 | target_5 | — | — | — | 0.61 | 9.4 | — | 8.96 | — | 18.97 |
| round_138b58bc3adb | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| round_138b58bc3adb:target_8 | target_8 | — | — | — | 0.61 | 9.6 | — | 9.22 | — | 19.41 |

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
- 帧间隔中位数：1.468 s（≈0.68 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_8
- captured_views：1
- rejected_views：20
- tf_failures：0
- tf_latency_ms：0.11277198791503906
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

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_3 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.204,-0.747,0.657 | 0.568,-0.608,0.720 | — | — |
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.569,-0.608,0.673 | 0.568,-0.608,0.720 | — | — |
| target_4 | permit | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.570,-0.608,0.672 | — | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.658,0.663 | 0.403,-0.666,0.718 | — | — |
| target_9 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.679 | 0.402,-0.666,0.718 | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.414,-0.638,0.666 | 0.402,-0.666,0.718 | — | — |

## 运行性能统计

- 性能采样条数：127
- CPU %：均值 26.37 / 峰值 45.3
- 内存 %：均值 40.93 / 峰值 42.4
- GPU 利用率 %：均值 15.02 / 峰值 40.0
- GPU 显存 MB：均值 2556.61 / 峰值 2616.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：474
- 路径长：4.3777 m
- 起止弦：0.0559 m
- 绕行比（路径/弦）：78.328（直线≈1）
- 相对弦最大偏离：0.447 m
- Z 范围：0.5342 ~ 0.7557 m（Δz=0.0019）
