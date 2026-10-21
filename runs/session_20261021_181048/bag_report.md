# 采摘会话分析 `session_20261021_181048`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_181048/bag`
- 开始时间：2026-10-21 18:10:48
- 结束时间：2026-10-21 18:13:13
- 总时长：144.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_334a5b2b922a | COMPLETED | 2026-10-21 18:11:15 | 2026-10-21 18:11:42 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_334a5b2b922a/ledger.json` |
| round_24df14d51bad | COMPLETED | 2026-10-21 18:12:02 | 2026-10-21 18:12:27 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_24df14d51bad/ledger.json` |
| round_faa20fea4595 | COMPLETED | 2026-10-21 18:12:43 | 2026-10-21 18:13:11 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_faa20fea4595/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.07 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 3 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_334a5b2b922a

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 21.28 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["observe", "finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.001, 0.259, 0.335, 11.079, 0.0, 9.533], "harvest_confirmed": false, "completion_level": 6} |

### round_24df14d51bad

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 3 | succeeded | 18.27 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 0.646, 8.954, 0.0, 8.542], "harvest_confirmed": false, "completion_level": 6} |

### round_faa20fea4595

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | succeeded | 20.65 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.256, 0.353, 10.8, 0.0, 9.204], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_334a5b2b922a | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_334a5b2b922a:target_0 | target_0 | — | 0.24 | 0.19 | 0.21 | 11.23 | — | 9.38 | — | 21.22 |
| round_24df14d51bad | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_24df14d51bad:target_5 | target_5 | — | 0.22 | — | 0.59 | 8.77 | — | 8.61 | — | 18.17 |
| round_faa20fea4595 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| round_faa20fea4595:target_8 | target_8 | — | 0.22 | 0.21 | 0.39 | 10.6 | — | 9.18 | — | 20.61 |

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
- 帧间隔中位数：0.934 s（≈1.07 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_8
- captured_views：1
- rejected_views：21
- tf_failures：0
- tf_latency_ms：0.07677078247070312
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
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.570,-0.608,0.672 | 0.568,-0.609,0.725 | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.679 | 0.299,-0.727,0.672 | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.298,-0.718,0.627 | 0.299,-0.727,0.672 | — | — |
| target_9 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.414,-0.638,0.666 | — | — | — |
| target_8 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.571,-0.608,0.672 | 0.569,-0.609,0.729 | — | — |

## 运行性能统计

- 性能采样条数：135
- CPU %：均值 27.38 / 峰值 47.0
- 内存 %：均值 41.63 / 峰值 43.0
- GPU 利用率 %：均值 25.08 / 峰值 52.0
- GPU 显存 MB：均值 3000.12 / 峰值 3201.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：476
- 路径长：4.5333 m
- 起止弦：0.0575 m
- 绕行比（路径/弦）：78.905（直线≈1）
- 相对弦最大偏离：0.4943 m
- Z 范围：0.499 ~ 0.7632 m（Δz=0.0019）
