# 采摘会话分析 `session_20261015_094451`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_094451/bag`
- 开始时间：2026-10-15 09:44:52
- 结束时间：2026-10-15 09:55:42
- 总时长：650.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_55c2d35b6b88 | COMPLETED | 2026-10-15 09:47:48 | 2026-10-15 09:48:45 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_55c2d35b6b88/ledger.json` |
| round_73a0ea8ae6f0 | RUNNING | 2026-10-15 09:48:55 | 2026-10-15 09:48:59 | unfinished=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_73a0ea8ae6f0/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.25 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_55c2d35b6b88

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 34.3 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.408, 16.786, 0.001, 14.328], "harvest_confirmed": false, "completion_level": 6} |

### round_73a0ea8ae6f0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | unfinished | — |  |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_55c2d35b6b88 | — | 0.3 | — | — | — | — | — | — | — | 0.3 |
| round_55c2d35b6b88:target_1 | target_1 | — | — | 2.59 | 0.39 | 16.57 | — | 14.43 | — | 33.97 |
| round_55c2d35b6b88:target_1 | — | 7.69 | — | — | — | — | — | — | — | 7.69 |
| round_73a0ea8ae6f0 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_73a0ea8ae6f0:target_2 | target_2 | — | — | 1.2 | 0.37 | 14.05 | — | 0.8 | — | 16.45 |

## 事件统计

- 事件总数：30
- severity AUDIT：8
- severity INFO：22
- `enables_changed`：4
- `photo_pose_reached`：8
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：8
- `target_dispatched`：4
- `target_succeeded`：2

## 感知统计

- 记录帧数：309
- 帧间隔中位数：0.801 s（≈1.25 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_2
- captured_views：1
- rejected_views：9
- tf_failures：0
- tf_latency_ms：0.08726119995117188
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：bag_mouth_geometry

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.376,-0.607,0.582 | 0.386,-0.627,0.635 | 0.374,-0.598,0.567 | 0.379,-0.609,0.595 |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.381,-0.616,0.607 | — | — | — |
| target_2 | approach | 是 | 工具档关；接触恢复未确认，不派下一颗 | 0.376,-0.607,0.583 | 0.385,-0.626,0.634 | 0.375,-0.598,0.568 | 0.379,-0.609,0.596 |

## 运行性能统计

- 性能采样条数：990
- CPU %：均值 39.06 / 峰值 98.5
- 内存 %：均值 47.7 / 峰值 75.4
- GPU 利用率 %：均值 39.03 / 峰值 97.0
- GPU 显存 MB：均值 2807.62 / 峰值 6035.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：392
- 路径长：2.6169 m
- 起止弦：0.3974 m
- 绕行比（路径/弦）：6.585（直线≈1）
- 相对弦最大偏离：0.2537 m
- Z 范围：0.4719 ~ 0.7776 m（Δz=-0.0139）
