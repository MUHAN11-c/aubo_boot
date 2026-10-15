# 采摘会话分析 `session_20261015_150848`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_150848/bag`
- 开始时间：2026-10-15 15:08:49
- 结束时间：2026-10-15 15:11:37
- 总时长：167.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_5cdf7d17e169 | RUNNING | 2026-10-15 15:11:27 | 2026-10-15 15:11:31 | unfinished=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_5cdf7d17e169/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.6 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐批次逐目标 outcome

### round_5cdf7d17e169

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | unfinished | — |  |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_5cdf7d17e169 | — | 0.31 | — | — | — | — | — | — | — | 0.31 |
| round_5cdf7d17e169:target_1 | target_1 | — | — | 0.98 | 0.23 | 4.5 | — | — | — | 5.72 |

## 事件统计

- 事件总数：12
- severity AUDIT：6
- severity INFO：6
- `enables_changed`：6
- `photo_pose_reached`：2
- `round_locked`：2
- `target_dispatched`：2

## 感知统计

- 记录帧数：80
- 帧间隔中位数：1.668 s（≈0.6 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：1
- rejected_views：13
- tf_failures：0
- tf_latency_ms：0.09489059448242188
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：bag_mouth_geometry

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.375,-0.608,0.580 | 0.385,-0.625,0.632 | 0.373,-0.599,0.566 | 0.378,-0.609,0.593 |
| target_1 | approach | 是 | 工具档关；工具档关闭：可靠近，不打末端 IO | 0.376,-0.608,0.582 | 0.385,-0.625,0.633 | 0.373,-0.599,0.566 | 0.378,-0.609,0.593 |

## 运行性能统计

- 性能采样条数：132
- CPU %：均值 82.68 / 峰值 95.1
- 内存 %：均值 58.49 / 峰值 70.6
- GPU 利用率 %：均值 54.94 / 峰值 79.0
- GPU 显存 MB：均值 3812.39 / 峰值 4486.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：31
- 路径长：0.2293 m
- 起止弦：0.2214 m
- 绕行比（路径/弦）：1.036（直线≈1）
- 相对弦最大偏离：0.0255 m
- Z 范围：0.5942 ~ 0.6888 m（Δz=-0.0946）
