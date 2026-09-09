# 采摘批次摘要 `field_pregrasp_20260831_1351`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 13:51:49
- 结束时间：2026-08-31 13:52:23
- 总时长：34.1 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 13.51 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 8.63 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1351:target_0 | target_0 | — | 12.89 | — | 0.2 | 0.2 | — | — | 0.2 | 13.49 |
| field_pregrasp_20260831_1351:target_0 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1351:target_1 | target_1 | — | 8.42 | — | — | — | — | — | — | 8.42 |
| field_pregrasp_20260831_1351:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：84
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.3972053527832031
- cloud_points：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：0
- skip_reasons：{}
- last_skip_code：
- last_skip_reason：

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 5 | 54 | READY | stale_frame {"same_stamp": 4, "near_duplicate": 3, "robot_not_static": 40, "stale_frame": 7} |
| target_1 | 4 | 40 | COLLECTING | same_stamp {"missing_mask": 1, "same_stamp": 4, "robot_not_static": 35} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；观察预算收口但重建未收敛: insufficient_angular_baseline | 0.208,-0.630,0.496 | 0.369,-0.676,0.553 | — |
| target_1 | lock | 否 | 工具档关；观察预算收口但重建未收敛: insufficient_angular_baseline | 0.279,-0.639,0.505 | 0.369,-0.676,0.553 | — |

## 运行性能统计

- 性能采样条数：33
- CPU %：均值 49.52 / 峰值 99.7
- 内存 %：均值 46.7 / 峰值 47.3
- GPU 利用率 %：均值 14.91 / 峰值 33.0
- GPU 显存 MB：均值 1438.52 / 峰值 1531.0
