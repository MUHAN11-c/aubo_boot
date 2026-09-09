# 采摘批次摘要 `field_pregrasp_20260831_1633`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 16:34:18
- 结束时间：2026-08-31 16:35:15
- 总时长：57.0 s
- 终局计数：skipped=3

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 9.81 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 9.03 | {"code": "target_skipped"} |
| target_2 | 3 | skipped | 20.63 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1633 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1633:target_0 | target_0 | — | 9.79 | — | — | — | — | — | — | 9.79 |
| field_pregrasp_20260831_1633:target_1 | target_1 | — | 8.83 | — | — | — | — | — | — | 8.83 |
| field_pregrasp_20260831_1633:target_2 | target_2 | — | 20.4 | — | — | — | — | — | — | 20.4 |
| field_pregrasp_20260831_1633:target_2 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：11
- severity INFO：11
- `round_locked`：5
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：127
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09202957153320312
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
| target_0 | 3 | 48 | COLLECTING | stale_frame {"stale_frame": 4, "same_stamp": 3, "near_duplicate": 1, "robot_not_static": 40} |
| target_1 | 4 | 41 | COLLECTING | same_stamp {"missing_mask": 1, "same_stamp": 4, "robot_not_static": 34, "stale_frame": 2} |
| target_2 | 1 | 64 | COLLECTING | missing_mask {"missing_mask": 38, "same_stamp": 1, "robot_not_static": 19, "stale_frame": 6} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.228,-0.624,0.520 | 0.563,-0.783,0.503 | — |
| target_1 | lock | 否 | 工具档关；观察预算收口但重建未收敛: insufficient_angular_baseline | 0.305,-0.641,0.497 | 0.404,-0.673,0.541 | — |
| target_2 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.604,-0.756,0.439 | 0.563,-0.783,0.503 | — |

## 运行性能统计

- 性能采样条数：55
- CPU %：均值 38.61 / 峰值 86.0
- 内存 %：均值 47.74 / 峰值 48.4
- GPU 利用率 %：均值 11.45 / 峰值 33.0
- GPU 显存 MB：均值 1682.69 / 峰值 1724.0
