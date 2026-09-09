# 采摘批次摘要 `field_pregrasp_20260831_1554`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 15:54:22
- 结束时间：2026-08-31 15:55:25
- 总时长：63.1 s
- 终局计数：skipped=3

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 14.43 | {"code": "target_skipped"} |
| target_0 | 2 | skipped | 8.64 | {"code": "target_skipped"} |
| target_2 | 3 | skipped | 19.23 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1554 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1554:target_1 | target_1 | — | 10.22 | 0.2 | 0.21 | 3.6 | — | — | 0.19 | 14.43 |
| field_pregrasp_20260831_1554:target_0 | target_0 | — | 8.41 | — | — | — | — | — | — | 8.41 |
| field_pregrasp_20260831_1554:target_2 | target_2 | — | 19.02 | — | — | — | — | — | — | 19.02 |
| field_pregrasp_20260831_1554:target_2 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：12
- severity INFO：12
- `round_locked`：6
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：141
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08916854858398438
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
| target_1 | 2 | 40 | READY | missing_mask {"missing_mask": 5, "stale_frame": 5, "same_stamp": 1, "robot_not_static": 29} |
| target_0 | 3 | 44 | COLLECTING | missing_mask {"missing_mask": 2, "same_stamp": 6, "robot_not_static": 36} |
| target_2 | 1 | 60 | COLLECTING | missing_mask {"missing_mask": 28, "same_stamp": 1, "robot_not_static": 28, "stale_frame": 3} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.225,-0.615,0.507 | 0.562,-0.781,0.502 | — |
| target_0 | lock | 否 | 工具档关；观察预算收口但重建未收敛: insufficient_angular_baseline | 0.307,-0.631,0.505 | 0.408,-0.677,0.552 | — |
| target_2 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.622,-0.759,0.450 | 0.562,-0.781,0.502 | — |

## 运行性能统计

- 性能采样条数：60
- CPU %：均值 34.89 / 峰值 96.9
- 内存 %：均值 46.77 / 峰值 47.3
- GPU 利用率 %：均值 9.77 / 峰值 35.0
- GPU 显存 MB：均值 1845.1 / 峰值 1873.0
