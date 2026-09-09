# 采摘批次摘要 `field_full_20260825_1735`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 17:36:16
- 结束时间：2026-08-25 17:37:06
- 总时长：50.6 s
- 终局计数：skipped=3

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | — | skipped | 2.82 | {"code": "target_skipped"} |
| target_3 | 1 | skipped | 15.42 | {"code": "target_skipped"} |
| target_4 | 2 | skipped | 16.23 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1735:target_0 | target_0 | — | 2.61 | — | — | — | — | — | — | 2.61 |
| field_full_20260825_1735:target_3 | target_3 | — | 14.0 | — | 0.4 | 0.6 | — | — | 0.2 | 15.2 |
| field_full_20260825_1735:target_3 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_full_20260825_1735:target_4 | target_4 | — | 16.02 | — | — | — | — | — | — | 16.02 |

## 事件统计

- 事件总数：11
- severity INFO：11
- `round_locked`：5
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：122
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.0858306884765625
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
| target_0 | 0 | 6 | COLLECTING | missing_mask {"missing_mask": 6} |
| target_3 | 3 | 47 | READY | robot_not_static {"missing_mask": 9, "same_stamp": 2, "robot_not_static": 42, "stale_frame": 2} |
| target_4 | 1 | 68 | COLLECTING | missing_mask {"missing_mask": 11, "same_stamp": 1, "robot_not_static": 33, "target_drift": 5, "icp_reject": 16, "stale_frame": 2} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | discover | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_changed | — | — | — |
| target_3 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.371,-0.654,0.530 | 0.287,-0.730,0.653 | — |
| target_4 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.268,-0.701,0.555 | 0.287,-0.730,0.653 | — |

## 运行性能统计

- 性能采样条数：48
- CPU %：均值 27.65 / 峰值 79.1
- 内存 %：均值 58.58 / 峰值 59.0
- GPU 利用率 %：均值 13.33 / 峰值 32.0
- GPU 显存 MB：均值 2348.58 / 峰值 2365.0
