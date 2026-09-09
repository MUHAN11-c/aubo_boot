# 采摘批次摘要 `field_full_20260825_1720`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 17:21:04
- 结束时间：2026-08-25 17:21:45
- 总时长：40.6 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 14.95 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 17.23 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1720:target_0 | target_0 | — | 13.74 | — | 0.4 | 0.6 | — | — | 0.2 | 14.94 |
| field_full_20260825_1720:target_1 | target_1 | — | 17.02 | — | — | — | — | — | — | 17.02 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：96
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09441375732421875
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
| target_0 | 3 | 54 | READY | robot_not_static {"no_frame": 2, "missing_mask": 7, "stale_frame": 6, "same_stamp": 3, "robot_not_static": 36} |
| target_1 | 3 | 58 | COLLECTING | same_stamp {"no_frame": 2, "missing_mask": 23, "stale_frame": 6, "same_stamp": 3, "robot_not_static": 36} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 2/1）: insufficient_angular_baseline | 0.398,-0.653,0.546 | 0.282,-0.714,0.644 | — |
| target_1 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 2/1）: insufficient_angular_baseline | 0.265,-0.732,0.536 | 0.282,-0.714,0.644 | — |

## 运行性能统计

- 性能采样条数：39
- CPU %：均值 30.29 / 峰值 64.3
- 内存 %：均值 57.65 / 峰值 58.1
- GPU 利用率 %：均值 12.95 / 峰值 35.0
- GPU 显存 MB：均值 2152.97 / 峰值 2214.0
