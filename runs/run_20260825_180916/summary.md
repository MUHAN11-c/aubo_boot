# 采摘批次摘要 `field_full_20260825_1808`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 18:09:16
- 结束时间：2026-08-25 18:09:38
- 总时长：22.2 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 6.32 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 4.02 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1808 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_full_20260825_1808:target_0 | target_0 | — | 6.32 | — | — | — | — | — | — | 6.32 |
| field_full_20260825_1808:target_1 | target_1 | — | 3.82 | — | — | — | — | — | — | 3.82 |
| field_full_20260825_1808:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：54
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08463859558105469
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
| target_0 | 3 | 30 | COLLECTING | same_stamp {"missing_mask": 1, "stale_frame": 6, "same_stamp": 5, "near_duplicate": 1, "robot_not_static": 17} |
| target_1 | 3 | 20 | COLLECTING | same_stamp {"missing_mask": 1, "same_stamp": 4, "robot_not_static": 15} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 1/1）: insufficient_angular_baseline | 0.386,-0.657,0.550 | 0.282,-0.719,0.633 | — |
| target_1 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 1/1）: insufficient_angular_baseline | 0.237,-0.682,0.560 | 0.282,-0.719,0.633 | — |

## 运行性能统计

- 性能采样条数：22
- CPU %：均值 25.33 / 峰值 60.9
- 内存 %：均值 52.95 / 峰值 53.2
- GPU 利用率 %：均值 10.27 / 峰值 28.0
- GPU 显存 MB：均值 2263.86 / 峰值 2278.0
