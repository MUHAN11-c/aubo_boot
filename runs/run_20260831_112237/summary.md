# 采摘批次摘要 `field_pregrasp_20260831`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 11:22:37
- 结束时间：2026-08-31 11:23:00
- 总时长：22.7 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 6.58 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 5.03 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831:target_0 | target_0 | — | 5.77 | — | 0.2 | 0.4 | — | — | 0.2 | 6.57 |
| field_pregrasp_20260831:target_0 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831:target_1 | target_1 | — | 4.22 | 0.2 | 0.0 | 0.2 | — | — | 0.2 | 4.82 |
| field_pregrasp_20260831:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：56
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.10919570922851562
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
| target_0 | 2 | 21 | READY | robot_not_static {"missing_mask": 1, "stale_frame": 2, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 14} |
| target_1 | 2 | 19 | READY | icp_reject {"missing_mask": 1, "same_stamp": 2, "robot_not_static": 13, "icp_reject": 3} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin to on-axis pregrasp (0/1):  | 0.223,-0.633,0.499 | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin to on-axis pregrasp (0/1):  | 0.265,-0.640,0.513 | 0.373,-0.674,0.549 | 0.380,-0.589,0.530 |

## 运行性能统计

- 性能采样条数：22
- CPU %：均值 43.12 / 峰值 74.2
- 内存 %：均值 47.21 / 峰值 47.8
- GPU 利用率 %：均值 12.95 / 峰值 31.0
- GPU 显存 MB：均值 1362.0 / 峰值 1369.0
