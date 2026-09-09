# 采摘批次摘要 `field_pregrasp_20260831_1405`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 14:05:32
- 结束时间：2026-08-31 14:06:14
- 总时长：42.7 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 13.16 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 17.04 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1405:target_0 | target_0 | — | 12.35 | — | 0.4 | 0.2 | — | — | 0.2 | 13.15 |
| field_pregrasp_20260831_1405:target_1 | target_1 | — | 15.01 | — | 1.4 | 0.2 | — | — | 0.2 | 16.81 |
| field_pregrasp_20260831_1405:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：95
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09632110595703125
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
| target_0 | 4 | 54 | READY | robot_not_static {"missing_mask": 1, "stale_frame": 2, "same_stamp": 3, "near_duplicate": 3, "robot_not_static": 45} |
| target_1 | 3 | 39 | READY | same_stamp {"missing_mask": 1, "same_stamp": 2, "robot_not_static": 36} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin align tool z (0/1):  | 0.211,-0.631,0.496 | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin align tool z (0/1):  | 0.273,-0.644,0.509 | 0.370,-0.675,0.555 | 0.411,-0.603,0.521 |

## 运行性能统计

- 性能采样条数：42
- CPU %：均值 43.72 / 峰值 82.4
- 内存 %：均值 47.06 / 峰值 47.6
- GPU 利用率 %：均值 13.17 / 峰值 33.0
- GPU 显存 MB：均值 1436.69 / 峰值 1449.0
