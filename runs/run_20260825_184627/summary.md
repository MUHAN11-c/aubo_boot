# 采摘批次摘要 `field_full_20260825_1846`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 18:46:27
- 结束时间：2026-08-25 18:46:59
- 总时长：32.3 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 15.21 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 5.42 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1846 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_full_20260825_1846:target_0 | target_0 | — | 15.21 | — | — | — | — | — | — | 15.21 |
| field_full_20260825_1846:target_1 | target_1 | — | 4.41 | — | 0.2 | 0.4 | — | — | 0.2 | 5.21 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：77
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.10418891906738281
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
| target_0 | 1 | 22 | COLLECTING | robot_not_static {"missing_mask": 1, "same_stamp": 1, "stale_frame": 17, "robot_not_static": 3} |
| target_1 | 2 | 17 | READY | robot_not_static {"missing_mask": 2, "same_stamp": 1, "robot_not_static": 14} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；MTC 接近/插入失败: 到入口失败: MTC short-path guard rejected: 预计时长 12.6061s > 12s；降级抓取(degraded_anchor) | 0.395,-0.653,0.544 | — | — |
| target_1 | lock | 否 | 工具档关；MTC 接近/插入失败: 到入口失败: MTC short-path guard rejected: 预计时长 12.6061s > 12s；降级抓取(degraded_anchor) | 0.243,-0.690,0.539 | 0.281,-0.718,0.639 | — |

## 运行性能统计

- 性能采样条数：31
- CPU %：均值 34.18 / 峰值 82.8
- 内存 %：均值 50.87 / 峰值 51.5
- GPU 利用率 %：均值 17.35 / 峰值 37.0
- GPU 显存 MB：均值 2060.97 / 峰值 2081.0
