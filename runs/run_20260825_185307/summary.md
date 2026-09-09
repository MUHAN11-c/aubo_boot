# 采摘批次摘要 `field_full_20260825_1851`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 18:53:07
- 结束时间：2026-08-25 18:53:52
- 总时长：44.4 s
- 终局计数：skipped=1, succeeded=1

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 4.62 | {"code": "target_skipped"} |
| target_1 | 2 | succeeded | 24.43 | {"code": "target_succeeded"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1851:target_0 | target_0 | — | 4.42 | — | — | — | — | — | — | 4.42 |
| field_full_20260825_1851:target_1 | target_1 | — | 3.82 | — | 0.2 | 15.6 | — | 4.4 | 0.2 | 24.22 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：1
- `target_succeeded`：1

## 感知统计

- 记录帧数：108
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08130073547363281
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
| target_0 | 3 | 22 | COLLECTING | same_stamp {"missing_mask": 1, "same_stamp": 4, "robot_not_static": 17} |
| target_1 | 2 | 16 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "robot_not_static": 14} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；抓取未许可：reconstruction_not_ready | 0.388,-0.655,0.547 | — | — |
| target_1 | lock | 是 | 工具档关；抓取已许可，等待进入靠近 | 0.249,-0.724,0.552 | 0.282,-0.718,0.638 | 0.298,-0.699,0.531 |

## 运行性能统计

- 性能采样条数：42
- CPU %：均值 21.2 / 峰值 60.9
- 内存 %：均值 52.34 / 峰值 52.9
- GPU 利用率 %：均值 15.07 / 峰值 35.0
- GPU 显存 MB：均值 2089.93 / 峰值 2103.0
