# 采摘批次摘要 `field_pregrasp_20260828`

## 批次概览

- 终局状态：未知/未终止
- 复扫轮数：1
- 开始时间：2026-08-28 12:00:02
- 结束时间：2026-08-28 12:00:57
- 总时长：54.7 s
- 终局计数：skipped=1, unfinished=1

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_10 | 1 | skipped | 23.8 | {"code": "target_skipped"} |
| target_1 | 2 | unfinished | — |  |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260828:target_10 | target_10 | — | 23.77 | — | — | — | — | — | — | 23.77 |
| field_pregrasp_20260828:target_10 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260828:target_1 | target_1 | — | 21.52 | — | — | — | — | — | — | 21.52 |

## 事件统计

- 事件总数：5
- severity INFO：5
- `round_locked`：2
- `target_dispatched`：2
- `target_skipped`：1

## 感知统计

- 记录帧数：88
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：COLLECTING
- target_id：target_10
- captured_views：1
- rejected_views：1
- tf_failures：0
- tf_latency_ms：0.09512901306152344
- cloud_points：0
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：1
- skip_reasons：{'missing_mask': 1}
- last_skip_code：missing_mask
- last_skip_reason：缺少所选 target_id 的同时间戳掩膜

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_10 | 1 | 1 | COLLECTING | missing_mask {"missing_mask": 1} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_1 | observe | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: reconstruction_data_stale | -0.082,-0.711,0.567 | 0.275,-0.651,0.635 | — |
| target_10 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: reconstruction_data_stale | 0.249,-0.640,0.526 | 0.275,-0.651,0.635 | — |

## 运行性能统计

- 性能采样条数：93
- CPU %：均值 11.4 / 峰值 45.6
- 内存 %：均值 38.53 / 峰值 39.4
- GPU 利用率 %：均值 8.2 / 峰值 43.0
- GPU 显存 MB：均值 1404.81 / 峰值 1428.0
