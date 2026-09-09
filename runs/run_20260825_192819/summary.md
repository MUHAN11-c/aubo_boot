# 采摘批次摘要 `field_full_20260825_1928`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 19:28:19
- 结束时间：2026-08-25 19:29:06
- 总时长：46.3 s
- 终局计数：skipped=3

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 16.72 | {"code": "target_skipped"} |
| target_2 | 2 | skipped | 11.82 | {"code": "target_skipped"} |
| target_0 | 3 | skipped | 0.42 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1928 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_full_20260825_1928:target_1 | target_1 | — | 16.7 | — | — | — | — | — | — | 16.7 |
| field_full_20260825_1928:target_2 | target_2 | — | 11.62 | — | — | — | — | — | — | 11.62 |
| field_full_20260825_1928:target_0 | target_0 | — | 0.23 | — | — | — | — | — | — | 0.23 |
| field_full_20260825_1928:target_0 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：12
- severity INFO：12
- `round_locked`：6
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：113
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：COLLECTING
- target_id：target_0
- captured_views：1
- rejected_views：3
- tf_failures：0
- tf_latency_ms：0.10633468627929688
- cloud_points：7615
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- tsdf_points：1529
- tsdf_integrate_time_s：0.008023977279663086
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：1
- skip_reasons：{'missing_mask': 1}
- last_skip_code：missing_mask
- last_skip_reason：缺少所选 target_id 的同时间戳掩膜

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 0 | 73 | COLLECTING | neighbor_gap {"missing_mask": 9, "neighbor_gap": 64} |
| target_2 | 0 | 40 | COLLECTING | missing_mask {"missing_mask": 17, "neighbor_gap": 23} |
| target_0 | 1 | 1 | COLLECTING | missing_mask {"missing_mask": 1} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；剩余候选视点均不可达或规划失败 | 0.321,-0.714,0.569 | 0.552,-0.650,0.595 | — |
| target_2 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.381,-0.714,0.629 | 0.397,-0.758,0.702 | — |
| target_0 | lock | 否 | 工具档关；剩余候选视点均不可达或规划失败 | 0.457,-0.616,0.523 | 0.552,-0.650,0.595 | — |

## 运行性能统计

- 性能采样条数：44
- CPU %：均值 23.1 / 峰值 46.2
- 内存 %：均值 54.23 / 峰值 54.5
- GPU 利用率 %：均值 12.32 / 峰值 36.0
- GPU 显存 MB：均值 2062.61 / 峰值 2070.0
