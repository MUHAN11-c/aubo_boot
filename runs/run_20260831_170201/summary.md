# 采摘批次摘要 `field_pregrasp_20260831_1700`

## 批次概览

- 终局状态：RECOVERY_REQUIRED
- 复扫轮数：1
- 开始时间：2026-08-31 17:02:01
- 结束时间：2026-08-31 17:02:39
- 总时长：38.3 s
- 终局计数：unfinished=1

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | unfinished | — |  |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1700 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1700:target_1 | target_1 | — | 11.02 | — | 0.41 | 21.79 | — | — | 0.2 | 33.42 |

## 事件统计

- 事件总数：2
- severity INFO：2
- `round_locked`：1
- `target_dispatched`：1

## 感知统计

- 记录帧数：91
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：2
- rejected_views：44
- tf_failures：0
- tf_latency_ms：0.08082389831542969
- cloud_points：14586
- max_baseline_deg：10.824727418280142
- mean_nearest_baseline_deg：10.824727418280142
- tsdf_points：1620
- tsdf_integrate_time_s：0.004992246627807617
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：46
- skip_reasons：{'missing_mask': 2, 'same_stamp': 1, 'near_duplicate': 2, 'stale_frame': 4, 'robot_not_static': 37}
- last_skip_code：robot_not_static
- last_skip_reason：机器人未静止：最大关节速度 0.2769 rad/s > 0.03

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 2 | 46 | READY | robot_not_static {"missing_mask": 2, "same_stamp": 1, "near_duplicate": 2, "stale_frame": 4, "robot_not_static": 37} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_1 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.261,-0.639,0.528 | 0.264,-0.616,0.617 | 0.238,-0.649,0.481 |

## 运行性能统计

- 性能采样条数：37
- CPU %：均值 42.1 / 峰值 99.0
- 内存 %：均值 50.31 / 峰值 50.8
- GPU 利用率 %：均值 9.92 / 峰值 34.0
- GPU 显存 MB：均值 1733.89 / 峰值 1881.0
