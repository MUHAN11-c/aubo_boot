# 采摘批次摘要 `field_full_20260825_1640`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 16:42:04
- 结束时间：2026-08-25 16:45:40
- 总时长：216.0 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 22.52 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 180.15 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1640:target_0 | target_0 | — | 22.51 | — | — | — | — | — | — | 22.51 |
| field_full_20260825_1640:target_1 | target_1 | — | 180.15 | — | — | — | — | — | — | 180.15 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：504
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：COLLECTING
- target_id：target_0
- captured_views：5
- rejected_views：37
- tf_failures：0
- tf_latency_ms：0.09298324584960938
- cloud_points：43299
- max_baseline_deg：10.106109589686216
- mean_nearest_baseline_deg：10.106109589686216
- tsdf_points：3094
- tsdf_integrate_time_s：0.031221866607666016
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：40
- skip_reasons：{'missing_mask': 1, 'stale_frame': 6, 'near_duplicate': 3, 'robot_not_static': 28, 'same_stamp': 2}
- last_skip_code：same_stamp
- last_skip_reason：缓存帧未更新（与上次采帧同帧），请等下一帧

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 5 | 40 | COLLECTING | same_stamp {"missing_mask": 1, "stale_frame": 6, "near_duplicate": 3, "robot_not_static": 28, "same_stamp": 2} |

## 运行性能统计

- 性能采样条数：208
- CPU %：均值 24.37 / 峰值 68.3
- 内存 %：均值 58.19 / 峰值 60.0
- GPU 利用率 %：均值 14.57 / 峰值 39.0
- GPU 显存 MB：均值 1881.96 / 峰值 1942.0
