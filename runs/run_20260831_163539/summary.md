# 采摘批次摘要 `field_pregrasp_20260831_1636`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 16:35:39
- 结束时间：2026-08-31 16:36:22
- 总时长：42.6 s
- 终局计数：skipped=3

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 2.83 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 8.93 | {"code": "target_skipped"} |
| target_2 | 3 | skipped | 19.23 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1636:target_0 | target_0 | — | 2.63 | — | — | — | — | — | — | 2.63 |
| field_pregrasp_20260831_1636:target_1 | target_1 | — | 8.73 | — | — | — | — | — | — | 8.73 |
| field_pregrasp_20260831_1636:target_2 | target_2 | — | 19.01 | — | — | — | — | — | — | 19.01 |
| field_pregrasp_20260831_1636:target_2 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：11
- severity INFO：11
- `round_locked`：5
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：94
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.16117095947265625
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
| target_0 | 1 | 9 | COLLECTING | missing_mask {"missing_mask": 9} |
| target_1 | 3 | 37 | COLLECTING | same_stamp {"missing_mask": 11, "no_frame": 1, "same_stamp": 3, "robot_not_static": 26, "stale_frame": 4} |
| target_2 | 1 | 58 | COLLECTING | missing_mask {"missing_mask": 28, "same_stamp": 3, "robot_not_static": 22, "stale_frame": 5} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.219,-0.631,0.517 | 0.562,-0.783,0.503 | — |
| target_1 | lock | 否 | 工具档关；观察预算收口但重建未收敛: insufficient_angular_baseline | 0.306,-0.633,0.506 | 0.412,-0.677,0.554 | — |
| target_2 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.609,-0.766,0.441 | 0.562,-0.783,0.503 | — |

## 运行性能统计

- 性能采样条数：41
- CPU %：均值 31.21 / 峰值 97.1
- 内存 %：均值 47.63 / 峰值 48.1
- GPU 利用率 %：均值 12.07 / 峰值 35.0
- GPU 显存 MB：均值 1684.66 / 峰值 1714.0
