# 采摘批次摘要 `field_pregrasp_20260828c`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-28 15:37:04
- 结束时间：2026-08-28 15:37:29
- 总时长：25.3 s
- 终局计数：skipped=1

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 16.65 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260828c | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260828c:target_0 | target_0 | — | 16.63 | — | — | — | — | — | — | 16.63 |

## 事件统计

- 事件总数：4
- severity INFO：4
- `round_locked`：2
- `target_dispatched`：1
- `target_skipped`：1

## 感知统计

- 记录帧数：61
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.10848045349121094
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
| target_0 | 0 | 80 | COLLECTING | tsdf_integrate {"missing_mask": 1, "stale_frame": 12, "tsdf_integrate": 36, "robot_not_static": 31} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.459,-0.604,0.497 | 0.494,-0.638,0.583 | — |

## 运行性能统计

- 性能采样条数：24
- CPU %：均值 54.7 / 峰值 83.5
- 内存 %：均值 44.01 / 峰值 44.3
- GPU 利用率 %：均值 11.5 / 峰值 31.0
- GPU 显存 MB：均值 1519.17 / 峰值 1532.0
