# 采摘批次摘要 `field_pregrasp_20260828f`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-28 17:26:01
- 结束时间：2026-08-28 17:26:32
- 总时长：31.4 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 1.63 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 23.38 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260828f | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260828f:target_0 | target_0 | — | 1.62 | — | — | — | — | — | — | 1.62 |
| field_pregrasp_20260828f:target_0 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260828f:target_1 | target_1 | — | 23.38 | — | — | — | — | — | — | 23.38 |
| field_pregrasp_20260828f:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：71
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08630752563476562
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
| target_0 | 0 | 6 | COLLECTING | missing_mask {"missing_mask": 6} |
| target_1 | 0 | 95 | COLLECTING | stale_frame {"missing_mask": 5, "tsdf_integrate": 31, "robot_not_static": 37, "stale_frame": 22} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.206,-0.625,0.505 | 0.376,-0.684,0.559 | — |
| target_1 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.271,-0.645,0.513 | 0.376,-0.684,0.559 | — |

## 运行性能统计

- 性能采样条数：30
- CPU %：均值 60.31 / 峰值 99.5
- 内存 %：均值 40.81 / 峰值 41.1
- GPU 利用率 %：均值 14.47 / 峰值 34.0
- GPU 显存 MB：均值 1558.63 / 峰值 1569.0
