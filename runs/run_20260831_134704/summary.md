# 采摘批次摘要 `field_pregrasp_20260831_1347`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-31 13:47:04
- 结束时间：2026-08-31 13:47:10
- 总时长：6.7 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 1.72 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 0.21 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260831_1347 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1347:target_0 | target_0 | — | 1.72 | — | — | — | — | — | — | 1.72 |
| field_pregrasp_20260831_1347:target_0 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260831_1347:target_1 | target_1 | — | 0.01 | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260831_1347:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：17
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08296966552734375
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
| target_0 | 1 | 1 | COLLECTING | same_stamp {"same_stamp": 1} |
| target_1 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；没有生成可用观察视点 | 0.201,-0.627,0.505 | 0.375,-0.684,0.559 | — |
| target_1 | observe | 否 | 工具档关；没有生成可用观察视点 | 0.262,-0.658,0.534 | 0.375,-0.684,0.559 | — |

## 运行性能统计

- 性能采样条数：6
- CPU %：均值 61.85 / 峰值 75.7
- 内存 %：均值 46.22 / 峰值 46.7
- GPU 利用率 %：均值 8.83 / 峰值 27.0
- GPU 显存 MB：均值 1534.67 / 峰值 1546.0
