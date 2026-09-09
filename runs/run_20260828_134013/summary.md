# 采摘批次摘要 `field_pregrasp_20260828b`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-28 13:40:13
- 结束时间：2026-08-28 13:40:38
- 总时长：24.2 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | skipped | 1.22 | {"code": "target_skipped"} |
| target_4 | 1 | skipped | 11.83 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260828b:target_0 | target_0 | — | 1.22 | — | — | — | — | — | — | 1.22 |
| field_pregrasp_20260828b:target_4 | target_4 | — | 11.62 | — | 0.0 | — | — | — | 0.2 | 11.82 |
| field_pregrasp_20260828b:target_4 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：55
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.0705718994140625
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
| target_4 | 4 | 40 | READY | robot_not_static {"missing_mask": 11, "robot_not_static": 28, "same_stamp": 1} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | discover | 否 | 抓取档关、工具档关；重建COLLECTING，尚未给出抓取许可 | — | 0.492,-0.638,0.581 | — |
| target_4 | lock | 否 | 工具档关；GraspDecision.allowed=false，禁止降级接触: refined_quality_not_allowed | 0.255,-0.622,0.544 | — | — |

## 运行性能统计

- 性能采样条数：23
- CPU %：均值 24.7 / 峰值 67.0
- 内存 %：均值 51.09 / 峰值 51.2
- GPU 利用率 %：均值 10.83 / 峰值 32.0
- GPU 显存 MB：均值 1440.61 / 峰值 1459.0
