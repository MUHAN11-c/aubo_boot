# 采摘批次摘要 `field_pregrasp_20260828e`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-28 17:18:50
- 结束时间：2026-08-28 17:21:55
- 总时长：184.9 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 2.02 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 180.0 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260828e:target_0 | target_0 | — | 2.02 | — | — | — | — | — | — | 2.02 |
| field_pregrasp_20260828e:target_1 | target_1 | — | 180.0 | — | — | — | — | — | — | 180.0 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：429
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- cloud_points：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：0
- skip_reasons：{}
- last_skip_code：
- last_skip_reason：

## 逐目标重建视角

- （无逐目标诊断）

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.206,-0.624,0.505 | — | — |
| target_1 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.259,-0.658,0.532 | — | — |

## 运行性能统计

- 性能采样条数：178
- CPU %：均值 58.49 / 峰值 100.0
- 内存 %：均值 40.69 / 峰值 43.4
- GPU 利用率 %：均值 19.72 / 峰值 71.0
- GPU 显存 MB：均值 2583.75 / 峰值 7981.0
