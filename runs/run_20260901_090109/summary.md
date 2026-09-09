# 采摘批次摘要 `field_pregrasp_20260901_0900`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-01 09:01:09
- 结束时间：2026-09-01 09:01:18
- 总时长：8.7 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 1.47 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 2.82 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_0900 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_pregrasp_20260901_0900:target_0 | target_0 | — | 1.47 | — | — | — | — | — | — | 1.47 |
| field_pregrasp_20260901_0900:target_1 | target_1 | — | 2.82 | — | — | — | — | — | — | 2.82 |
| field_pregrasp_20260901_0900:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：18
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：0
- rejected_views：7
- tf_failures：0
- cloud_points：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：5
- skip_reasons：{'missing_mask': 5}
- last_skip_code：missing_mask
- last_skip_reason：缺少所选 target_id 的同时间戳掩膜

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 6 | COLLECTING | missing_mask {"missing_mask": 6} |
| target_1 | 0 | 5 | COLLECTING | missing_mask {"missing_mask": 5} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；剩余候选视点均不可达或规划失败 | 0.273,-0.648,0.505 | 0.570,-0.771,0.522 | — | — |
| target_1 | lock | 否 | 工具档关；剩余候选视点均不可达或规划失败 | 0.626,-0.731,0.451 | 0.570,-0.771,0.522 | — | — |

## 运行性能统计

- 性能采样条数：9
- CPU %：均值 31.96 / 峰值 51.8
- 内存 %：均值 32.14 / 峰值 32.4
- GPU 利用率 %：均值 14.78 / 峰值 32.0
- GPU 显存 MB：均值 1344.0 / 峰值 1356.0

## 末端 TCP 轨迹

- 采样点数：14
- 路径长：0.0011 m
- 起止弦：0.0011 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.7075 ~ 0.7082 m（Δz=0.0007）
