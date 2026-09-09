# 采摘批次摘要 `field_pregrasp_20260901_0907`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-01 09:07:27
- 结束时间：2026-09-01 09:07:57
- 总时长：30.1 s
- 终局计数：skipped=2

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 2.82 | {"code": "target_skipped"} |
| target_1 | 2 | skipped | 16.43 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_0907:target_0 | target_0 | — | 2.62 | — | — | — | — | — | — | 2.62 |
| field_pregrasp_20260901_0907:target_1 | target_1 | — | 16.21 | — | — | — | — | — | — | 16.21 |
| field_pregrasp_20260901_0907:target_1 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：72
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

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 8 | COLLECTING | missing_mask {"missing_mask": 8} |
| target_1 | 0 | 41 | COLLECTING | missing_mask {"missing_mask": 41} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.272,-0.647,0.505 | 0.568,-0.775,0.526 | — | — |
| target_1 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.622,-0.724,0.467 | 0.568,-0.775,0.526 | — | — |

## 运行性能统计

- 性能采样条数：29
- CPU %：均值 21.0 / 峰值 59.0
- 内存 %：均值 32.48 / 峰值 32.9
- GPU 利用率 %：均值 10.83 / 峰值 33.0
- GPU 显存 MB：均值 1319.86 / 峰值 1331.0

## 末端 TCP 轨迹

- 采样点数：198
- 路径长：0.3758 m
- 起止弦：0.0004 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1854 m
- Z 范围：0.6526 ~ 0.7111 m（Δz=0.0001）
