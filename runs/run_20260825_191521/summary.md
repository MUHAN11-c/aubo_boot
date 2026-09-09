# 采摘批次摘要 `field_full_20260825_1914`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-08-25 19:15:21
- 结束时间：2026-08-25 19:16:11
- 总时长：49.3 s
- 终局计数：skipped=1, succeeded=1

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 22.33 | {"code": "target_succeeded"} |
| target_1 | 2 | skipped | 12.03 | {"code": "target_skipped"} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_full_20260825_1914 | — | 0.0 | — | — | — | — | — | — | — | 0.0 |
| field_full_20260825_1914:target_0 | target_0 | — | 5.91 | — | 0.2 | 12.2 | — | 3.8 | 0.2 | 22.31 |
| field_full_20260825_1914:target_1 | target_1 | — | 11.82 | — | — | — | — | — | — | 11.82 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：1
- `target_succeeded`：1

## 感知统计

- 记录帧数：119
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09298324584960938
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
| target_0 | 2 | 22 | READY | robot_not_static {"no_frame": 1, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 17} |
| target_1 | 0 | 31 | COLLECTING | missing_mask {"missing_mask": 31} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 抓取进入 |
|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.353,-0.670,0.546 | 0.283,-0.728,0.641 | — |
| target_1 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.237,-0.742,0.558 | 0.283,-0.728,0.641 | — |

## 运行性能统计

- 性能采样条数：47
- CPU %：均值 19.18 / 峰值 58.6
- 内存 %：均值 55.3 / 峰值 55.7
- GPU 利用率 %：均值 11.62 / 峰值 33.0
- GPU 显存 MB：均值 2080.04 / 峰值 2102.0
