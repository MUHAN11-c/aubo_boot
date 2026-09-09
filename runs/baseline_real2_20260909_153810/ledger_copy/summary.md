# 采摘批次摘要 `field_pregrasp_20260909_1542`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 15:42:05
- 结束时间：2026-09-09 15:43:15
- 总时长：70.3 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1542/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | skipped | 16.33 | skipped_unreachable |
| target_2 | 3 | skipped | 13.42 | skipped_unreachable |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1542 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| field_pregrasp_20260909_1542:target_1 | target_1 | — | 11.32 | — | 0.0 | 4.8 | — | — | 0.2 | 16.31 |
| field_pregrasp_20260909_1542:target_1 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| field_pregrasp_20260909_1542:target_2 | target_2 | — | 8.21 | — | 0.2 | 4.6 | — | — | 0.2 | 13.21 |
| field_pregrasp_20260909_1542:target_2 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |

## 事件统计

- 事件总数：16
- severity INFO：16
- `photo_pose_reached`：4
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2
- `targets_filtered`：4

## 感知统计

- 记录帧数：157
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09059906005859375
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
| target_1 | 3 | 48 | READY | robot_not_static {"no_frame": 1, "missing_mask": 1, "near_duplicate": 3, "stale_frame": 3, "robot_not_static": 40} |
| target_2 | 2 | 37 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "near_duplicate": 1, "robot_not_static": 34} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
corridor transfer 2 (0/1):  | 0.593,-0.695,0.715 | — | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
corridor transfer 2 (0/1):  | 0.255,-0.752,0.588 | 0.221,-0.753,0.639 | 0.292,-0.741,0.573 | 0.271,-0.745,0.594 |
| target_2 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
corridor transfer 2 (0/1):  | 0.517,-0.762,0.607 | 0.517,-0.769,0.643 | 0.510,-0.754,0.584 | 0.515,-0.760,0.613 |

## 运行性能统计

- 性能采样条数：68
- CPU %：均值 24.58 / 峰值 58.8
- 内存 %：均值 40.99 / 峰值 41.6
- GPU 利用率 %：均值 13.0 / 峰值 31.0
- GPU 显存 MB：均值 1897.72 / 峰值 2029.0

## 末端 TCP 轨迹

- 采样点数：336
- 路径长：0.5507 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1673 m
- Z 范围：0.673 ~ 0.7143 m（Δz=0.0）
