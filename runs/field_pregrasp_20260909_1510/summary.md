# 采摘批次摘要 `field_pregrasp_20260909_1510`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 15:10:47
- 结束时间：2026-09-09 15:12:34
- 总时长：106.4 s
- 终局计数：skipped=3
- 会话根：`runs/field_pregrasp_20260909_1510/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | skipped | 12.28 | skipped_unreachable |
| target_2 | 3 | skipped | 21.63 | skipped_unreachable |
| target_3 | 4 | skipped | 20.02 | observe_failed |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1510 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| field_pregrasp_20260909_1510:target_1 | target_1 | — | 7.87 | — | 0.2 | 4.0 | — | — | 0.2 | 12.27 |
| field_pregrasp_20260909_1510:target_1 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| field_pregrasp_20260909_1510:target_2 | target_2 | — | 13.81 | — | 0.2 | 7.2 | — | — | 0.2 | 21.41 |
| field_pregrasp_20260909_1510:target_2 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| field_pregrasp_20260909_1510:target_3 | target_3 | — | 19.81 | — | — | — | — | — | — | 19.81 |
| field_pregrasp_20260909_1510:target_3 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |

## 事件统计

- 事件总数：20
- severity INFO：20
- `photo_pose_reached`：5
- `round_locked`：5
- `target_dispatched`：3
- `target_skipped`：3
- `targets_filtered`：4

## 感知统计

- 记录帧数：250
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09107589721679688
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
| target_1 | 3 | 33 | READY | robot_not_static {"no_frame": 2, "near_duplicate": 3, "robot_not_static": 28} |
| target_2 | 2 | 57 | READY | robot_not_static {"no_frame": 1, "missing_mask": 1, "same_stamp": 1, "robot_not_static": 53, "stale_frame": 1} |
| target_3 | 0 | 51 | COLLECTING | missing_mask {"missing_mask": 51} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.592,-0.695,0.733 | -0.905,-1.888,0.162 | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
cartesian corridor 2/4 (0/1):  | 0.244,-0.756,0.591 | 0.219,-0.755,0.638 | 0.267,-0.764,0.553 | 0.254,-0.761,0.579 |
| target_2 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
cartesian corridor 2/4 (0/1):  | 0.515,-0.758,0.603 | 0.519,-0.770,0.646 | 0.491,-0.736,0.595 | 0.503,-0.749,0.619 |
| target_3 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | -0.945,-1.876,0.126 | -0.905,-1.888,0.162 | — | — |

## 运行性能统计

- 性能采样条数：103
- CPU %：均值 19.66 / 峰值 56.7
- 内存 %：均值 45.2 / 峰值 46.9
- GPU 利用率 %：均值 12.39 / 峰值 37.0
- GPU 显存 MB：均值 1338.8 / 峰值 1388.0

## 末端 TCP 轨迹

- 采样点数：539
- 路径长：0.8857 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.2135 m
- Z 范围：0.6447 ~ 0.7096 m（Δz=0.0）
