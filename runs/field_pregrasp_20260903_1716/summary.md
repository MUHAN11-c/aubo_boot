# 采摘批次摘要 `field_pregrasp_20260903_1716`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-03 17:17:15
- 结束时间：2026-09-03 17:18:02
- 总时长：46.8 s
- 终局计数：skipped=1
- 会话根：`runs/field_pregrasp_20260903_1716/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 15.03 | skipped_unreachable |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1716 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260903_1716:target_0 | target_0 | — | 10.21 | — | 0.4 | 4.0 | — | — | 0.2 | 14.81 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `photo_pose_reached`：3
- `round_locked`：2
- `target_dispatched`：1
- `target_skipped`：1

## 感知统计

- 记录帧数：102
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.10275840759277344
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
| target_0 | 2 | 41 | READY | robot_not_static {"no_frame": 1, "missing_mask": 2, "same_stamp": 1, "near_duplicate": 11, "robot_not_static": 20, "stale_frame": 6} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
ptp to on-axis pregrasp (0/1):  | 0.384,-0.586,0.562 | — | — | — |

## 运行性能统计

- 性能采样条数：45
- CPU %：均值 37.94 / 峰值 90.5
- 内存 %：均值 47.23 / 峰值 47.4
- GPU 利用率 %：均值 16.29 / 峰值 38.0
- GPU 显存 MB：均值 2205.62 / 峰值 2237.0

## 末端 TCP 轨迹

- 采样点数：181
- 路径长：0.2447 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1199 m
- Z 范围：0.6624 ~ 0.7106 m（Δz=0.0）
