# 采摘批次摘要 `field_pregrasp_20260909_1349`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 13:49:18
- 结束时间：2026-09-09 13:50:22
- 总时长：64.4 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1349/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 15.05 | skipped_unreachable |
| target_1 | 2 | skipped | 10.42 | skipped_unreachable |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1349 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1349:target_0 | target_0 | — | 9.84 | — | 0.2 | 4.8 | — | — | 0.2 | 15.04 |
| field_pregrasp_20260909_1349:target_0 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1349:target_1 | target_1 | — | 6.41 | — | 0.2 | 3.4 | — | — | 0.2 | 10.21 |

## 事件统计

- 事件总数：12
- severity INFO：12
- `photo_pose_reached`：4
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：156
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08368492126464844
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
| target_0 | 2 | 43 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 37, "stale_frame": 1} |
| target_1 | 2 | 28 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "robot_not_static": 26} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin to on-axis pregrasp (0/1):  | 0.569,-0.613,0.548 | — | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
lin to on-axis pregrasp (0/1):  | 0.236,-0.700,0.566 | 0.224,-0.700,0.621 | 0.255,-0.704,0.549 | 0.243,-0.700,0.576 |

## 运行性能统计

- 性能采样条数：63
- CPU %：均值 31.63 / 峰值 56.6
- 内存 %：均值 40.27 / 峰值 40.6
- GPU 利用率 %：均值 11.73 / 峰值 34.0
- GPU 显存 MB：均值 1344.95 / 峰值 1350.0

## 末端 TCP 轨迹

- 采样点数：330
- 路径长：0.5894 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.164 m
- Z 范围：0.6581 ~ 0.7086 m（Δz=0.0）
