# 采摘批次摘要 `field_pregrasp_20260903_1604`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-03 16:06:16
- 结束时间：2026-09-03 16:08:13
- 总时长：116.2 s
- 终局计数：skipped=3
- 会话根：`runs/field_pregrasp_20260903_1604/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | skipped | 21.21 | observe_failed |
| target_2 | 3 | skipped | 16.82 | observe_failed |
| target_1 | 1 | skipped | 17.43 | observe_failed |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1604 | — | 0.51 | — | — | — | — | — | — | — | 0.51 |
| field_pregrasp_20260903_1604:target_0 | target_0 | — | 21.18 | — | — | — | — | — | — | 21.18 |
| field_pregrasp_20260903_1604:target_0 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| field_pregrasp_20260903_1604:target_2 | target_2 | — | 16.6 | — | — | — | — | — | — | 16.6 |
| field_pregrasp_20260903_1604:target_1 | target_1 | — | 17.21 | — | — | — | — | — | — | 17.21 |

## 事件统计

- 事件总数：21
- severity INFO：21
- `photo_pose_reached`：6
- `round_locked`：6
- `target_dispatched`：3
- `target_skipped`：3
- `targets_filtered`：3

## 感知统计

- 记录帧数：267
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

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
| target_0 | 0 | 93 | COLLECTING | robot_not_static {"missing_mask": 13, "neighbor_gap": 79, "robot_not_static": 1} |
| target_2 | 0 | 44 | COLLECTING | missing_mask {"missing_mask": 44} |
| target_1 | 0 | 43 | COLLECTING | missing_mask {"missing_mask": 43} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.328,-0.589,0.568 | 0.349,-0.592,0.600 | — | — |
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.624,-0.607,0.672 | 0.544,-0.603,0.712 | — | — |
| target_2 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.612,-0.658,0.569 | 0.595,-0.664,0.588 | — | — |

## 运行性能统计

- 性能采样条数：112
- CPU %：均值 36.92 / 峰值 72.2
- 内存 %：均值 42.45 / 峰值 42.8
- GPU 利用率 %：均值 13.54 / 峰值 34.0
- GPU 显存 MB：均值 2014.46 / 峰值 2045.0

## 末端 TCP 轨迹

- 采样点数：575
- 路径长：1.0333 m
- 起止弦：0.0001 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.242 m
- Z 范围：0.6554 ~ 0.7222 m（Δz=0.0）
