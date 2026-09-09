# 采摘批次摘要 `field_pregrasp_20260909_1606`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 16:06:55
- 结束时间：2026-09-09 16:07:46
- 总时长：50.7 s
- 终局计数：skipped=1
- 会话根：`runs/field_pregrasp_20260909_1606/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 4 | skipped | 15.5 | observe_failed |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1606 | — | 0.31 | — | — | — | — | — | — | — | 0.31 |
| field_pregrasp_20260909_1606:target_5 | target_5 | — | 15.48 | — | — | — | — | — | — | 15.48 |
| field_pregrasp_20260909_1606:target_5 | — | 0.61 | — | — | — | — | — | — | — | 0.61 |

## 事件统计

- 事件总数：10
- severity INFO：10
- `photo_pose_reached`：3
- `round_locked`：3
- `target_dispatched`：1
- `target_skipped`：1
- `targets_filtered`：2

## 感知统计

- 记录帧数：121
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 4

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
| target_5 | 0 | 40 | COLLECTING | missing_mask {"missing_mask": 40} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.594,-0.694,0.731 | -0.900,-1.878,0.250 | — | — |
| target_5 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | -0.886,-1.906,0.177 | -0.900,-1.878,0.250 | — | — |

## 运行性能统计

- 性能采样条数：49
- CPU %：均值 35.51 / 峰值 62.4
- 内存 %：均值 47.91 / 峰值 48.5
- GPU 利用率 %：均值 14.27 / 峰值 38.0
- GPU 显存 MB：均值 1832.82 / 峰值 1934.0

## 末端 TCP 轨迹

- 采样点数：187
- 路径长：0.4296 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.2141 m
- Z 范围：0.654 ~ 0.7082 m（Δz=0.0）
