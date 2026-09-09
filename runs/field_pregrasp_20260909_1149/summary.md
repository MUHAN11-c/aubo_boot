# 采摘批次摘要 `field_pregrasp_20260909_1149`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 11:49:54
- 结束时间：2026-09-09 11:54:07
- 总时长：253.8 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1149/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 33.68 | observe_failed |
| target_1 | 2 | skipped | 180.0 | build_rejected |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1149 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1149:target_0 | target_0 | — | 33.65 | — | — | — | — | — | — | 33.65 |
| field_pregrasp_20260909_1149:target_1 | target_1 | — | 180.0 | — | — | — | — | — | — | 180.0 |

## 事件统计

- 事件总数：11
- severity INFO：11
- `photo_pose_reached`：4
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：599
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：COLLECTING
- target_id：target_0
- captured_views：0
- rejected_views：1
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
| target_0 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: reconstruction_data_stale | 0.572,-0.622,0.555 | 0.550,-0.629,0.593 | — | — |
| target_1 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: reconstruction_data_stale | 0.248,-0.700,0.585 | 0.550,-0.629,0.593 | — | — |

## 运行性能统计

- 性能采样条数：245
- CPU %：均值 37.26 / 峰值 61.7
- 内存 %：均值 40.65 / 峰值 41.3
- GPU 利用率 %：均值 12.64 / 峰值 34.0
- GPU 显存 MB：均值 1348.01 / 峰值 1659.0

## 末端 TCP 轨迹

- 采样点数：469
- 路径长：0.3694 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1641 m
- Z 范围：0.6499 ~ 0.7082 m（Δz=0.0）
