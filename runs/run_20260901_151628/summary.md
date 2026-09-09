# 采摘批次摘要 `field_pregrasp_20260901_1540`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-01 15:16:28
- 结束时间：2026-09-01 15:17:13
- 总时长：44.7 s
- 终局计数：skipped=1
- 配对账本：`runs/field_pregrasp_20260901_1540/ledger.json`（结构化 outcome 细节在彼处）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 2 | skipped | 22.08 | observe_failed |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_1540 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| field_pregrasp_20260901_1540:target_2 | target_2 | — | 22.07 | — | — | — | — | — | — | 22.07 |
| field_pregrasp_20260901_1540:target_2 | — | 0.6 | — | — | — | — | — | — | — | 0.6 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `round_locked`：3
- `target_dispatched`：1
- `target_skipped`：1
- `targets_filtered`：3

## 感知统计

- 记录帧数：109
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
| target_2 | 0 | 111 | COLLECTING | neighbor_gap {"no_frame": 4, "missing_mask": 2, "neighbor_gap": 105} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_3 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.573,-0.603,0.706 | 0.542,-0.616,0.729 | — | — |
| target_2 | lock | 否 | 工具档关；达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views | 0.585,-0.577,0.637 | 0.542,-0.616,0.729 | — | — |

## 运行性能统计

- 性能采样条数：43
- CPU %：均值 45.68 / 峰值 75.4
- 内存 %：均值 51.64 / 峰值 52.0
- GPU 利用率 %：均值 14.88 / 峰值 37.0
- GPU 显存 MB：均值 2040.28 / 峰值 2122.0

## 末端 TCP 轨迹

- 采样点数：233
- 路径长：0.3203 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1486 m
- Z 范围：0.6844 ~ 0.7082 m（Δz=0.0）
