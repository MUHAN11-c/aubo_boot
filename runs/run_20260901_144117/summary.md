# 采摘批次摘要 `field_pregrasp_20260901_1440`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-01 14:41:17
- 结束时间：2026-09-01 14:43:05
- 总时长：108.2 s
- 终局计数：skipped=4
- 配对账本：`runs/field_pregrasp_20260901_1440/ledger.json`（结构化 outcome 细节在彼处）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 11.11 | build_start_timeout |
| target_13 | 2 | skipped | 11.03 | skipped_unreachable |
| target_12 | 3 | skipped | 23.84 | skipped_unreachable |
| target_14 | 4 | skipped | 20.02 | observe_failed |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_1440:target_0 | target_0 | — | 11.1 | — | — | — | — | — | — | 11.1 |
| field_pregrasp_20260901_1440:target_13 | target_13 | — | 7.63 | — | 0.2 | 3.0 | — | — | 0.2 | 11.03 |
| field_pregrasp_20260901_1440:target_12 | target_12 | — | 16.22 | — | 0.2 | 7.0 | — | — | 0.2 | 23.62 |
| field_pregrasp_20260901_1440:target_14 | target_14 | — | 19.81 | — | — | — | — | — | — | 19.81 |

## 事件统计

- 事件总数：13
- severity INFO：13
- `round_locked`：5
- `target_dispatched`：4
- `target_skipped`：4

## 感知统计

- 记录帧数：257
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08940696716308594
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
| target_0 | 1 | 23 | COLLECTING | stale_frame {"no_frame": 2, "missing_mask": 6, "stale_frame": 11, "same_stamp": 1, "near_duplicate": 3} |
| target_13 | 2 | 34 | READY | robot_not_static {"no_frame": 1, "missing_mask": 2, "same_stamp": 1, "robot_not_static": 30} |
| target_12 | 2 | 77 | READY | robot_not_static {"no_frame": 2, "missing_mask": 3, "stale_frame": 3, "same_stamp": 1, "near_duplicate": 1, "robot_not_static": 67} |
| target_14 | 0 | 62 | COLLECTING | missing_mask {"missing_mask": 32, "robot_not_static": 17, "stale_frame": 2, "target_drift": 11} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.296,-0.635,0.493 | -0.935,-1.892,0.170 | — | — |
| target_13 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
ptp to on-axis pregrasp (0/1):  | 0.622,-0.561,0.645 | 0.542,-0.617,0.733 | 0.543,-0.466,0.582 | 0.538,-0.536,0.654 |
| target_12 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
ptp to on-axis pregrasp (0/1):  | 0.573,-0.697,0.471 | 0.558,-0.687,0.591 | 0.613,-0.776,0.401 | 0.583,-0.733,0.486 |
| target_14 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | -0.977,-1.838,0.186 | -0.935,-1.892,0.170 | — | — |

## 运行性能统计

- 性能采样条数：104
- CPU %：均值 33.51 / 峰值 96.9
- 内存 %：均值 53.66 / 峰值 54.1
- GPU 利用率 %：均值 13.56 / 峰值 36.0
- GPU 显存 MB：均值 2010.31 / 峰值 2117.0

## 末端 TCP 轨迹

- 采样点数：521
- 路径长：0.8775 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.2139 m
- Z 范围：0.6451 ~ 0.7082 m（Δz=0.0）
