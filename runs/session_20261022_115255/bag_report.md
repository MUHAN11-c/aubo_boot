# 采摘会话分析 `session_20261022_115255`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_115255/bag`
- 开始时间：2026-10-22 11:52:55
- 结束时间：2026-10-22 11:55:51
- 总时长：175.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| sim4_0_1792641214 | RECOVERY_REQUIRED | 2026-10-22 11:53:50 | 2026-10-22 11:54:12 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/sim4_0_1792641214/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.75 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 1 | = 0 | ✗ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### sim4_0_1792641214

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | failed | 16.08 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| sim4_0_1792641214 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| sim4_0_1792641214:target_0 | target_0 | — | 0.24 | — | 0.63 | 15.2 | — | — | — | 16.07 |

## 事件统计

- 事件总数：12
- severity AUDIT：4
- severity ERROR：2
- severity INFO：6
- `enables_changed`：2
- `photo_pose_reached`：2
- `recovery_required`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：2
- 帧间隔中位数：1.334 s（≈0.75 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_0
- captured_views：1
- rejected_views：1287
- tf_failures：1
- tf_latency_ms：1004.2335987091064
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | approach | 否 | 工具档关；预抓取已验证 · 周期失败 · 等待人工恢复确认 | 0.413,-0.668,0.679 | 0.405,-0.666,0.717 | — | — |

## 运行性能统计

- 性能采样条数：165
- CPU %：均值 44.0 / 峰值 88.1
- 内存 %：均值 42.05 / 峰值 43.4
- GPU 利用率 %：均值 27.07 / 峰值 54.0
- GPU 显存 MB：均值 1675.58 / 峰值 1786.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：137
- 路径长：0.739 m
- 起止弦：0.4436 m
- 绕行比（路径/弦）：1.666（直线≈1）
- 相对弦最大偏离：0.2152 m
- Z 范围：0.5342 ~ 0.7554 m（Δz=0.0648）
