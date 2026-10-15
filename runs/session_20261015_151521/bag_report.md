# 采摘会话分析 `session_20261015_151521`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_151521/bag`
- 开始时间：2026-10-15 15:15:22
- 结束时间：2026-10-15 15:16:42
- 总时长：80.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_de06a39d5bb1 | INTERRUPTED | 2026-10-15 15:15:55 | 2026-10-15 15:16:25 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_de06a39d5bb1/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.36 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_de06a39d5bb1

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | failed | 24.5 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_de06a39d5bb1 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_de06a39d5bb1:target_1 | target_1 | — | — | 0.99 | 0.38 | 17.95 | — | 4.83 | — | 24.17 |

## 事件统计

- 事件总数：22
- severity AUDIT：12
- severity ERROR：4
- severity INFO：6
- `enables_changed`：8
- `photo_pose_reached`：2
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：2
- `survey_failed`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：100
- 帧间隔中位数：0.734 s（≈1.36 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.1049041748046875
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.374,-0.607,0.579 | 0.385,-0.627,0.634 | 0.372,-0.599,0.564 | 0.377,-0.609,0.592 |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期失败 | 0.380,-0.617,0.607 | — | — | — |

## 运行性能统计

- 性能采样条数：90
- CPU %：均值 53.79 / 峰值 80.8
- 内存 %：均值 53.98 / 峰值 56.0
- GPU 利用率 %：均值 59.16 / 峰值 75.0
- GPU 显存 MB：均值 3358.33 / 峰值 3589.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：177
- 路径长：1.1132 m
- 起止弦：0.3754 m
- 绕行比（路径/弦）：2.965（直线≈1）
- 相对弦最大偏离：0.2553 m
- Z 范围：0.4715 ~ 0.753 m（Δz=-0.1187）
