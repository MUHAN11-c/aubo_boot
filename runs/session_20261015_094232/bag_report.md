# 采摘会话分析 `session_20261015_094232`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_094232/bag`
- 开始时间：2026-10-15 09:42:33
- 结束时间：2026-10-15 09:43:15
- 总时长：42.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| optimize_frozen_fixed_20261015 | COMPLETED | 2026-10-15 09:42:42 | 2026-10-15 09:43:12 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/optimize_frozen_fixed_20261015/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.0 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### optimize_frozen_fixed_20261015

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 3.06 | skipped_unreachable |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| optimize_frozen_fixed_20261015 | — | 1.09 | — | — | — | — | — | — | — | 1.09 |
| optimize_frozen_fixed_20261015:target_0 | target_0 | — | — | 1.81 | 0.21 | — | — | — | — | 2.02 |
| optimize_frozen_fixed_20261015:target_0 | — | 7.92 | — | — | — | — | — | — | — | 7.92 |

## 事件统计

- 事件总数：20
- severity AUDIT：4
- severity INFO：16
- `enables_changed`：4
- `photo_pose_reached`：6
- `round_locked`：6
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：40
- 帧间隔中位数：1.0 s（≈1.0 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.12564659118652344
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | observe | 否 | 工具档关；尚未到达预抓取 · SKIPPED_UNREACHABLE | 0.349,-0.802,0.618 | — | — | — |

## 运行性能统计

- 性能采样条数：56
- CPU %：均值 51.41 / 峰值 67.8
- 内存 %：均值 63.41 / 峰值 65.0
- GPU 利用率 %：均值 56.93 / 峰值 75.0
- GPU 显存 MB：均值 4567.43 / 峰值 4831.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
