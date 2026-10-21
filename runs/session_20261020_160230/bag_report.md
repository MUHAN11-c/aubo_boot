# 采摘会话分析 `session_20261020_160230`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261020_160230/bag`
- 开始时间：2026-10-20 16:02:31
- 结束时间：2026-10-20 16:03:43
- 总时长：71.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_be214cfe7bcd | COMPLETED | 2026-10-20 16:02:57 | 2026-10-20 16:03:34 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_be214cfe7bcd/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.75 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_be214cfe7bcd

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 2 | skipped | 32.03 | skipped_quality |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_be214cfe7bcd | — | 1.92 | — | — | — | — | — | — | — | 1.92 |
| round_be214cfe7bcd:target_0 | target_0 | — | — | 29.84 | 0.23 | — | — | — | — | 30.06 |

## 事件统计

- 事件总数：16
- severity AUDIT：8
- severity INFO：8
- `enables_changed`：8
- `photo_pose_reached`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：6
- 帧间隔中位数：1.334 s（≈0.75 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | permit | 否 | 工具档关；reconstruction_not_ready | 0.282,-0.570,0.609 | 0.442,-0.512,0.722 | — | — |
| target_0 | permit | 否 | 工具档关；reconstruction_not_ready | 0.443,-0.506,0.668 | 0.442,-0.512,0.722 | — | — |

## 运行性能统计

- 性能采样条数：68
- CPU %：均值 26.47 / 峰值 38.8
- 内存 %：均值 47.78 / 峰值 49.3
- GPU 利用率 %：均值 21.84 / 峰值 49.0
- GPU 显存 MB：均值 1750.94 / 峰值 2097.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
