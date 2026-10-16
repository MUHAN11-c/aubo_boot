# 采摘会话分析 `session_20261016_180102`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_180102/bag`
- 开始时间：2026-10-16 18:01:03
- 结束时间：2026-10-16 18:05:14
- 总时长：251.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_c2de5d7b51f0 | INTERRUPTED | 2026-10-16 18:04:36 | 2026-10-16 18:05:14 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_c2de5d7b51f0/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.68 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_c2de5d7b51f0

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 25.56 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_c2de5d7b51f0 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_c2de5d7b51f0:target_1 | target_1 | — | 0.27 | 25.17 | — | — | — | — | — | 25.43 |

## 事件统计

- 事件总数：14
- severity AUDIT：6
- severity INFO：8
- `enables_changed`：6
- `photo_pose_reached`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：66
- 帧间隔中位数：1.468 s（≈0.68 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.07343292236328125
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 0 | READY | — |
| target_1 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.401,-0.533,0.668 | 0.417,-0.545,0.726 | 0.411,-0.525,0.657 | 0.413,-0.534,0.685 |
| target_1 | done | 否 | 工具档关；interrupted | 0.400,-0.533,0.667 | — | — | — |

## 运行性能统计

- 性能采样条数：212
- CPU %：均值 65.44 / 峰值 75.7
- 内存 %：均值 35.93 / 峰值 38.3
- GPU 利用率 %：均值 28.02 / 峰值 60.0
- GPU 显存 MB：均值 2507.56 / 峰值 2671.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
