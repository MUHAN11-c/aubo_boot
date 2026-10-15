# 采摘会话分析 `session_20261015_145828`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_145828/bag`
- 开始时间：2026-10-15 14:58:28
- 结束时间：2026-10-15 15:02:26
- 总时长：237.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_d5b489cd081d | INTERRUPTED | 2026-10-15 14:59:03 | 2026-10-15 15:02:09 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_d5b489cd081d/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.36 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_d5b489cd081d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | failed | 180.17 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_d5b489cd081d | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_d5b489cd081d:target_1 | target_1 | — | — | 6.4 | 0.59 | 172.31 | — | — | — | 179.31 |
| round_d5b489cd081d:target_1 | — | — | — | — | — | 15.83 | — | — | — | 15.83 |

## 事件统计

- 事件总数：18
- severity AUDIT：8
- severity ERROR：4
- severity INFO：6
- `enables_changed`：8
- `photo_pose_reached`：2
- `round_locked`：2
- `survey_failed`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：294
- 帧间隔中位数：0.734 s（≈1.36 FPS）
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
| target_1 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.374,-0.607,0.578 | 0.385,-0.624,0.630 | 0.370,-0.599,0.563 | 0.376,-0.609,0.591 |
| target_1 | done | 否 | 抓取档关、工具档关；interrupted | 0.380,-0.617,0.605 | — | — | — |

## 运行性能统计

- 性能采样条数：216
- CPU %：均值 69.19 / 峰值 97.2
- 内存 %：均值 49.81 / 峰值 52.7
- GPU 利用率 %：均值 33.17 / 峰值 53.0
- GPU 显存 MB：均值 3019.04 / 峰值 3261.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
