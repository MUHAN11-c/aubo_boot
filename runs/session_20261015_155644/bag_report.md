# 采摘会话分析 `session_20261015_155644`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_155644/bag`
- 开始时间：2026-10-15 15:56:44
- 结束时间：2026-10-15 15:58:10
- 总时长：85.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_6b1ce386ab4f | COMPLETED | 2026-10-15 15:57:17 | 2026-10-15 15:58:09 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_6b1ce386ab4f/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.25 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_6b1ce386ab4f

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 29.95 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.371, 15.788, 0.0, 11.891], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_6b1ce386ab4f | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_6b1ce386ab4f:target_1 | target_1 | — | — | 1.6 | 0.39 | 15.83 | — | 11.93 | — | 29.71 |
| round_6b1ce386ab4f:target_1 | — | 7.78 | — | — | — | — | — | — | — | 7.78 |

## 事件统计

- 事件总数：28
- severity AUDIT：12
- severity INFO：16
- `enables_changed`：8
- `photo_pose_reached`：6
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：6
- `target_dispatched`：2
- `target_succeeded`：2

## 感知统计

- 记录帧数：93
- 帧间隔中位数：0.801 s（≈1.25 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.15783309936523438
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
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.373,-0.605,0.579 | 0.385,-0.625,0.632 | 0.374,-0.599,0.566 | 0.378,-0.609,0.594 |
| target_1 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.380,-0.616,0.606 | — | — | — |

## 运行性能统计

- 性能采样条数：80
- CPU %：均值 54.8 / 峰值 65.4
- 内存 %：均值 56.67 / 峰值 58.7
- GPU 利用率 %：均值 27.68 / 峰值 54.0
- GPU 显存 MB：均值 2552.93 / 峰值 2806.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：217
- 路径长：1.2881 m
- 起止弦：0.0001 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.4009 m
- Z 范围：0.4704 ~ 0.6909 m（Δz=0.0）
