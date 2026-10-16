# 采摘会话分析 `session_20261016_182749`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_182749/bag`
- 开始时间：2026-10-16 18:27:49
- 结束时间：2026-10-16 18:32:22
- 总时长：272.2 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_af1e512e0b80 | COMPLETED | 2026-10-16 18:28:40 | 2026-10-16 18:29:37 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_af1e512e0b80/ledger.json` |
| round_7fb52a134c85 | COMPLETED | 2026-10-16 18:30:04 | 2026-10-16 18:30:56 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_7fb52a134c85/ledger.json` |
| round_f3d22e0c0eeb | COMPLETED | 2026-10-16 18:31:31 | 2026-10-16 18:32:07 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f3d22e0c0eeb/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.06 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_af1e512e0b80

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 33.93 | skipped_quality |

### round_7fb52a134c85

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | — | skipped | 30.2 | skipped_quality |

### round_f3d22e0c0eeb

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 26.96 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.471, 12.126, 0.0, 11.325], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_af1e512e0b80 | — | 11.63 | — | — | — | — | — | — | — | 11.63 |
| round_af1e512e0b80:target_0 | target_0 | — | — | 29.94 | 0.27 | — | — | — | — | 30.22 |
| round_7fb52a134c85 | — | 6.83 | — | — | — | — | — | — | — | 6.83 |
| round_7fb52a134c85:target_1 | target_1 | — | — | 29.84 | — | — | — | — | — | 29.84 |
| round_f3d22e0c0eeb | — | 0.3 | — | — | — | — | — | — | — | 0.3 |
| round_f3d22e0c0eeb:target_2 | target_2 | — | — | 1.6 | 1.57 | 12.01 | — | 11.4 | — | 26.56 |

## 事件统计

- 事件总数：42
- severity AUDIT：10
- severity INFO：32
- `enables_changed`：6
- `photo_pose_reached`：10
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：10
- `target_dispatched`：6
- `target_skipped`：4
- `target_succeeded`：2

## 感知统计

- 记录帧数：5
- 帧间隔中位数：16.046 s（≈0.06 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.12874603271484375
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 0 | COLLECTING | — |
| target_1 | 0 | 0 | COLLECTING | — |
| target_2 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.398,-0.533,0.667 | 0.417,-0.545,0.724 | — | — |
| target_1 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.405,-0.538,0.688 | — | — | — |
| target_2 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.398,-0.533,0.666 | 0.417,-0.545,0.724 | 0.410,-0.525,0.656 | 0.412,-0.534,0.684 |
| target_3 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.397,-0.533,0.665 | — | — | — |

## 运行性能统计

- 性能采样条数：223
- CPU %：均值 90.15 / 峰值 98.0
- 内存 %：均值 80.29 / 峰值 93.6
- GPU 利用率 %：均值 84.13 / 峰值 100.0
- GPU 显存 MB：均值 20243.24 / 峰值 23985.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：146
- 路径长：1.2477 m
- 起止弦：0.0001 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.3358 m
- Z 范围：0.5583 ~ 0.7775 m（Δz=0.0）
