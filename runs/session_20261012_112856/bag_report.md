# 采摘会话分析 `session_20261012_112856`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261012_112856/bag`
- 开始时间：2026-10-12 11:28:56
- 结束时间：2026-10-12 11:29:52
- 总时长：55.2 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| frozen_v1_1791775738 | INTERRUPTED | 2026-10-12 11:29:10 | 2026-10-12 11:29:51 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/frozen_v1_1791775738/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.0 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### frozen_v1_1791775738

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 29.66 | {"stage_names": ["approach_insert", "reconfirm", "tool", "retreat"], "stage_durations": [15.338, 0.201, 0.0, 12.646], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| frozen_v1_1791775738 | — | 1.22 | — | — | — | — | — | — | — | 1.22 |
| frozen_v1_1791775738:target_0 | target_0 | — | — | 0.18 | 0.42 | 15.18 | — | 12.59 | — | 28.35 |

## 事件统计

- 事件总数：14
- severity AUDIT：6
- severity INFO：8
- `enables_changed`：2
- `photo_pose_reached`：2
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_succeeded`：2

## 感知统计

- 记录帧数：25
- 帧间隔中位数：1.0 s（≈1.0 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_0
- captured_views：1
- rejected_views：1
- tf_failures：0
- tf_latency_ms：0.08893013000488281
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：dynamic_budget_accept

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 是 | 工具档关；interrupted | 0.339,-0.791,0.592 | 0.346,-0.822,0.643 | 0.355,-0.783,0.579 | 0.356,-0.797,0.606 |

## 运行性能统计

- 性能采样条数：86
- CPU %：均值 20.73 / 峰值 45.5
- 内存 %：均值 40.37 / 峰值 41.6
- GPU 利用率 %：均值 20.21 / 峰值 40.0
- GPU 显存 MB：均值 2306.05 / 峰值 2456.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：227
- 路径长：1.504 m
- 起止弦：0.0561 m
- 绕行比（路径/弦）：26.807（直线≈1）
- 相对弦最大偏离：0.5655 m
- Z 范围：0.4855 ~ 0.7121 m（Δz=0.0019）
