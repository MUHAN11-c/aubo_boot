# 采摘会话分析 `session_20261012_161132`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261012_161132/bag`
- 开始时间：2026-10-12 16:11:32
- 结束时间：2026-10-12 16:15:34
- 总时长：241.7 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| e2e_20261012_1610 | INTERRUPTED | 2026-10-12 16:11:46 | 2026-10-12 16:12:27 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e_20261012_1610/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.0 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### e2e_20261012_1610

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 29.64 | {"stage_names": ["approach_insert", "reconfirm", "tool", "retreat"], "stage_durations": [15.363, 0.2, 0.0, 12.548], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| e2e_20261012_1610 | — | 1.2 | — | — | — | — | — | — | — | 1.2 |
| e2e_20261012_1610:target_0 | target_0 | — | — | — | 0.4 | 15.37 | — | 12.56 | — | 28.33 |

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

- 记录帧数：26
- 帧间隔中位数：1.0 s（≈1.0 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_0
- captured_views：2
- rejected_views：1
- tf_failures：0
- tf_latency_ms：0.06628036499023438
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：dynamic_budget_accept

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 是 | 工具档关；interrupted | 0.339,-0.791,0.592 | 0.346,-0.822,0.643 | 0.355,-0.783,0.579 | 0.356,-0.797,0.606 |

## 运行性能统计

- 性能采样条数：366
- CPU %：均值 17.47 / 峰值 40.3
- 内存 %：均值 47.69 / 峰值 49.6
- GPU 利用率 %：均值 30.02 / 峰值 45.0
- GPU 显存 MB：均值 1871.5 / 峰值 1958.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：236
- 路径长：1.5058 m
- 起止弦：0.0567 m
- 绕行比（路径/弦）：26.534（直线≈1）
- 相对弦最大偏离：0.5658 m
- Z 范围：0.4856 ~ 0.7121 m（Δz=0.0019）
