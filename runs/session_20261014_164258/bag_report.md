# 采摘会话分析 `session_20261014_164258`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261014_164258/bag`
- 开始时间：2026-10-14 16:42:59
- 结束时间：2026-10-14 16:54:03
- 总时长：664.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| rviz_repeat_20261014_1632_round_1 | COMPLETED | 2026-10-14 16:45:54 | 2026-10-14 16:46:45 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/rviz_repeat_20261014_1632_round_1/ledger.json` |
| rviz_repeat_20261014_1632_round_2 | COMPLETED | 2026-10-14 16:46:49 | 2026-10-14 16:47:06 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/rviz_repeat_20261014_1632_round_2/ledger.json` |
| rviz_repeat_20261014_1632_round_3 | COMPLETED | 2026-10-14 16:47:10 | 2026-10-14 16:48:01 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/rviz_repeat_20261014_1632_round_3/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.67 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 2 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### rviz_repeat_20261014_1632_round_1

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 32.56 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.341, 16.232, 0.0, 12.974], "harvest_confirmed": false, "completion_level": 6} |

### rviz_repeat_20261014_1632_round_2

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 0.86 | skipped_quality |

### rviz_repeat_20261014_1632_round_3

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | succeeded | 33.21 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.533, 16.967, 0.0, 14.616], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| rviz_repeat_20261014_1632_round_1 | — | 1.47 | — | — | — | — | — | — | — | 1.47 |
| rviz_repeat_20261014_1632_round_1:target_0 | target_0 | — | — | 1.41 | 0.43 | 16.17 | — | 12.97 | — | 30.95 |
| rviz_repeat_20261014_1632_round_1:target_0 | — | 6.17 | — | — | — | — | — | — | — | 6.17 |
| rviz_repeat_20261014_1632_round_2 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| rviz_repeat_20261014_1632_round_2:target_1 | target_1 | — | — | 0.62 | — | — | — | — | — | 0.62 |
| rviz_repeat_20261014_1632_round_2:target_1 | — | 6.13 | — | — | — | — | — | — | — | 6.13 |
| rviz_repeat_20261014_1632_round_3 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| rviz_repeat_20261014_1632_round_3:target_2 | target_2 | — | — | 0.82 | 0.59 | 16.75 | — | 14.66 | — | 32.85 |
| rviz_repeat_20261014_1632_round_3:target_2 | — | 6.24 | — | — | — | — | — | — | — | 6.24 |

## 事件统计

- 事件总数：62
- severity AUDIT：14
- severity INFO：48
- `enables_changed`：6
- `photo_pose_reached`：18
- `recovery_acknowledged`：4
- `recovery_required`：4
- `round_locked`：18
- `target_dispatched`：6
- `target_skipped`：2
- `target_succeeded`：4

## 感知统计

- 记录帧数：1001
- 帧间隔中位数：0.6 s（≈1.67 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08749961853027344
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |
| target_2 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.380,-0.617,0.606 | — | — | — |
| target_1 | lock | 否 | 工具档关；GraspDecision.allowed=false，禁止降级接触: pregrasp_geometry_ready | 0.380,-0.616,0.606 | — | — | — |
| target_2 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.380,-0.617,0.606 | — | — | — |

## 运行性能统计

- 性能采样条数：726
- CPU %：均值 55.7 / 峰值 79.7
- 内存 %：均值 50.67 / 峰值 54.1
- GPU 利用率 %：均值 29.34 / 峰值 54.0
- GPU 显存 MB：均值 2188.06 / 峰值 2814.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：568
- 路径长：3.0547 m
- 起止弦：0.0001 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.402 m
- Z 范围：0.4696 ~ 0.7848 m（Δz=0.0）
