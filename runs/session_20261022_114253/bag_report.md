# 采摘会话分析 `session_20261022_114253`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261022_114253/bag`
- 开始时间：2026-10-22 11:42:54
- 结束时间：2026-10-22 11:47:43
- 总时长：289.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| sim_click_1792640648 | COMPLETED | 2026-10-22 11:44:22 | 2026-10-22 11:46:59 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/sim_click_1792640648/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.15 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### sim_click_1792640648

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 4 | failed | 23.66 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| sim_click_1792640648 | — | 0.18 | — | — | — | — | — | — | — | 0.18 |
| sim_click_1792640648:target_2 | target_2 | — | 0.22 | — | 0.4 | 14.21 | — | 8.78 | — | 23.61 |

## 事件统计

- 事件总数：14
- severity AUDIT：6
- severity ERROR：2
- severity INFO：6
- `enables_changed`：2
- `photo_pose_reached`：2
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：2
- 帧间隔中位数：0.867 s（≈1.15 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.14638900756835938
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_2 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.414,-0.638,0.666 | — | — | — |
| target_1 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.270,-0.643,0.669 | — | — | — |
| target_2 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期失败 | 0.219,-0.782,0.649 | 0.193,-0.758,0.683 | — | — |

## 运行性能统计

- 性能采样条数：271
- CPU %：均值 44.21 / 峰值 87.6
- 内存 %：均值 42.72 / 峰值 43.6
- GPU 利用率 %：均值 27.42 / 峰值 56.0
- GPU 显存 MB：均值 1848.41 / 峰值 2099.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：236
- 路径长：1.4119 m
- 起止弦：0.0848 m
- 绕行比（路径/弦）：16.648（直线≈1）
- 相对弦最大偏离：0.5162 m
- Z 范围：0.5324 ~ 0.7048 m（Δz=0.0053）
