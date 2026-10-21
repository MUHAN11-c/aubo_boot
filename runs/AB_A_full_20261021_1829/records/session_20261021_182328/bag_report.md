# 采摘会话分析 `session_20261021_182328`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_182328/bag`
- 开始时间：2026-10-21 18:23:29
- 结束时间：2026-10-21 18:26:13
- 总时长：164.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_f62781ee3ad2 | COMPLETED | 2026-10-21 18:23:51 | 2026-10-21 18:25:11 | skipped=3，succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f62781ee3ad2/ledger.json` |
| round_16f0c8a329b8 | COMPLETED | 2026-10-21 18:25:16 | 2026-10-21 18:26:12 | skipped=1，succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_16f0c8a329b8/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.65 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 2 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_f62781ee3ad2

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | skipped | 5.54 | skipped_unreachable |
| target_0 | 2 | skipped | 3.51 | build_failed |
| target_1 | 3 | succeeded | 22.19 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [2.188, 0.499, 10.119, 0.0, 9.147], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 4 | skipped | 2.28 | build_failed |

### round_16f0c8a329b8

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | succeeded | 21.29 | {"stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.801, 0.322, 10.03, 0.0, 8.992], "harvest_confirmed": false, "completion_level": 6} |
| target_4 | 2 | skipped | 2.76 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_f62781ee3ad2 | — | 0.36 | — | — | — | — | — | — | — | 0.36 |
| round_f62781ee3ad2:target_3 | target_3 | — | 1.29 | 1.39 | 0.24 | 2.62 | — | — | — | 5.54 |
| round_f62781ee3ad2:target_3 | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_f62781ee3ad2:target_0 | target_0 | — | — | 3.26 | — | — | — | — | — | 3.26 |
| round_f62781ee3ad2:target_0 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_f62781ee3ad2:target_1 | target_1 | — | — | 2.0 | 0.85 | 9.99 | — | 9.23 | — | 22.06 |
| round_f62781ee3ad2:target_1 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| round_f62781ee3ad2:target_2 | target_2 | — | — | 2.03 | — | — | — | — | — | 2.03 |
| round_f62781ee3ad2:target_2 | — | 10.16 | — | — | — | — | — | — | — | 10.16 |
| round_16f0c8a329b8 | — | 0.19 | — | — | — | — | — | — | — | 0.19 |
| round_16f0c8a329b8:target_5 | target_5 | — | 0.26 | 1.61 | 0.38 | 10.01 | — | 9.02 | — | 21.28 |
| round_16f0c8a329b8:target_5 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_16f0c8a329b8:target_4 | target_4 | — | — | 2.47 | 0.22 | — | — | — | — | 2.69 |
| round_16f0c8a329b8:target_4 | — | 10.15 | — | — | — | — | — | — | — | 10.15 |

## 事件统计

- 事件总数：80
- severity AUDIT：16
- severity INFO：64
- `enables_changed`：8
- `photo_pose_reached`：20
- `recovery_acknowledged`：4
- `recovery_required`：4
- `round_locked`：20
- `target_dispatched`：12
- `target_skipped`：8
- `target_succeeded`：4

## 感知统计

- 记录帧数：19
- 帧间隔中位数：1.535 s（≈0.65 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.08082389831542969
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_3 | 1 | 0 | READY | — |
| target_0 | 1 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | READY | — |
| target_2 | 2 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | READY | — |
| target_4 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.572,-0.608,0.672 | 0.403,-0.667,0.718 | — | — |
| target_3 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.204,-0.746,0.657 | — | — | — |
| target_1 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.405,-0.658,0.663 | 0.300,-0.731,0.672 | — | — |
| target_2 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.298,-0.721,0.626 | 0.300,-0.731,0.672 | — | — |
| target_5 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.407,-0.653,0.681 | — | — | — |
| target_4 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.571,-0.608,0.672 | 0.569,-0.609,0.726 | — | — |

## 运行性能统计

- 性能采样条数：152
- CPU %：均值 29.8 / 峰值 44.2
- 内存 %：均值 43.51 / 峰值 45.3
- GPU 利用率 %：均值 25.59 / 峰值 51.0
- GPU 显存 MB：均值 3097.1 / 峰值 3323.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：333
- 路径长：2.7758 m
- 起止弦：0.0003 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.4225 m
- Z 范围：0.5518 ~ 0.7573 m（Δz=0.0）
