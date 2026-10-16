# 采摘会话分析 `session_20261016_180542`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_180542/bag`
- 开始时间：2026-10-16 18:05:42
- 结束时间：2026-10-16 18:13:18
- 总时长：456.0 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_888777f66e63 | COMPLETED | 2026-10-16 18:06:20 | 2026-10-16 18:07:28 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_888777f66e63/ledger.json` |
| round_87ea6c57c077 | COMPLETED | 2026-10-16 18:07:50 | 2026-10-16 18:08:55 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_87ea6c57c077/ledger.json` |
| round_fe395f8d34d9 | COMPLETED | 2026-10-16 18:09:19 | 2026-10-16 18:10:23 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_fe395f8d34d9/ledger.json` |
| round_79d37aabe4c1 | COMPLETED | 2026-10-16 18:10:46 | 2026-10-16 18:11:50 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_79d37aabe4c1/ledger.json` |
| round_83f39eae8a08 | COMPLETED | 2026-10-16 18:12:13 | 2026-10-16 18:13:17 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_83f39eae8a08/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.03 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_888777f66e63

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | succeeded | 31.57 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [1.36, 10.359, 0.0, 10.251], "harvest_confirmed": false, "completion_level": 6} |

### round_87ea6c57c077

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 30.17 | skipped_quality |

### round_fe395f8d34d9

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | skipped | 30.14 | skipped_quality |

### round_79d37aabe4c1

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | skipped | 30.14 | skipped_quality |

### round_83f39eae8a08

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | skipped | 30.18 | skipped_quality |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_888777f66e63 | — | 1.51 | — | — | — | — | — | — | — | 1.51 |
| round_888777f66e63:target_0 | target_0 | — | — | 7.8 | 1.6 | 10.4 | — | 10.19 | — | 29.98 |
| round_888777f66e63:target_0 | — | 10.12 | — | — | — | — | — | — | — | 10.12 |
| round_87ea6c57c077 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| round_87ea6c57c077:target_1 | target_1 | — | — | 29.85 | — | — | — | — | — | 29.85 |
| round_87ea6c57c077:target_1 | — | 10.14 | — | — | — | — | — | — | — | 10.14 |
| round_fe395f8d34d9 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_fe395f8d34d9:target_2 | target_2 | — | — | 29.79 | — | — | — | — | — | 29.79 |
| round_fe395f8d34d9:target_2 | — | 10.07 | — | — | — | — | — | — | — | 10.07 |
| round_79d37aabe4c1 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_79d37aabe4c1:target_3 | target_3 | — | — | 29.74 | — | — | — | — | — | 29.74 |
| round_79d37aabe4c1:target_3 | — | 10.09 | — | — | — | — | — | — | — | 10.09 |
| round_83f39eae8a08 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| round_83f39eae8a08:target_4 | target_4 | — | — | 29.81 | — | — | — | — | — | 29.81 |
| round_83f39eae8a08:target_4 | — | 10.12 | — | — | — | — | — | — | — | 10.12 |

## 事件统计

- 事件总数：92
- severity AUDIT：12
- severity INFO：80
- `enables_changed`：8
- `photo_pose_reached`：30
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：30
- `target_dispatched`：10
- `target_skipped`：8
- `target_succeeded`：2

## 感知统计

- 记录帧数：15
- 帧间隔中位数：28.852 s（≈0.03 FPS）
- 目标数范围：1 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09179115295410156
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 0 | 0 | COLLECTING | — |
| target_2 | 0 | 0 | COLLECTING | — |
| target_3 | 0 | 0 | COLLECTING | — |
| target_4 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.406,-0.538,0.689 | — | — | — |
| target_1 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.406,-0.538,0.689 | — | — | — |
| target_2 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.406,-0.538,0.689 | — | — | — |
| target_3 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.405,-0.538,0.688 | — | — | — |
| target_4 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.404,-0.538,0.688 | — | — | — |

## 运行性能统计

- 性能采样条数：377
- CPU %：均值 68.59 / 峰值 80.9
- 内存 %：均值 40.51 / 峰值 43.0
- GPU 利用率 %：均值 28.94 / 峰值 61.0
- GPU 显存 MB：均值 2654.07 / 峰值 2953.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：166
- 路径长：1.1979 m
- 起止弦：0.0024 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.3345 m
- Z 范围：0.559 ~ 0.7762 m（Δz=0.0001）
