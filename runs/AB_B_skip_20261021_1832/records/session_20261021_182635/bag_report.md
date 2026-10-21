# 采摘会话分析 `session_20261021_182635`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_182635/bag`
- 开始时间：2026-10-21 18:26:35
- 结束时间：2026-10-21 18:31:45
- 总时长：309.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_2856deebe575 | COMPLETED | 2026-10-21 18:26:57 | 2026-10-21 18:29:15 | succeeded=4 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_2856deebe575/ledger.json` |
| round_e88b4a3ea53d | COMPLETED | 2026-10-21 18:29:20 | 2026-10-21 18:31:43 | succeeded=4 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_e88b4a3ea53d/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.03 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 8 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_2856deebe575

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 16.87 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.004, 0.518, 8.162, 0.0, 7.949], "harvest_confirmed": false, "completion_level": 6} |
| target_0 | 2 | succeeded | 23.73 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.256, 3.539, 10.77, 0.0, 9.032], "harvest_confirmed": false, "completion_level": 6} |
| target_2 | 3 | succeeded | 18.69 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.008, 1.381, 8.914, 0.0, 8.241], "harvest_confirmed": false, "completion_level": 6} |
| target_3 | 4 | succeeded | 16.66 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.011, 1.377, 7.799, 0.0, 7.347], "harvest_confirmed": false, "completion_level": 6} |

### round_e88b4a3ea53d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | succeeded | 20.89 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.257, 0.433, 10.806, 0.0, 9.244], "harvest_confirmed": false, "completion_level": 6} |
| target_5 | 2 | succeeded | 22.13 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.005, 4.688, 8.841, 0.0, 8.41], "harvest_confirmed": false, "completion_level": 6} |
| target_6 | 3 | succeeded | 18.74 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.003, 2.123, 8.424, 0.0, 8.022], "harvest_confirmed": false, "completion_level": 6} |
| target_7 | 4 | succeeded | 18.5 | {"skip_reconstruction": true, "geometry_source": "scene_observation", "stage_names": ["finalize", "reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.008, 2.344, 8.49, 0.0, 7.553], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_2856deebe575 | — | 0.2 | — | — | — | — | — | — | — | 0.2 |
| round_2856deebe575:target_1 | target_1 | — | 0.22 | — | 0.43 | 8.23 | — | 8.02 | — | 16.86 |
| round_2856deebe575:target_1 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| round_2856deebe575:target_0 | target_0 | — | — | 0.2 | 3.6 | 10.84 | — | 9.02 | — | 23.64 |
| round_2856deebe575:target_0 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_2856deebe575:target_2 | target_2 | — | — | — | 1.19 | 9.03 | — | 8.18 | — | 18.41 |
| round_2856deebe575:target_2 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_2856deebe575:target_3 | target_3 | — | — | — | 1.39 | 7.79 | — | 7.39 | — | 16.56 |
| round_2856deebe575:target_3 | — | 10.13 | — | — | — | — | — | — | — | 10.13 |
| round_e88b4a3ea53d | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| round_e88b4a3ea53d:target_4 | target_4 | — | 0.24 | 0.2 | 0.41 | 10.82 | — | 9.18 | — | 20.84 |
| round_e88b4a3ea53d:target_4 | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| round_e88b4a3ea53d:target_5 | target_5 | — | — | — | 4.79 | 8.83 | — | 8.41 | — | 22.03 |
| round_e88b4a3ea53d:target_5 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| round_e88b4a3ea53d:target_6 | target_6 | — | — | — | 2.24 | 8.36 | — | 7.99 | — | 18.6 |
| round_e88b4a3ea53d:target_6 | — | 0.3 | — | — | — | — | — | — | — | 0.3 |
| round_e88b4a3ea53d:target_7 | target_7 | — | — | — | 2.22 | 8.62 | — | 7.37 | — | 18.19 |
| round_e88b4a3ea53d:target_7 | — | 10.13 | — | — | — | — | — | — | — | 10.13 |

## 事件统计

- 事件总数：120
- severity AUDIT：40
- severity INFO：80
- `enables_changed`：8
- `photo_pose_reached`：24
- `recovery_acknowledged`：16
- `recovery_required`：16
- `round_locked`：24
- `target_dispatched`：16
- `target_succeeded`：16

## 感知统计

- 记录帧数：10
- 帧间隔中位数：28.785 s（≈0.03 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.05221366882324219
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 1 | 0 | COLLECTING | — |
| target_0 | 1 | 0 | COLLECTING | — |
| target_2 | 3 | 0 | COLLECTING | — |
| target_3 | 4 | 0 | COLLECTING | — |
| target_4 | 1 | 0 | COLLECTING | — |
| target_5 | 2 | 0 | COLLECTING | — |
| target_6 | 1 | 0 | COLLECTING | — |
| target_7 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.571,-0.608,0.672 | 0.570,-0.608,0.725 | — | — |
| target_1 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.407,-0.651,0.681 | — | — | — |
| target_2 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.297,-0.719,0.627 | 0.300,-0.731,0.672 | — | — |
| target_3 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.209,-0.743,0.646 | 0.193,-0.755,0.682 | — | — |
| target_4 | done | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.571,-0.608,0.676 | — | — | — |
| target_5 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.406,-0.656,0.663 | 0.404,-0.666,0.718 | — | — |
| target_6 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.297,-0.718,0.627 | 0.300,-0.727,0.672 | — | — |
| target_7 | lock | 否 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.209,-0.743,0.646 | 0.193,-0.755,0.682 | — | — |

## 运行性能统计

- 性能采样条数：278
- CPU %：均值 33.88 / 峰值 47.4
- 内存 %：均值 44.68 / 峰值 46.1
- GPU 利用率 %：均值 24.22 / 峰值 51.0
- GPU 显存 MB：均值 3403.58 / 峰值 3502.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1194
- 路径长：12.1948 m
- 起止弦：0.0028 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.5411 m
- Z 范围：0.4982 ~ 0.7623 m（Δz=0.0001）
