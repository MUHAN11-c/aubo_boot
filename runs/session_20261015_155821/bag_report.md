# 采摘会话分析 `session_20261015_155821`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261015_155821/bag`
- 开始时间：2026-10-15 15:58:22
- 结束时间：2026-10-15 15:59:47
- 总时长：85.5 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_76020d222ce2 | INTERRUPTED | 2026-10-15 15:58:57 | 2026-10-15 15:59:44 | succeeded=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_76020d222ce2/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.25 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐批次逐目标 outcome

### round_76020d222ce2

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | succeeded | 33.97 | {"stage_names": ["reconfirm", "approach_insert", "tool", "retreat"], "stage_durations": [0.525, 17.843, 0.0, 13.766], "harvest_confirmed": false, "completion_level": 6} |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_76020d222ce2 | — | 0.14 | — | — | — | — | — | — | — | 0.14 |
| round_76020d222ce2:target_1 | target_1 | — | — | 1.65 | 0.77 | 17.73 | — | 13.6 | — | 33.73 |
| round_76020d222ce2:target_1 | — | 2.38 | — | — | — | — | — | — | — | 2.38 |

## 事件统计

- 事件总数：24
- severity AUDIT：12
- severity INFO：12
- `enables_changed`：8
- `photo_pose_reached`：4
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：4
- `target_dispatched`：2
- `target_succeeded`：2

## 感知统计

- 记录帧数：100
- 帧间隔中位数：0.799 s（≈1.25 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：1
- rejected_views：17
- tf_failures：0
- tf_latency_ms：0.11658668518066406
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：bag_mouth_geometry

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |
| target_1 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；抓取已许可，等待进入靠近 | 0.374,-0.606,0.578 | 0.385,-0.626,0.633 | 0.372,-0.599,0.565 | 0.378,-0.609,0.593 |
| target_1 | done | 是 | 工具档关；撤退已确认（采摘未确认） · 周期完成 | 0.380,-0.617,0.607 | 0.385,-0.627,0.634 | 0.374,-0.599,0.566 | 0.379,-0.609,0.594 |

## 运行性能统计

- 性能采样条数：76
- CPU %：均值 57.08 / 峰值 69.5
- 内存 %：均值 59.31 / 峰值 63.3
- GPU 利用率 %：均值 32.76 / 峰值 54.0
- GPU 显存 MB：均值 2982.95 / 峰值 3337.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：277
- 路径长：1.8732 m
- 起止弦：0.0028 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.4015 m
- Z 范围：0.4711 ~ 0.7746 m（Δz=0.0001）
