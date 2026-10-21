# 采摘会话分析 `session_20261020_160711`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261020_160711/bag`
- 开始时间：2026-10-20 16:07:11
- 结束时间：2026-10-20 16:33:41
- 总时长：1589.9 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow_20261020_161233 | COMPLETED | 2026-10-20 16:12:40 | 2026-10-20 16:15:31 | skipped=4 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow_20261020_161233/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.57 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### flow_20261020_161233

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 31.28 | skipped_quality |
| target_0 | 2 | skipped | 30.15 | skipped_quality |
| target_3 | 3 | skipped | 30.09 | skipped_quality |
| target_2 | 4 | skipped | 30.22 | skipped_quality |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow_20261020_161233 | — | 1.13 | — | — | — | — | — | — | — | 1.13 |
| flow_20261020_161233:target_1 | target_1 | — | — | 29.84 | 0.26 | — | — | — | — | 30.1 |
| flow_20261020_161233:target_1 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| flow_20261020_161233:target_0 | target_0 | — | — | 29.83 | — | — | — | — | — | 29.83 |
| flow_20261020_161233:target_0 | — | 0.23 | — | — | — | — | — | — | — | 0.23 |
| flow_20261020_161233:target_3 | target_3 | — | — | 29.79 | — | — | — | — | — | 29.79 |
| flow_20261020_161233:target_3 | — | 0.15 | — | — | — | — | — | — | — | 0.15 |
| flow_20261020_161233:target_2 | target_2 | — | — | 29.86 | 0.22 | — | — | — | — | 30.08 |
| flow_20261020_161233:target_2 | — | 9.38 | — | — | — | — | — | — | — | 9.38 |

## 事件统计

- 事件总数：44
- severity AUDIT：4
- severity INFO：40
- `enables_changed`：4
- `photo_pose_reached`：12
- `round_locked`：12
- `target_dispatched`：8
- `target_skipped`：8

## 感知统计

- 记录帧数：19
- 帧间隔中位数：1.749 s（≈0.57 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 0 | 0 | COLLECTING | — |
| target_0 | 0 | 0 | COLLECTING | — |
| target_3 | 0 | 0 | COLLECTING | — |
| target_2 | 0 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | permit | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.290,-0.569,0.627 | — | — | — |
| target_0 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.443,-0.504,0.668 | 0.574,-0.505,0.723 | — | — |
| target_3 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.571,-0.506,0.705 | 0.573,-0.505,0.723 | — | — |
| target_2 | lock | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.238,-0.562,0.653 | 0.223,-0.567,0.686 | — | — |

## 运行性能统计

- 性能采样条数：1480
- CPU %：均值 33.46 / 峰值 78.5
- 内存 %：均值 49.32 / 峰值 51.5
- GPU 利用率 %：均值 21.26 / 峰值 42.0
- GPU 显存 MB：均值 1802.36 / 峰值 2378.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
