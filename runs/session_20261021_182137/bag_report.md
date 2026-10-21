# 采摘会话分析 `session_20261021_182137`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_182137/bag`
- 开始时间：2026-10-21 18:21:38
- 结束时间：2026-10-21 18:22:40
- 总时长：62.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_93b15c28efb6 | RUNNING | 2026-10-21 18:21:59 | 2026-10-21 18:22:19 | skipped=1，unfinished=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_93b15c28efb6/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.56 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_93b15c28efb6

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 5.79 | build_failed |
| target_1 | 2 | unfinished | — |  |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_93b15c28efb6 | — | 0.17 | — | — | — | — | — | — | — | 0.17 |
| round_93b15c28efb6:target_0 | target_0 | — | 1.34 | 4.2 | 0.23 | — | — | — | — | 5.78 |
| round_93b15c28efb6:target_0 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_93b15c28efb6:target_1 | target_1 | — | — | 1.2 | 0.8 | 9.78 | — | 8.8 | — | 20.59 |

## 事件统计

- 事件总数：18
- severity AUDIT：4
- severity INFO：14
- `enables_changed`：4
- `photo_pose_reached`：4
- `round_locked`：4
- `target_dispatched`：4
- `target_skipped`：2

## 感知统计

- 记录帧数：7
- 帧间隔中位数：1.802 s（≈0.56 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：1
- rejected_views：16
- tf_failures：0
- tf_latency_ms：0.07224082946777344
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：True
- grasp_reason：bag_mouth_geometry

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | permit | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.571,-0.608,0.672 | 0.403,-0.664,0.718 | — | — |
| target_1 | approach | 是 | 工具档关；接触恢复未确认，不派下一颗 | 0.406,-0.658,0.663 | 0.403,-0.664,0.718 | 0.403,-0.631,0.652 | 0.402,-0.639,0.681 |

## 运行性能统计

- 性能采样条数：58
- CPU %：均值 29.58 / 峰值 44.6
- 内存 %：均值 41.79 / 峰值 43.8
- GPU 利用率 %：均值 23.34 / 峰值 50.0
- GPU 显存 MB：均值 2819.09 / 峰值 2919.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：151
- 路径长：1.3161 m
- 起止弦：0.0546 m
- 绕行比（路径/弦）：24.089（直线≈1）
- 相对弦最大偏离：0.4218 m
- Z 范围：0.5543 ~ 0.7557 m（Δz=0.0018）
