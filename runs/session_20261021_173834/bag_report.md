# 采摘会话分析 `session_20261021_173834`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_173834/bag`
- 开始时间：2026-10-21 17:38:35
- 结束时间：2026-10-21 17:40:12
- 总时长：96.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_bfd2188de3bd | COMPLETED | 2026-10-21 17:39:03 | 2026-10-21 17:39:15 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_bfd2188de3bd/ledger.json` |
| round_03d7b684f2e2 | COMPLETED | 2026-10-21 17:39:31 | 2026-10-21 17:39:41 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_03d7b684f2e2/ledger.json` |
| round_1412a7b8fa06 | COMPLETED | 2026-10-21 17:39:57 | 2026-10-21 17:40:09 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_1412a7b8fa06/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.04 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_bfd2188de3bd

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 5.73 | build_failed |

### round_03d7b684f2e2

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 3 | skipped | 4.48 | build_failed |

### round_1412a7b8fa06

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 2 | skipped | 6.35 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_bfd2188de3bd | — | 1.3 | — | — | — | — | — | — | — | 1.3 |
| round_bfd2188de3bd:target_0 | target_0 | — | — | 4.22 | 0.27 | — | — | — | — | 4.49 |
| round_03d7b684f2e2 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| round_03d7b684f2e2:target_5 | target_5 | — | — | 4.24 | — | — | — | — | — | 4.24 |
| round_1412a7b8fa06 | — | 0.12 | — | — | — | — | — | — | — | 0.12 |
| round_1412a7b8fa06:target_8 | target_8 | — | — | 6.04 | 0.21 | — | — | — | — | 6.26 |

## 事件统计

- 事件总数：32
- severity AUDIT：8
- severity INFO：24
- `enables_changed`：8
- `photo_pose_reached`：6
- `round_locked`：6
- `target_dispatched`：6
- `target_skipped`：6

## 感知统计

- 记录帧数：4
- 帧间隔中位数：24.553 s（≈0.04 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：COLLECTING
- target_id：target_8
- captured_views：1
- rejected_views：41
- tf_failures：0
- tf_latency_ms：0.08177757263183594
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：bag_axis_envelope_ill_conditioned

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 0 | COLLECTING | — |
| target_5 | 2 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.569,-0.608,0.673 | 0.569,-0.610,0.726 | — | — |
| target_3 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.209,-0.742,0.646 | — | — | — |
| target_7 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.569,-0.608,0.679 | 0.299,-0.730,0.672 | — | — |
| target_5 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.298,-0.720,0.626 | 0.299,-0.730,0.672 | — | — |
| target_9 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.408,-0.647,0.681 | 0.570,-0.610,0.729 | — | — |
| target_8 | done | 否 | 工具档关；重建COLLECTING，尚未给出抓取几何 | 0.570,-0.608,0.672 | 0.570,-0.610,0.729 | — | — |

## 运行性能统计

- 性能采样条数：90
- CPU %：均值 28.56 / 峰值 41.1
- 内存 %：均值 44.87 / 峰值 46.1
- GPU 利用率 %：均值 23.47 / 峰值 49.0
- GPU 显存 MB：均值 2666.52 / 峰值 2740.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
