# 采摘会话分析 `session_20261016_085748`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_085748/bag`
- 开始时间：2026-10-16 08:57:48
- 结束时间：2026-10-16 09:09:20
- 总时长：692.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_84be9da6e850 | COMPLETED | 2026-10-16 09:02:05 | 2026-10-16 09:02:33 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_84be9da6e850/ledger.json` |
| round_79b33095820f | COMPLETED | 2026-10-16 09:02:49 | 2026-10-16 09:03:17 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_79b33095820f/ledger.json` |
| round_61bcad663d94 | COMPLETED | 2026-10-16 09:03:33 | 2026-10-16 09:04:00 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_61bcad663d94/ledger.json` |
| round_c7e0807fc249 | COMPLETED | 2026-10-16 09:04:15 | 2026-10-16 09:04:41 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_c7e0807fc249/ledger.json` |
| round_dd841af93905 | COMPLETED | 2026-10-16 09:04:57 | 2026-10-16 09:05:24 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_dd841af93905/ledger.json` |
| round_5ab1baa5f1a3 | COMPLETED | 2026-10-16 09:05:40 | 2026-10-16 09:06:07 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_5ab1baa5f1a3/ledger.json` |
| round_516e340ce5fe | COMPLETED | 2026-10-16 09:06:23 | 2026-10-16 09:06:50 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_516e340ce5fe/ledger.json` |
| round_20ceeb2fb87d | COMPLETED | 2026-10-16 09:07:06 | 2026-10-16 09:07:32 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_20ceeb2fb87d/ledger.json` |
| round_da7aac2c3031 | COMPLETED | 2026-10-16 09:07:48 | 2026-10-16 09:08:17 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_da7aac2c3031/ledger.json` |
| round_f51191d9e826 | COMPLETED | 2026-10-16 09:08:33 | 2026-10-16 09:08:59 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_f51191d9e826/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.79 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_84be9da6e850

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 2.66 | build_failed |

### round_79b33095820f

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 1 | skipped | 2.26 | build_failed |

### round_61bcad663d94

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_3 | 1 | skipped | 2.11 | build_failed |

### round_c7e0807fc249

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_4 | 1 | skipped | 2.06 | build_failed |

### round_dd841af93905

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_5 | 1 | skipped | 1.69 | build_failed |

### round_5ab1baa5f1a3

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_6 | 1 | skipped | 4.08 | build_failed |

### round_516e340ce5fe

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | skipped | 2.88 | build_failed |

### round_20ceeb2fb87d

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_8 | 1 | skipped | 2.06 | build_failed |

### round_da7aac2c3031

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_9 | 1 | skipped | 4.48 | build_failed |

### round_f51191d9e826

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_10 | 1 | skipped | 1.45 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_84be9da6e850 | — | 0.3 | — | — | — | — | — | — | — | 0.3 |
| round_84be9da6e850:target_1 | target_1 | — | — | 2.4 | — | — | — | — | — | 2.4 |
| round_84be9da6e850:target_1 | — | 7.39 | — | — | — | — | — | — | — | 7.39 |
| round_79b33095820f | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_79b33095820f:target_2 | target_2 | — | — | 2.01 | — | — | — | — | — | 2.01 |
| round_79b33095820f:target_2 | — | 7.52 | — | — | — | — | — | — | — | 7.52 |
| round_61bcad663d94 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| round_61bcad663d94:target_3 | target_3 | — | — | 1.82 | 0.22 | — | — | — | — | 2.04 |
| round_61bcad663d94:target_3 | — | 7.26 | — | — | — | — | — | — | — | 7.26 |
| round_c7e0807fc249 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_c7e0807fc249:target_4 | target_4 | — | — | 1.82 | — | — | — | — | — | 1.82 |
| round_c7e0807fc249:target_4 | — | 7.17 | — | — | — | — | — | — | — | 7.17 |
| round_dd841af93905 | — | 0.28 | — | — | — | — | — | — | — | 0.28 |
| round_dd841af93905:target_5 | target_5 | — | 0.27 | 1.43 | — | — | — | — | — | 1.7 |
| round_dd841af93905:target_5 | — | 7.2 | — | — | — | — | — | — | — | 7.2 |
| round_5ab1baa5f1a3 | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_5ab1baa5f1a3:target_6 | target_6 | — | — | 3.8 | — | — | — | — | — | 3.8 |
| round_5ab1baa5f1a3:target_6 | — | 6.67 | — | — | — | — | — | — | — | 6.67 |
| round_516e340ce5fe | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_516e340ce5fe:target_7 | target_7 | — | — | 2.61 | — | — | — | — | — | 2.61 |
| round_516e340ce5fe:target_7 | — | 7.01 | — | — | — | — | — | — | — | 7.01 |
| round_20ceeb2fb87d | — | 0.26 | — | — | — | — | — | — | — | 0.26 |
| round_20ceeb2fb87d:target_8 | target_8 | — | — | 1.81 | — | — | — | — | — | 1.81 |
| round_20ceeb2fb87d:target_8 | — | 7.09 | — | — | — | — | — | — | — | 7.09 |
| round_da7aac2c3031 | — | 0.27 | — | — | — | — | — | — | — | 0.27 |
| round_da7aac2c3031:target_9 | target_9 | — | — | 4.21 | — | — | — | — | — | 4.21 |
| round_da7aac2c3031:target_9 | — | 6.69 | — | — | — | — | — | — | — | 6.69 |
| round_f51191d9e826 | — | 0.25 | — | — | — | — | — | — | — | 0.25 |
| round_f51191d9e826:target_10 | target_10 | — | — | 1.22 | — | — | — | — | — | 1.22 |
| round_f51191d9e826:target_10 | — | 7.78 | — | — | — | — | — | — | — | 7.78 |

## 事件统计

- 事件总数：168
- severity AUDIT：8
- severity INFO：160
- `enables_changed`：8
- `photo_pose_reached`：60
- `round_locked`：60
- `target_dispatched`：20
- `target_skipped`：20

## 感知统计

- 记录帧数：181
- 帧间隔中位数：1.268 s（≈0.79 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.7367134094238281
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |
| target_2 | 1 | 0 | COLLECTING | — |
| target_3 | 1 | 0 | COLLECTING | — |
| target_4 | 2 | 0 | COLLECTING | — |
| target_5 | 1 | 0 | COLLECTING | — |
| target_6 | 1 | 0 | COLLECTING | — |
| target_7 | 1 | 0 | COLLECTING | — |
| target_8 | 1 | 0 | COLLECTING | — |
| target_9 | 1 | 0 | COLLECTING | — |
| target_10 | 2 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 否 | 抓取档关、工具档关；重建COLLECTING，尚未给出抓取几何 | 0.411,-0.657,0.666 | 0.423,-0.671,0.726 | — | — |
| target_1 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.416,-0.664,0.687 | — | — | — |
| target_2 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.663,0.687 | — | — | — |
| target_3 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.664,0.688 | — | — | — |
| target_4 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.664,0.688 | — | — | — |
| target_5 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.416,-0.664,0.688 | — | — | — |
| target_6 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.663,0.688 | — | — | — |
| target_7 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.663,0.688 | — | — | — |
| target_8 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.663,0.688 | — | — | — |
| target_9 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.664,0.688 | — | — | — |
| target_10 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.415,-0.663,0.688 | — | — | — |

## 运行性能统计

- 性能采样条数：631
- CPU %：均值 40.95 / 峰值 80.2
- 内存 %：均值 62.92 / 峰值 68.0
- GPU 利用率 %：均值 28.56 / 峰值 54.0
- GPU 显存 MB：均值 2490.77 / 峰值 2836.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
