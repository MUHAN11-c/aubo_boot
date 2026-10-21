# 采摘会话分析 `session_20261021_160208`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261021_160208/bag`
- 开始时间：2026-10-21 16:02:09
- 结束时间：2026-10-21 16:06:36
- 总时长：267.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| flow10_160309_r01 | INTERRUPTED | 2026-10-21 16:03:16 | 2026-10-21 16:06:19 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_160309_r01/ledger.json` |
| flow10_160619_r02 | INTERRUPTED | 2026-10-21 16:06:19 | 2026-10-21 16:06:19 | 无 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_160619_r02/ledger.json` |
| flow10_160619_r03 | INTERRUPTED | 2026-10-21 16:06:19 | 2026-10-21 16:06:19 | 无 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/flow10_160619_r03/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.6 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### flow10_160309_r01

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | failed | 180.03 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| flow10_160309_r01 | — | 0.24 | — | — | — | — | — | — | — | 0.24 |
| flow10_160309_r01:target_0 | target_0 | — | — | — | 0.61 | 178.98 | — | — | — | 179.56 |
| flow10_160309_r01:target_0 | — | — | — | — | — | 16.7 | — | — | — | 16.7 |

## 事件统计

- 事件总数：18
- severity AUDIT：4
- severity ERROR：8
- severity INFO：6
- `enables_changed`：4
- `photo_pose_reached`：2
- `round_locked`：2
- `survey_failed`：6
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：2
- 帧间隔中位数：1.669 s（≈0.6 FPS）
- 目标数范围：4 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.11682510375976562
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 抓取档关、工具档关；interrupted | 0.570,-0.608,0.675 | — | — | — |

## 运行性能统计

- 性能采样条数：250
- CPU %：均值 52.02 / 峰值 79.8
- 内存 %：均值 50.38 / 峰值 51.9
- GPU 利用率 %：均值 24.79 / 峰值 55.0
- GPU 显存 MB：均值 2738.78 / 峰值 2922.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
