# 采摘会话分析 `session_20260917_171626`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20260917_171626/bag`
- 开始时间：2026-09-17 17:16:26
- 结束时间：2026-09-17 17:23:38
- 总时长：432.3 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| e2e25r_pregrasp3_171710 | INTERRUPTED | 2026-09-17 17:17:24 | 2026-09-17 17:17:42 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e25r_pregrasp3_171710/ledger.json` |
| e2e25r_pregrasp4_171833 | INTERRUPTED | 2026-09-17 17:18:41 | 2026-09-17 17:19:06 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e25r_pregrasp4_171833/ledger.json` |
| e2e25r_pregrasp5_171948 | INTERRUPTED | 2026-09-17 17:20:01 | 2026-09-17 17:23:06 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e25r_pregrasp5_171948/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.0 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### e2e25r_pregrasp3_171710

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 13.04 | observe_build_view_race |

### e2e25r_pregrasp4_171833

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 2 | skipped | 18.97 | observe_build_view_race |

### e2e25r_pregrasp5_171948

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 2 | skipped | 180.02 | build_timeout:reconstruction |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| e2e25r_pregrasp3_171710 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| e2e25r_pregrasp3_171710:target_0 | target_0 | — | 13.04 | — | — | — | — | — | — | 13.04 |
| e2e25r_pregrasp4_171833 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| e2e25r_pregrasp4_171833:target_2 | target_2 | — | 18.97 | — | — | — | — | — | — | 18.97 |
| e2e25r_pregrasp5_171948 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| e2e25r_pregrasp5_171948:target_2 | target_2 | — | 180.02 | — | — | — | — | — | — | 180.02 |

## 事件统计

- 事件总数：15
- severity INFO：13
- severity WARNING：2
- `enables_changed`：1
- `observe_build_view_race`：2
- `photo_pose_reached`：3
- `round_locked`：3
- `target_dispatched`：3
- `target_skipped`：3

## 感知统计

- 记录帧数：686
- 帧间隔中位数：0.5 s（≈2.0 FPS）
- 目标数范围：0 ~ 4

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09417533874511719
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 0 | COLLECTING | — |
| target_2 | 3 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_2 | done | 否 | 抓取档关、工具档关；interrupted | 0.477,-0.680,0.662 | — | — | — |
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.676,-0.549,0.573 | — | — | — |

## 运行性能统计

- 性能采样条数：415
- CPU %：均值 16.24 / 峰值 33.3
- 内存 %：均值 43.75 / 峰值 49.3
- GPU 利用率 %：均值 15.32 / 峰值 100.0
- GPU 显存 MB：均值 1685.43 / 峰值 6861.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：103
- 路径长：0.3886 m
- 起止弦：0.0024 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0945 m
- Z 范围：0.6742 ~ 0.7059 m（Δz=0.0007）
