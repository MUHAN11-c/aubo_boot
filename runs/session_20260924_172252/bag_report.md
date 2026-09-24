# 采摘会话分析 `session_20260924_172252`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20260924_172252/bag`
- 开始时间：2026-09-24 17:22:52
- 结束时间：2026-09-24 17:25:48
- 总时长：175.6 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| e2e_full_unrefined_20260924T172313 | RECOVERY_REQUIRED | 2026-09-24 17:23:24 | 2026-09-24 17:23:44 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e_full_unrefined_20260924T172313/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.87 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### e2e_full_unrefined_20260924T172313

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | failed | 9.83 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| e2e_full_unrefined_20260924T172313 | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| e2e_full_unrefined_20260924T172313:target_1 | target_1 | — | — | — | 0.2 | 9.36 | — | — | — | 9.56 |

## 事件统计

- 事件总数：14
- severity AUDIT：2
- severity ERROR：2
- severity INFO：10
- `enables_changed`：4
- `photo_pose_reached`：2
- `recovery_required`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：291
- 帧间隔中位数：0.534 s（≈1.87 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：2
- rejected_views：1726
- tf_failures：0
- tf_latency_ms：0.10251998901367188
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 2 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.409,-0.775,0.547 | — | — | — |
| target_1 | approach | 否 | 工具档关；接触恢复未确认，不派下一颗 | — | 0.616,-0.726,0.588 | — | — |

## 运行性能统计

- 性能采样条数：212
- CPU %：均值 60.1 / 峰值 91.1
- 内存 %：均值 61.29 / 峰值 70.2
- GPU 利用率 %：均值 23.74 / 峰值 50.0
- GPU 显存 MB：均值 2210.69 / 峰值 2598.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：101
- 路径长：0.806 m
- 起止弦：0.582 m
- 绕行比（路径/弦）：1.385（直线≈1）
- 相对弦最大偏离：0.1286 m
- Z 范围：0.4336 ~ 0.7075 m（Δz=-0.1871）
