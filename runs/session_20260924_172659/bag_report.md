# 采摘会话分析 `session_20260924_172659`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20260924_172659/bag`
- 开始时间：2026-09-24 17:26:59
- 结束时间：2026-09-24 17:27:56
- 总时长：56.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| e2e_full_unrefined_20260924T172716 | RUNNING | 2026-09-24 17:27:26 | 2026-09-24 17:27:48 | failed=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/e2e_full_unrefined_20260924T172716/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.67 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### e2e_full_unrefined_20260924T172716

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | failed | 9.85 | full_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| e2e_full_unrefined_20260924T172716 | — | 0.22 | — | — | — | — | — | — | — | 0.22 |
| e2e_full_unrefined_20260924T172716:target_1 | target_1 | — | — | — | 0.54 | 9.06 | — | — | — | 9.6 |

## 事件统计

- 事件总数：14
- severity AUDIT：4
- severity ERROR：2
- severity INFO：8
- `enables_changed`：2
- `photo_pose_reached`：2
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_failed`：2

## 感知统计

- 记录帧数：70
- 帧间隔中位数：0.6 s（≈1.67 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：COLLECTING
- target_id：target_1
- captured_views：4
- rejected_views：207
- tf_failures：0
- tf_latency_ms：0.09202957153320312
- max_baseline_deg：0.0
- mean_nearest_baseline_deg：0.0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 4 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.412,-0.768,0.547 | — | — | — |
| target_1 | tool | 否 | 工具档关；已记录现场人工撤离确认；本服务不发送任何运动命令 | 0.597,-0.734,0.549 | 0.617,-0.726,0.590 | — | — |

## 运行性能统计

- 性能采样条数：66
- CPU %：均值 66.74 / 峰值 86.7
- 内存 %：均值 61.01 / 峰值 70.2
- GPU 利用率 %：均值 24.33 / 峰值 43.0
- GPU 显存 MB：均值 2173.52 / 峰值 2502.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：161
- 路径长：1.4251 m
- 起止弦：0.061 m
- 绕行比（路径/弦）：23.345（直线≈1）
- 相对弦最大偏离：0.5871 m
- Z 范围：0.4348 ~ 0.7075 m（Δz=-0.0018）
