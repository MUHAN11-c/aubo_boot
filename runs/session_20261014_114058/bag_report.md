# 采摘会话分析 `session_20261014_114058`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261014_114058/bag`
- 开始时间：2026-10-14 11:40:59
- 结束时间：2026-10-14 11:45:16
- 总时长：257.0 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| live_cam_shear_full_20261014_1143 | INTERRUPTED | 2026-10-14 11:41:56 | 2026-10-14 11:44:47 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/live_cam_shear_full_20261014_1143/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.25 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### live_cam_shear_full_20261014_1143

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 18.92 | skipped_unreachable |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| live_cam_shear_full_20261014_1143 | — | 12.36 | — | — | — | — | — | — | — | 12.36 |
| live_cam_shear_full_20261014_1143:target_0 | target_0 | — | — | 3.26 | 0.71 | 11.84 | — | — | — | 15.83 |
| live_cam_shear_full_20261014_1143:target_0 | — | 1.89 | — | — | — | — | — | — | — | 1.89 |

## 事件统计

- 事件总数：24
- severity AUDIT：8
- severity INFO：16
- `enables_changed`：4
- `photo_pose_reached`：6
- `recovery_acknowledged`：2
- `recovery_required`：2
- `round_locked`：6
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：292
- 帧间隔中位数：0.801 s（≈1.25 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.13375282287597656
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 1 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；预抓取已验证 · SKIPPED_UNREACHABLE | 0.360,-0.852,0.585 | — | — | — |

## 运行性能统计

- 性能采样条数：240
- CPU %：均值 77.44 / 峰值 81.7
- 内存 %：均值 83.1 / 峰值 84.8
- GPU 利用率 %：均值 43.53 / 峰值 67.0
- GPU 显存 MB：均值 5635.64 / 峰值 6123.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：167
- 路径长：1.6238 m
- 起止弦：0.0023 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.6322 m
- Z 范围：0.4614 ~ 0.7009 m（Δz=0.0006）
