# 采摘会话分析 `session_20261016_091537`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261016_091537/bag`
- 开始时间：2026-10-16 09:15:37
- 结束时间：2026-10-16 09:16:56
- 总时长：78.9 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| round_059b5707111c | COMPLETED | 2026-10-16 09:16:26 | 2026-10-16 09:16:55 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/round_059b5707111c/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 0.83 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### round_059b5707111c

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 1 | skipped | 3.28 | build_failed |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| round_059b5707111c | — | 0.29 | — | — | — | — | — | — | — | 0.29 |
| round_059b5707111c:target_1 | target_1 | — | — | 3.04 | — | — | — | — | — | 3.04 |
| round_059b5707111c:target_1 | — | 7.21 | — | — | — | — | — | — | — | 7.21 |

## 事件统计

- 事件总数：24
- severity AUDIT：8
- severity INFO：16
- `enables_changed`：8
- `photo_pose_reached`：6
- `round_locked`：6
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：9
- 帧间隔中位数：1.201 s（≈0.83 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.12350082397460938
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 4 | 0 | COLLECTING | — |
| target_1 | 1 | 0 | COLLECTING | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 否 | 抓取档关、工具档关；重建COLLECTING，尚未给出抓取几何 | 0.412,-0.659,0.666 | 0.423,-0.673,0.728 | — | — |
| target_1 | done | 否 | 工具档关；尚未到达预抓取 · SKIPPED_QUALITY | 0.416,-0.664,0.688 | — | — | — |

## 运行性能统计

- 性能采样条数：72
- CPU %：均值 41.06 / 峰值 53.1
- 内存 %：均值 62.91 / 峰值 64.6
- GPU 利用率 %：均值 29.21 / 峰值 47.0
- GPU 显存 MB：均值 2361.33 / 峰值 2588.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
