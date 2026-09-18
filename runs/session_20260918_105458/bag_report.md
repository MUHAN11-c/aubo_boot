# 采摘会话分析 `session_20260918_105458`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20260918_105458/bag`
- 开始时间：2026-09-18 10:54:58
- 结束时间：2026-09-18 11:02:18
- 总时长：440.1 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| field_pregrasp_20260918_1055 | INTERRUPTED | 2026-09-18 10:56:39 | 2026-09-18 10:57:04 | skipped=1 | `/home/mu/Desktop/aubo_e5_jazzy_ws/runs/field_pregrasp_20260918_1055/ledger.json` |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.0 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐批次逐目标 outcome

### field_pregrasp_20260918_1055

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 15.05 | skipped_unreachable |


## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260918_1055 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260918_1055:target_0 | target_0 | — | 8.24 | — | 0.4 | 6.39 | — | — | — | 15.03 |

## 事件统计

- 事件总数：8
- severity INFO：8
- `photo_pose_reached`：2
- `round_locked`：2
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：1708
- 帧间隔中位数：0.5 s（≈2.0 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.07581710815429688
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 3 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | done | 否 | 工具档关；interrupted | 0.445,-0.594,0.651 | — | — | — |

## 运行性能统计

- 性能采样条数：530
- CPU %：均值 18.9 / 峰值 32.1
- 内存 %：均值 35.76 / 峰值 36.6
- GPU 利用率 %：均值 19.57 / 峰值 46.0
- GPU 显存 MB：均值 1465.31 / 峰值 1591.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：52
- 路径长：0.1784 m
- 起止弦：0.0001 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0866 m
- Z 范围：0.6683 ~ 0.7056 m（Δz=-0.0）
