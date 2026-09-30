# 采摘会话分析 `session_20260930_174150`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/plans/v1_三末端混合测试/results/runs/session_20260930_174150/bag`
- 开始时间：2026-09-30 17:41:51
- 结束时间：2026-09-30 17:42:39
- 总时长：47.7 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| — | — | — | — | — | — |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.36 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐批次逐目标 outcome

- （无派发目标）

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| — | — | — | — | — | — | — | — | — | — | — |

## 事件统计

- 事件总数：0

## 感知统计

- 记录帧数：53
- 帧间隔中位数：0.734 s（≈1.36 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：3
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.0
- max_baseline_deg：20.0
- mean_nearest_baseline_deg：12.0
- grasp_allowed：True
- grasp_reason：sim_budget_accept

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 3 | 0 | READY | — |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 是 | 抓取档关、工具档关；到预抓取失败: MTC short-path guard rejected: 工具筒体接触果实胶囊 s=0.0141582m r=0.144555m 间隙=-0.00544527m (果半径 0.04m) | 0.170,-0.791,0.620 | 0.305,-0.610,0.566 | 0.000,0.000,0.000 | 0.000,0.000,0.000 |
| target_1 | discover | 是 | 抓取档关、工具档关；到预抓取失败: MTC short-path guard rejected: 工具筒体接触果实胶囊 s=0.0141582m r=0.144555m 间隙=-0.00544527m (果半径 0.04m) | 0.304,-0.614,0.536 | 0.305,-0.610,0.566 | 0.000,0.000,0.000 | 0.000,0.000,0.000 |

## 运行性能统计

- 性能采样条数：38
- CPU %：均值 75.45 / 峰值 84.7
- 内存 %：均值 54.66 / 峰值 57.1
- GPU 利用率 %：均值 13.95 / 峰值 30.0
- GPU 显存 MB：均值 1681.53 / 峰值 1839.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：7
- 路径长：0.0585 m
- 起止弦：0.0584 m
- 绕行比（路径/弦）：1.002（直线≈1）
- 相对弦最大偏离：0.0015 m
- Z 范围：0.7056 ~ 0.7075 m（Δz=-0.0019）
