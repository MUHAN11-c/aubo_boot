# 采摘会话分析 `session_20261014_154710`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261014_154710/bag`
- 开始时间：2026-10-14 15:47:10
- 结束时间：2026-10-14 15:52:55
- 总时长：344.8 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| — | — | — | — | — | — |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.15 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐批次逐目标 outcome

- （无派发目标）

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| — | — | — | — | — | — | — | — | — | — | — |

## 事件统计

- 事件总数：2
- severity AUDIT：2
- `enables_changed`：2

## 感知统计

- 记录帧数：381
- 帧间隔中位数：0.867 s（≈1.15 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready

## 逐目标重建视角

- （无逐目标诊断）

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | discover | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.377,-0.625,0.576 | — | — | — |

## 运行性能统计

- 性能采样条数：300
- CPU %：均值 81.2 / 峰值 89.2
- 内存 %：均值 82.64 / 峰值 84.9
- GPU 利用率 %：均值 56.08 / 峰值 76.0
- GPU 显存 MB：均值 6171.69 / 峰值 6392.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
