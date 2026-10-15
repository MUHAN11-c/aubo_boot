# 采摘会话分析 `session_20261014_164050`

## 会话概览

- bag：`/home/mu/Desktop/aubo_e5_jazzy_ws/runs/session_20261014_164050/bag`
- 开始时间：2026-10-14 16:40:51
- 结束时间：2026-10-14 16:40:57
- 总时长：6.2 s

## 批次（request）一览

| request_id | 终局状态 | 开始 | 结束 | 终局计数 | 账本 |
|---|---|---|---|---|---|
| — | — | — | — | — | — |

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | — | ≥ 2.0 | — |
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

- 记录帧数：0
- 帧间隔中位数：None s（≈None FPS）
- 目标数范围：None ~ None

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

- （无作业票记录）

## 运行性能统计

- 性能采样条数：10
- CPU %：均值 32.72 / 峰值 60.0
- 内存 %：均值 43.3 / 峰值 44.4
- GPU 利用率 %：均值 30.8 / 峰值 45.0
- GPU 显存 MB：均值 1242.4 / 峰值 1272.0

## 末端 TCP 轨迹（/tf 重算）

- 采样点数：1
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.6888 ~ 0.6888 m（Δz=0.0）
