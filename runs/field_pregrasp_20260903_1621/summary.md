# 采摘批次摘要 `field_pregrasp_20260903_1621`

## 批次概览

- 终局状态：COMPLETED
- 复扫轮数：1
- 开始时间：2026-09-03 16:22:54
- 结束时间：2026-09-03 16:23:27
- 总时长：32.9 s
- 终局计数：无
- 会话根：`runs/field_pregrasp_20260903_1621/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.0 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1621 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |

## 事件统计

- 事件总数：4
- severity INFO：4
- `photo_pose_reached`：2
- `round_locked`：2

## 感知统计

- 记录帧数：29
- 帧间隔中位数：1.0 s（≈1.0 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.10704994201660156
- cloud_points：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：0
- skip_reasons：{}
- last_skip_code：
- last_skip_reason：

## 逐目标重建视角

- （无逐目标诊断）

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_7 | lock | 否 | 工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.402,-0.606,0.567 | — | — | — |

## 运行性能统计

- 性能采样条数：31
- CPU %：均值 25.17 / 峰值 59.9
- 内存 %：均值 44.37 / 峰值 44.7
- GPU 利用率 %：均值 10.94 / 峰值 26.0
- GPU 显存 MB：均值 2129.06 / 峰值 2276.0

## 末端 TCP 轨迹

- 采样点数：34
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.7082 ~ 0.7082 m（Δz=0.0）
