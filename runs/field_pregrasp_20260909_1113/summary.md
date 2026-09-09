# 采摘批次摘要 `field_pregrasp_20260909_1113`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 11:13:12
- 结束时间：2026-09-09 11:15:03
- 总时长：111.1 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1113/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 12.03 | build_start_timeout |
| target_1 | 2 | skipped | 66.43 | build_rejected |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1113 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1113:target_0 | target_0 | — | 12.03 | — | — | — | — | — | — | 12.03 |
| field_pregrasp_20260909_1113:target_1 | target_1 | — | 66.43 | — | — | — | — | — | — | 66.43 |

## 事件统计

- 事件总数：12
- severity INFO：12
- `photo_pose_reached`：4
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：255
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- cloud_points：0
- grasp_allowed：False
- grasp_reason：reconstruction_not_ready
- skipped_views：0
- skip_reasons：{}
- last_skip_code：
- last_skip_reason：

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 0 | 11 | COLLECTING | missing_mask {"missing_mask": 11} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.550,-0.624,0.565 | — | — | — |
| target_1 | lock | 否 | 抓取档关、工具档关；重建COLLECTING，尚未给出抓取几何 | 0.242,-0.703,0.573 | 0.551,-0.630,0.592 | — | — |

## 运行性能统计

- 性能采样条数：107
- CPU %：均值 38.77 / 峰值 99.6
- 内存 %：均值 39.42 / 峰值 39.7
- GPU 利用率 %：均值 11.95 / 峰值 33.0
- GPU 显存 MB：均值 1344.44 / 峰值 1426.0

## 末端 TCP 轨迹

- 采样点数：111
- 路径长：0.0 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.0 m
- Z 范围：0.7082 ~ 0.7082 m（Δz=0.0）
