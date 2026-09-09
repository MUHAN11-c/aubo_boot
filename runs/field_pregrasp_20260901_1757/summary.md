# 采摘批次摘要 `field_pregrasp_20260901_1757`

## 批次概览

- 终局状态：RECOVERY_REQUIRED
- 复扫轮数：1
- 开始时间：2026-09-01 17:58:04
- 结束时间：2026-09-01 17:58:30
- 总时长：26.4 s
- 终局计数：unfinished=1
- 会话根：`runs/field_pregrasp_20260901_1757/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | unfinished | — |  |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_1757 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| field_pregrasp_20260901_1757:target_1 | target_1 | — | 11.91 | — | 0.2 | 10.0 | — | — | 0.2 | 22.31 |

## 事件统计

- 事件总数：3
- severity INFO：3
- `round_locked`：1
- `target_dispatched`：1
- `targets_filtered`：1

## 感知统计

- 记录帧数：64
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：2
- rejected_views：51
- tf_failures：0
- tf_latency_ms：0.08296966552734375
- cloud_points：11990
- max_baseline_deg：11.939622780173933
- mean_nearest_baseline_deg：11.939622780173933
- tsdf_points：1582
- tsdf_integrate_time_s：0.0048291683197021484
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：54
- skip_reasons：{'missing_mask': 1, 'stale_frame': 12, 'same_stamp': 1, 'near_duplicate': 3, 'robot_not_static': 37}
- last_skip_code：stale_frame
- last_skip_reason：缓存帧龄期 2.36 s > max_frame_age_s=2.0（陈帧拒采）

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_1 | 2 | 54 | READY | stale_frame {"missing_mask": 1, "stale_frame": 12, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 37} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.328,-0.598,0.569 | 0.307,-0.626,0.601 | 0.304,-0.614,0.536 | 0.304,-0.614,0.536 |
| target_2 | lock | 否 | 抓取档关、工具档关；接触未许可（reconstruction_not_ready）；有几何则可去预抓取 | 0.531,-0.598,0.697 | — | — | — |

## 运行性能统计

- 性能采样条数：25
- CPU %：均值 50.21 / 峰值 99.0
- 内存 %：均值 53.06 / 峰值 53.6
- GPU 利用率 %：均值 21.8 / 峰值 42.0
- GPU 显存 MB：均值 2145.92 / 峰值 2304.0

## 末端 TCP 轨迹

- 采样点数：227
- 路径长：0.6572 m
- 起止弦：0.4191 m
- 绕行比（路径/弦）：1.568（直线≈1）
- 相对弦最大偏离：0.0995 m
- Z 范围：0.5365 ~ 0.7115 m（Δz=-0.1717）
