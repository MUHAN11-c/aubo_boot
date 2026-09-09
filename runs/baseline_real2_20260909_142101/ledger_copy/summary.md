# 采摘批次摘要 `field_pregrasp_20260909_1424`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 14:24:55
- 结束时间：2026-09-09 14:29:02
- 总时长：246.9 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1424/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | skipped | 23.25 | observe_failed |
| target_1 | 2 | skipped | 180.17 | build_rejected |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1424 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1424:target_0 | target_0 | — | 10.01 | 13.23 | — | — | — | — | — | 23.24 |
| field_pregrasp_20260909_1424:target_0 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260909_1424:target_1 | target_1 | — | 180.18 | — | — | — | — | — | — | 180.18 |

## 事件统计

- 事件总数：11
- severity INFO：11
- `photo_pose_reached`：4
- `round_locked`：3
- `target_dispatched`：2
- `target_skipped`：2

## 感知统计

- 记录帧数：577
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 2

## 重建关键指标终值

- state：COLLECTING
- target_id：target_0
- captured_views：4
- rejected_views：33
- tf_failures：0
- tf_latency_ms：0.07605552673339844
- cloud_points：12095
- max_baseline_deg：10.79855892120749
- mean_nearest_baseline_deg：10.79855892120749
- tsdf_points：1422
- tsdf_integrate_time_s：0.016561508178710938
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：35
- skip_reasons：{'same_stamp': 1, 'near_duplicate': 3, 'robot_not_static': 26, 'stale_frame': 2, 'missing_mask': 3}
- last_skip_code：robot_not_static
- last_skip_reason：机器人未静止：最大关节速度 0.1763 rad/s > 0.03

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 4 | 35 | COLLECTING | robot_not_static {"same_stamp": 1, "near_duplicate": 3, "robot_not_static": 26, "stale_frame": 2, "missing_mask": 3} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；observe_only 未等到绑定目标的 TSDF/精化几何: target_0 | 0.578,-0.625,0.550 | 0.551,-0.629,0.591 | 0.518,-0.587,0.567 | 0.529,-0.609,0.583 |
| target_1 | lock | 否 | 工具档关；observe_only 未等到绑定目标的 TSDF/精化几何: target_0 | 0.240,-0.703,0.573 | 0.551,-0.629,0.591 | 0.518,-0.587,0.567 | 0.529,-0.609,0.583 |

## 运行性能统计

- 性能采样条数：239
- CPU %：均值 37.38 / 峰值 61.3
- 内存 %：均值 39.52 / 峰值 40.4
- GPU 利用率 %：均值 12.82 / 峰值 33.0
- GPU 显存 MB：均值 1331.48 / 峰值 1548.0

## 末端 TCP 轨迹

- 采样点数：400
- 路径长：0.3307 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1642 m
- Z 范围：0.658 ~ 0.7086 m（Δz=0.0）
