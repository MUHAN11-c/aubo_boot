# 采摘批次摘要 `field_pregrasp_20260901_1704`

现场目视（2026-09-01 17:08）：`target_1` **方向与位置良好，轨迹可行**（筒口对袋轴、定位可用、拍照位→预抓取 PTP 可走）。`target_2` 未到位（精化预抓取超程，MTC 0 解）。无 SetIO。

## 批次概览

- 终局状态：RECOVERY_REQUIRED
- 复扫轮数：1
- 开始时间：2026-09-01 17:03:47
- 结束时间：2026-09-01 17:04:28
- 总时长：41.1 s
- 终局计数：skipped=1, succeeded=1
- 会话根：`runs/field_pregrasp_20260901_1704/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_2 | 2 | skipped | 11.92 | skipped_unreachable |
| target_1 | 1 | succeeded | 19.43 | {"build_view_count": 2, "build_status": "局部重建完成：2 帧，11629 点（少于推荐 5 帧）；重叠 mean=4.4mm p95=17.9mm；TSDF 1651 点 / mesh 2652 顶点（累计积分 0.02s）；refit ACCEPT（袋模型 2 视）", "build_duration_s": 9.016, "stage_names": ["prepare", "observe", "finalize", "reconfirm", "approach_insert"], "stage_durations": [0.0, 8.825, 0.0, 0.329, 9.987], "harvest_confirmed": false, "completion_level": 1, "cut_confirmed": false, "retreat_confirmed": false} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260901_1704 | — | 0.21 | — | — | — | — | — | — | — | 0.21 |
| field_pregrasp_20260901_1704:target_2 | target_2 | — | 8.31 | — | 0.2 | 3.2 | — | — | 0.2 | 11.91 |
| field_pregrasp_20260901_1704:target_2 | — | 0.11 | — | — | — | — | — | — | — | 0.11 |
| field_pregrasp_20260901_1704:target_1 | target_1 | — | 8.81 | — | 0.21 | 10.0 | — | — | 0.2 | 19.22 |

## 事件统计

- 事件总数：7
- severity INFO：7
- `round_locked`：2
- `target_dispatched`：2
- `target_skipped`：1
- `target_succeeded`：1
- `targets_filtered`：1

## 感知统计

- 记录帧数：101
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：READY
- target_id：target_1
- captured_views：2
- rejected_views：40
- tf_failures：0
- tf_latency_ms：0.09179115295410156
- cloud_points：11629
- max_baseline_deg：10.186953909291795
- mean_nearest_baseline_deg：10.186953909291795
- tsdf_points：1651
- tsdf_integrate_time_s：0.015105962753295898
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：40
- skip_reasons：{'no_frame': 1, 'missing_mask': 1, 'same_stamp': 1, 'robot_not_static': 34, 'stale_frame': 3}
- last_skip_code：robot_not_static
- last_skip_reason：机器人未静止：最大关节速度 0.0909 rad/s > 0.03

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_2 | 2 | 37 | READY | robot_not_static {"no_frame": 1, "missing_mask": 1, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 31} |
| target_1 | 2 | 40 | READY | robot_not_static {"no_frame": 1, "missing_mask": 1, "same_stamp": 1, "robot_not_static": 34, "stale_frame": 3} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.236,-0.579,0.514 | 0.310,-0.625,0.605 | 0.298,-0.639,0.367 | 0.301,-0.624,0.466 |
| target_2 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
ptp to on-axis pregrasp (0/1):  | 0.612,-0.563,0.630 | 0.541,-0.617,0.728 | 0.533,-0.444,0.614 | 0.535,-0.526,0.670 |

## 运行性能统计

- 性能采样条数：40
- CPU %：均值 45.35 / 峰值 72.6
- 内存 %：均值 50.97 / 峰值 51.4
- GPU 利用率 %：均值 14.35 / 峰值 34.0
- GPU 显存 MB：均值 2057.45 / 峰值 2079.0

## 末端 TCP 轨迹

- 采样点数：364
- 路径长：1.0889 m
- 起止弦：0.5306 m
- 绕行比（路径/弦）：2.052（直线≈1）
- 相对弦最大偏离：0.1491 m
- Z 范围：0.3675 ~ 0.7113 m（Δz=-0.3407）
