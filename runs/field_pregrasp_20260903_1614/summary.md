# 采摘批次摘要 `field_pregrasp_20260903_1614`

## 批次概览

- 终局状态：RECOVERY_REQUIRED
- 复扫轮数：1
- 开始时间：2026-09-03 16:14:36
- 结束时间：2026-09-03 16:15:11
- 总时长：35.7 s
- 终局计数：unfinished=1
- 会话根：`runs/field_pregrasp_20260903_1614/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | unfinished | — |  |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1614 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260903_1614:target_7 | target_7 | — | 9.41 | — | 0.2 | 9.21 | — | — | 0.2 | 19.02 |

## 事件统计

- 事件总数：3
- severity INFO：3
- `photo_pose_reached`：1
- `round_locked`：1
- `target_dispatched`：1

## 感知统计

- 记录帧数：85
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：READY
- target_id：target_7
- captured_views：2
- rejected_views：33
- tf_failures：0
- tf_latency_ms：0.10704994201660156
- cloud_points：14022
- max_baseline_deg：8.934697148649665
- mean_nearest_baseline_deg：8.934697148649665
- tsdf_points：1564
- tsdf_integrate_time_s：0.009226799011230469
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：42
- skip_reasons：{'missing_mask': 1, 'same_stamp': 1, 'near_duplicate': 9, 'robot_not_static': 30, 'stale_frame': 1}
- last_skip_code：robot_not_static
- last_skip_reason：机器人未静止：最大关节速度 0.3903 rad/s > 0.03

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_7 | 2 | 42 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "near_duplicate": 9, "robot_not_static": 30, "stale_frame": 1} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_1 | lock | 否 | 工具档关；目标身份/可见性安全门失败: selected_target_stale | 0.328,-0.589,0.568 | — | — | — |
| target_7 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.392,-0.575,0.575 | 0.399,-0.597,0.610 | 0.397,-0.594,0.543 | 0.397,-0.594,0.543 |

## 运行性能统计

- 性能采样条数：34
- CPU %：均值 43.63 / 峰值 90.1
- 内存 %：均值 43.19 / 峰值 43.6
- GPU 利用率 %：均值 15.03 / 峰值 34.0
- GPU 显存 MB：均值 2028.41 / 峰值 2041.0

## 末端 TCP 轨迹

- 采样点数：247
- 路径长：0.6801 m
- 起止弦：0.4094 m
- 绕行比（路径/弦）：1.661（直线≈1）
- 相对弦最大偏离：0.1198 m
- Z 范围：0.5425 ~ 0.7105 m（Δz=-0.1657）
