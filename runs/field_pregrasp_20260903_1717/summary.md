# 采摘批次摘要 `field_pregrasp_20260903_1717`

## 批次概览

- 终局状态：RECOVERY_REQUIRED
- 复扫轮数：1
- 开始时间：2026-09-03 17:18:05
- 结束时间：2026-09-03 17:18:42
- 总时长：36.6 s
- 终局计数：unfinished=1
- 会话根：`runs/field_pregrasp_20260903_1717/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | — |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_0 | 1 | unfinished | — |  |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1717 | — | 0.01 | — | — | — | — | — | — | — | 0.01 |
| field_pregrasp_20260903_1717:target_0 | target_0 | — | 10.22 | — | 0.39 | 9.22 | — | — | 0.21 | 20.04 |

## 事件统计

- 事件总数：3
- severity INFO：3
- `photo_pose_reached`：1
- `round_locked`：1
- `target_dispatched`：1

## 感知统计

- 记录帧数：89
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_0
- captured_views：2
- rejected_views：34
- tf_failures：0
- tf_latency_ms：0.10442733764648438
- cloud_points：13809
- max_baseline_deg：9.870352113319932
- mean_nearest_baseline_deg：9.870352113319932
- tsdf_points：1643
- tsdf_integrate_time_s：0.010094165802001953
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：45
- skip_reasons：{'missing_mask': 1, 'same_stamp': 1, 'near_duplicate': 11, 'robot_not_static': 32}
- last_skip_code：robot_not_static
- last_skip_reason：机器人未静止：最大关节速度 0.0589 rad/s > 0.03

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_0 | 2 | 45 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "near_duplicate": 11, "robot_not_static": 32} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.409,-0.587,0.586 | 0.398,-0.598,0.607 | 0.388,-0.598,0.510 | 0.395,-0.595,0.539 |

## 运行性能统计

- 性能采样条数：35
- CPU %：均值 46.37 / 峰值 73.5
- 内存 %：均值 47.24 / 峰值 47.4
- GPU 利用率 %：均值 19.74 / 峰值 38.0
- GPU 显存 MB：均值 2207.0 / 峰值 2225.0

## 末端 TCP 轨迹

- 采样点数：244
- 路径长：0.6969 m
- 起止弦：0.4254 m
- 绕行比（路径/弦）：1.638（直线≈1）
- 相对弦最大偏离：0.1182 m
- Z 范围：0.5099 ~ 0.7106 m（Δz=-0.1983）
