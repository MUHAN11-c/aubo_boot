# 采摘批次摘要 `field_pregrasp_20260903_1624`

## 批次概览

- 终局状态：INTERRUPTED
- 复扫轮数：1
- 开始时间：2026-09-03 16:25:32
- 结束时间：2026-09-03 16:27:38
- 总时长：125.2 s
- 终局计数：succeeded=1
- 会话根：`runs/field_pregrasp_20260903_1624/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 1.25 | ≥ 2.0 | ✗ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 1 | ≥1（有派发目标时） | ✓ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_7 | 1 | succeeded | 22.65 | {"build_view_count": 2, "build_status": "局部重建完成：2 帧，13711 点（少于推荐 5 帧）；重叠 mean=3.5mm p95=7.0mm；TSDF 1649 点 / mesh 2961 顶点（累计积分 0.02s）；refit ACCEPT（袋模型 2 视）", "build_duration_s": 12.231, "stage_names": ["prepare", "observe", "finalize", "reconfirm", "approach_insert"], "stage_durations": [0.0, 12.074, 0.0, 1.175, 9.176], "harvest_confirmed": false, "completion_level": 1, "cut_confirmed": false, "retreat_confirmed": false} |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260903_1624:target_7 | target_7 | — | 12.23 | — | 1.0 | 9.2 | — | — | 0.2 | 22.63 |

## 事件统计

- 事件总数：6
- severity INFO：6
- `photo_pose_reached`：2
- `round_locked`：2
- `target_dispatched`：1
- `target_succeeded`：1

## 感知统计

- 记录帧数：45
- 帧间隔中位数：0.8 s（≈1.25 FPS）
- 目标数范围：0 ~ 1

## 重建关键指标终值

- state：READY
- target_id：target_7
- captured_views：2
- rejected_views：36
- tf_failures：0
- tf_latency_ms：0.09131431579589844
- cloud_points：13711
- max_baseline_deg：10.051843194406004
- mean_nearest_baseline_deg：10.051843194406004
- tsdf_points：1649
- tsdf_integrate_time_s：0.018177509307861328
- grasp_allowed：False
- grasp_reason：bag_d95_exceeds_tool
- skipped_views：36
- skip_reasons：{'missing_mask': 6, 'stale_frame': 29, 'same_stamp': 1}
- last_skip_code：stale_frame
- last_skip_reason：缓存帧龄期 3.34 s > max_frame_age_s=2.0（陈帧拒采）

## 逐目标重建视角

| target_id | captured_views_max | skipped_views_max | last_state | last_skip |
|---|---:|---:|---|---|
| target_7 | 2 | 36 | READY | stale_frame {"missing_mask": 6, "stale_frame": 29, "same_stamp": 1} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_7 | approach | 否 | 工具档关；停在预抓取，ACK 后再 Survey | 0.391,-0.584,0.564 | 0.399,-0.599,0.605 | 0.397,-0.599,0.543 | 0.397,-0.599,0.543 |

## 运行性能统计

- 性能采样条数：47
- CPU %：均值 31.01 / 峰值 79.9
- 内存 %：均值 44.7 / 峰值 45.0
- GPU 利用率 %：均值 14.49 / 峰值 35.0
- GPU 显存 MB：均值 2139.15 / 峰值 2172.0

## 末端 TCP 轨迹

- 采样点数：257
- 路径长：0.7079 m
- 起止弦：0.4272 m
- 绕行比（路径/弦）：1.657（直线≈1）
- 相对弦最大偏离：0.1226 m
- Z 范围：0.5134 ~ 0.7104 m（Δz=-0.1948）
