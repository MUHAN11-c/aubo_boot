# 采摘批次摘要 `field_pregrasp_20260909_1527`

## 批次概览

- 终局状态：COMPLETED
- 开始时间：2026-09-09 15:27:39
- 结束时间：2026-09-09 15:28:50
- 总时长：70.4 s
- 终局计数：skipped=2
- 会话根：`runs/field_pregrasp_20260909_1527/`（账本/感知/会话同根，R7 单根目录）

## 验收门对照（口径见 docs/testing.md 量化基线）

| 指标 | 实测 | 门 | 判定 |
|---|---:|---|---|
| 感知帧率 FPS | 2.5 | ≥ 2.0 | ✓ |
| 重建 TF 失败次数 | 0 | = 0 | ✓ |
| 到预抓取停住（succeeded） | 0 | ≥1（有派发目标时） | ✗ |

## 逐目标 outcome

| target_id | 优先级 | outcome | 耗时 s | 原因 |
|---|---:|---|---:|---|
| target_1 | 2 | skipped | 15.82 | skipped_unreachable |
| target_2 | 3 | skipped | 13.22 | skipped_unreachable |

## 每阶段耗时统计（按目标周期）

| 周期 | 目标 | SELECTING | OBSERVING | FINALIZING | VALIDATING | APPROACHING | TOOL_ACTION | RETREATING | COMPLETING | 合计 s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| field_pregrasp_20260909_1527 | — | 0.1 | — | — | — | — | — | — | — | 0.1 |
| field_pregrasp_20260909_1527:target_1 | target_1 | — | 11.4 | — | 0.0 | 4.2 | — | — | 0.2 | 15.8 |
| field_pregrasp_20260909_1527:target_1 | — | 0.13 | — | — | — | — | — | — | — | 0.13 |
| field_pregrasp_20260909_1527:target_2 | target_2 | — | 8.41 | — | 0.2 | 4.2 | — | — | 0.2 | 13.01 |
| field_pregrasp_20260909_1527:target_2 | — | 0.22 | — | — | — | — | — | — | — | 0.22 |

## 事件统计

- 事件总数：15
- severity INFO：15
- `photo_pose_reached`：4
- `round_locked`：4
- `target_dispatched`：2
- `target_skipped`：2
- `targets_filtered`：3

## 感知统计

- 记录帧数：167
- 帧间隔中位数：0.4 s（≈2.5 FPS）
- 目标数范围：0 ~ 3

## 重建关键指标终值

- state：IDLE
- target_id：
- captured_views：0
- rejected_views：0
- tf_failures：0
- tf_latency_ms：0.09822845458984375
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
| target_1 | 2 | 44 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "near_duplicate": 3, "robot_not_static": 39} |
| target_2 | 2 | 36 | READY | robot_not_static {"missing_mask": 1, "same_stamp": 1, "robot_not_static": 34} |

## 逐目标作业票（感知→抓取）

| target_id | 当前环节 | 抓取许可 | 原因/档位 | 感知入口 | 重建中心 | 预抓取 | 抓取进入 |
|---|---|---|---|---|---|---|---|
| target_0 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
cartesian corridor 2/4 (0/1):  | 0.591,-0.696,0.714 | — | — | — |
| target_1 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
cartesian corridor 2/5 (0/1):  | 0.252,-0.754,0.587 | 0.223,-0.751,0.633 | 0.288,-0.739,0.567 | 0.269,-0.743,0.589 |
| target_2 | lock | 否 | 工具档关；到预抓取失败: MTC planning failed: Failing stage(s):
cartesian corridor 2/4 (0/1):  | 0.521,-0.763,0.612 | 0.518,-0.771,0.645 | 0.514,-0.744,0.578 | 0.516,-0.754,0.606 |

## 运行性能统计

- 性能采样条数：68
- CPU %：均值 25.18 / 峰值 54.7
- 内存 %：均值 46.18 / 峰值 46.9
- GPU 利用率 %：均值 14.06 / 峰值 36.0
- GPU 显存 MB：均值 1440.57 / 峰值 1717.0

## 末端 TCP 轨迹

- 采样点数：331
- 路径长：0.548 m
- 起止弦：0.0 m
- 绕行比（路径/弦）：—（直线≈1）
- 相对弦最大偏离：0.1672 m
- Z 范围：0.6748 ~ 0.7143 m（Δz=0.0）
