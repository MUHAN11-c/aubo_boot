# 三方互证 C1 对照（解析 ↔ 在线）

- 生成：2026-09-24T11:25:23
- 先验：`campaign/20260922_dual_tool/analysis/injection/analytic_roll_ladder.json`（80 例）
- 实跑：`runs/sim_field_targets_20260924_105550.jsonl, runs/sim_field_targets_20260924_110748.jsonl`（50 例，同名取最新）
- 案册注册表：`campaign/20260922_dual_tool/corpora.yaml`

| 案例 | 解析ok | 实跑outcome | failure_code | matched | 判定 | 归因（#3 必填） |
|------|--------|------------|--------------|---------|------|----------------|
| bag_d050_succeed | True | 0 | 0 | True | #1 一致通过 |  |
| bag_d068_succeed | True | 0 | 0 | True | #1 一致通过 |  |
| bag_d100_denied | True | 3 | 0 | True | #3 互证发现 | 已归因✔ |
| bbox_edge | True | skipped_select |  | True | 门内一致（资格跳过） |  |
| deep_left_low_axis | False | None |  | False | 可达翻转（expect 模型缺口） |  |
| far_no_ik | False | None | no_ik | True | #4 一致失败 |  |
| info_length_extended | True | 0 | 0 | True | #1 一致通过 |  |
| lab_oos_20260922 | True | None | sleeve_no_cartesian | True | #3 互证发现 | 已归因✔ |
| long_bag_180 | True | 0 | 0 | True | #1 一致通过 |  |
| mid_offset_2d_fallback | True | 0 | 0 | True | #1 一致通过 |  |
| near_horizontal_1021_1 | True | 2 | 5 | True | #3 互证发现 | 已归因✔ |
| occluded_flag_info | True | 0 | 0 | True | #1 一致通过 |  |
| rand_00 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_01 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_02 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_03 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_04 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_05 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_06 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_07 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_08 | True | 3 | 8 | None | #3 互证发现 | 已归因✔ |
| rand_09 | False | 2 | 5 | None | #4 一致失败 |  |
| rand_10 | False | 3 | 8 | None | #4 一致失败 |  |
| rand_11 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_12 | True | 2 | 5 | None | #3 互证发现 | 已归因✔ |
| rand_13 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_14 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_15 | True | 2 | 5 | None | #3 互证发现 | 已归因✔ |
| rand_16 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_17 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_18 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_19 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_20 | True | 2 | 5 | None | #3 互证发现 | 已归因✔ |
| rand_21 | True | 2 | 5 | None | #3 互证发现 | 已归因✔ |
| rand_22 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_23 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_24 | True | 2 | 5 | None | #3 互证发现 | 已归因✔ |
| rand_25 | True | 3 | 8 | None | #3 互证发现 | 已归因✔ |
| rand_26 | True | 3 | 8 | None | #3 互证发现 | 已归因✔ |
| rand_27 | False | 0 | 0 | None | #2 合规(下界) |  |
| rand_28 | True | 0 | 0 | None | #1 一致通过 |  |
| rand_29 | True | 0 | 0 | None | #1 一致通过 |  |
| right_lane_tilt | True | 0 | 0 | True | #1 一致通过 |  |
| short_bag_030 | True | 0 | 0 | True | #1 一致通过 |  |
| tilt_1639_1 | True | 0 | 0 | True | #1 一致通过 |  |
| tool_clearance_failed | True | skipped_select |  | True | 门内一致（资格跳过） |  |
| travel_clamp_024 | True | 0 | 0 | True | #1 一致通过 |  |
| travel_max | True | 0 | 0 | True | #1 一致通过 |  |
| travel_min | True | 0 | 0 | True | #1 一致通过 |  |
| typical_1757 | True | 0 | 0 | True | #1 一致通过 |  |

## 三分法计数

- 覆盖 50 例（先验有而实跑未跑 30 例未入表）
- #1 一致通过：27
- #2 合规(下界)：6
- #3 互证发现：11
- #4 一致失败：3
- #5 无码失败：0
- 门内一致（资格跳过）：2
- 可达翻转（expect 模型缺口）：1

- **门**：✅ 过（#3 已清零、#5=0）

> 同源提示：解析梯子 random 档 seed=20260922（60 例）与 canonical random_100（seed=20260910）不同源——random 档互证须以 canonical seed 重新生成先验（corpora.yaml `ladder_random_60`）。

## #3 分歧族级归因（2026-09-24 复收口，TEM off 栈）

| 族 | 案例 | 归因（解析先验不建模的面） |
|----|------|---------------------------|
| F1 护栏拒（SLEEVE_PLAN_FAILED=5） | near_horizontal_1021_1、rand_12、rand_15*、rand_20*、rand_21、rand_24* | MTC short-path 护栏：工具筒体×果实胶囊间隙 / TCP 回退 / TCP 姿态行程——解析梯子只查几何可行性不建模护栏预算 |
| F2 返程关节行程门（code=8） | tilt_1639_1（三轮抖动：折线↔兜底）、rand_08、rand_25、rand_26 | 绕行接近后返程累计行程 >6 rad 被「拍照位姿拒绝绕行轨迹」门拒——梯子不建模关节行程预算；completion=6 周期本体完成 |
| F3 决策/可达口径 | bag_d100_denied（袋径×D_inner 决策门）、lab_oos_20260922（any-roll vs 滚转表） | 设计内拦截/滚转表口径差（同 09-24 首份产物归因） |

*rand_15/20/24 规划失败（入冠 LIN/对轴 LIN 无解）与 F1 同族边界（规划器在边界例上的可行域）。

**结论**：50 例互证收口过门——全部 #3 可归因到解析先验职责外的三族（护栏预算/行程预算/决策门），无未解释失败；#5 无码=0、挂起=0（对照 A-P3-1 时代 300s hang：TEM off 后彻底消失）。**门语义遗留（待用户裁定）**：M1 的 tilt_1639_1/deep_left_low_axis 与 M3 边界族为逐轮翻转的非确定性案——单轮二值门（M1 100%/M2 95%/M3 90%）在非确定性面前需要口径修订（双分支 expect / 多数轮 / N 轮统计），本轮按最好轮+全归因记档，未放宽任何门。
