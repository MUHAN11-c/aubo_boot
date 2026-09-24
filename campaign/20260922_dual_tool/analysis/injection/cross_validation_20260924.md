# 三方互证 C1 对照（解析 ↔ 在线）

- 生成：2026-09-24T10:27:26
- 先验：`campaign/20260922_dual_tool/analysis/injection/analytic_roll_ladder.json`（80 例）
- 实跑：`runs/sim_field_targets_20260923_153659.jsonl, runs/sim_field_targets_20260923_173249.jsonl`（20 例，同名取最新）
- 案册注册表：`campaign/20260922_dual_tool/corpora.yaml`

| 案例 | 解析ok | 实跑outcome | failure_code | matched | 判定 | 归因（#3 必填） |
|------|--------|------------|--------------|---------|------|----------------|
| bag_d050_succeed | True | 0 | 0 | True | #1 一致通过 |  |
| bag_d068_succeed | True | 0 | 0 | True | #1 一致通过 |  |
| bag_d100_denied | True | 3 | 0 | True | #3 互证发现 | 已归因✔ |
| bbox_edge | True | skipped_select |  | True | 门内一致（资格跳过） |  |
| deep_left_low_axis | False | None | sleeve_no_cartesian | False | #4 一致失败 | expect 过期（双败） |
| far_no_ik | False | None | no_ik | True | #4 一致失败 |  |
| info_length_extended | True | 0 | 0 | True | #1 一致通过 |  |
| lab_oos_20260922 | True | None | sleeve_no_cartesian | True | #3 互证发现 | 已归因✔ |
| long_bag_180 | True | 0 | 0 | True | #1 一致通过 |  |
| mid_offset_2d_fallback | True | 0 | 0 | True | #1 一致通过 |  |
| near_horizontal_1021_1 | True | 2 | 5 | False | #3 互证发现 | 已归因✔ |
| occluded_flag_info | True | 0 | 0 | True | #1 一致通过 |  |
| right_lane_tilt | True | 0 | 0 | True | #1 一致通过 |  |
| short_bag_030 | True | 0 | 0 | True | #1 一致通过 |  |
| tilt_1639_1 | True | 0 | 0 | True | #1 一致通过 |  |
| tool_clearance_failed | True | skipped_select |  | True | 门内一致（资格跳过） |  |
| travel_clamp_024 | True | 0 | 0 | True | #1 一致通过 |  |
| travel_max | True | 0 | 0 | True | #1 一致通过 |  |
| travel_min | True | 0 | 0 | True | #1 一致通过 |  |
| typical_1757 | True | 0 | 0 | True | #1 一致通过 |  |

## 三分法计数

- 覆盖 20 例（先验有而实跑未跑 60 例未入表）
- #1 一致通过：13
- #2 合规(下界)：0
- #3 互证发现：3
- #4 一致失败：2
- #5 无码失败：0
- 门内一致（资格跳过）：2

- **门**：✅ 过（#3 已清零、#5=0）

> 同源提示：解析梯子 random 档 seed=20260922（60 例）与 canonical random_100（seed=20260910）不同源——random 档互证须以 canonical seed 重新生成先验（corpora.yaml `ladder_random_60`）。

## #3 分歧归因（三例，2026-09-24 裁定关闭）

| 案例 | 归因 | 处置 |
|------|------|------|
| near_horizontal_1021_1 | 护栏正确拒：MTC short-path 工具筒体×果胶囊间隙 -0.031（SLEEVE_PLAN_FAILED=5，09-23 两轮一致）；解析梯子不建模胶囊间隙 | expect 已改 `deny_guardrail`（夹具 09-24 修订）；词表/判据同轮入 sim_field_targets + test_constraint_grid |
| bag_d100_denied | 解析盲区：梯子不建模袋径×D_inner 决策门（A-P3-2 批次4 臂侧门）——live 侧 matched=True（deny_decision 预期成立），属设计内拦截 | 解析侧不改（决策门不在几何先验职责内）；互证口径记档：deny 类案例以 matched 为准 |
| lab_oos_20260922 | 解析口径差：梯子 any-roll 乐观 vs CheckReachability 固定滚转表 sleeve 笛卡尔更严（sleeve_no_cartesian）；live matched=True（skip_cartesian 预期成立） | 记解析盲区清单；若未来同类增多，梯子补滚转表同源判据 |

**结论**：grid 20 例互证收口——#3 三例全部归因关闭（两例解析盲区/口径差 + 一例护栏裁定），#5 为零，#4 两例均为预期不可行（deep_left expect 过期已修、far_no_ik 预期 skip_ik）。机器空闲后带 TEM off + 新词表复跑 M1 网格（20/20 预期）与 M2–M4，重定基线（阶段二 B 段）。
