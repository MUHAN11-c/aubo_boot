# 2026-09-18 真机预抓取轮（field_pregrasp_20260918_1029 / 1039 / 1055）

操作：ZCode 代理（用户口头授权=发起本轮目标）。工具档 adaptive_cylinder_v1（launch 默认）。
场景：实验室单袋 target_0，袋底 r≈0.732 m / z≈0.641 m，拟合直径 0.055 m，光照偏低
（掩膜内有效深度占比 EMA≈0.30–0.32 < 0.35 门，感知持续告警）。全程 tool.enabled=false、
execute_pregrasp_only=true、无 SetIO。web 8090 监控全流程在线（流水线/事件/账本/六关节实时）。

## 结果一览

| 批次 | 结果 | 直接原因 |
|------|------|----------|
| 1029 | ABORTED（进 target_0 观察段即崩） | `executor_node.py:1317` `now().nanoseconds()` 误当方法调用（rclpy 属性）→ run_harvest 回调抛异常；已修复为 `.nanoseconds` 并重建 |
| 1039 | target_0 SKIPPED_UNREACHABLE（12 次重试全拒，真机有运动） | 接近全部 MTC 0 解：`<octomap> × wrist2_Link` 真实碰撞（袋上方 z0.70–1.01 有真实枝叶回波，相机距≈0.55 m，非 1.01/2.02 鬼影带） |
| 1055 | target_0 SKIPPED_UNREACHABLE（octomap 已关，臂有运动） | 审查门全过后 Pilz 生成轴向 LIN `NO_IK_SOLUTION`（goal 位姿 IK 无解）；伴随 table_link×upperArm / foreArm×wrist2 边界构型自碰采样 |

## 根因链（1055 主线）

1. 今日袋位比成功参考 1757（09-01，hollow 工具）多要 ~8 cm 伸展：肩到预抓取 0.846 vs 0.786 m，
   加 adaptive 档 TCP +17.6 mm。
2. 选果预检（单点停位 IK、种子=当前关节）放行，但 Pilz LIN goal IK 更严 → 拒。
   预检门缝：CheckReachability 单点 ≠ LIN 全路径可行性（改进项）。
3. 关掉的只是 octomap；table_link×upperArm 是 URDF 自碰（工作台是机器人模型一部分），
   边界构型才蹭到，正常构型（1757 类）不会碰——与用户经验一致「之前不可能碰」。

## 点云统计（结合 PS800-E1 硬件参数）

- 稀疏云 ~24.7k 点（≈640×480 的 8%，驱动端已滤无效深度，point_step=20）
- 单位自检 ✓：果 0.53 m 相机距落 0.50–0.75 m 档（无 ÷4000 vs ÷1000 的 4× 单位错）
- 1.01 m 鬼影带 2.5%、2.02 m 带 ≈0%（存在但非主质量）
- 远端离群超窗：p99 z=3.92 m、x 到 −2.26 m（> 0.3–2.5 m 标称窗；octomap 侧 max_range=2.0 已裁）
- 袋 XY 15cm 走廊：z<0.62 空（袋下无回波，被袋自身遮挡）；z0.70–1.01 有 ~1.8k 真实点（枝叶）

## 果胶囊门设计复核（用户质询「护圈应采用感知拟合圆柱」）

- 果侧 ✓：`fruitRadiusM(diameter, inflation=0.01, fallback=0.12)`——拟合直径优先，仅无效时回退并告警。
- 工具侧 ⚠ 两处债：①实心圆柱模型（kToolBodyRadiusM=0.060/L=0.200，空心筒 D_inner=0.116）——
  侧向防撞语义正确，但「袋进筒口」的口侧准备会被实心假设多拦；②常量硬编码，未走工具档案
  单一事实源（当前两档 D_outer/L 相同故数值未脱钩，改档案即脱钩）。
- 拒例复核：1639_1（−24.6mm，轴距≈7.5cm）即使空心环模型仍拒（落在壁环带）=真侧贴，护栏正确；
  12 rad 行程门与 0.25 m 弦偏离门是 09-11 收紧后的设计值（当时即拒 12.7–12.9 rad 案），
  「14/14→7/14」主要是护栏收紧所致，非 adaptive 回归（两档外径同）。

## mock 回放（sim_field_targets --case all --velocity 1.0，adaptive）

- 0/14→7/14 三段式：①09-18 身份元组新门拒全部（回放架未填 model_revision 等 4 字段），
  已补注入；②decision 缺 valid_until → model_not_executable 4 例，已补 now+30s（1 例仍竞态，
  待放宽）；③最终 7/14 过：3 行程门、1 弦偏离、1 筒体压胶囊、1 规划 0 解、1 有效期竞态。
- 回放修复随本轮入库：goal/decision 补身份元组与有效期（sim_field_targets.py）。

## 其他缺陷与改进项

- [P1] 取消路径终局空：CANCEL_NOW 后 ABORTED、termination_reason=''、summary 全 0（1039/1055 复现 2 次）。
- [P2] 批次停滞：1055 收尾段 5 min 无臂活动、无日志推进，per_target_timeout 未兜住（与「禁 >20s 阻塞」裁定相悖）。
- [P2] web 前端「柜侧硬件」面板光照字段显示 `[object Object]`（纯展示 bug）。
- [P2] CheckReachability 预检门缝（见根因链 2）。
- [P3] 光照偏低影响拟合质量（本轮 axis_confidence 0.46、REOBSERVE、σ_axis 8.5°）——建议现场补光。
- 减速：用户裁定暂时不减速——运行期 `ros2 param set /peach_arm moveit.approach_near_velocity_scaling 1.0`
  （默认 0.05 不动仓库）。

## 临时态（须恢复）

- `src/aubo_e5_moveit_config/config/sensors_3d.yaml` `sensors: []`（用户指令「碰撞先不开」；
  恢复=把 camera_pointcloud 加回 sensors）。生产/有人近臂时必须恢复。

## 过程数据

runs/field_pregrasp_20260918_{1029,1039,1055}/（round_report.md 已归集）；会话 bag 三段随栈停自动封口。

## 17:07–17:13 相机前端感知对比（不动臂）

Percipio vs peach_stereo，同场景静态腕相机，使能关。两前端各锁 1 个中间袋；debug 10s 墙钟 21 vs 37 帧；相机距 0.553 vs 0.565 m。拼接视频 `runs/camera_ab_20260918/comparison_2x2.mp4`。

