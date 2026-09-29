# 视觉：看一帧 / 建一颗

包：`peach_harvester`。节点：`peach_scene_perception_node`、`peach_target_reconstruction_node`。大脑：`brain.py` 同一 `MultiThreadedExecutor`，图名与独立进程时相同。

## 感知（L1 事实）

入口：`ScenePerceptionNode`。纯核门面：`pipeline.PerceptionPipeline.process(SyncedRgbd) → PerceptionResult`。节点只 decode RGB-D、查 TF、发布。

一帧：

1. `message_filters` 同步 RGB + 深度（slop 0.05 s KEEP）
2. 深度 uint16 **毫米**；有效窗裁前景
3. YOLO-det 检测 → 去重 → MobileSAM 实例掩膜（无 SAM 则深度带连通域降级，须标 `mask_source`）
4. `make_pipeline`：`bag` = `RobustBagPosePipeline`（圆柱 RANSAC 定轴）；`fruit` = `RobustFruitPosePipeline`（球拟合+梗洼定向）。产品范围是套袋；裸果不进选果
5. 单帧状态 ACCEPT / REOBSERVE / REJECT——**不得据此接触**
6. `identity.TargetRegistry`：世界系 χ²≤9 + 匈牙利 1-1 + 类别 + 歧义比 1.2。新目标 `target_{N}`；`clear()` **不复位**计数（防换场撞号）
7. `CollectLockPolicy` / `GlobalHarvestPlan`：收齐窗关闭锁定，`snapshot_id` +1，`harvest_run_id` 每轮重铸
8. 发布：`/peach/perception/target_observations`（调度选果+重建+臂缓存）、`initial_pose`（**只进重建**）、diagnostics、debug_image

BeginScene：Active 才受理；`scene_epoch++`；换 `scene_key` 清身份表。调度 `HarvestState.target_id` 反哺覆盖感知 selected，旧目标 `mark_completed`。

静止门 / 有效深度占比 / 贴边框不攒确认：见 `image_gates.py`。精确 stamp TF 失败则 `tf_status!='ok'`，**不进身份链**。

## 重建（L1 模型）

入口：`TargetReconstructionNode`。宿主：`ReconstructionCore`。会话：`ReconstructionSession`。融合：`RefitOrchestrator`。

流程：

1. `BuildTargetModel` 或 `initial_pose` 自动绑定（`capture.auto_mode`）
2. 每唯一 RGB-D stamp：锁外查 **base←camera 精确时刻 TF**（失败跳帧，禁止运动中 latest）
3. 质量门 `StrictMaskGate`（五道）→ FK 变到 base → 有界 ICP（只修小刚性误差）→ 在线 TSDF
4. finalize：抽网格 + 柱/球 refit（`refine.py` 的 `REFITTERS_BY_IMPL`，UNWIND dict）
5. 动态径向/轴向预算 `tool_budget.py` → `GraspDecision`

发布（均为 latched）：`grasp_decision`、`refined_pose`、`refined_axis`、`refined_diagnostics`、`tsdf_cloud`、`pregrasp_verification`、`shape_hypothesis`。心跳约 1 Hz **不得续签** `valid_until`。

`allowed` 是 geometry∧sleeve∧cut 汇总展示。臂侧分档：套入看 `sleeve_capability` / `radial_margin_m>0`；剪切看 `cut_capability` / `axial_margin_m>0` + `pregrasp_verified`。负余量禁止对应阶段。禁止单帧/unrefined 降级接触（除非显式 `skip_reconstruction` + 臂 `allow_unrefined_geometry`）。

## 缝位（不要再扩）

| yaml | 实现 |
|------|------|
| `pipeline.*_impl` | `pose_pipelines.PIPELINES_BY_IMPL` |
| `refitter.*_impl` | `refine.REFITTERS_BY_IMPL` |

新可替换算法走 pluginlib。YOLO/SAM/匹配器/TSDF/ICP 现行直接构造。

## 工具档案

`tool_profiles.py`：三把剪切手 `shear_v1` / `bite_shear_v1`（MTC 沿轴 LIN 套入）/ `adaptive_shear_v1`（默认；预抓取后 `imu_follow` 窗，禁止与 MTC 同时写控制器）。单一事实源 `aubo_description/config/<profile_id>.yaml`；launch 期注入 scene `tool.D_inner` / `tool.L_insert` / `tool.L_blade`（径向走廊门 + 行程/刀口轴向，2026-09-29 扩）、recon `tool.budget.d_inner` + `tool.profile_id`。停位：`grasp_standoffs.yaml`（预抓取沿 −axis 后撤，现行 0.03 m）。
