# 视觉：看一帧 / 建一颗 / 场景障碍

包：`peach_harvester`。大脑进程三节点：`peach_scene_perception_node`、`peach_target_reconstruction_node`、`peach_supervisor`（`brain.py`）。**第四个视觉进程** `peach_scene_obstacles` 独立、不进 lifecycle、不进 brain。

## 感知（L1 事实）

入口：`ScenePerceptionNode`。纯核门面：`pipeline.PerceptionPipeline.process(SyncedRgbd) → PerceptionResult`。节点只 decode RGB-D、查 TF、发布。

一帧：

1. `message_filters` 同步 RGB + 深度（slop 0.05 s KEEP）
2. 深度 uint16 **毫米**；有效窗裁前景
3. YOLO-det 检测 → 去重 → MobileSAM 实例掩膜（无 SAM 则深度带连通域降级，须标 `mask_source`）
4. `make_pipeline`：`bag` = `RobustBagPosePipeline`（圆柱 RANSAC 定轴）；`fruit` = `RobustFruitPosePipeline`（球拟合+梗洼定向）。产品执行范围是套袋；裸果可显示，不进选果
5. 单帧状态 ACCEPT / REOBSERVE / REJECT——**不得据此接触**
6. `identity.TargetRegistry`：世界系 χ²≤9 + 匈牙利 1-1 + 类别 + 歧义比 1.2。新目标 `target_{N}`；`clear()` **不复位**计数
7. `CollectLockPolicy` / `GlobalHarvestPlan`：收齐窗关闭锁定，`snapshot_id` +1，`harvest_run_id` 每轮重铸
8. 发布：`/peach/perception/target_observations`（调度选果+重建+臂缓存）、`initial_pose`（**只进重建**）、diagnostics、debug_image

BeginScene：Active 才受理；`scene_epoch++`；换 `scene_key` 清身份表。调度 `HarvestState.target_id` 反哺覆盖感知 selected，旧目标 `mark_completed`。

静止门 / 有效深度占比 / 贴边框不攒确认：`image_gates.py`。精确 stamp TF 失败则 `tf_status!='ok'`，**不进身份链**。

## 重建（L1 模型）

入口：`TargetReconstructionNode`。宿主：`ReconstructionCore`。会话：`ReconstructionSession`。融合：`RefitOrchestrator`。

流程：

1. `BuildTargetModel` 或 `initial_pose` 自动绑定（`capture.auto_mode`）
2. 每唯一 RGB-D stamp：锁外查 **base←camera 精确时刻 TF**（失败跳帧，禁止运动中 latest）
3. 质量门 `StrictMaskGate`（五道）→ FK 变到 base → 有界 ICP → 在线 TSDF
4. finalize：抽网格 + 柱/球 refit（`refine.py` 的 `REFITTERS_BY_IMPL`，UNWIND dict）
5. 动态径向/轴向预算 `tool_budget.py`（内径随 `tool.budget.d_inner` 档案注入）→ `GraspDecision`

发布（latched）：`grasp_decision`、`refined_pose`（臂 + **scene_obstacles 胶囊** + 观测）、`refined_axis`、`refined_diagnostics`、`tsdf_cloud`、`pregrasp_verification`、`shape_hypothesis`。心跳约 1 Hz **不得续签** `valid_until`。

`allowed` = geometry∧sleeve∧cut 汇总。臂侧分档：套入看 `sleeve_capability` / `radial_margin_m>0`；剪切看 `cut_capability` / `axial_margin_m>0` + `pregrasp_verified`。禁止单帧/unrefined 降级接触（除非 `skip_reconstruction` + 臂 `allow_unrefined_geometry`）。

## 场景障碍（③层，0035/0036）

节点：`peach_scene_obstacles`（`vision/scene_obstacles/`）。`camera_enabled` 才随 harvest_system 起。不进 lifecycle。

触发：supervisor Survey **成功**后发 `/peach/scene/obstacles_refresh`（Empty，TL）。源帧=最近 `/camera/depth_registered/points`（订阅 **RELIABLE**，两前端发布端均 RELIABLE）。

纯核 `core.build_snapshot` 滤除链（顺序即语义；体素先行）：

1. 体素化 0.06 m + 工作空间裁剪
2. 自身滤除：体素中心距 collision mesh+FK ≤ `self_filter_margin_m` + 体素半对角（防方块角切工具致 FCL 起点碰撞）
3. 已精化目标膨胀胶囊滤除（`refined_pose` 累积；未精化不滤）
4. 上限 3000 box

经 `/apply_planning_scene` 原子写入对象 id=`peach_scene_obstacles`（首写 ADD，此后同请求 REMOVE+ADD）。作业期冻结；新批次 Survey 重建；新目标精化后同帧重写。只为保护 **相机**；臂/末端豁免。

## 缝位（不要再扩）

| yaml | 实现 |
|------|------|
| `pipeline.*_impl` | `pose_pipelines.PIPELINES_BY_IMPL` |
| `refitter.*_impl` | `refine.REFITTERS_BY_IMPL` |

新可替换算法走 pluginlib。YOLO/SAM/匹配器/TSDF/ICP/障碍滤除现行直接构造。

## 工具档案

`tool_profiles.py` 读 `aubo_description/config/<profile_id>.yaml`：

| profile | 套入 |
|---------|------|
| `shear_v1` | MTC 沿轴 LIN |
| `bite_shear_v1` | MTC 沿轴 LIN |
| `adaptive_shear_v1`（默认） | 预抓取后 `imu_follow` 窗；禁止与 MTC 同时写控制器 |

launch 注入：scene `tool.D_inner` / `L_insert` / `L_blade`；recon `tool.budget.d_inner` + `profile_id`；臂 `tool.body_*`。停位：`grasp_standoffs.yaml`（预抓取沿 −axis 后撤，现行 0.03 m）。
