# peach_arm：一颗 ExecuteTarget

节点图名 `peach_arm`，类 `ManipulationSkillsNode`。动作受理 `cycle.cpp`，阶段序列 `stages.cpp::executeCycle`。MTC 接触 `grasp_task.cpp`。缓存 `target_cache.cpp`。纯核门 `safety_gate` / `quality_gate` / `trajectory_guard`。

## 受理门（onActionGoal）

非 Active / 周期占用 / `contact_recovery_required` → REJECT。

mode ∈ {PREVIEW, OBSERVE_ONLY, FULL, PREGRASP_ONLY}。FULL/PREGRASP 须模型七元组 `identityComplete`（非空 presence-check）。目标须命中锁定集锚点，否则回退 selected 缓存。

`execution_enabled` 时一次性 `execution_armed`；周期结束自动解除。PREVIEW：`execution_enabled=false` 只规划。

## executeCycle 阶段

```
stagePrepareCycle          钉 target 快照、生成视点
  execution_enabled? 否 → stagePlanPreview（终结）
  skip_observation? 否 → stageAcquireViews（最近短 LIN，最多两次）
stageFinalizeAndValidate   等精化、质量门
  OBSERVE_ONLY → ReportObserveOnly
  grasp_enabled 关 → ReportReady
  接触：
    stageReconfirmTarget
    stageMovePregrasp          PREGRASP 级授权
    stageVerifyPregrasp        残差门；最多两次增量
    PREGRASP_ONLY → Hold（停住不 stow 不 SetIO；仍置 recovery）
    FULL：
      stagePlanSleeveAndReverseRetreat
      stageSleeveLinear        CONTACT 级（令牌 sleeve/径向）
      stageActuateCutter       TOOL 级（令牌 cut/轴向 + verified）
      stageVerifyCut
      stageExecuteReservedReverseRetreat
      stageReturnHarvestStow
      stageReleasePayload / VerifyHarvestOutcome / CompleteTarget
```

终局优先级：接触 recovery（PREGRASP 成功除外）> 取消 > 成功 > 失败。

## authorizeStage（命令门）

TRANSIT / PREGRASP → `authorizeTransit`（使能×Active×robotReady×¬cancel；**不**复检 GraspDecision）。

CONTACT / TOOL：

1. `motionOutputAllowed`（Active + armed/enabled）
2. `safetyReady`（`robot_status`：抱闸/motion_possible/急停**状态**；mock 可 `require_robot_status=false`）
3. ¬cancel、`grasp_enabled`
4. 令牌优先：`allowed`、target 绑定、`valid_until`、`model_stamp` 新鲜度窗
5. CONTACT：`radial_margin_m>0`；TOOL：`axial_margin_m>0` + `pregrasp_verified` + `tool_enabled`
6. 无令牌：快照 `grasp_allowed` + 能力三态

过期 → `StageDenial::EXPIRED`（可重派）；其余 DENIED。`requireStageAuthority` 映射到 SKIPPED_QUALITY / FAILED。

**套入/SetIO 不得绕过此门。** 8090 调试运动另需 yaml `debug.motion_enabled`，仍进同一门。

## 接近主路径（2026-09-23 v4）

观察停在 look-at，现场常无 IK → **先回拍照位**。

然后唯一主档（无候选扫描/多级兜底）：

1. Pilz PTP 到中段点正下方（树冠外；滚转梯子 {0,±30°,±60°}）
2. 世界垂直 LIN 入冠（`approach_canopy_entry_m`）
3. 沿轴 LIN 进预抓取（`approach_final_axial_m`；对轴 20° OrientationConstraint）

冠内 LIN 优先 Pilz sequence blend；失败降级单任务 MTC LIN。护栏：果实胶囊（仅接近段）、反爬、场景障碍（③层=Survey 快照对象 `peach_scene_obstacles`，仅 `camera_body_link` 受查、`moveit.obstacle_guard_enabled` 可关；滤除在体素中心上做=余量+半对角（0036，防方块切工具致起点碰撞）、ACM 激活即 3 次幂等重试应用（0035/0036)）、绕行累计 12 rad / 单轴 6.1 rad。套入/撤退豁免胶囊（故意进囊）。不走 CIRC/STOMP/OMPL 兜底接触。

套入：shear/bite（非 IMU 档）= MTC 沿轴 LIN；adaptive_shear_v1 = `imu_follow` enable→insert→cut→retract→disable，**禁止与 MTC 同时写 JTC**。

`ContactMonitor` 腕轴电流默认 **关**，不是柜急停。

## SurveyScene / MoveTo

Survey：`goToPhotoPose` + 等新 snapshot。MoveTo：赶路（拍照位/stow），TRANSIT 授权。`CheckReachability`：当前关节为种子，入口→预抓取停位 IK；`require_sleeve` 再检套入终点+沿轴笛卡尔。不动臂。

## 停轨

取消 = 透传 abort + 硬件 `RobotMoveStop`（失败再 `robotMoveFastStop`）。bringup **不起** `aubo_dashboard`。故障后禁止 resume 原轨迹；须 ACK、人确认、重新下发。
