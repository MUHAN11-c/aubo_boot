# peach_arm：一颗 ExecuteTarget

节点图名 `peach_arm`，类 `ManipulationSkillsNode`。动作受理 `cycle.cpp`，阶段序列 `stages.cpp::executeCycle`。MTC 接触 `grasp_task.cpp::tryStagingTransit`（**接近路径真源**）。缓存 `target_cache.cpp`。纯核门 `safety_gate` / `quality_gate` / `trajectory_guard` / `acm_policy`。

`grasp_task.hpp` 文件头与 `peach_arm/README.md` 仍写「斜插 + PTP staging 兜底」——**已过期**，以 `tryStagingTransit` 为准。

## 受理门（onActionGoal）

非 Active / 周期占用 / `contact_recovery_required` → REJECT。`onStart` 拒绝带 `FailureCode::START_NOT_READY`。

mode ∈ {PREVIEW, OBSERVE_ONLY, FULL, PREGRASP_ONLY}。FULL/PREGRASP 须模型七元组 `identityComplete`。目标须命中锁定集锚点，否则回退 selected 缓存。

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
    stageMovePregrasp          PREGRASP 级授权（先回拍照位）
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

TRANSIT / PREGRASP → `authorizeTransit`（使能×Active×robotReady×¬cancel；**不**复检 GraspDecision）。MoveTo 失败用 `TRANSIT_FAILED`，不要挪用 `SLEEVE_PLAN_FAILED`。

CONTACT / TOOL：

1. `motionOutputAllowed`（Active + armed/enabled）
2. `safetyReady`（`robot_status`：抱闸/motion_possible/急停**状态**；mock 可 `require_robot_status=false`）
3. ¬cancel、`grasp_enabled`
4. 令牌优先：`allowed`、target 绑定、`valid_until`、`model_stamp` 新鲜度窗
5. CONTACT：`radial_margin_m>0`；TOOL：`axial_margin_m>0` + `pregrasp_verified` + `tool_enabled`
6. 无令牌：快照 `grasp_allowed` + 能力三态（unrefined 由袋径×D_inner 门兜）

过期 → `StageDenial::EXPIRED`（可重派）；其余 DENIED。套入/SetIO **不得绕过此门**。8090 调试运动另需 yaml `debug.motion_enabled`，仍进同一门。

## 接近主路径（源码：tryStagingTransit，2026-09-23 v4/v4d）

观察停在 look-at，现场常无 IK → **先回拍照位**（有记录的接近则倒放；否则 PTP 0.5 s，失败 OMPL 3.0 s）。

然后唯一主档（无候选扫描）：

1. 滚转梯子 IK（keep-roll 优先，±30°/±60°）落到 **中段点正下方**（树冠外、世界垂直线）
2. Pilz PTP 到该关节目标（弧过果实胶囊 `staging_guard`）
3. 冠内走廊：世界垂直 LIN 入冠（`approach_canopy_entry_m`）+ 沿轴 LIN 进预抓取（`approach_final_axial_m`，对轴 20°）
4. 冠内两段优先 Pilz sequence blend(0.02)；失败降级单任务 MTC LIN

短修正（已在袋底侧、直连不穿囊）走直连 LIN 几何分档，不是兜底链。不走 CIRC/STOMP/OMPL 兜底接触。护栏：①果实胶囊（仅接近段）②反爬 ③场景障碍 ACM ④近果降速 + ContactMonitor（默认关）。

套入：`shear_v1`/`bite_shear_v1` = MTC 沿轴 LIN；`adaptive_shear_v1` = `imu_follow` enable→insert→cut→retract→disable，**禁止与 MTC 同时写 JTC**。

## ③层障碍 ACM（避障只为保护相机）

不用 octomap updater（`sensors_3d.yaml` `sensors: []`）。对象由 `peach_scene_obstacles` 写入。`peach_arm` 激活后 3 次幂等重试应用 ACM：`tool.links`（全机器人−相机）豁免 × `{peach_scene_obstacles}`；唯一受查对=`camera_body_link`×障碍。开关 `moveit.obstacle_guard_enabled`（默认 true；关则相机也豁免，对象仍留场景）。

`ContactMonitor` 腕轴电流默认 **关**，不是柜急停。

## SurveyScene / MoveTo

Survey：`goToPhotoPose` + 等新 snapshot。result `scene_epoch` **恒填 0**（ID-4）。MoveTo：赶路（拍照位/stow），TRANSIT 授权。`CheckReachability`：当前关节为种子，入口→预抓取停位 IK；`require_sleeve` 再检套入终点+沿轴笛卡尔。不动臂。

## 停轨

取消 = 透传 abort + 硬件 `RobotMoveStop`（失败再 `robotMoveFastStop`）。bringup **不起** `aubo_dashboard`。故障后禁止 resume 原轨迹；须 ACK、人确认、重新下发。

关停期例外（决策 0038）：SIGINT 后 `rcl_shutdown` 已发生，`requestCancelAll` 里 `stop()`/`cancel()` 失败只 DEBUG 留痕（try/catch 防御，原 exit -6 根因），**清理路径不得终止进程**。detach 线程关停后竞态（e1 exit_codes 存量红）是挂账 F3，未修。
