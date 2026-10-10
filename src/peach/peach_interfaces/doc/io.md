# `peach_interfaces` io（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/io.md` 拆写入包。跨包边界在 [peach_bringup/doc/io.md](../../peach_bringup/doc/io.md)。**改字段只改本包 IDL，先编本包再编下游；同轮改 README、manifest 与本文。**

无节点、无 launch、无运行参数。跨包唯一 IDL；清单 57 active + 5 reserved（预留=四个导航 IDL + `peach_sim/joint_states` gz 桥登记），脚本双向核对（`scripts/check_interface_manifest.py` 已挂 CTest——改 IDL 忘改清单 `colcon test` 会红；可视化话题 8 条、rcl 参数服务后缀、legacy 别名 5 条 `/peach_executor/*` 为脚本内建豁免，旧名 bag 兼容由 `peach_observability/bag_reader.py` 承接）。每根管子与每个字段的含义写在 [peach_interfaces/README.md](../../peach_interfaces/README.md)。改字段只改本包，先编本包再编下游；同轮改 README、manifest 与本文。

| 动作 | 服务端所在包 | 含义 |
|------|--------------|------|
| `RunHarvest` | `peach_harvester`（supervisor） | **开一批采摘。** launch 默不自动发（`autostart:=true` 选配档例外：整栈客户端硬等自检绿后才发）。goal：`request_id`（账本目录名，须唯一；空则 `auto_<时间戳>` 兜底、账本不复用——批次7 ID-1）、`scene_key`、`profile_id`（**批次参数剖面**，现行未消费；与工具档案 `tool_profile_id` 是两个字段，勿混）、`intent`（PICK_ALL / PICK_SELECTED / SURVEY_ONLY）、可选 `target_ids`。结果：至少成功一颗才 `success` |
| `NavigateToWorksite` | （预留，导航包已归档） | **走到作业位。** 固定座调度直通 `NAV_OK`，不发动作、无服务端 |
| `SurveyScene` | `peach_arm` | **去全局拍照位并复核关节已静止。** 给感知准备发现 FOV。PAUSE 会取消，恢复后重试。失败整批 `survey_failed`，不 Begin |
| `BuildTargetModel` | `peach_harvester`（vision） | **绑一颗、收合格机位后 finalize。** 与 OBSERVE_ONLY 并行。积分只用精确 stamp TF。反馈 `view_count` 是机位数 |
| `ExecuteTarget` | `peach_arm` | **对当前 `target_id` 跑一周期。** `PREVIEW` 只规划；`OBSERVE_ONLY` 只补视角；`PREGRASP_ONLY` 停预抓取不 SetIO（默认干跑）；`FULL` 套入/刀/撤退（shear/bite MTC LIN；adaptive_shear_v1 imu_follow 窗：预抓取→insert→回预抓取）。终局 `SUCCEEDED` / `SKIPPED_*` / `FAILED` / `CANCELED`。`harvest.grasped` 仅切断且撤退确认 |

| 服务 | 服务端所在包 | 含义 |
|------|--------------|------|
| `BeginScene` | `peach_harvester`（vision） | **重启收齐窗。** 推进 `scene_epoch`。同 `scene_key` 保留身份；换场才清表。非 Active 拒绝。调度仅 DISCOVERY 首巡 Survey 到位后调用一次 |
| `CheckReachability` | `peach_arm` | **选果：预抓取停位 IK；FULL 时再检套入终点 IK 与沿轴笛卡尔。** 预抓取后撤 `mtc_approach_along_axis_m`，姿态 `alignFrameZ` 不抄感知滚转；keep-roll 无解再 ±30°/±60° 滚转。`require_sleeve`（调度在非 `execute_pregrasp_only` 时置位）时套入终点 = 入口沿袋轴 + clamp(travel)，再从预抓取 IK 解沿轴 interpolate（步长/完成比与接触 LIN 同参）。不装配 MTC、不动臂。失败码 `no_ik` / `sleeve_no_ik` / `sleeve_no_cartesian`。服务不可用时调度回退半径窗（FULL 另卡套入终点半径） |
| `ControlTask` | `peach_harvester`（supervisor） | **人工控批（监控不发）。** PAUSE / RESUME / CANCEL_NOW / SKIP_TARGET / ACKNOWLEDGE_RECOVERY。`expected_state_seq` 须对上，防过期点击。`CANCEL_NOW` 只置 `_cancel`；客户端未发 action cancel 时终局 `abort()`，不得从 EXECUTING 直接 `canceled()` |
| `ManageLifecycleNodes` | `peach_harvester`（supervisor，仅随包内独立 launch） | **整栈 configure/activate/拆除。** PAUSE=节点 Inactive，不是批次暂停。不发 `RunHarvest`。现行整栈由 nav2_lifecycle_manager 承载（同名节点用自带 `nav2_msgs` 服务，非本 IDL；本 IDL 服务端无现行生产方，manifest 登记 producers 空） |

| 消息 | 含义 |
|------|------|
| `PeachTargetObservation*` | **场景里有哪些桃。** 稳定 `target_id`、跟踪态、掩膜、单帧几何；数组带 `scene_epoch`。调度据此选果（须世代对齐且已锁定）；重建据此对齐掩膜 |
| `BagGraspCandidate` / `BagFitting` | **单帧袋/果几何与拟合诊断。** `status` ACCEPT/REOBSERVE/REJECT 只当初值与画面，不发运动 |
| `HarvestState` | **批次唯一快照。** `target_id` 是感知/重建作业绑定；`batch_state` / `target_phase` 只由 FSM 推导；`blockers` 自 2026-09-29 起填充（软门机读口径，词表：`stack_not_ready` / `recovery_required` / `mode_paused` / `mode_maintenance` / `ledger_write_failed`，源 `harvest_fsm.blockers_for`） |
| `HarvestSummary` / `TargetOutcome` | 一批结算与单颗入账结果 |
| `CanonicalEvent` | **可检索事件流。** 派发/成功/跳过/失败/暂停/ACK；终局 `message` 带 `failure_code`；severity 四档（2026-09-29 起全码表映射） |
| `SceneSnapshot` | WAIT_LOCK 结束或回访 dwell 后的锁定集快照；`scene_epoch` 须为 Begin 之后 |
| `ReconstructionStatus` | **重建心跳。** 绑定目标、机位数、基线、TF 失败次数。技能看覆盖 |
| `GraspDecision` | **融合几何 + 套入许可。** 身份元组 + 能力三态；`allowed` 只由 geometry∧sleeve∧cut VALID 派生（pregrasp 不进，以免拦 PREGRASP_ONLY；dict 诊断侧同源派生，M11），只拦套入/剪切——批次4 起为汇总位：臂侧 CONTACT 另查 sleeve/径向余量、TOOL 另查 cut/轴向余量+预备位残差，不再单条放行剪切；`valid_until` 心跳不得续签（窗口 2026-09-20 参数化 `decision.validity_s` 默认 120s——G1：原 5s 与接近链时长错配）；`model_revision` 含单调 finalize 计数（G3，消费方按不透明字符串比较） |
| `PregraspVerification` | 重建侧预抓取残差观测；技能 VerifyPregrasp **未订**本话题，用工具 TF |
| `TargetModel` / `TargetQuality` | Build 结果模型与质量档；含 `run_id` / 修订 / `generated_at` / `valid_until` / 能力三态 |
| `FailureCode` | 失败码枚举（0-23；21-23 为 2026-09-29 观测性轮新增：`TRANSIT_FAILED` / `START_NOT_READY` / `CANCELED`） |
| `ShapeHypothesis` | 契约预留形状假说；重建发，尚未当批次门 |
| `GraspHypothesis` | 技能本周期抓取假说；监控订阅，尚未当批次门 |
| 保留/低频：`BagGrasp2D` / `HarvestResult` / `DepositResult` / `Verification` | 像素键点 / 周期结果（EvidenceTarget）/ 卸果站预留（W7 起不随 ExecuteTarget.Result 携带）/ 周期验证块——字段见 README，勿当已删 |

事件码：`target_dispatched` / `target_succeeded` / `target_skipped` / `target_failed` / `target_canceled` / `target_operator_skipped` / `photo_pose_reached` / `round_locked` / `survey_failed` / `begin_scene_failed` / `navigate_failed`（后两码 2026-09-29 补齐——此前 FSM `event_code` 为空，批次中断只体现在 `termination_reason`，事件时间线无归因）；单果时限码 `target_timeout`（WARNING 级）。人工操作审计码：`batch_paused` / `batch_resumed`（含 from/to 态）、`recovery_required`（真运动后停驻）、`recovery_acknowledged`（人工 ACK 完成）、`enables_changed` / `batch_policy_updated` / `fire_step` / `ledger_restored`——审计码 details 带 ControlTask `reason`（请求填了才写）；选果过滤码：`targets_filtered`（details 列出超窗目标与原因 `out_of_reach_window` / `out_of_depth_window` / `ik_no_solution`）。终局目标事件的 `message` JSON 并入 outcome 细节（`failure_code` 等），summary「原因」列取之。轨道身份歧义/占用在感知 `identity.py` 用分配代价比值判定（无 IDL；`JobIntent` / `HarvestEvent` / `MatchStatus` 已随 e2f37ff 删除，`intent` 常量以 `RunHarvest.action` 的 `INTENT_*` 为唯一权威）。

导航预留（manifest `reserved_interfaces`，无生产方；调度 NAV 直通）：

| 名字 | 种类 | 含义 |
|------|------|------|
| `/peach_navigation_node/navigate_to_worksite` | action `NavigateToWorksite` | 底盘走到作业位；现行不发送 |
| `/peach/navigation/target_report` | `HarvestTargetReport` | 向导航报当前目标；无节点 |
| `/peach/navigation/vehicle_state` | `VehicleState` | 底盘位姿/速度；无节点 |
| `/peach/navigation/arm_status` | `HarvestOperationStatus` | 臂作业状态给导航；无节点 |

清洁重写轮契约接线状态（**阶段 2b/3 已全部落码**：服务端在 `peach_supervisor`，8090 页面尚未转发这些入口——人工经 `ros2 service call` 可用）：

| 类型 | 图名 | 服务端 | 状态 | 语义 |
|------|------|--------|------|------|
| `MoveTo.action` | `/peach_arm/move_to` | `peach_arm` | **已接线（2b）；supervisor 为消费方（2026-10-08 起，fast 档观察/回拍照位）** | KIND_NAMED（goToPhotoPose 通用命名位，拍照位带原路返程）/ KIND_POSE（lin_only 只 LIN，失败不回退；非 base 系自动换系；camera_frame=true 走相机光学位姿 planOrMoveCamera）；KIND_JOINTS 预留未实现；取消=abort+RobotMoveStop；与接触周期互斥（running/recovery 期拒） |
| `Enables.msg` | `/peach/batch/enables` | `peach_supervisor`（Active 且 override 非空时 1Hz 心跳重发；lifecycle publisher） | **已接线（2b 臂侧订阅 + supervisor 发布/心跳）** | transient_local；臂侧收到过即覆盖本地参数（意图源=大脑），无发布者时本地参数保持唯一权威。**缺心跳=故障（09-18）**：臂侧 `peach_arm.execution.enables_heartbeat_timeout_s`（默认 5.0，<=0 关闭）超时未收到广播即回落本地参数权威并 WARN；广播源在权时本地参数 set 不覆盖使能（双写竞争已修） |
| `Clearance.msg` | （随 ExecuteTarget goal） | `peach_supervisor` 装配 | **已接线（2b；09-18 加绑定）** | 接触许可令牌：goal 填写 model_stamp 即启用 CONTACT/TOOL 级令牌复检——`target_id` 须等于当前目标（装配端不绑定不装，臂侧不匹配拒）、`valid_until`（GraspDecision 原值冻结不续签；窗口=`decision.validity_s` 默认 120s，G1）过期拒、allowed 拒；批次4 单条 allowed 拆档位门：CONTACT 另查令牌径向余量 >0、TOOL 另查轴向余量 >0 + 预备位残差过门 + `tool_enabled`。装配时令牌已过期仍照装（臂侧拒），supervisor 打一次过期 WARN 供现场归因（按 target+revision 去重防 1Hz 刷屏；不拒发不重排，G1 预警）。model_stamp 新鲜窗=effectiveTargetMaxAgeS，不重算几何；未装（绑定不对齐，或 `skip_reconstruction` FULL——不把未融合决策装成令牌）回退 GraspDecision 话题快照复检（skip 路径该快照=臂侧 promoteUnrefined 钉住的 `grasp_allowed`） |
| `ExecuteTarget` 扩展字段 | 现名不变 | `peach_arm` | **已接线（2b）** | `mode` 是接触深度权威（FULL 不被默认 `profile=0` HOLD 盖成停预抓取）；`PROFILE_FULL` 强制套入。`feedback.checkpoint` 随行下发——已记 AT_PREGRASP/SLEEVE_PLANNED/SLEEVED/CUT_ACCEPTED/RETREATED/STOWED；AT_STAGING 待 GraspTask staging 到位回调，CUT_CONFIRMED 已随批次3 接线（`stageVerifyCut` 等到刀具 DI 本命令后新上升沿即记，超时 `CUT_UNCONFIRMED` 保守收口）；RETAINED 待承接 DI（仍预留） |
| `RunHarvest` 扩展字段 | 现名不变 | `peach_supervisor` | **已接线（3）** | `INTENT_*` 常量（JobIntent 删除后唯一权威）+ 批次策略 `target_harvest_ratio`/`per_target_timeout_s`/`sector_timeout_s`/`view_policy`（VIEW_FAST 默认/VIEW_CONSERVATIVE 原值）——`cycle_core/batch_policy.py` 解析、单果时限与 `target_timeout` 事件已投产 |
| `SetEnables.srv` | `/peach_supervisor/set_enables` | `peach_supervisor` | **已接线（3）** | 操作台使能开关；广播 `/peach/batch/enables` +审计 |
| `SetBatchPolicy.srv` | `/peach_supervisor/set_batch_policy` | `peach_supervisor` | **已接线（3）** | 运行期改批次策略（当前批或下批默认） |
| `FireStep.srv` | `/peach_supervisor/fire_step` | `peach_supervisor` | **仅 PHOTO 已接线**（MoveTo KIND_NAMED 到 `global_photo_pose`）；VIEWPOINT/BUILD/APPROACH/PREVIEW/TOOL_DEBUG 占位，调用即拒（response 提示走 8090 既有单步）；8090 页面转发未接——人工 `ros2 service call` | 操作台单步；PHOTO 走 MoveTo（TRANSIT 级授权），未来运动类档位须过臂侧命令门 |
