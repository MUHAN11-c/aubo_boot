# 输入输出

> 2026-09-30 套袋专项 v10：纠正 v9 厚盒底和撑满侧壁，改为薄封边、内果局部鼓起和下部余纸，独立三袋模型含原尺寸内果/果梗/扎丝；新增正侧面检查。仅套袋资产更新，既有整园/field/五光照仍属 v9 历史结果，不是 v10 验收。命令与证据见 [peach_sim README](../src/peach_sim/README.md)。

> 2026-09-24 果园外观重建：当前 Blender 入口为 `src/peach_sim/reconstruction/`，不复用旧模型/贴图/随机布局。RGB-D 约束局部可见袋面；整树、遮挡部分与树行是标注清楚的推断。旧 Gazebo SDF/GT 仍属旧管线，尚未接入此次新场景；无 ROS IDL、运动或驱动变更。数据口径、离线命令与验收边界见 [peach_sim README](../src/peach_sim/README.md)。根目录 `blender_orchard/` 观感预览链已于 2026-09-28 并入 `src/peach_sim/reconstruction/`（袋面折痕、程序贴图、PeachDataSet 先验统计迁入；数据口径以 priors 分位为准），果园外观建模收敛为单管线；光照/叶遮挡/枝干扰为受控变量，多视角感知验证矩阵与基线回归门见 README。

现行系统（SNAPSHOT）：源码、各包 `config/*.yaml`、[`peach_interfaces/config/interface_manifest.yaml`](../src/peach_interfaces/config/interface_manifest.yaml)。字段级目录：[peach_interfaces/README.md](../src/peach_interfaces/README.md)。清单漂移：`python3 src/peach_interfaces/scripts/check_interface_manifest.py`。与 [architecture.md](architecture.md)、[testing.md](testing.md) 构成仅有的三份活文档；**源码与本文互相更新，改接口/话题/TF 或改本文须同一轮改另一边**。**如何演化**以 [AGENTS.md](../AGENTS.md) 为准：非完美适配当前真机/产品则跟 ROS 2 / 优秀 GitHub 主流（标准 msg、QoS、相对名+remap）。真机轮次：[testing-log.md](testing-log.md)。工程整理过程：[REFACTORING.md](REFACTORING.md)（不驱动现行设计）。

对象是套袋桃。跨包只走 `peach_interfaces`。能力包不互发批次命令；作业目标只认调度 `~/state.target_id`。包职责见 [architecture.md](architecture.md) §3。

下表生产/消费名与清单一致。**名字之外须能看出功能含义**（这一口数据/命令干什么、谁据此做什么）。改接口先改 IDL 再改清单再改本文。运行参数部署值在各包 `config/<节点>.yaml`（nav2 式全量清单）；Python 节点 `attach(node)` 按该 yaml 声明叶子（实现单源 `peach_common.yaml_params`，W1；包内旧路径 shim），`peach_arm` 仍 GPL `arm_parameters.yaml`。下表只列改变作业行为的键。

| 节 | 包 | 节点 |
|----|----|------|
| §2 | `peach_interfaces` | 无（IDL + 清单） |
| §3 | `peach_harvester`（vision） | `peach_scene_perception_node`、`peach_target_reconstruction_node` |
| §3.2 | `peach_vegetation` | `peach_vegetation`（GPU 枝/叶掩膜；不进 harvest_system） |
| §4 | `peach_arm` | `peach_arm` |
| §5 | `peach_harvester`（supervisor） | `peach_harvester`（supervisor）、`peach_lifecycle_manager` |
| §5.3 | `peach_observability` | `peach_observability`（8090 / 会话 bag） |
| §6 | 驱动九包 | 臂 / 相机 / TF（只读红线见 AGENTS） |
| §7 | `serial_imu` | 随 `harvest_system`（`imu_enabled`），不进 lifecycle |
| §8 | 旁路视觉抓取 | `ivg_pose_estimation`、`ivg_graspnet`（IDL=`ivg_interfaces`；数学=scipy） |

各节点流程图在对应小节（入口 → 处理 → 输出）。跨包谁叫谁见 §1。批次时序与技能阶段序列见 [architecture.md](architecture.md) 图 C / 图 D。

---

## 1. 跨包边界

| 包 | 对外提供 | 对外消费 | 不提供 |
|----|----------|----------|--------|
| `peach_interfaces` | IDL + `interface_manifest.yaml` | — | 运行时节点 |
| `peach_harvester`（vision） | `BeginScene`；`/peach/perception/*`；`BuildTargetModel`；`/peach/reconstruction/*` | RGB-D、`HarvestState`、精确 stamp TF | 运动动作、`ledger.json`、选下一颗 |
| `peach_arm` | `SurveyScene`、`ExecuteTarget`、`CheckReachability`、`grasp_hypothesis`、预览/使能/ACK 服务 | 观测、`GraspDecision`、`refined_*` | `RunHarvest`、重建 Trigger 客户端、`ledger.json` |
| `peach_harvester`（supervisor） | `RunHarvest`、`ControlTask`、`HarvestState`/`events`、lifecycle | 观测（选果，仅 `target_observations`）、动作结果 | RGB-D 处理、MoveIt 规划接触、Nav2 规划 |
| `peach_bringup` | 整栈 launch / 预检 | Include 只读 `aubo_e5_bringup` | 不自动 RunHarvest |
| `peach_observability` | 8090 转发、独立 rosbag2 | 调度/技能只读话题 | 不发运动 |
| `peach_vegetation` | `/peach/vegetation/{leaf_mask,branch_mask,overlay,status}` | 彩色图 `image`（remap） | 运动、PlanningScene、octomap、选果 |
| `peach_sim` | 果园世界 `worlds/peach_orchard.sdf`、采摘工位 xacro、目标清单 GT（`peach_orchard.manifest.yaml`：袋底=entry/袋颈/轴/果心/`reachable`） | `config/orchard.yaml`（场景参数唯一事实源） | ROS 业务话题/动作/IDL；仅桥 `/clock` 与 `/joint_states`（gz `gz.msgs.Model`→`sensor_msgs/JointState`），不进 `harvest_system` |

导航预留（`peach_navigation` 已归档 `_archive/parked_2026-09/`）：曾提供 `NavigateToWorksite` 与 `/peach/navigation/target_report` / `arm_status` / `vehicle_state`；现仅在 manifest `reserved_interfaces` 区留名，无生产方。调度是批次侧**唯一**动作客户端。技能不调重建 `reset`/`finalize` Trigger。到位一步无导航动作：`_cmd_navigate` 固定座直通 `NAV_OK`。`harvest_plan` 只做收齐窗口与锁定集，不选下一颗。

```mermaid
flowchart LR
  Op[人工] -->|RunHarvest / ControlTask| Ex[peach_supervisor]
  LCM[peach_lifecycle_manager] -->|managed_nodes_activated| Ex
  Ex -->|SurveyScene| Skill[peach_arm]
  Ex -->|BeginScene| Perc[peach_scene_perception_node]
  Ex -->|BuildTargetModel| Rec[peach_target_reconstruction_node]
  Ex -->|ExecuteTarget OBSERVE / FULL / PREGRASP_ONLY| Skill
  Perc -->|target_observations| Ex
  Perc -->|initial_pose| Rec
  Perc --> Rec
  Perc --> Skill
  Rec -->|refined_* / grasp_decision / diagnostics| Skill
  Ex -->|HarvestState.target_id| Perc
  Ex -->|HarvestState.target_id| Rec
  Skill -->|grasp_hypothesis| Obs[peach_observability]
  Ex --> Obs
  Perc --> Obs
  Rec --> Obs
```

**读图：** 左到右是一次开批谁叫谁。粗箭头是动作/服务（只有调度发出）。细回流是观测和许可话题：调度订 `target_observations` 选果、`grasp_decision`（闩锁许可令牌缓存，非选果输入）；`initial_pose` 只进重建。监控在最右，只收不发。`NavigateToWorksite` 预留（图上无导航节点）；`ExecuteTarget` 干跑走 `PREGRASP_ONLY`，不是图上三种同时发。

| 调用 | 服务端 | 发起方 | 何时 |
|------|--------|--------|------|
| `NavigateToWorksite` | （预留，导航包已归档） | `_cmd_navigate` | `Command.NAVIGATE`；固定座直通 `NAV_OK`，不发送动作 |
| `CheckReachability` | `peach_arm` | `_query_reachability`（SELECT 段） | 批量 TCP IK 预检（请求=感知入口 + 行程；服务端预抓取停位：后撤 + `alignFrameZ` + 滚转扫描；`require_sleeve` 时再检套入终点 IK **和**沿轴笛卡尔，失败码 `sleeve_no_ik` / `sleeve_no_cartesian`；种子=当前关节；不装配 MTC、不动臂）；不可用回退标定半径窗（FULL 另卡套入终点半径） |
| `SurveyScene` | 技能 `~/survey_scene` | `_survey_body` | `Command.SURVEY`；首巡 `NAV_OK` 后，回访 `CYCLE_DONE` / `NO_TARGET` |
| `BeginScene` | 感知 `~/begin_scene` | 调度 `_cmd_begin` | 仅首巡 `SURVEY_AT_POSE` 后一次 |
| 等锁 | （消费观测） | `_wait_lock` | `Command.WAIT_LOCK`；谓词：`scene_epoch` 对齐且 `target_set_locked` |
| `BuildTargetModel` | 重建 `~/build_target_model` | `_cmd_dispatch` | 与 OBSERVE_ONLY **并行** |
| `ExecuteTarget` OBSERVE_ONLY | 技能 `~/execute_target` | `_cmd_dispatch` | 主动视点给重建凑 `min_views` 机位 |
| `ExecuteTarget` FULL | 技能 `~/execute_target` | `_cmd_full` | 观察+模型都过门之后；仅 `execute_pregrasp_only=false` |
| `ExecuteTarget` PREGRASP_ONLY | 技能 `~/execute_target` | `_cmd_full` | 默认 `execute_pregrasp_only=true` 时替代 FULL；停预抓取等 ACK |
| （可选）`skip_reconstruction` | 调度参数 | `_cmd_dispatch` | launch `skip_reconstruction:=true` 时 **不发** Build/OBSERVE，直接 `READY_FULL`；接触几何=锁定集场景观测。brain 用 `peach_supervisor` 键的 ParameterFile overlay（unnamed Node 上嵌套 dict 打不进进程内调度节点）。默认 false |
| `ControlTask` | 调度 `~/control` | 人工（监控只读不发） | PAUSE / SKIP / CANCEL… |
| `ManageLifecycleNodes` | 管理器 `~/manage_nodes` | 人工 | STARTUP/PAUSE/RESUME/RESET/SHUTDOWN；不发 RunHarvest |

---

## 2. `peach_interfaces`

无节点、无 launch、无运行参数。跨包唯一 IDL；清单 56 active + 5 reserved（预留=四个导航 IDL + `peach_sim/joint_states` gz 桥登记），脚本双向核对。每根管子与每个字段的含义写在 [peach_interfaces/README.md](../src/peach_interfaces/README.md)。改字段只改本包，先编本包再编下游；同轮改 README、manifest 与本文。

| 动作 | 服务端所在包 | 含义 |
|------|--------------|------|
| `RunHarvest` | `peach_harvester`（supervisor） | **开一批采摘。** launch 绝不自动发。goal：`request_id`（账本目录名，须唯一；空则 `auto_<时间戳>` 兜底、账本不复用——批次7 ID-1）、`scene_key`、`profile_id`（**批次参数剖面**，现行未消费；与工具档案 `tool_profile_id` 是两个字段，勿混）、`intent`（PICK_ALL / PICK_SELECTED / SURVEY_ONLY）、可选 `target_ids`。结果：至少成功一颗才 `success` |
| `NavigateToWorksite` | （预留，导航包已归档） | **走到作业位。** 固定座调度直通 `NAV_OK`，不发动作、无服务端 |
| `SurveyScene` | `peach_arm` | **去全局拍照位并复核关节已静止。** 给感知准备发现 FOV。PAUSE 会取消，恢复后重试。失败整批 `survey_failed`，不 Begin |
| `BuildTargetModel` | `peach_harvester`（vision） | **绑一颗、收合格机位后 finalize。** 与 OBSERVE_ONLY 并行。积分只用精确 stamp TF。反馈 `view_count` 是机位数 |
| `ExecuteTarget` | `peach_arm` | **对当前 `target_id` 跑一周期。** `PREVIEW` 只规划；`OBSERVE_ONLY` 只补视角；`PREGRASP_ONLY` 停预抓取不 SetIO（默认干跑）；`FULL` 套入/刀/撤退（shear/bite MTC LIN；adaptive_shear_v1 imu_follow 窗：预抓取→insert→回预抓取）。终局 `SUCCEEDED` / `SKIPPED_*` / `FAILED` / `CANCELED`。`harvest.grasped` 仅切断且撤退确认 |

| 服务 | 服务端所在包 | 含义 |
|------|--------------|------|
| `BeginScene` | `peach_harvester`（vision） | **重启收齐窗。** 推进 `scene_epoch`。同 `scene_key` 保留身份；换场才清表。非 Active 拒绝。调度仅 DISCOVERY 首巡 Survey 到位后调用一次 |
| `CheckReachability` | `peach_arm` | **选果：预抓取停位 IK；FULL 时再检套入终点 IK 与沿轴笛卡尔。** 预抓取后撤 `mtc_approach_along_axis_m`，姿态 `alignFrameZ` 不抄感知滚转；keep-roll 无解再 ±30°/±60° 滚转。`require_sleeve`（调度在非 `execute_pregrasp_only` 时置位）时套入终点 = 入口沿袋轴 + clamp(travel)，再从预抓取 IK 解沿轴 interpolate（步长/完成比与接触 LIN 同参）。不装配 MTC、不动臂。失败码 `no_ik` / `sleeve_no_ik` / `sleeve_no_cartesian`。服务不可用时调度回退半径窗（FULL 另卡套入终点半径） |
| `ControlTask` | `peach_harvester`（supervisor） | **人工控批（监控不发）。** PAUSE / RESUME / CANCEL_NOW / SKIP_TARGET / ACKNOWLEDGE_RECOVERY。`expected_state_seq` 须对上，防过期点击 |
| `ManageLifecycleNodes` | `peach_harvester`（supervisor） | **整栈 configure/activate/拆除。** PAUSE=节点 Inactive，不是批次暂停。不发 `RunHarvest` |

| 消息 | 含义 |
|------|------|
| `PeachTargetObservation*` | **场景里有哪些桃。** 稳定 `target_id`、跟踪态、掩膜、单帧几何；数组带 `scene_epoch`。调度据此选果（须世代对齐且已锁定）；重建据此对齐掩膜 |
| `BagGraspCandidate` / `BagFitting` | **单帧袋/果几何与拟合诊断。** `status` ACCEPT/REOBSERVE/REJECT 只当初值与画面，不发运动 |
| `HarvestState` | **批次唯一快照。** `target_id` 是感知/重建作业绑定；`batch_state` / `target_phase` 只由 FSM 推导；`blockers` 自 2026-09-29 起填充（软门机读口径，词表：`stack_not_ready` / `recovery_required` / `mode_paused` / `mode_maintenance` / `ledger_write_failed`，源 `harvest_fsm.blockers_for`） |
| `HarvestSummary` / `TargetOutcome` | 一批结算与单颗入账结果 |
| `CanonicalEvent` | **可检索事件流。** 派发/成功/跳过/失败/暂停/ACK；终局 `message` 带 `failure_code`；severity 四档（2026-09-29 起全码表映射） |
| `SceneSnapshot` | WAIT_LOCK 结束或回访 dwell 后的锁定集快照；`scene_epoch` 须为 Begin 之后 |
| `ReconstructionStatus` | **重建心跳。** 绑定目标、机位数、基线、TF 失败次数。技能看覆盖 |
| `GraspDecision` | **融合几何 + 套入许可。** 身份元组 + 能力三态；`allowed` 只由 geometry∧sleeve∧cut VALID 派生（pregrasp 不进，以免拦 PREGRASP_ONLY；dict 诊断侧同源派生，M11），只拦套入/剪切；`valid_until` 心跳不得续签（窗口 2026-09-20 参数化 `decision.validity_s` 默认 120s——G1：原 5s 与接近链时长错配）；`model_revision` 含单调 finalize 计数（G3，消费方按不透明字符串比较） |
| `PregraspVerification` | 重建侧预抓取残差观测；技能 VerifyPregrasp **未订**本话题，用工具 TF |
| `TargetModel` / `TargetQuality` | Build 结果模型与质量档；含 `run_id` / 修订 / `generated_at` / `valid_until` / 能力三态 |
| `FailureCode` | 失败码枚举（0-23；21-23 为 2026-09-29 观测性轮新增：`TRANSIT_FAILED` / `START_NOT_READY` / `CANCELED`） |
| `ShapeHypothesis` | 契约预留形状假说；重建发，尚未当批次门 |
| `GraspHypothesis` | 技能本周期抓取假说；监控订阅，尚未当批次门 |

事件码：`target_dispatched` / `target_succeeded` / `target_skipped` / `target_failed` / `target_canceled` / `target_operator_skipped` / `photo_pose_reached` / `round_locked` / `survey_failed` / `begin_scene_failed` / `navigate_failed`（后两码 2026-09-29 补齐——此前 FSM `event_code` 为空，批次中断只体现在 `termination_reason`，事件时间线无归因）；单果时限码 `target_timeout`（WARNING 级）。人工操作审计码：`batch_paused` / `batch_resumed`（含 from/to 态）、`recovery_required`（真运动后停驻）、`recovery_acknowledged`（人工 ACK 完成）、`enables_changed` / `batch_policy_updated` / `fire_step` / `ledger_restored`——审计码 details 带 ControlTask `reason`（请求填了才写）；选果过滤码：`targets_filtered`（details 列出超窗目标与原因 `out_of_reach_window` / `out_of_depth_window` / `ik_no_solution`）。终局目标事件的 `message` JSON 并入 outcome 细节（`failure_code` 等），summary「原因」列取之。轨道身份歧义/占用在感知 `identity.py` 用分配代价比值判定（无 IDL；`JobIntent` / `HarvestEvent` / `MatchStatus` 已随 e2f37ff 删除，`intent` 常量以 `RunHarvest.action` 的 `INTENT_*` 为唯一权威）。

导航预留（manifest `reserved_interfaces`，无生产方；调度 NAV 直通）：

| 名字 | 种类 | 含义 |
|------|------|------|
| `/peach_navigation_node/navigate_to_worksite` | action `NavigateToWorksite` | 底盘走到作业位；现行不发送 |
| `/peach/navigation/target_report` | `HarvestTargetReport` | 向导航报当前目标；无节点 |
| `/peach/navigation/vehicle_state` | `VehicleState` | 底盘位姿/速度；无节点 |
| `/peach/navigation/arm_status` | `HarvestOperationStatus` | 臂作业状态给导航；无节点 |

清洁重写轮契约接线状态（设计全文见 REFACTORING.md 重写轮节；**阶段 2b 已接线臂侧**，其余随阶段 3/4 落地）：

| 类型 | 图名 | 服务端 | 状态 | 语义 |
|------|------|--------|------|------|
| `MoveTo.action` | `/peach_arm/move_to` | `peach_arm` | **已接线（2b）** | KIND_NAMED（goToPhotoPose 通用命名位，拍照位带原路返程）/ KIND_POSE（lin_only 只 LIN，失败不回退；非 base 系自动换系）；KIND_JOINTS 预留未实现；取消=abort+RobotMoveStop；与接触周期互斥（running/recovery 期拒） |
| `Enables.msg` | `/peach/batch/enables` | `peach_supervisor`（Active 且 override 非空时 1Hz 心跳重发） | **臂侧已订阅（2b）** | transient_local；臂侧收到过即覆盖本地参数（意图源=大脑），无发布者时本地参数保持唯一权威。**缺心跳=故障（09-18）**：臂侧 `peach_arm.execution.enables_heartbeat_timeout_s`（默认 5.0，<=0 关闭）超时未收到广播即回落本地参数权威并 WARN；广播源在权时本地参数 set 不覆盖使能（双写竞争已修） |
| `Clearance.msg` | （随 ExecuteTarget goal） | `peach_supervisor` 装配 | **已接线（2b；09-18 加绑定）** | 接触许可令牌：goal 填写 model_stamp 即启用 CONTACT/TOOL 级令牌复检——`target_id` 须等于当前目标（装配端不绑定不装，臂侧不匹配拒）、`valid_until`（GraspDecision 原值冻结不续签；窗口=`decision.validity_s` 默认 120s，G1）过期拒、allowed 拒；批次4 单条 allowed 拆档位门：CONTACT 另查令牌径向余量 >0、TOOL 另查轴向余量 >0 + 预备位残差过门 + `tool_enabled`。装配时令牌已过期仍照装（臂侧拒），supervisor 打一次过期 WARN 供现场归因（按 target+revision 去重防 1Hz 刷屏；不拒发不重排，G1 预警）。model_stamp 新鲜窗=effectiveTargetMaxAgeS，不重算几何；未装（绑定不对齐，或 `skip_reconstruction` FULL——不把未融合决策装成令牌）回退 GraspDecision 话题快照复检（skip 路径该快照=臂侧 promoteUnrefined 钉住的 `grasp_allowed`） |
| `ExecuteTarget` 扩展字段 | 现名不变 | `peach_arm` | **已接线（2b）** | `mode` 是接触深度权威（FULL 不被默认 `profile=0` HOLD 盖成停预抓取）；`PROFILE_FULL` 强制套入。`feedback.checkpoint` 随行下发——已记 AT_PREGRASP/SLEEVE_PLANNED/SLEEVED/CUT_ACCEPTED/RETREATED/STOWED；AT_STAGING 待 GraspTask staging 到位回调，CUT_CONFIRMED 已随批次3 接线（`stageVerifyCut` 等到刀具 DI 本命令后新上升沿即记，超时 `CUT_UNCONFIRMED` 保守收口）；RETAINED 待承接 DI（仍预留） |
| `RunHarvest` 扩展字段 | 现名不变 | `peach_supervisor` | 3 | `INTENT_*` 常量（JobIntent 删除后唯一权威）+ 批次策略 `target_harvest_ratio`/`per_target_timeout_s`/`sector_timeout_s`/`view_policy`（VIEW_FAST 默认/VIEW_CONSERVATIVE 原值） |
| `SetEnables.srv` | `/peach_supervisor/set_enables` | `peach_supervisor` | 3 | 操作台使能开关；广播 `/peach/batch/enables` +审计 |
| `SetBatchPolicy.srv` | `/peach_supervisor/set_batch_policy` | `peach_supervisor` | 3 | 运行期改批次策略（当前批或下批默认） |
| `FireStep.srv` | `/peach_supervisor/fire_step` | `peach_supervisor` | 3/4 | 操作台单步（PHOTO/VIEWPOINT/BUILD/APPROACH/PREVIEW/TOOL_DEBUG）；运动类过臂侧命令门 |



---

## 3. `peach_harvester`（vision）

一包两节点。不发运动、不选下一颗、不写 `ledger.json`（可向 `runs/<request_id>/perception_data/` 追加 `HarvestDataStore` 事件）。感知几何优先 stamp TF，失败 latest 并 `tf_stale`；重建积分必须图像时刻精确 TF，禁止 latest。

驱动 RGB-D（§6）：

| 名字 | 含义 |
|------|------|
| `/camera/color/image_raw` | 彩色（前端二选一，话题同构） |
| `/camera/depth/image_raw` | 配准深度（uint16 或 32FC1） |
| `/camera/color/camera_info` | 彩色内参 K |
| `/camera/depth/camera_info` | 深度内参（配准后与彩色同 K；stereo 前端同发） |
| `/camera/depth_registered/points` | 配准彩色点云（percipio 默认开；stereo 同名，点云多一个 `confidence` 字段，见下） |

**相机前端（2026-09-17 起，harvest_system `camera_frontend:=percipio|stereo`；**默认 stereo**，2026-09-29 用户裁定；percipio 显式传参仍可用）**：percipio=设备端 18 图案深度（~2.43 fps，额定量程 0.4–0.8 m；09-21 修复 `parameters.xml` 调参残留下发致深度大面积无效——该 XML 是无条件下发通道，实验值勿残留）；stereo=`peach_stereo` 主机单图案立体（~13.7 fps，hh4 档；话题与 percipio 同构：`color/image_raw`、`depth/image_raw`、`{color,depth}/camera_info`、`depth_registered/points`——stereo 点云比 percipio 多一个 `confidence` FLOAT32 字段（point_step 24=xyz12+pad4+rgb4+confidence4——confidence 在 rgb 槽之后，勿与 rgb@16 重叠；`temporal_k>1` 时=窗内采样占比×取值一致性、与发布深度逐像素对齐，`=1` 时恒 1.0；消费方按字段名读，octomap/RViz/PCL 兼容）；深度口径 uint16×0.25 mm；只发 raw Image + 点云，不用 image_transport；静态 TF 同名链；激光满功率点亮、停栈自动复位；与 percipio 相机连接互斥；SGBM 档 `sgbm.mode`/`sgbm.uniqueness_ratio`/`median_ksize` + 配准后滑窗时域中值 `temporal_k`（1=关/3/5 非法拒启；逐像素有效中值、每帧照常发布不除率；部署 yaml=3，09-21 live A/B：entry std z 0.41→0.21mm、袋半径 std 0.27→0.14mm、覆盖 +0.4pp、13.7gps 无回归），参数依据见该包 README 与 yaml 注释）。规格档案与实测数据见 `src/peach_stereo/README.md`。感知/重建订阅零改动。

感知/重建 ApproximateTime slop **0.05 s**。驱动 QoS 字符串 `default`（RELIABLE）；订户手写 RELIABLE、depth=10。跨包轴向后撤只改 `src/peach_harvester/config/grasp_standoffs.yaml` 两行（`entry_standoff_m` / `pregrasp_standoff_m`），launch 注入各节点已声明参数。

- 深度 uint16：**raw × `depth_scale_unit=0.25` = 毫米**（Percipio）。数据集真毫米设 1.0。32FC1 按米 ×1000，该参数不生效。
- 有效深度：非 0、非 65535；管线窗现行 `pipeline.min_depth_m`/`max_depth_m` **0.3–1.5 m**（`scene_perception.yaml`）。禁止网络补深度。`depth_fallback` 只出现在构造 `hybrid_dilated` 时的深度连通域标签，以及管线 `_foreground` 在「外部掩膜 ∩ 有效深度 <50 像素」时的内部降级；节点主路径 SAM 缺失走 `mask_unavailable`，不把深度带当几何。
- Percipio launch **请求** `frame_rate:=5.0`。归档现场彩色流约 **2.43 fps**，感知帧间隔中位 **0.4 s ≈ 2.5 FPS**。设计节拍用实测，不把 5.0 当已测 Hz。未授权不改 Percipio `frame_rate`。
- 重建 `capture.max_views: 24` 是帧栈上限不是节拍。停稳门打开后有效视角常 4–6。

话题三类语义（最终架构 R3/R4）：**稳定流**（confirmed-only，进 RViz）`markers`、`single_cloud`、`debug_image`；**真相流**（全量含未确认，进记录层与 L2 选果，不进 RViz）`detections`、`masks`、`observations`、`initial_pose`、`debug_image_raw`（与稳定流成对落盘供筛选前后对比）；**模型流**（几何+许可）`grasp_decision`、`refined_*`、`tsdf_cloud`。豁免话题含义见 §3.1。

### 3.1 `peach_scene_perception_node`（看）

消费：RGB-D；`HarvestState`；精确或 latest TF。Lifecycle 非 Active：同步回调直接 return，不积分、不 `BeginScene`。

```mermaid
flowchart TB
  act{"Lifecycle Active?"}
  act -->|否| idle["回调直接 return"]
  act -->|是| src{"入口"}
  src -->|BeginScene| clr["重启收齐窗 scene_epoch++；换场才清身份"]
  src -->|HarvestState| lock["作业 target_id 只作锁定显示 不选下一颗"]
  src -->|RGB-D 同步| pipe["一帧管线"]
  pipe --> out["observations / initial_pose / diagnostics / debug_image"]
  clr --> wait["等下一帧"]
```

**读图：** 节点三入口。`BeginScene` 重启收齐窗；换场才清身份。`HarvestState` 只告诉「当前作业是哪颗」，不在这里选果。真正产出在 RGB-D 回调（下图）。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_scene_perception_node/begin_scene` | service | 重启收齐窗、推进 `scene_epoch`；换场才清身份 | peach_scene_perception | peach_supervisor |
| `/peach/perception/target_observations` | topic | 全量观测（锁定前 `observations[]` 空）：稳定 ID、跟踪态、掩膜、`scene_epoch`。调度选果须世代对齐且已锁定；重建对齐、技能新鲜度 | peach_scene_perception | peach_supervisor, peach_target_reconstruction, peach_arm, peach_observability |
| `/peach/perception/initial_pose` | topic | 单帧袋入口/轴初值。重建当起点；不授权运动 | peach_scene_perception | peach_target_reconstruction |
| `/peach/perception/diagnostics` | topic | 单帧拟合诊断（直径/RMSE/内点） | peach_scene_perception | peach_target_reconstruction |
| `/peach/perception/harvest_state` | topic | 感知侧计划 JSON（锁定集镜像，给监控） | peach_scene_perception | peach_observability |
| `/peach/perception/debug_image` | topic | 检/分割叠加（confirmed-only，进 RViz） | peach_scene_perception | peach_observability |

清单未列、源码仍发（可视化豁免）：

| 名字 | 含义 |
|------|------|
| `/peach/perception/detections` | 本帧检测框（真相流，不进 RViz） |
| `/peach/perception/debug_image_raw` | 筛选前叠加，与 `debug_image` 成对落盘 |
| `/peach/perception/masks` | SAM 像素掩膜 |
| `/peach/perception/single_cloud` | 检测框深度反投影，对 TF/深度，不是重建 |
| `/peach/perception/markers` | 锁定目标 3D：绿 ACCEPT、黄 REOBSERVE、红 REJECT |

```mermaid
flowchart TB
  sync["同步 RGB-D + 彩色 K + TF"] --> yolo["YOLO conf 0.35"]
  yolo --> filt["min_detection_conf 0.40"]
  filt --> dedup["IoS 0.6 去重"]
  dedup --> sam["MobileSAM 最多 16 框"]
  sam --> fg["hybrid_dilated = SAM ∩ 膨胀深度连通域；SAM 缺失 → mask_unavailable"]
  fg --> bag{"袋圆柱 / 果球"}
  bag --> gate["单帧 ACCEPT / REOBSERVE / REJECT"]
  gate --> tfw{"世界系 TF?"}
  tfw -->|unavailable| skip["几何留相机系 不注册"]
  tfw -->|stale| staleid["几何可变到 output_frame；身份不提交 打 tf_stale+target_untracked"]
  tfw -->|ok| id["整帧 χ²门+匈牙利 ID + EMA"]
  id --> obs["/peach/perception/target_observations"]
  gate --> init["/peach/perception/initial_pose"]
```

**读图：** 一帧相机数据从左到右变成「有哪些桃」。检测框 → 分割掩膜 → 袋/果几何 → 单帧门（只给画面，不授权运动）→ **仅精确 stamp TF（ok）** 才登记世界系身份。右边两条话题：观测给调度选果和技能；初值位姿给重建当起点。

- YOLO 异常：整帧跳过，不炸 worker。无框不补假框。
- SAM 只出像素掩膜。截断超 16 框的目标常变 OCCLUDED。前景几何只用 `hybrid_dilated`；SAM 缺失或交后像素 < `min_mask_points` 给 `mask_unavailable` / REOBSERVE，**不**走深度-only 几何回退。`depth_fallback` 只出现在构造 hybrid 时的深度连通域标签。
- 单帧门（2026-09-21 第二轮起）：`status` 只看**门控类** flag（净空/行程/误差预算/有效深度/截断/2D 校验/长度一致性等）；过程类 flag（`taper_*`/`polarity_*`/`axis_from_pca`/`fruit_prior_auxiliary`/`*_from_band`/`gravity_defaulted`/`length_extended_from_2d`）只进 `diagnostic_flags` 供追溯、不再压状态——修正前生产 56/56 恒 REOBSERVE（过程 flag 全帧在场）、refine 的 ACCEPT 偏好分支死码。`valid_depth_ratio` 同轮改**目标级口径** |SAM∩valid|/|SAM|（旧 ROI 均值被窗外背景稀释一个量级，连带压垮 confidence/σ/遮挡分类）；袋长塌缩新增显式 `axis_length_inconsistent` 门（3D 长度 vs 框 2D 展程×深度/焦距粗测）。**袋长 2D+3D 融合**（09-21 第三轮）：点云只覆盖有深度的可见段，袋长以原始 SAM 剪影沿投影轴的展程为准、被检测框限幅（`raw_mask` 经 build_masks 透传），两端只延不缩——修掉活流对拍 len −29% 的系统性偏短；直径不融合（径向 P95 稳健且直接进净空安全门）。
- 身份：整帧一次全局 1-1 分配（`identity.assign_detections`：同类 + 马氏 χ² 门 9≈3σ + 歧义比 1.2）。节点不传 `covariance`，σ=`match_radius/3`×`recovery_scale`。`SpatialEmaMatcher` 欧氏两段匹配**不**走帧路径（只提供半径属性）。`tf_unavailable` 与 `tf_stale` 均不改权威身份。确认 `confirm_frames=5`；贴边帧不攒确认。摆动连续 3 帧残差 >0.03 m → `target_swinging`（不可选）。
- 跟踪 token：OUT_OF_VIEW / LOST / OCCLUDED / DEPTH_VOID / OBSERVED。
- 锁定：`CollectLockPolicy`；`tf_stale` / `tf_unavailable` / `target_untracked` / `target_swinging` / `bbox_edge` 不可选。锁定后新 ID 不入集。`observations[]` 只发锁定 ID；锁定前数组空。

作业参数（部署值 `config/scene_perception.yaml`；`ScenePerceptionParams.attach`）：

| 参数 | 含义 |
|------|------|
| `yolo_conf` / `min_detection_conf` | 检测两级置信度。过低闪框，过高逆光漏检 |
| `yolo_model_path` / `sam_model_path` | 权重路径。yaml 写 `$(find-pkg-share peach_harvester)/model/…`；空串在参数层拒绝启动；节点不回落、不在编排层判空 |
| `confirm_frames` | 累计命中才转正进锁定/候选；短暂闪现不占 ID |
| `target_memory.match_radius_m` | 同类且距离内复用同一 `target_id` |
| `harvest.min_collect_frames` / `lock_settle_frames` / `max_collect_s` | 收齐窗口：最少帧、无新 ID 稳定帧、超时兜底 |
| `pipeline.min_depth_m` / `max_depth_m` | 位姿管线有效深度窗（现行 0.3 / **1.5** m） |
| `pipeline.bag_impl` / `fruit_impl` | 袋/果线映射名（`PIPELINES_BY_IMPL`）；果线另受 `from_params(..., enable_fruit=False)`（改 `True` 才构造） |
| `depth_scale_unit` | uint16：raw × 本值 = 毫米（Percipio 0.25） |
| `sync_slop_s` | RGB-D 近似同步允差（0.05 s） |
| `tool.entry_d_tool` / `entry_d_s` | 入口相对袋底。由 `grasp_standoffs.yaml` 注入，勿只改这里 |
| `tool.D_inner` | 工具内径（径向走廊门）。整栈由 launch `tool_profile` 档案注入覆盖（`aubo_description/config/<profile>.yaml` 单一事实源；三把剪切手 0.030/0.080/0.120，基础值=adaptive_shear_v1 0.120） |
| `tool.L_insert` / `tool.L_blade` | 最大插入深度 / TCP 到剪切平面轴向距离。整栈由档案注入覆盖（三把 0.030/0.030/0.090 与 0.037/0.030/0.079，基础值=adaptive 档案；2026-09-29 扩注入面） |

### 3.2 `peach_target_reconstruction_node`（建）

消费：同一套 RGB-D；感知观测；`HarvestState.target_id`；`BuildTargetModel`。

```mermaid
flowchart TB
  act{"Lifecycle Active?"}
  act -->|否| idle["不积分 不受理 Build"]
  act -->|是| src{"入口"}
  src -->|BuildTargetModel| slot{"已有 Build 在跑?"}
  slot -->|是| rej["拒绝 单槽"]
  slot -->|否| bind["绑定 target_id COLLECTING"]
  src -->|HarvestState 换目标| drop["放弃旧会话 不把半成品续绑"]
  src -->|RGB-D 且已绑定| cap["采帧门"]
  cap -->|过| icp["有界 ICP"]
  icp -->|过| tsdf["LocalTsdf 在线积分"]
  tsdf --> cov{"机位 >= min_views?"}
  cov -->|否| wait["继续收帧"]
  cov -->|是| fin["finalize 提云 + 袋融合"]
  fin --> pub["grasp_decision / refined_* / tsdf_cloud"]
  bind --> wait
```

**读图：** `BuildTargetModel` 一次只绑一颗。RGB-D 过门才积分；机位够了才 finalize 出发许可话题。换目标丢旧会话。finalize 后的几何/许可分岔见下图。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_target_reconstruction_node/build_target_model` | action | 绑 `target_id`，收满机位后 finalize 出模型 | peach_target_reconstruction | peach_supervisor |
| `/peach/reconstruction/diagnostics` | topic | 结构化心跳：绑定态、机位数、基线、TF 失败 | peach_target_reconstruction | peach_arm, peach_observability |
| `/peach/reconstruction/diagnostics_debug` | topic | 调试 JSON 明细（TSDF/ICP/逐机位），不进决策 | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/status` | topic | 短状态 String（MCAP 白名单用这个，不是 diagnostics） | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/grasp_decision` | topic | 融合入口/轴/预抓取/剪切 + `allowed` 汇总位（分档复检语义见 §5 `GraspDecision` 行） | peach_target_reconstruction | peach_arm, peach_supervisor（闩锁令牌缓存）, peach_observability |
| `/peach/reconstruction/pregrasp_verification` | topic | 重建侧残差观测；技能 VerifyPregrasp 用工具 TF，未订本话题 | peach_target_reconstruction | （观测） |
| `/peach/reconstruction/refined_pose` | topic | 融合后袋位姿（精化候选） | peach_target_reconstruction | peach_arm, peach_scene_obstacles, peach_observability |
| `/peach/scene/obstacles_refresh` | topic | Survey 快照建图触发（Empty，transient_local；Survey 成功=拍照位刚拍完，此刻点云帧即障碍快照源，0035） | peach_supervisor | peach_scene_obstacles |
| `/peach/reconstruction/refined_axis` | topic | 融合袋轴，给监控三维 | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/refined_diagnostics` | topic | 精化拟合诊断 | peach_target_reconstruction | peach_arm, peach_observability |
| `/peach/reconstruction/tsdf_cloud` | topic | 绑定目标 TSDF 表面（批次结束会复位变空） | peach_target_reconstruction | peach_observability |
| `/peach/reconstruction/markers` | topic | 相机轨迹与精化示意 | peach_target_reconstruction | （可视化） |
| `/peach/reconstruction/shape_hypothesis` | topic | 形状假说（契约预留，未当批次门） | peach_target_reconstruction | （无消费方） |

采帧门（`target_reconstruction/capture.py`，自动失败=skip）：满栈 → 无帧 → 掩膜 → 同 stamp → 帧龄>2 s → 静止（`/joint_states` 最大 `|vel|`>0.03 rad/s）→ 空 frame_id → **精确 TF**。

```mermaid
flowchart TB
  obs["同戳掩膜 + RGB-D"] --> gate["采帧门"]
  gate -->|过| icp["有界 ICP"]
  icp -->|拒| skip["跳帧 不硬套"]
  icp -->|过| tsdf["LocalTsdf"]
  tsdf --> fin["finalize 提云"]
  fin --> landmarks["宽头=底、窄头=口；轴袋底→袋口且不朝下；对打否决该帧"]
  landmarks --> geom["入口侧向贴体积 + 剪切参考=袋口/框极限"]
  geom --> pre["PREGRASP_ONLY 真机评方向定位"]
  geom --> envelope["TSDF 包络主方向否决"]
  envelope -->|病态跳过12°| budget["动态径向/轴向预算"]
  envelope -->|夹角>12°| veto["keypoint_cloud_axis_conflict"]
  envelope -->|一致| budget
  veto --> gd{"GraspDecision.allowed 只拦套入"}
  budget --> gd
```

**读图：** 重建把多机位收成「这一颗怎么套」。过采帧门才积分；ICP 拒了就丢这一帧，不硬套进模型。下面分岔：左边融合几何给预抓取评方向；右边包络轴只否决、不授权。最下菱形 `allowed` **只决定能不能套入/剪切**，关了仍可去预抓取。

**两层三态不要混：**

| 层 | 字段 | 谁消费 |
|----|------|--------|
| 感知单帧 | `BagGraspCandidate.status` ACCEPT/REOBSERVE/REJECT | 初值、可视化。不发运动 |
| 融合几何 | `GraspDecision` 入口/轴/预抓取/剪切参考（融合成功即填；后撤由 `grasp_standoffs.yaml` 注入，现行入口 0=拟合袋底、预抓取相对入口 0.03 m） | `PREGRASP_ONLY` 到预抓取停住；RViz/监控目视。方向定位对错以真机预抓取实测为准。停袋底对照轮次见 [testing-log.md](testing-log.md) 1757 |
| 接触许可 | `GraspDecision.allowed` | 只授权套入/剪切；禁止降级接触 |

融合成功时 entry/axis/pregrasp/cut_pose 有效，即使 `allowed=false`。无几何时入口/轴填零，只信 `reason` / `failure_code`。常见 reason：`reconstruction_not_ready`、`refined_geometry_unavailable`、`bag_model_unavailable`、`dynamic_budget_negative`、`keypoint_cloud_axis_conflict`（包络轴与关键点轴 >12° 且包络有长径比）、`cut_plane_fruit_clearance` / `cut_band_unavailable`。`envelope_axis_ill_conditioned` / `envelope_too_few_slices` 只诊断，不单独关 `allowed`。>35° 只打 `diagnostic_axis_mismatch`，不单独把 `allowed` 打成 false。通过接触：`dynamic_budget_accept` / `refined_geometry_accept`。软件预算与夹角不代替预抓取位的真机精度评定。

`allowed=true` 之后仍可能 `skipped_quality`（再确认/预抓取残差）或 `skipped_unreachable`（MTC 护栏）。心跳里大量 `not_ready` 不等于精化从未 ACCEPT；看逐目标 max，不要看收工 IDLE。

作业参数（部署值 `config/target_reconstruction.yaml`；`TargetReconstructionParams.attach`）：

| 参数 | 含义 |
|------|------|
| `capture.min_views` | finalize 所需独立机位数（默认 2 = 当前位+一次短移） |
| `capture.max_views` | 帧栈上限，不是节拍 |
| `capture.require_robot_static` | `/joint_states` 最大 \|vel\| 超门则跳帧 |
| `capture.min_neighbor_gap_m` / `neighbor_gap_area_ratio` | 邻锚过近拒帧；小框面积比豁免，近距双检不互锁 |
| `tf_timeout_sec` | 按深度 stamp 精确查 TF；失败跳帧，禁止 latest |
| `refit.entry_standoff_m` / `refit.pregrasp_standoff_m` | 入口相对袋底、预抓取相对入口。只改 `grasp_standoffs.yaml` |
| `tool.budget.d_inner` | GraspDecision 动态预算许可的工具内径。整栈由 launch `tool_profile` 档案注入覆盖（三把剪切手 0.030/0.080/0.120，基础值=adaptive 0.120） |
| `tool.profile_id` | 档案标签（`GraspDecision`/`PregraspVerification`/`TargetModel` 的 `tool_profile_id`）。整栈由 launch `tool_profile` 档案注入 |
| `refitter.cylinder_impl` / `sphere_impl` | 柱/球精化映射名（`REFITTERS_BY_IMPL`）；其余算法直接构造 |

### 3.2 `peach_vegetation`（枝/叶掩膜，影子能力）

独立 Lifecycle 节点，**不进** `harvest_system` / lifecycle 名单。只做二维分割，不写 PlanningScene、不删 octomap 叶、不发运动。输入相对名 `image`（launch remap 到 `/camera/color/image_raw`，禁止话题名参数）；订阅 QoS 为传感档 BEST_EFFORT+VOLATILE（`peach_common.qos.sensor`，W12 起对齐 AGENTS 传感默认；与 RELIABLE 发布端兼容）。首版直接构造 Frangi（torch Hessian，`device:=auto` 有 CUDA 用 `cuda:0`）+ Excess Green/HSV 叶；GPU 推理跑在专用互斥回调组（双线程执行器，长帧推理不饿死 diagnostics）。同卡时不要和场景感知对打。健康走 `/diagnostics`。

```mermaid
flowchart LR
  img["image bgr8"] --> act{"Lifecycle Active?"}
  act -->|否| idle["回调 return"]
  act -->|是| split["Frangi GPU + ExG/HSV"]
  split --> leaf["/peach/vegetation/leaf_mask"]
  split --> wood["/peach/vegetation/branch_mask"]
  split --> ov["/peach/vegetation/overlay"]
  split --> st["/peach/vegetation/status"]
```

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach/vegetation/leaf_mask` | topic | 叶掩膜 mono8（255=叶） | peach_vegetation | （核内无订） |
| `/peach/vegetation/branch_mask` | topic | 木质脊掩膜 mono8（细枝为主，非直径分类） | peach_vegetation | （核内无订） |
| `/peach/vegetation/overlay` | topic | 绿叶红枝叠加 bgr8 | peach_vegetation | （核内无订） |
| `/peach/vegetation/status` | topic | JSON：infer_ms / 覆盖率 / dropped | peach_vegetation | peach_observability（自检新鲜度探针，2026-09-29 起；在图才查） |

作业参数（部署值 `peach_vegetation/config/vegetation.yaml`；`peach_vegetation.attach`）：

| 参数 | 含义 |
|------|------|
| `device` | `auto` / `cpu` / `cuda` / `cuda:0`。显式 CUDA 无卡则 configure 失败 |
| `leaf.exg_min` / `leaf.h_*` / `leaf.s_min` / `leaf.v_min` | 叶：Excess Green 下限 ∪ HSV 绿窗 |
| `branch.sigmas` / `beta` / `gamma` / `percentile` / `dilate_px` / `dark_max` | Frangi 尺度与暗脊门 |
| `branch.exclude_leaf` | 枝掩膜减去叶 |
| `publish_overlay` | 关则零序列化 overlay |

---

## 4. `peach_arm`

单节点。不写 `ledger.json`、不调重建 Trigger、不当 `BeginScene` / `RunHarvest` 客户端。规划 tip 为 URDF `tcp`。作业目标以 **goal.target_id** 为准。

```mermaid
flowchart TB
  act{"Lifecycle Active?"}
  act -->|否| idle["拒运动类入口"]
  act -->|是| src{"入口"}
  src -->|CheckReachability| ik["预抓取 IK；FULL 再套入 IK+沿轴笛卡尔 不动臂"]
  src -->|SurveyScene| photo["goToPhotoPose 原路返程或 PTP；PTP 行程门 6 / 2.5；成功出口 atNamedTarget"]
  src -->|ExecuteTarget| cycle["executeCycle"]
  cycle --> en{"execution_enabled?"}
  en -->|否| preview["PlanPreview 终结"]
  en -->|是| skip{"skip_observation?"}
  skip -->|否 OBSERVE| views["AcquireViews 最多两次短移"]
  skip -->|是| fin["FinalizeAndValidate"]
  views --> fin
  fin --> mode{"mode / 档位"}
  mode -->|OBSERVE_ONLY| repO["ReportObserveOnly"]
  mode -->|grasp_enabled 关| repR["ReportReady"]
  mode -->|接触档| rec["Reconfirm → MovePregrasp → VerifyPregrasp"]
  rec --> pg{"PREGRASP_ONLY?"}
  pg -->|是 默认干跑| hold["HoldPregrasp 停住 不 SetIO"]
  pg -->|FULL| sleeve["PlanSleeve → LIN 套入 → 刀 → 原路撤 → stow"]
  hold --> done["CompleteTarget outcome"]
  sleeve --> done
  repO --> done
  repR --> done
  done --> hyp["grasp_hypothesis / status"]
```

**读图：** 三个对外入口。可达性预检不动臂。拍照是 Survey。一颗桃的接触/观察全在 `executeCycle`（与 [architecture.md](architecture.md) 图 D 同构）：关执行只规划；`OBSERVE_ONLY` 看完就停；干跑默认 `PREGRASP_ONLY` 停预抓取；FULL 才套入打刀。套入/剪切前复检 `GraspDecision.allowed`。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_arm/survey_scene` | action | 去 SRDF 拍照位并复核当前关节；不重启收齐窗 | peach_arm | peach_supervisor |
| `/peach_arm/execute_target` | action | 对一颗桃：观察 / 预抓取 / 套入剪切，按 `mode` 短路 | peach_arm | peach_supervisor |
| `/peach_arm/check_reachability` | service | 批量 TCP IK：预抓取停位；FULL 再套入终点；不动臂 | peach_arm | peach_supervisor |
| `/peach_arm/acknowledge_recovery` | service | 技能确认停驻已看过；调度 ACK 会调它，成功才消耗 `state_seq` | peach_arm | peach_supervisor |
| `/peach/manipulation/grasp_hypothesis` | topic | 本周期抓取假说（监控三维；未当批次门） | peach_arm | peach_observability |
| `/peach_arm/status` | topic | 技能短状态 JSON，作业票「靠近/工具」用 | peach_arm | peach_observability |
| `/diagnostics` | topic | 节点诊断标准面（diagnostic_updater 周期任务；2026-09-29 观测性轮起全覆盖）：`peach_arm`（9 任务：观测流/TF、目标缓存、回调耗时、接触电流、使能、**robot_status 断流、工具链、move_group 在场、最近拒因锁存**）、`peach_scene_perception_node` / `peach_target_reconstruction_node` / `peach_supervisor` / `peach_lifecycle_manager`（watchdog）/ `peach_scene_obstacles`（快照结局）/ `peach_observability`（bag 队列、摄入活度、**selfcheck 裁决**）/ `peach_vegetation` / `peach_stereo`（帧率/丢弃/激光配置）/ `serial_imu`。不进接口清单：核对器只扫 `/peach` 前缀绝对名字面量，`/diagnostics` 属 ROS 标准诊断面（同 `serial_imu` 先例，W5 验证不登记也绿）。**消费端**：`peach_observability` 订阅聚合进 `/api/state` `diagnostics` 分区与 8090 面板（此前全栈无消费方）+ 会话 bag | peach_arm 等 | peach_observability（聚合）+ 通用诊断工具 |

另有 Trigger（已进清单；8090 调试面调用，调度主路径走动作 cancel）：

| 名字 | 含义 |
|------|------|
| `cancel_cycle` | 手动停一周期（调度走动作 cancel，不走此口；调试面在用） |
| `go_to_photo_pose` | 只回拍照位（有接近记录则原路返程，否则 PTP），不采锁定集 |
| `preview_approach_insert` / `preview_full_contact` | 只规划接触预览 |
| `set_execution_armed` | 本周期武装；`execution.enabled` 后每个周期还须调一次 |

订阅：感知观测；重建 `grasp_decision` / `refined_*` / `diagnostics`。柜侧：`RobotStatus` 做安全门；`/aubo_io_controller/joint_status` 电流环形缓存给 ④层接触检测（默认关，只缓存不判定）；工具闭合调 `/aubo_io_controller/set_io`。TF：`tf2::TimePointZero`（规划下一视点，不是积分旧深度）。

`stages.cpp` 的 `executeCycle(ctx)` 显式模式 switch，周期状态全在 `CycleContext`（action 受理时创建、worker 单写者）：PrepareCycle →（`execution_enabled` 关则 PlanPreview 终结）→（未 `skip_observation` 则 AcquireViews）→ FinalizeAndValidate →（OBSERVE_ONLY → Report / `grasp_enabled` 关 → ReportReady / Reconfirm → MovePregrasp → VerifyPregrasp →（PREGRASP_ONLY 则 `HoldPregrasp` 停住 | PlanSleeve → SleeveLinear → ActuateCutter → VerifyCut → ReverseRetreat → ReturnStow → VerifyHarvestOutcome））→ CompleteTarget。FULL 套入/撤退：shear/bite 走 MTC LIN；adaptive_shear_v1 FULL 在预抓取验证后开 imu_follow 窗（enable→insert_start→剪切→insert_retract→disable），回到预抓取才关窗，**先倒放回拍照位再短 PTP `harvest_stow`**；套入/撤退用实测行程判据（批次5）：进度主判据 FK（等待起点 TCP 沿锁定轴向的位移投影），回退订 `/imu_follow/insert_progress`（目标积分），时间只作截止，停滞窗（3 s 实测增益 <1 mm）进度收口 UNKNOWN。shear/bite 两档走 MTC LIN，永不调、不建 `/imu_follow` 客户端。未 enable 时 follow_node pause servo，禁止与 MTC 同时写 JTC。运动/IO 入口逐阶段过 `ExecutionAuthority`（套入/剪切前复检 `GraspDecision.allowed`；撤离 TRANSIT 级不做决策复检）。

- OBSERVE_ONLY：当前位先采帧；基线未过最多两次最近短移（只 LIN，失败换候选），沿当前相机直线截到 `max_camera_step_m`（默认 0.15 m），评分以行程最短为主；朝当前目标检测框内分割更满的方向微偏。禁止 OMPL、对侧兜圈、贴 0.40 m 球面环绕、PTP 兜底。覆盖门 `minimum_baseline_deg: 8`。停准则：覆盖达标或 `maximum_moves` 用尽；不做墙钟预算/移动+等帧 EMA 预测收口（`time_budget_s` 键已随死分支删除，2026-09-20 W5）。到位后等新机位（`view_directions` 增加），同机位连帧不加覆盖。成功：重建已绑定、独立机位已满 `min_views`、TSDF/精化已发布。观察成功但 Build `view_count`（机位数）`< min_views` → `observe_build_view_race`。`captured_views` 仍是积分帧数。
- PREGRASP_ONLY：有融合几何即去预抓取（入口在拟合袋底，预抓取相对入口后撤 0.03 m）；先回拍照位（有记录的接近则原路返程，否则 PTP 0.5 s / 失败 OMPL 3.0 s），再走接近主路径：**PTP 关节空间到中段点正下方（树冠外，滚转梯子 {0,±30°,±60°}+随机重启 IK）+ 世界垂直 LIN 入冠（伸进果树里）+ 沿轴 LIN 对轴进入预抓取**（`staging.*` 已删；不满足即失败收口）（已齐 LIN 挂相对目标 20° 姿态路径约束）。执行路径 MTC `plan(1)`。起点已在袋底侧、直连不穿囊的短修正走直连 LIN（未齐先 LIN 原地对齐工具 Z；keep-roll 自碰则换滚转）。不走 CIRC/STOMP/OMPL。拍照位失败则从当前位规划。不要求 `allowed`。工具 TF 残差超门则按**最新精化快照**重算 entry/pregrasp 做增量修正（最多两次）；残差未过门也停在预抓取（不回 `harvest_stow`），`pregrasp.passed=false` 且 `completion_level=LEVEL_PREGRASP_REACHED`（不得抬到 `LEVEL_PREGRASP_VERIFIED`；`passed` 与 `completion_level` 不得互相抬级）。任何路径不 SetIO。到位终局 `SUCCEEDED` 且 `recovery_required`，ACK 前调度不 Survey。现行不是两帧精确 TF RGB-D 重估。
- FULL：`skip_observation`。shear/bite 沿轴 MTC LIN 套入/原路撤退；adaptive_shear_v1 在预抓取验证后开 imu_follow 窗（`enable`→`insert_start`→剪切→`insert_retract`→`disable`），回到预抓取才关窗，**先倒放回拍照位再短 PTP `harvest_stow`**（stow 与 photo 已分叉，直达会跳过原路返程撞 `transit_max`）。结果填 `HarvestResult` / `Verification` / `PregraspVerification` / `outcome_record`；W7 起顶层 bool 镜像与 `DepositResult` 字段已删（cut/retreat/harvest 证据单源 harvest/verification 块；`DepositResult` 消息保留卸果站预留，不随 Result 携带）。`harvest.grasped` 仅切断证据∧撤退证据。SetIO 超时 → 工具 UNKNOWN，不自动重发、不自动撤退。SetIO ACK 只产生 `CUT_COMMAND_ACCEPTED`；切断确认已接 `/aubo_io_controller/io_states`（批次3：`tool_io_states[pin0]` **本命令后新上升沿**，早已卡高不算），`stageVerifyCut` 有界等沿 2 s（`tool.feedback_timeout_s`）超时 `CUT_UNCONFIRMED` 保守收口——不撤退不重发，终局 `FAILED`/`CUT_FEEDBACK_TIMEOUT` 待人工（详见下「工具三轴状态」行）。
- 接触 ACM：只对指定目标对象 × 指定工具链接 × 套入/剪切阶段放行（场景障碍豁免见下条，0035 起为独立面）。
- 场景障碍 ACM（0035，2026-09-29 起现行）：避障目的=**保护相机**（臂/末端轻微碰撞用户已接受）。`tool.links` 部署值=全机器人连杆−`camera_body_link`，豁免×`{<octomap>, peach_scene_obstacles}`；唯一受查对=相机×障碍。开关 `moveit.obstacle_guard_enabled`（默认 true；false=相机一并豁免恢复可达性，对象保留场景，重开即恢复保护；空闲 `ros2 param set` 生效，运动中拒改，下个规划前重写 ACM）。豁免条目组装纯核=`acm_policy::obstacleExemptionEntries`。**PlanningScene 写域分域**：`peach_scene_obstacles` 节点只写 world.collision_objects（快照对象）、`peach_arm` 只写 ACM diff，双写者互不覆盖。
- ExecuteTarget FULL/PREGRASP_ONLY 要求完整模型身份元组；空版本拒执行（PREVIEW / OBSERVE_ONLY 除外）。`plan_id` 绑定（2026-09-20 修复轮收紧口径）：**只有 PREVIEW 档 goal 记 preview 绑定**，FULL **在自身携带 plan_id 时**与其全字段比对（mismatch → `failure_code=PLAN_MISMATCH(20)`）；绑定存在而 FULL 缺 plan_id 时比对不生效、直接放行（ID-2 缺口，architecture「ID 与身份确定」链 C；绑定写入亦先于预览成败）。OBSERVE_ONLY 只携带 plan_id+身份（观察会动臂、不构成计划预览，模型建好前本就带不了三修订）；FULL/PREGRASP_ONLY 周期终局清绑定。授权拒绝分级：许可过期（valid_until / model_stamp 超窗）→ SKIPPED_QUALITY（可重派），明确不允许 → FAILED。
- 新鲜度门：`SafetyGate` 比较 `clock - freshnessStamp`。OBSERVED 且 `updated_s` 更新时用 `updated_s`，否则末次有效观测 `received_s`。门限 `effectiveTargetMaxAgeS()`：未测得 EMA 用 yaml 3.0 s，测得后只放宽。`assumed_frame_interval_s: 0.4` 只估等待窗口，不预填 EMA。
- 工具三轴状态（批次3，2026-09-23）：`/peach_arm/tool_state`（ToolState.msg，latched，on-change）——刀/保持/载荷三轴各自 UNKNOWN 优先；刀闭合证据=`/aubo_io_controller/io_states` 的 `tool_io_states[pin0]` **本命令后新上升沿**（早已卡高不算，§12.2 新事件门在 ToolActuator 内强制）；闭合≠分离（M0 台架后升级组合判别）。`stageVerifyCut` 有界等沿（`tool.feedback_timeout_s` 2.0s），超时=`CUT_UNCONFIRMED` 保守收口：**不撤退、不重发、不 resume**，FAILED+CUT_FEEDBACK_TIMEOUT 待人工；`stageReleasePayload` 在 harvest_stow 后开刀释放（授权矩阵同 TOOL 级）。SetIO ACK 仍≠任何确认。
- 接触护栏（yaml）：绕腕看累计 12 rad、单轴 6.1 rad（URDF ±3.05 满行程）；段间接缝计入 `|Δq|`。口侧/上方看①②层果实胶囊，**逐段审查**（工具有限圆柱 vs 感知胶囊；反爬 s 不得超过本段起点 max(s,0)+2 cm——v4 两段 LIN 逐跳都查；PTP 首段只查筒体接触，不查反爬；套入/撤退不审）。笛卡尔绕行比 2.6 / 偏离 0.32 m / 回退 0.12 m（09-18 标定；接近与转移段同值，逐段审；0=不查）；TCP 姿态行程绝对 110°（相对起止余量 20°，0=不查）。09-11 mock typical 打开默认门后，从拍照位成功接近绕行比 ≤1.70、姿态 ≤71°；超门拒发见 testing-log。不按时长（`mtc_approach_max_duration_s` 默认 0）。预抓取先回拍照位（有记录的接近则原路返程，否则 PTP 0.5 s / 失败 OMPL 3.0 s），再主路径 PTP+垂直入冠+沿轴到预抓取，再一段沿轴 LIN 套入；反向同轨迹（含入冠/沿轴段）回预抓取。已齐 LIN 加 tip 姿态 OrientationConstraint（对目标姿态，容差 `mtc_approach_max_align_deg` 20°）。主路径未齐先 LIN 原地对齐再平移（对齐跳不挂门）；兜底直连 LIN 同样。直连 LIN 弦长/弧长上限 0.80 m（v4 两段 LIN 同限；PTP 不受此限）。观察短移：只 LIN；行程 `observe_max_*` 4.0 rad / 1.5 rad（09-01 现场把 2.5 会拒的合法短移固化进 yaml）。`goToPhotoPose`：原路返程不过 `transit_max_*`；新规划先 PTP（`photo_ptp_planning_time_s` 0.5 s）失败再 OMPL（`photo_planning_time_s` 3.0 s），行程门 6 rad / 2.5 rad。成功出口核当前关节（`photo_pose_joint_tolerance_rad` / `photo_pose_max_joint_vel_rad_s`，`execute=false` 仍核）。超行程或不在拍照位不报成功。③层场景障碍=Survey 快照对象（0035）：`sensors_3d.yaml` 保持 `sensors: []`（不用 octomap updater，两轮翻车教训见该文件注释）；障碍由 `peach_scene_obstacles` 节点写入（见下节），受查面与开关见上「场景障碍 ACM」。④层近果速度档只套入/撤退 0.05（接近折线走 `velocity_scaling` 0.10）；接触检测订 `joint_status`，默认关。

默认 `execution/grasp/tool=false`：只规划、不接触、不 SetIO。

### peach_scene_obstacles（场景障碍快照，0035）

独立进程（`peach_harvester` 包 console_script，不进 lifecycle 名单、不随 brain 进程）；`harvest_system` 在 `camera_enabled:=true` 时拉起（须位于 aubo include 之前，stereo 分支参数泄漏坑同见 launch 注释）。无状态快照服务：订阅 `/camera/depth_registered/points`（**RELIABLE** VOLATILE depth 5——两前端发布端均 RELIABLE，BE 订户大帧假性丢包（09-29 勘定），缓存最近一帧）、`/peach/reconstruction/refined_pose`（latched，累积已精化目标胶囊）、`/joint_states`+`/robot_description`（自身滤除 FK/mesh）、`/peach/scene/obstacles_refresh`（触发）；经 `/apply_planning_scene` 写 BOX 对象 `peach_scene_obstacles`（首写 ADD、此后同请求 REMOVE+ADD 原子替换）。参数 `config/scene_obstacles.yaml`（0024 直读 attach：体素 0.06、自身余量 0.05、胶囊余量 0.10/0.10、半径 1.5、上限 3000、帧龄门 2s；**滤除在体素中心上做，阈值=余量+体素半对角**——保证方块含角点不切机器人表面，0036）。触发后处理链在工作线程单飞（Open3D 突发不占 executor）；新目标精化后同帧重写；作业期冻结。**只写 world.collision_objects，不碰 ACM**（ACM 归 peach_arm，分域防覆盖；peach_arm 激活即 3 次幂等重试应用 26 条豁免并打 INFO）。纯核 `vision/scene_obstacles/core.py`（零 ROS，pytest 19 例）。

作业参数（部署值 `config/peach_arm.yaml`；默认/校验/描述权威 = GPL `peach_arm/src/arm_parameters.yaml` 单源生成 `include/peach_arm/arm_parameters.hpp`，清洁重写轮 2c 回迁）：

| 参数 | 含义 |
|------|------|
| `execution.enabled` | 开了才真动；每个周期还须 `set_execution_armed`。须与调度 `execution_enabled` 同时开 |
| `grasp.enabled` | 关则停在 READY、不到预抓取。开要求 execution 已开 |
| `tool.enabled` | 关则跳过 SetIO，不得宣称采摘成功 |
| `mtc_approach_along_axis_m` | 预抓取相对入口后撤。由 `grasp_standoffs.yaml` 注入，现行 0.03 m |
| `mtc_approach_max_total_joint_travel_rad` / `max_single_joint_travel_rad` | 接近绕腕护栏 12 / 6.1；超则不执行 |
| `mtc_approach_keepout_radius_m` / `keepout_axial_m` | ①层：axial>0 启用果实胶囊/反爬审查；radius 是感知直径无效时的回退半径（0.12 m），不再当半无限圆柱 |
| `approach_near_velocity_scaling` | ④层近果速度档，默认 0.05；仅套入/撤退 LIN。斜插封顶 0.20/0.10（失败再 0.10/0.04）；沿轴跳已对轴走更高档；staging PTP 用 `velocity_scaling` 0.10 |
| `grasp.fruit_inflation_m` | ①层果实胶囊固定膨胀，默认 0.01 m |
| `grasp.contact_detect.*` | ④层电流特征止损（enabled 默认 false；斜率/尖峰阈值默认 0=不判）；订 `/aubo_io_controller/joint_status` |
| `mtc_approach_max_detour_ratio` / `max_chord_deviation_m` / `max_recede_m` | 接近笛卡尔绕行比三项，默认 2.6 / 0.32 m / 0.12 m（09-18 标定；0=不查） |
| `mtc_approach_transit_max_detour_ratio` / `max_chord_deviation_m` / `max_recede_m` | 转移段（v4 接近主路径 PTP 弧+两 LIN、回拍照位返程）绕行比三项，默认同接近段 2.6 / 0.32 / 0.12 |
| `approach_final_axial_m` / `approach_canopy_entry_m` | v4 接近两段长：末段沿轴（中段点→预抓取）默认 0.05；世界垂直入冠段（PTP 落点在中段点正下方）默认 0.05 |
| `mtc_approach_max_tcp_rotation_deg` | 接近段相对起点 TCP 姿态测地线绝对上限，默认 110°（0=不查）。水平袋 keep-roll≈90°、±60°滚转≈105° |
| `mtc_approach_tcp_rotation_slack_deg` | 相对本段起止测地线的路径余量，默认 20°（0=只看绝对上限） |
| `mtc_approach_cartesian_max_distance_m` | 接触笛卡尔弦长/弧长上限 0.80 m；超过不改无约束 PTP |
| `observe_max_total_joint_travel_rad` / `max_single_joint_travel_rad` | 观察短移护栏 4.0 / 1.5 |
| `transit_max_total_joint_travel_rad` / `max_single_joint_travel_rad` | 回拍照位新规划 PTP/OMPL 护栏 6 / 2.5；原路返程不过此门 |
| `max_camera_step_m` | 下一视点沿当前相机直线截步（默认 0.15 m） |
| `maximum_moves` | 观察移动次数封顶；覆盖达标即停 |
| `quality.minimum_baseline_deg` | 覆盖门 8° |
| `quality.allow_unrefined_geometry` | 默认 false；true 时用场景观测当精化入口并钉住 `grasp_allowed` 与 `bag_diameter_upper_m`（`skip_reconstruction` 打开） |
| `photo_pose_named_target` | Survey / 回拍照位的 SRDF 名（`global_photo_pose`） |
| `photo_pose_joint_tolerance_rad` / `photo_pose_max_joint_vel_rad_s` | `goToPhotoPose` 成功出口每轴 \|Δq\| / \|qdot\| 上限（默认 0.05） |

---

## 5. `peach_harvester`（supervisor）

调度与 lifecycle 管理器同包。8090 实现在 `peach_observability`；参数 yaml 也在 `peach_observability/config/observability.yaml`（W11 起随包走），节点 `ObservabilityParams.attach`。调度是批次唯一所有者；lifecycle 不发 `RunHarvest`；监控只读。

### 5.1 `peach_harvester`（supervisor）（批）

```mermaid
flowchart TB
  rh["RunHarvest"] --> flag{"managed_nodes_activated?"}
  flag -->|否且 require_managed_stack| rej["拒绝开批"]
  flag -->|是| nav["_cmd_navigate 直通 NAV_OK"]
  nav --> survey["SurveyScene 复核关节"]
  survey -->|失败| fail["整批 survey_failed"]
  survey -->|成功| begin["BeginScene 重启收齐窗"]
  begin --> waitLock["WAIT_LOCK 世代对齐且锁定"]
  waitLock --> intent{"intent / execution_enabled?"}
  intent -->|SURVEY_ONLY 或执行关| settle["结算 不选果"]
  intent -->|PICK| sel["SELECT 深度窗 ∩ CheckReachability"]
  sel -->|无候选| empty{"empty_survey_limit?"}
  empty -->|否| revisit["SurveyScene 回访 不 Begin"]
  empty -->|是| settle
  sel -->|有| disp["并行 BuildTargetModel + ExecuteTarget OBSERVE"]
  disp --> full{"execute_pregrasp_only?"}
  full -->|true 默认| pg["ExecuteTarget PREGRASP_ONLY"]
  full -->|false| fl["ExecuteTarget FULL"]
  pg --> recov{"recovery_required?"}
  recov -->|是 未 ACK| hold["不 Survey 不派下一颗"]
  recov -->|ACK| ledger["写 ledger.json"]
  fl --> ledger
  ledger --> revisit
  revisit --> sel
  ctl["ControlTask PAUSE/SKIP/CANCEL/ACK"] -.-> recov
```

**读图：** 调度是唯一动作客户端。开批先看 lifecycle 旗标（软件就绪，不是柜侧上电）；到位无导航动作。**先到拍照位再开收齐窗**；锁定后才选果。默认停预抓取，须 ACK 才再 Survey（回访不 Begin）。状态机细节见 [architecture.md](architecture.md) 图 C。虚线是人工 `ControlTask`，监控不发。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_supervisor/run_harvest` | action | 显式开一批；不自动发 | peach_supervisor | 人工 |
| `/peach_supervisor/control` | service | 暂停/跳过/取消/ACK；须带对的 `state_seq` | peach_supervisor | 人工 |
| `/peach_supervisor/state` | topic | 批次快照：`target_id`、档位、`recovery_required`、permissions | peach_supervisor | peach_scene_perception, peach_target_reconstruction, peach_observability |
| `/peach_supervisor/events` | topic | 可检索事件（拍照到位/锁定/派发/终局/过滤/ACK） | peach_supervisor | peach_observability |
| `/peach_supervisor/scene_snapshot` | topic | WAIT_LOCK 或回访 dwell 后的锁定集快照 | peach_supervisor | （无订阅方；账本/MCAP） |

客户端（仅本节点）：`BeginScene`、`SurveyScene`、`BuildTargetModel`（与 OBSERVE_ONLY 并行）、`ExecuteTarget`、`CheckReachability`。批次6 `reconstruct_in_trajectory`（默认关）：true 时 DISPATCH 起 Build 后不再阻塞等 Build 完成，立即派 FULL（goal 自带 `skip_observation=true`）——重建与接近并行，臂侧 FinalizeAndValidate 有界等精化兜底，周期收口取消并等 Build 结束（单槽约束）。账本：`runs/<request_id>/ledger.json`。

作业参数（部署值 `config/peach_supervisor.yaml`；`peach_supervisor.attach`。`tool.profile_id` 基础值是 adaptive_shear_v1 标签，整栈由 launch `tool_profile` 注入覆盖）：

| 参数 | 含义 |
|------|------|
| `execution_enabled` | false：WAIT_LOCK 后结算，不选果、不派 ExecuteTarget。运行期 `ros2 param set`，不改仓库默认 |
| `execute_pregrasp_only` | true（默认）：接触槽发 PREGRASP_ONLY；false 才 FULL 套入 |
| `skip_reconstruction` | false（默认）：true 时 DISPATCH 不发 Build/补视，用场景观测进接触；FULL 不装重建 GraspDecision 令牌 |
| `require_managed_stack` | 整栈 launch 为 true：未收到 lifecycle 旗标拒绝开批 |
| `survey_wait_s` | WAIT_LOCK 上限（默认 15 s） |
| `survey_dwell_s` | 已锁回访到位后驻留（默认 2 s） |
| `empty_survey_limit` | 连续空扫次数上限，到则结算 |
| `build_start_timeout_s` | Build 须在此时限内进 COLLECTING/READY，否则取消并等结束再派下一颗 |
| `selection_depth_min_m` / `selection_depth_max_m` | 选果相机距离窗（默认 0.30–1.60 m） |
| `selection_reach_min_m` / `selection_reach_max_m` | 仅 IK 服务不可用时的半径回退窗 |
| `check_reachability_service` | TCP IK 预检服务名 |

### 5.2 `peach_lifecycle_manager`（管）

```mermaid
flowchart TB
  boot["节点起来 定时器踢一次 STARTUP"] --> cfg["正向 configure"]
  cfg --> act["正向 activate"]
  act --> flag["managed_nodes_activated"]
  flag --> wait["等人 ManageLifecycleNodes"]
  wait -->|PAUSE| deact["逆向 deactivate → 旗标 false"]
  wait -->|RESUME| react["正向 activate → 旗标"]
  wait -->|RESET| rst["逆向 deactivate+cleanup 再 configure/activate"]
  wait -->|SHUTDOWN| td["逆向 teardown → 旗标 false"]
```

**读图：** 名单感知 → 重建 → 技能 → 调度；observability 不进名单。PAUSE 是节点 Inactive，不是批次暂停。不管柜侧上电，不发 `RunHarvest`。**整栈由 `nav2_lifecycle_manager` 承载**（节点名同为 `peach_lifecycle_manager`，bond_timeout=0，名单硬编码在整栈 launch；下表闩锁在整栈由 `peach_lifecycle_flag_bridge` 发出）；本包内管理器保留独立 launch（`peach_harvester/launch/lifecycle_manager.launch.py`），服务语义不变。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach_lifecycle_manager/manage_nodes` | service | STARTUP/PAUSE/RESUME/RESET/SHUTDOWN 整栈；不发 RunHarvest | peach_lifecycle_manager | 人工 |
| `/peach/lifecycle/managed_nodes_activated` | topic | 名单节点是否都 Active；调度开批闸门 | peach_lifecycle_manager（独立）/ peach_lifecycle_flag_bridge（整栈） | peach_supervisor |

名单默认：场景感知 → 重建 → 技能 → 调度。observability **不进名单**。

| 参数 | 含义 |
|------|------|
| `node_names` | 有序 configure/activate 名单 |
| `startup_timeout_s` | 单次状态转换等待上限 |

### 5.3 `peach_observability`（监）

```mermaid
flowchart LR
  sub["订阅各包状态 / 观测 / GraspDecision / TF / joint_states / joint_status"] --> st["ObservabilityState"]
  st --> http["HTTP GET /api/state /api/trajectory"]
  st --> rec["会话 bag：随节点启停开合 runs/session_*/bag（MCAP 限速进袋，决策 0031）"]
  rec --> rep["栈停自动出 bag_report.md/json + 体积预算回收"]
  st --> viz["/peach/observability/tcp_path + markers"]
  st --> jobpub["/peach/observability/job + metrics（String JSON，随 bag 录制）"]
  dbg["POST /api/debug/action"] --> bridge["调试桥 转发既有动作/服务"]
  bridge --> targets["调度/感知/重建/技能 既有入口"]
```

**读图：** 过程页只收不发。状态汇进作业票和末端主平面轨迹；录制绑定节点启停（决策 0019），不再按批次开合。节点 `main()` 自行 configure/activate。调试 POST（无令牌；动臂须 `debug.motion_enabled`）是**纯转发客户端**——目标全部是各包既有动作/服务，技能 ExecutionAuthority 等门照常生效；每次操作（含被拒）审计落 `runs/debug_audit/<日期>.jsonl`。

| 名字 | 种类 | 含义 | 生产 | 消费 |
|------|------|------|------|------|
| `/peach/observability/tcp_path` | topic | latest TF 末端轨迹 nav_msgs/Path（RViz 默认关，避免与 Marker 双绘） | peach_observability | （RViz） |
| `/peach/observability/markers` | topic | TCP 已采样路径/当前点/预抓取→入口轴，与网页主平面同源；不画未走到的首末弦 | peach_observability | （RViz） |
| `/peach/observability/job` | topic | 作业票 String JSON（指纹变化发布） | peach_observability | 会话 bag（`peach_bag_report` 离线消费） |
| `/peach/observability/metrics` | topic | 性能采样 String JSON（1s） | peach_observability | 会话 bag（同上） |
| `/peach/observability/selfcheck_passed` | topic | **启动自检裁决闩锁**（Bool，TL d1；P0 观测性轮新增）。初次（托管栈激活+静置后）与每 30s 周期复检，或 8090 `POST /api/debug/selfcheck` 手动触发；无 FAIL 即 true。检查项全只读：生命周期/控制器/关节序/TF 对档案/相机帧率/move_group/四动作服务端/robot_status（真机）/IMU/磁盘/权重文件/参数一致性 | peach_observability | peach_bringup（autostart 硬等） |

监控+调试 HTTP（默认 `127.0.0.1:8090`）+ 会话 bag 录制（随栈启停开合；2026-09-24 起进袋限速三族：控制流 20 Hz／PlanningScene 1 Hz／感知派生 2 Hz，决策 0031，只限进袋不影响实时流）。过程页只订不发；调试 POST 无令牌。`debug.enabled` 默认 true（false→503）；运动类另需 `debug.motion_enabled`（默认 false→423）。健康走 `/diagnostics`（W15 双轨 + 2026-09-29 扩充：`session_recorder` 队列/丢帧、`ingest_liveness` 摄入活度、`selfcheck` 裁决；`/diagnostics` 全栈订阅聚合进 `/api/state`）。会话工件（2026-09-29 新增）：`runs/session_*/startup.json`（launch 注入的启动事实：模式/档案/相机/git 提交/域 ID——离线排障回答"当时跑的什么配置"）、`selfcheck.json` + `selfcheck_history.jsonl`（历次自检报告）。

| 参数 | 含义 |
|------|------|
| `host` / `port` | HTTP 监听；默认回环 8090。局域网须显式 `0.0.0.0` |
| `record.enabled` | 会话 bag 录制总开关（默认 true；关则不建录制订阅） |
| `record.level` | 录制档：`std`=通配发现全话题+相机 raw 限 1Hz（默认）；`all`=相机 raw 不限速（限速族仍生效，决策 0031）；`core`=仅固定订阅集（镜像订阅+派生 job/metrics） |
| `record.max_total_bag_gb` | bag 二进制总量预算（GB，默认 100；0=禁用回收）：超限从最旧删 `session_*/bag` 与旧 `mcap_*`，报告/账本/文本永不删，逐条审计。分析完成后 `scripts/purge_analyzed_bags.py` 删 MCAP 留 `bag_report` |
| `record.queue_depth` | bag 写队列深度上限（默认 512）：盘速掉队时丢最旧保最新，丢帧计数进 `/api/state` 的 `record.info.drops` |
| `record.save_images` / `record.save_clouds` | 调试图/TSDF 点云进 bag 的门控（键名沿用，语义已从「落 jpg/ply 文件」改为「进 bag」） |
| `trajectory.enabled` | latest TF 采 TCP 轨迹给 Web/RViz（bag 侧由 `/tf` 离线重算，同源） |
| `joint_states_topic` / `joint_status_topic` | 硬件表：实际角/速度与柜侧电流（SDK 原单位）/温度/跟随误差；镜像只在 Web，原始话题随 bag 录制 |
| `debug.enabled` | 调试 POST 总开关；默认 true |
| `debug.motion_enabled` | 运动类放行；默认 false→423 |
| `debug.audit_enabled` | 审计落盘 `runs/debug_audit/`；默认开（含被拒，含 `enabled=false` 的 503） |
| `debug.endpoints.*` | 调试桥目标（18 个既有动作/服务名，params.py 默认=现行契约名） |

HTTP `/api/state` 区段：`perception` / `reconstruction` / `refined` / `manipulation` / `task_executor`（含 `state_seq`/`scene_epoch` 镜像）/ `robot`（含 `tcp` 摘要与 `joints` 六轴）/ `metrics` / `record` / `params` / **`pipeline`**（全流程阶段时序：调度 FSM 与技能周期各一条服务器侧时间线，每次状态转移记一条、段时长=到下一转移的间隔，页面刷新不丢）/ **`ledger`**（批次账本直播：`runs/<request_id>/ledger.json` 按 mtime 增量重读，per-target 结果/原因/失败码/耗时/阶段耗时与总计；`run_id` 即账本目录名）/ **`job`**（当前果实作业票：过程线状态、档位、`why`、感知入口/重建中心/预抓取/抓取进入点，`base_link` 米）/ **`startup`**（启动事实镜像，2026-09-29 起）/ **`selfcheck`**（最近一次自检报告：逐项 pass/warn/fail/skip + 汇总）/ **`diagnostics`**（`/diagnostics` 聚合：按 hardware_id 归组的任务级状态，2026-09-29 起）/ **`debug`**（`enabled` / `motion_enabled` + 最近操作环形缓冲）。**页面（2026-09-30 web 升级轮）**：事件时间线逐条带 severity 徽标（信息/警告/错误/审计）与事件码中文标签（`begin_scene_failed`→开场景失败等全词表）；`blockers` 红显且中文（`mode_paused`→批次已暂停等五词表）；账本失败码数值映射常量名（0-23，含 21 TRANSIT_FAILED/22 START_NOT_READY/23 CANCELED）；SELFCHECK 面板头部附启动事实摘要行（模式/末端/相机/git 短哈希/域）。`reconstruction.grasp_decision` 镜像含 `valid_until`/`model_revision`/`failure_code`（许可有效期倒计时，心跳不续签）。`GET /api/trajectory` 给俯视页：TCP 点列、起止弦、路标、Marker 字典（绕行比、Δz）。过程页首屏按作业票展示；抓取档关闭时靠近/工具为 gated，不是已完成；其后为阶段时序（FSM/技能两列）、批次账本表（阶段耗时可展开）、感知节拍（fps/检测/分割/几何耗时、掉锚/陈旧锚）与重建进度（机位/拒帧/TF 失败/基线/许可倒计时）、系统负载与参数镜像（折叠）。机械臂硬件表订 `/joint_states`（角/速度）与 `/aubo_io_controller/joint_status`（电流 SDK 原单位/温度/跟随误差），镜像只在 Web、原始话题随 bag 录制。`POST /api/debug/<action>`：`enabled=false→503`、运动类未放行→`423`、未知端点→`404`、未知 mode/intent/command→`400`。无令牌、无 401。运动类 = RunHarvest 全部档位（含 `SURVEY_ONLY`——会 Survey 移到拍照位，09-18 收紧）、Survey、Execute 非 `PREVIEW`（`OBSERVE_ONLY` 算运动）、`go_to_photo_pose`、arm、ControlTask 的 `RESUME`/`EXIT_MAINTENANCE`。页面只暴露本管线按钮：批次 ControlTask 含 PAUSE/RESUME/SKIP_TARGET/ACK 恢复/CANCEL_NOW（PAUSE 与 CANCEL_NOW 带 `expected_state_seq=0` 不做过期拦截，其余带最新镜像 `state_seq`，过期由调度拒）；ExecuteTarget 组含 `set_execution_armed` 武装/解除（属运动类，照常 423 门）；后端端点清单见 `config/observability.yaml` 的 `debug.endpoints.*`。

| 产物 | 路径 |
|------|------|
| 账本 | `runs/<request_id>/ledger.json` |
| 会话 bag | `runs/session_<YYYYMMDD>_<HHMMSS>/bag/`（`bag_0.mcap` + `metadata.yaml`；节点启动开、关闭收尾，非正常退出可 `ros2 bag reindex` 恢复） |
| 会话报告 | `runs/session_*/bag_report.md` + `bag_report.json`（停栈自动生成；写盘原子 tmp+rename，停栈窗内未完成则留「无新报告」而非半份，`ros2 run peach_observability peach_bag_report <bag>` 可复跑；observability 进程 SIGTERM 窗放宽至 60s 对齐报告线程；按 request_id 分批还原 outcome/阶段耗时/验收门对照/TCP 轨迹；回读对 DDS 侧按 msg 命名空间登记的 action 生成类型（如 `msg/RunHarvest_FeedbackMessage`）回退 action 命名空间解析，仍未知类型折叠为仅时间戳，不断整份报告） |
| 回收审计 | `runs/retention_audit.jsonl`（逐条记录删除目录/字节数/预算） |
| 重建 session | 同根；含 `geometry.jsonl`（袋底/颈/轴/剪切点/D95/预算/单帧 flags；复算脚本已归档 `_archive/offline_2026-09/`，写入保留）。三维点按 `list[float]` 写，缺失用 `is None` 回退，不得对 ndarray 用 Python `or`（真值歧义会把已积分体积回滚，RViz TSDF Cloud 变空） |

根：工作区 `runs/`（`peach_harvester.vision.common.runtime.default_runs_root`）。历史 `_archive/runs/`，不要删。录制生命周期绑定节点启停（决策 0019）：`on_configure` 开会话 bag、`on_shutdown`/`destroy_node` 收尾并自动出报告 + 按预算回收；批次边界由消息自带 `request_id` 还原，不再由记录器开合目录。归档里若有 `approach.jsonl`，那是旧技能状态文件名；`runs/run_*`（9 路 jsonl）是 2026-09-15 前的旧格式，历史数据不迁移。

会话 bag 录制话题（固定订阅集，2026-09-20 起 `record.bag_topics` 键已删、档位由 `record.level` 门控）：`events`、`state`、`scene_snapshot`、`target_observations`（含 mask）、`harvest_state`、`recon_status`、`recon_diagnostics`、`recon_debug`、`grasp_decision`、`refined_pose`、`refined_axis`、`refined_diagnostics`、`manipulation_status`、`grasp_hypothesis`、`tf`、`tf_static`、`joint_states`、`robot_status`、`joint_status`、`job`、`metrics`、`debug_image`、`debug_image_raw`（后两者受 `record.save_images`）、`tsdf_cloud`（受 `record.save_clouds`）；`scene_snapshot` 落盘订阅用 transient_local（M13：单发闩锁话题，VOLATILE 会在记录节点晚启时永久丢快照）；别名→话题/类型注册表（报告读取侧）见 `observability/bag_reader.py`。`std`/`all` 档另加通配发现订阅域内其余话题。报告侧离线重算：作业票/许可合并体照 `observability_node` 镜像逻辑重放，TCP 轨迹由 `/tf`+`/tf_static` 树合成（3mm 静止门槛，同在线采样器）。

---

## 6. 驱动层

给采摘核提供手臂、相机、TF。红线包只读。技能不直接写关节命令。

```
base_link → 臂链 → wrist3_Link
  → camera_link（extrinsics_publisher 标定权威）
     → camera_color_frame → camera_color_optical_frame（感知 yaml）
     → camera_depth_frame → camera_depth_optical_frame（深度 header、技能 yaml）
  → tool_axis → cutting_plane / tcp / sleeve_mouth / tool_body_link
     （按 launch tool_profile 选档案，帧名共用：shear_v1 TCP (0, 47.90, 151.07) mm Rx(−90°)（传承旧 hollow 基准）/
      bite_shear_v1 TCP (0, 30.24, 165.5) mm / adaptive_shear_v1 TCP (0, 47, 168.66) mm Rx(−90°)；
      Z=开口，XY=刀口；TCP=刀口/工作口设计基准（adaptive 与旧圆柱同基准、bite 总装测量点；shear_v1 为 CAD 推导）；tool_body_link 挂 wrist3_Link 载 CAD 网格）
```

无 `active.yaml` 时名义 TF：`wrist3_Link→camera_link` 平移 2 cm、单位四元数。现场标定约 `[0.045, 0.108, 0.002]`。驱动两光学系相对 `camera_link` 平移为 0（源码如此；未 live echo 不改名）。

不要统一成一种 TF 查询：

```mermaid
flowchart TB
  stamp["深度 header.stamp"]
  subgraph p["感知"]
    p1["base ← camera_color_optical_frame 超时 0.5s"]
    p2{"stamp 有 TF?"}
    p3["latest 并标 tf_stale"]
    p4["相机系 tf_unavailable 不注册 ID"]
    p1 --> p2
    p2 -->|有| okp["输出 base_link"]
    p2 -->|无| p3
    p3 --> p2b{"latest 有?"}
    p2b -->|有| okp
    p2b -->|无| p4
  end
  subgraph r["重建积分"]
    r1["base ← depth.header.frame_id 超时 1.0s"]
    r2{"stamp 有 TF?"}
    r1 --> r2
    r2 -->|有| tsdf["TSDF"]
    r2 -->|无| drop["跳帧 禁止 latest"]
  end
  subgraph s["技能 / MoveIt"]
    s1["TimePointZero 当前状态"]
    s1 --> plan["视点 / TCP 规划"]
  end
  stamp --> p1
  stamp --> r1
```

**读图：** 同一时刻的深度，三处用法不同。感知：优先图像时刻 TF，没有就用最新并打 `tf_stale`，再没有就不给这颗桃世界系 ID。重建积分：必须图像时刻精确 TF，失败直接丢帧，禁止用最新。技能规划：问「现在臂在哪」，用当前 TF，不拿旧深度去积分。

| 包 | 策略 |
|----|------|
| 重建积分 | 图像时刻精确 TF；失败跳帧，禁止 latest |
| 感知几何 | 优先 stamp，失败 latest 并 `tf_stale`；再失败不注册世界系 ID |
| 技能 | `tf2::TimePointZero`（规划下一视点，不是积分旧深度） |

归档接触轮 `tf_failures=0`。瓶颈不在 TF。

柜侧接口在 `aubo_msgs`，不是采摘业务类型：

| 名字 | 含义 |
|------|------|
| `/aubo_io_controller/robot_status` | 技能安全门：抱闸、`motion_possible`、急停**状态观测**（不是急停通道；急停在示教器/柜，见 AGENTS 第 2 章）；监控 Web 柜侧灯。mock bringup 不起本控制器（发布者计数 0）；`harvest_system` mock 把 `peach_arm` `execution.require_robot_status` 置 false，真机 KEEP true |
| `/aubo_io_controller/joint_status` | 监控 Web：关节电流（SDK 原单位）、温度、跟随误差。技能 ④层接触检测只读 `current[6]`（默认关，只缓存） |
| `/aubo_io_controller/set_io` | 工具闭合；仅 `tool.enabled` 且 FULL 切断阶段 |
| `/joint_states` | 重建静止门、技能规划当前关节；监控 Web 实际角/速度 |
| `/camera/depth_registered/points` | move_group ③层 octomap（`aubo_e5_moveit_config/config/sensors_3d.yaml`，pluginlib `occupancy_map_monitor/PointCloudOctomapUpdater`，分辨率 0.04 m；地图系=规划系 `world`，URDF 固定到 `base_link`）；mock 无点云=空地图。RViz Camera Points / 旁路 GraspNet 同名 |

采摘 IDL 在 `peach_interfaces`。

---

## 7. `serial_imu`（可选）

不在 peach 清单、采摘核不订、不进 lifecycle、不进只读 bringup。`harvest_system` 默认 `imu_enabled:=true`（mock / real 相同）：`use_rviz:=false`、`tf_parent_frame:=tcp`、`align_to_parent:=true`。无 USB 时节点每 2 s 重试串口，不挡整栈。关掉：`imu_enabled:=false`。单独看 IMU 仍可用 `ros2 launch serial_imu serial_imu.launch.py`（不要与整栈同时起）。整栈 RViz 看 Peach → **Imu**（`/imu/data`），不要找 TF 里的静止 `imu_link`。

| 名字 | 含义 |
|------|------|
| `/imu/data` | **修正** IMU（Reliable+Volatile，`frame_id=imu_link`）：`frame_rpy_deg` 默认 Rx(180°) 后静止比力 +Z，再乘可选 `align_to_parent`；默认 `gyro_available: false` → `angular_velocity_covariance[0]=-1` |
| `/imu/data_raw` | **原始** IMU：模组体轴，协议原样（含姿态）；陀螺协方差同 `data` |
| `/imu/mag` | 磁力计（特斯拉；已随修正转到 `imu_link`；未知方差全 0） |
| `/imu/temp` | 温度 |
| `/force/points` | 5 点力（`std_msgs/Float64MultiArray`，Reliable+Volatile）：**牛顿**（REP-103），顺序通道 1/2/3/5/7；2026-09-29 实测 A5 力帧与 A4 IMU 帧共用 `/dev/imu` 同口，本节点 `feed_mux` 分流 |
| `/diagnostics` | `diagnostic_updater`：串口开闭 + `imu/data` 帧率（不进采摘 observability） |
| `imu/align_to_parent` 服务 | `std_srvs/Trigger`：把当前 IMU↔parent（tcp）姿态差当误差清掉；`align_to_parent` 时提供 |

静态 TF `world`（或 `base_link` / `tcp`）→`imu_link`（单位姿态，不把融合四元数写进此帧）。不并进臂链，除非 `tf_parent_frame:=base_link` 或 `tcp`。坐标系修正与对齐在 `frame.py`（零 ROS）：倒装 Rx(180°) 与 parent 误差清零分开；贴歪/无磁 yaw 不要写进 `frame_rpy_deg`。叠 TCP 时 `align_to_parent:=true`（启动自动采，或调 `imu/align_to_parent`）。不做自适应工具偏移。手册：[src/serial_imu/README.md](../src/serial_imu/README.md)。

### `serial_force_node`（咬合末端 5 点力，只看力时的独立用法）

咬合末端盘面 5 点力传感器 IMS-C04A（量程 50g–2kg），采集板 0xA5 定长帧（17B：通道号 1/2/3/5/7 + int16 小端 kgf×100 + 累加和）100Hz 主动上报。**与 IMU 帧共用 `/dev/imu` 同一条串口（2026-09-29 实测）**：默认由 `serial_imu_node` 分流发 `/force/points`（上表）；本节点是"只看力"的独立用法，开同一条口、逐帧打印 kgf（点位标签 `channel_labels` 从 CAD 截图读出，现场按压校对）。**勿与整栈/`serial_imu` 节点并行**（抢同一口）。不进 `harvest_system` / lifecycle / peach 清单。

| 名字 | 含义 |
|------|------|
| `/force/points` | `std_msgs/Float64MultiArray`（Reliable+Volatile）：5 值 **牛顿**（REP-103），顺序同通道 1/2/3/5/7 |
| `/diagnostics` | `diagnostic_updater`：串口开闭 + `/force/points` 帧率（期望 100Hz，窗 0.6×–1.4×） |

---

## 8. 旁路视觉抓取（不进采摘）

不在 peach 清单、采摘核不订、不进 `harvest_system` / lifecycle。三包：`ivg_interfaces`（估姿 1 msg + 5 srv）、`ivg_pose_estimation`、`ivg_graspnet`（旋转数学统一 scipy，`ivg_utils` 已于 2026-09-17 精简轮删除）。与采摘共用 Percipio 图像/点云与 `extrinsics_publisher` TF；检测 launch **不**再起相机或手眼节点。

### `ivg_pose_estimation`

节点名 `ivg_pose_estimation`。Web 另进程，默认 `http://127.0.0.1:8089/`（与手眼标定网关 8088 错开）。**写端点认证**：`/exit`、`/api/save_template_pose`、`/api/standardize_template`、`/api/debug/update_params`、`/api/debug/save_thresholds` 需 `vpe_auth` SameSite=Strict cookie（同源 UI 自动携带）或 `X-Auth-Token` 头（token 见启动日志；`VPE_WEB_TOKEN` 环境变量可固定）；CORS 已收紧为同源。流水线分段（2026-09-29 前沿化轮）：Segmenter（`depth_band` 默认/`rembg_u2net`/`mobile_sam`）→ Matcher（`geometric` 默认/`dinov2_template`）→ pose 求解；配置单源 `config/pose_estimation.yaml`。软触发发 `std_msgs/String` 到 `/camera/soft_trigger`（与 `percipio_camera` 一致）；无订阅者时不阻断。`T_B_C` 优先查 `base_link` ← `camera_color_optical_frame`（按图像 stamp，失败回退 latest+WARN）。Web 上 `/api/get_robot_status`、`/api/set_robot_pose`、`/api/set_robot_io`、`/api/execute_pose_sequence` 与抓取/快换路由一律 **501**（运动走 harvest 8090 调试操作面或 `ivg_graspnet`）。

```mermaid
flowchart LR
  cam["/camera/{color,depth}/image_raw"] --> node["ivg_pose_estimation"]
  node -->|"estimate_pose / estimate_pose_2d"| caller["人工 / Web"]
  node -->|"list_templates / standardize_template / update_params"| caller
  node -->|"system_status"| log["String"]
```

| 名字 | 含义 |
|------|------|
| `estimate_pose`（根命名空间，非私有） | 深度(+可选彩色) → 6D 与抓取/放置笛卡尔位（米）；请求/响应带 `std_msgs/Header`，响应 stamp=图像采集时刻、frame=`base_link`；响应含 `message` 状态/失败原因字段 |
| `~/estimate_pose_2d` | RGB → 像素中心与转角 |
| `~/list_templates` | 列出模板库 |
| `~/standardize_template` | 标准化某工件模板 |
| `~/update_params` | 更新算法参数段 |
| `~/system_status` | 运行日志 String |
| `/camera/color/image_raw`、`/camera/depth/image_raw` | 与采摘同一相机话题（参数可改） |
| `/camera/soft_trigger` | Percipio 软触发（`std_msgs/String`） |

模板根：launch `template_root` → 环境变量 `VPE_TEMPLATE_ROOT` → `web_ui/configs/app_config.json` → `ivg_pose_estimation/templates`。

### `ivg_graspnet`

检测节点 `graspnet_demo_points_node`；执行客户端 `publish_grasps_client`（须外部 `move_group`，规划组 `manipulator_e5`、末端 `tcp`）。**双后端**（`backend` 参数 / 注册表 `backends/`）：`graspnet_torch`（默认；前向+解码）与 `contact_graspnet`（vendored）；碰撞/NMS/topK 在模型无关的 `postprocess.py`；耦合超参随 `models/checkpoint-rs.yaml` manifest。`publish_grasps_client` 有 opt-in 使能门 `require_enable`（默认 false）与 `apply_grasp_z_flip`（对齐后端 `approach_flip_z180`；graspnet_torch=true、contact_graspnet=false 待真机核验）；MoveIt 超时自动 `cancel_goal`。采集默认待命。

```mermaid
flowchart LR
  cloud["/camera/depth_registered/points"] --> det["graspnet_demo_points_node"]
  ctl["/graspnet_capture_control SetBool"] --> det
  det --> mk["grasp_markers"]
  det --> pa["grasp_poses_base"]
  pa --> cli["publish_grasps_client"]
  cli -->|"MoveGroup / CartesianPath"| mg["move_group"]
```

| 名字 | 含义 |
|------|------|
| `/camera/depth_registered/points` | 输入点云（与 RViz Camera Points 同名） |
| `/graspnet_capture_control` | `std_srvs/SetBool`：True 开始一组采集 |
| `grasp_markers` | 相机系 MarkerArray（兜底 frame `camera_depth_optical_frame`） |
| `grasp_poses_base` | `base_link` 系 PoseArray |
| `grasp_pose_i` | 动态 TF（候选抓取） |

`publish_grasps_client` 凑满 `min_groups_before_pick` 组后会走 MoveIt 接近。未授权不得真机运动。手册：[src/ivg_graspnet/README.md](../src/ivg_graspnet/README.md)。

---

## 9. `imu_follow`（可选；仅自适应档案随整栈）

不在 peach 清单、不进 lifecycle。`harvest_system` **仅** `tool_profile:=adaptive_shear_v1` 时 Include `imu_follow_servo.launch.py`（含 moveit_servo）；非 IMU 末端不起。独立 launch 仍可用于人工调试。`motion.enabled` 默认 false 只算不发；**peach 不自动开门**。mock 下发用 `ros2 param set /imu_follow motion.enabled true`。真机换 fjt 后端（透传只有 FJT 动作口）并另行人工授权。

peach `ExecuteTarget` FULL 在自适应档案上于预抓取验证后调用下列服务（非 IMU 末端永不调用、**也不建**这些 Trigger 客户端——非 IMU 栈 `ros2 service list` 不得出现 `/imu_follow/*`）：`/imu_follow/enable` → `/imu_follow/insert_start` →（剪切）→ `/imu_follow/insert_retract` → `/imu_follow/disable`。回到预抓取才关窗，再 MTC 回 `harvest_stow`。未 enable 时 follow_node pause servo，禁止与 MTC 同时写 JTC。follow_node 另发 `~/insert_progress`（`/imu_follow/insert_progress`，Float64，on-change；插入行程进度目标积分）——peach 侧套入/撤退行程判据的回退源（批次5：主判据 FK 沿轴投影，见 §5 周期）。

| 名字 | 含义 |
|------|------|
| `/imu/data` | 输入（订，Reliable+Volatile；serial_imu 修正话题；勿与其他发布器混流，双流会被平滑成中间值） |
| `/joint_states` | 当前关节（订；新鲜度门与 fjt 种子） |
| `~/enable` | `std_srvs/Trigger`：前置全就绪才采参考开始；servo 后端自动 `switch_command_type(TWIST)` + 确保未暂停 |
| `~/disable` | `std_srvs/Trigger`：停跟随（servo 补零速刹车并 pause；fjt 取消在途 goal；disable 后在途回执不补发） |
| `~/insert_start` | `std_srvs/Trigger`：插入推进开始——锁当前工具开口方向（tip +Z，base 系），位置目标沿它按 `insert.speed_m_s`（0.01 m/s）推进、钳 `insert.max_travel_m`（0.20 m）；姿态照常跟 IMU。须先 `~/enable` |
| `~/insert_stop` | `std_srvs/Trigger`：停推进（跟随保持）；disable/断流/达行程上限亦自动停 |
| `~/insert_retract` | `std_srvs/Trigger`：沿锁定开口把行程收回 enable 参考点（跟随保持）；须先 `~/enable` |
| `~/target_pose` | `geometry_msgs/PoseStamped`（base_link）：平滑后 TCP 目标（位置=参考点或插入推进点，姿态跟 IMU） |
| `~/command_twist` | `geometry_msgs/TwistStamped`（tcp 系）：P 控制输出（dry 镜像） |
| `/moveit_servo/delta_twist_cmds` | servo 输入（**BEST_EFFORT** 发布——可靠 QoS 与其订阅不兼容收不到；开门时发） |
| `/moveit_servo/status` | servo 状态（0=No warnings） |
| `/joint_trajectory_controller/joint_trajectory` | servo 100 Hz 输出（JTC 话题流式；mock） |
| `execution.follow_joint_trajectory_action` | fjt 后端动作：mock `/joint_trajectory_controller/...`；真机 `/aubo_passthrough_trajectory_controller/...` |

跟随链：Δ(conj(q_ref)·q_now) → 符号映射 `follow.invert_*` → 死区 → 锥钳 → 平滑 → 目标姿态；servo 后端对当前 TF 求体轴误差按 `servo.orientation_gain` P 控制成角速度（钳 `execution.max_omega_rad_s`；位置小增益 `servo.position_gain` 防漂移，插入推进期间目标点随行程前移）；fjt 后端 `/compute_ik` + 单步钳制流式 FJT。IMU / 关节状态断流、连续 IK 失败自动 disable。参数全量：包内 `config/imu_follow.yaml`（节点，决策 0017 口径手写 `params.py`）与 `config/moveit_servo.yaml`（servo；此版参数名自带 `moveit_servo.` 前缀）。手册：[src/imu_follow/README.md](../src/imu_follow/README.md)。

## 10. IVG 演示栈（2026-09-30 自 aubo_boot 移植；不进采摘）

四新包 `ivg_demo_services` / `latte_backend` / `tool_changer` / `aubo_ros2_web_dashboard` + `ivg_interfaces` 扩展（+10 srv +3 msg：拉花/快换/抓取/监控族）。独立 launch 栈（入口 `scripts/start_ivg_demo.sh`，默认 mock 且默认**独立 DDS 域 96**；`io_simulated:=true` 时 IO 旁路）；FK/IK/上下电走 `aubo_dashboard /aubo/*`，IO 走 `aubo_msgs/SetIO` `/aubo_io_controller/set_io`（板载 DO）。

**IVG/peach 隔离裁定（2026-09-30）**：IVG 演示资产只落 IVG 包——工具 mesh 在 `tool_changer/meshes/`、演示辅助脚本在 `ivg_demo_services/scripts/`；**不向共享包（aubo_description / aubo_e5_moveit_config / peach_*）添加任何 IVG 专有内容**；与 peach 不同域并行（peach 现役 77，演示栈 96），勿同域混跑；URDF 换装改 TF 树期间禁止 peach 预检/整栈同域启动。共享面仅限通用基础设施：bringup/moveit 控制器栈、aubo_dashboard、aubo_msgs。

| 名字 | 类型 | 提供方 → 消费方 |
|------|------|----------------|
| `/latte/run_workflow` | `ivg_interfaces/RunLatteWorkflow` | latte_backend → Web 咖啡拉花面板/CLI |
| `/set_latte_do2` `/set_latte_do4` | `std_srvs/SetBool` | latte_backend latte_io_node → 面板（转 SetIO pin2/4） |
| `/latte_di_status` | `std_msgs/String`（JSON） | latte_io_node → 面板 DI 指示灯 |
| `/run_gripper_swap` `/change_tool` `/get_current_tool` | `ivg_interfaces` 三 srv | tool_changer → Web 视觉抓取面板 |
| `/scene_attach` `/scene_detach` `/set_display_tool` | `ivg_interfaces/ChangeTool` | tool_changer scene_attach_worker → swap worker/面板 |
| `/tool_changer_status` | `ivg_interfaces/ToolChangerStatus` | gripper_swap_worker（5s 周期）→ scene_attach/面板 |
| `/execute_single_grasp` | `ivg_interfaces/ExecuteGraspPose` | ivg_demo_services grasp_trigger → 面板（视觉源 `/grasp_poses_base`） |
| `/loop_grasp_control` `/publish_grasps_worker_loop_control` | `std_srvs/SetBool` | grasp_trigger → 面板循环开关 |
| `/move_to_pose` `/debug/move_to_xyz` | `ivg_interfaces/MoveToPose` | move_service_node → 面板手动运动 |
| `/get_current_state` `/read_robot_io` `/set_robot_enable` `/set_speed_factor` | `ivg_interfaces` 四 srv | move_service_node → 面板/位姿轮询（500ms） |
| `/system/node_status` `/system/log` | `ivg_interfaces/NodeStatus/SystemLog` | system_monitor_node → 面板监控/日志 |
| 端口 **8095** | 网关（uvicorn） | aubo_ros2_web_dashboard（**8090 仍归 peach_observability**） |
| 端口 9090 / 8765 / 8089 | rosbridge / foxglove_bridge / web_video_server | 网关上游（apt `ros-jazzy-rosbridge-suite` 等，env_bootstrap 已列） |
