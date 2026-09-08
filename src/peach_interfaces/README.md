# peach_interfaces

采摘四包之间**唯一允许互发的类型**。无节点、无 launch、**无 ROS 运行参数**。感知 / 重建 / 技能 / 调度 / 监控只依赖本包消息，禁止互相 import 业务结构体。

| 你要查的 | 在哪 |
|----------|------|
| 管子叫什么、谁发谁订、QoS | 下面「接线」+ [`config/interface_manifest.yaml`](config/interface_manifest.yaml) |
| 管子里每个字段什么意思 | `msg/` `srv/` `action/` 文件头（话题/谁发谁订）+ 字段行内注释；下表是同一套说明 |
| 算法阈值、后撤距离、选果窗 | **不在本包**。见文末「运行参数在哪」 |
| 一批谁先叫谁 | [docs/io.md](../../docs/io.md) §1；架构 [docs/architecture.md](../../docs/architecture.md) §3 |

改字段：先改本包 IDL → 本 README 与 manifest → [docs/io.md](../../docs/io.md) → 各包 pub/sub。先编本包再编下游。漂移检查：

```bash
python3 src/peach_interfaces/scripts/check_interface_manifest.py
```

---

## 本包没有运行参数

没有 `*_parameters.yaml`，节点也不会 `declare_parameter`。清单里的 `qos:` 是**跨包必须对齐的传输约定**，不是可调阈值。

真正的运行参数在能力包 GPL yaml（声明/默认/校验）和同名部署覆盖 yaml：

| 文件 | 管什么 |
|------|--------|
| `peach_perception/config/scene_perception_parameters.yaml` | 检测、锁定窗、深度窗、袋/果管线 |
| `peach_perception/config/target_reconstruction_parameters.yaml` | 采帧门、TSDF、机位数 |
| `peach_perception/config/grasp_standoffs.yaml` | 入口相对袋底、预抓取后撤（launch 注入各节点已声明参数） |
| `peach_manipulation/config/manipulation_parameters.yaml` | 视点、MTC、接触、刀具 IO |
| `peach_executor/config/executor_parameters.yaml` | 批次、选果、`execute_pregrasp_only` |
| `peach_executor/config/lifecycle_manager_parameters.yaml` | lifecycle 名单与顺序 |
| `peach_executor/config/observability_parameters.yaml` | 监控 Web；`debug.*` 默认关 |

---

## 接线（图上的名字）

每一行 = 一根管子。`consumers: []` = 有人发、现行核内无人订（预留、只给 RViz 手订、或仅动作 Result 携带同类型）。

QoS 缩写：`R` reliable；`TL` transient_local（晚订户仍拿得到最后一帧）；`V` volatile；数字是 depth。未写 qos 的动作/服务走 ROS 默认。

### 开批 / 控批（调度）

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_executor/run_harvest` | action | `RunHarvest` | **唯一开批开关。** launch 绝不自动发。至少成功一颗才 `success` | 调度 | （人工/客户端） | — |
| `/peach_executor/control` | service | `ControlTask` | 暂停/恢复/取消/跳过/ACK 恢复。`expected_state_seq` 须对上当前快照 | 调度 | （人工；监控默认不发） | — |
| `/peach_executor/state` | topic | `HarvestState` | **批次唯一快照。** 能力包只认其中的 `target_id` 当当前作业 | 调度 | 感知、重建、监控；调度自订 | R/TL/1 |
| `/peach_executor/events` | topic | `CanonicalEvent` | 可检索事件流（派发/成功/失败/暂停/ACK/过滤） | 调度 | 监控 | R/TL/50 |
| `/peach_executor/scene_snapshot` | topic | `SceneSnapshot` | WAIT_LOCK 结束或回访后的锁定集快照 | 调度 | （核内无订） | R/TL/1 |

### 生命周期

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_lifecycle_manager/manage_nodes` | service | `ManageLifecycleNodes` | 整栈 configure/activate/拆除。这里的 PAUSE 是节点 Inactive，不是批次暂停；不发 `RunHarvest` | lifecycle | （人工） | — |
| `/peach/lifecycle/managed_nodes_activated` | topic | `std_msgs/Bool` | 名单都 Active 后才允许接开批 | lifecycle | 调度 | R/TL/1 |

### 看场景

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_scene_perception_node/begin_scene` | service | `BeginScene` | 重启收齐窗，`scene_epoch+1`。同场保留身份；换 `scene_key` 才清表。仅首巡 Survey 到位后调一次 | 感知 | 调度 | — |
| `/peach/perception/target_observations` | topic | `PeachTargetObservationArray` | **园子里有哪些桃。** 调度**只订这一根**选果（须 `scene_epoch` 对齐且已锁定） | 感知 | 调度、重建、技能、监控 | R/V/10 |
| `/peach/perception/initial_pose` | topic | `BagGraspCandidateArray` | 单帧袋入口/轴**初值**。只给重建当起点；`ACCEPT` 也不授权运动 | 感知 | **仅重建** | R/TL/1 |
| `/peach/perception/diagnostics` | topic | `BagFittingArray` | 单帧拟合诊断（直径/RMSE/内点） | 感知 | 重建 | — |
| `/peach/perception/harvest_state` | topic | `std_msgs/String` | 感知侧锁定集 JSON，给监控看 | 感知 | 监控 | R/TL/1 |
| `/peach/perception/debug_image` | topic | `sensor_msgs/Image` | 检/分割叠加（confirmed-only） | 感知 | 监控 | R/V/10 |

### 建当前目标

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_target_reconstruction_node/build_target_model` | action | `BuildTargetModel` | 绑 `target_id`，收够机位后 finalize。与 OBSERVE_ONLY **并行**。只用图像时刻精确 TF | 重建 | 调度 | — |
| `/peach/reconstruction/grasp_decision` | topic | `GraspDecision` | **融合入口/轴/预抓取/剪切 + `allowed`。** `allowed` 只拦套入和刀，不拦预抓取 | 重建 | 技能、监控 | R/TL/1 |
| `/peach/reconstruction/refined_pose` | topic | `BagGraspCandidateArray` | 融合后袋位姿 | 重建 | 技能、监控 | R/TL/1 |
| `/peach/reconstruction/refined_axis` | topic | `Vector3Stamped` | 融合袋轴，给监控三维 | 重建 | 监控 | R/TL/1 |
| `/peach/reconstruction/refined_diagnostics` | topic | `BagFittingArray` | 精化拟合诊断 | 重建 | 技能、监控 | R/TL/1 |
| `/peach/reconstruction/diagnostics` | topic | `ReconstructionStatus` | 绑定态、机位数、基线、TF 失败。技能看覆盖 | 重建 | 技能、监控 | R/TL/1 |
| `/peach/reconstruction/diagnostics_debug` | topic | `std_msgs/String` | TSDF/ICP 明细 JSON，不进决策 | 重建 | 监控 | R/TL/1 |
| `/peach/reconstruction/status` | topic | `std_msgs/String` | 短状态；MCAP 白名单用这个 | 重建 | 监控 | R/TL/1 |
| `/peach/reconstruction/pregrasp_verification` | topic | `PregraspVerification` | 重建算的预抓取残差。技能**不订**，到位后用工具 TF 自验；同类型出现在 `ExecuteTarget.Result` | 重建 | （核内无订） | R/TL/1 |
| `/peach/reconstruction/tsdf_cloud` | topic | `PointCloud2` | 绑定目标 TSDF 表面（批次结束复位变空） | 重建 | 监控 | R/TL/1 |
| `/peach/reconstruction/markers` | topic | `MarkerArray` | 相机轨迹与精化示意 | 重建 | （可视化手订） | R/TL/1 |
| `/peach/reconstruction/shape_hypothesis` | topic | `ShapeHypothesis` | 形状假说。已发，**尚未当批次门** | 重建 | （核内无订） | R/TL/1 |

### 动手（技能）

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_manipulation_node/survey_scene` | action | `SurveyScene` | 去全局拍照位并确认关节已静止。失败整批 `survey_failed`，不 Begin | 技能 | 调度 | — |
| `/peach_manipulation_node/execute_target` | action | `ExecuteTarget` | 对当前 `target_id` 跑一周期：预览 / 补视角 / 停预抓取 / 套入 | 技能 | 调度 | — |
| `/peach_manipulation_node/check_reachability` | service | `CheckReachability` | 选果：入口换成与 Hold 同一停位后，当前关节有没有 IK。不规划、不动臂 | 技能 | 调度 | — |
| `/peach_manipulation_node/acknowledge_recovery` | service | `std_srvs/Trigger` | 接触或预抓取停住后，调度转发人工 ACK | 技能 | 调度 | — |
| `/peach/manipulation/grasp_hypothesis` | topic | `GraspHypothesis` | 本周期技能打算怎么抓。**只有监控订**，FSM 不靠它做决定 | 技能 | 监控 | R/TL/1 |
| `/peach_manipulation_node/status` | topic | `std_msgs/String` | 技能短状态 | 技能 | 监控 | R/TL/1 |

### 监控自产

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach/observability/tcp_path` | topic | `nav_msgs/Path` | TCP 轨迹 | 监控 | （核内无订） | R/TL/1 |
| `/peach/observability/markers` | topic | `MarkerArray` | 监控叠加 | 监控 | （核内无订） | R/TL/1 |

清单故意不收的可视化话题（源码仍发，RViz 用手订）：`/peach/perception/{axis,debug_image_raw,detections,markers,masks,single_cloud}`、`/peach/reconstruction/local_cloud`、`/peach_manipulation_node/planned_views`。

导航四条在文末「预留」。

---

## 动作字段

### `RunHarvest` — 开一批

Goal：

| 字段 | 含义 |
|------|------|
| `request_id` | 本批 ID，同时作账本目录名，须唯一 |
| `scene_key` | 场景键，传给 `BeginScene` / `SurveyScene`；换键=换场清身份 |
| `profile_id` | 参数剖面（现行调度未消费） |
| `intent` | 见 `JobIntent`：`PICK_ALL=0` 锁定集逐颗；`PICK_SELECTED=1` 只采 `target_ids`；`SURVEY_ONLY=2` 只扫不派 |
| `selection_mode` | 预留 `AUTO/MANUAL`；现行：`target_ids` 非空即优先按列表 |
| `target_ids` | 可选允许列表；空=不限定 |

Result：`success` = 至少成功一颗；`termination_reason`（`completed` / `canceled` / `survey_failed` / `begin_scene_failed` / `no_targets_succeeded` 等）；`summary` 见 `HarvestSummary`。

Feedback：与 `~/state` 同源的 `HarvestState`。

### `SurveyScene` — 去拍照位

Goal：`request_id`、`scene_key`（须与 Begin 相同）。

Result：`snapshot_id`、`scene_epoch`、`degraded`（空集/超时/未锁）、`message`。

Feedback：`observation_count`、`status`（`moving` / `capturing` / …）。

PAUSE 会取消本动作，恢复后重试。成功还须当前关节过命名拍照位。

### `BuildTargetModel` — 绑一颗并出模型

Goal：`request_id`、`target_id`（须等于 `HarvestState.target_id`，空则失败）、`scene_epoch`（须等于当前 Begin 世代）。

Result：`success`、`quality_level`（`TargetQuality.LOW..HIGH_CONFIDENCE`）、`message`、`model`（`TargetModel`）。

Feedback：`view_count` = 已采**机位数**（不是原始帧数）；`status` 重建短名（`COLLECTING` / `READY` / …）。

### `ExecuteTarget` — 对当前目标跑一周期

Goal：

| 字段 | 含义 |
|------|------|
| `request_id` / `run_id` / `cycle_id` / `target_id` | 批次与单颗身份 |
| `mode` | `PREVIEW=0` 只规划；`OBSERVE_ONLY=1` 只补视角；`FULL=2` 套入/刀/撤退；`PREGRASP_ONLY=3` 停预抓取、不 SetIO、不回 stow（现行默认干跑） |
| `skip_observation` | FULL 时可跳过观察段 |
| `scene_epoch` | 须与当前场世代一致 |
| `model_revision` / `tool_profile_id` | 模型与工具剖面 |

Result 终局 `outcome`：`SUCCEEDED=0` / `SKIPPED_QUALITY=1` / `SKIPPED_UNREACHABLE=2` / `FAILED=3` / `CANCELED=4`。

`completion_level` 从 `LEVEL_NONE` 到 `LEVEL_HARVEST_CONFIRMED`（预抓取已验 → 套入 → 刀指令受理 → 切断确认 → 撤退确认 → 采摘确认）。产品成功看 `harvest.grasped`：仅 `cut_confirmed && retreat_confirmed`。`recovery_required` 为真时须人工 ACK 才允许下一颗。

其余 Result 块：`harvest` / `deposit` / `verification` / `outcome_record` / `pregrasp`（技能自填的残差，不是订重建话题）。

Feedback：`HarvestState`。

### `NavigateToWorksite` — 预留

Goal：`pose`、`site_id`。现行固定座调度直通 `NAV_OK`，**不发送**本动作，无服务端。

---

## 服务字段

### `BeginScene`

请求：`request_id`、`scene_key`。响应：`accepted`（非 Active / 忙则 false）、`scene_epoch`（接受后从 1 递增）、`message`。

### `CheckReachability`

请求：`header`；`tcp_poses[]` = 感知入口（位置=袋底，姿态 Z=袋轴，滚转任意）；`timeout_s`（≤0 用服务端 0.1 s）。

服务端换成与 Hold 同一停位再 IK：沿 −Z 后撤 `mtc_approach_along_axis_m`（`grasp_standoffs.yaml`，现行 0.03 m）；姿态 `alignFrameZ(当前 TCP, 袋轴)`，不抄感知滚转。

响应：`reachable[]` 与请求等长；`error_codes[]`（`no_ik` / `invalid_frame` / `moveit_unavailable`）；`message`。服务不可用时调度回退半径窗。

### `ControlTask`

`command`：`PAUSE=0`（接触段标 PAUSE_PENDING）/ `RESUME=1` / `ENTER_MAINTENANCE=2` / `EXIT_MAINTENANCE=3` / `CANCEL_NOW=4` / `SKIP_TARGET=5` / `ACKNOWLEDGE_RECOVERY=6`。

`expected_state_seq` 须等于当前 `HarvestState.state_seq`（0=不检查）。命令还须出现在快照 `permissions[]` 里。ACK 失败不消耗序号。

响应：`accepted`、`message`、`state_seq`、完整 `state`。

### `ManageLifecycleNodes`

`STARTUP=0` 先全部 configure 再 activate；`PAUSE=1` 逆序 deactivate；`RESUME=2` 顺序 activate；`RESET=3` 拆除再 STARTUP；`SHUTDOWN=4` 停在 Unconfigured。

---

## 消息字段

### 批次

**`HarvestState`** — 批次唯一快照。`batch_state` / `target_phase` 只由 `harvest_fsm.react` 推导，节点禁止手写。

| 字段 | 含义 |
|------|------|
| `target_id` | 当前作业目标；空=未选。感知焦点、重建绑定、观测 `selected` 都只认它 |
| `run_id` / `cycle_id` | 本批；单颗周期为 `run_id:target_id` |
| `state_seq` | 乐观并发；`ControlTask` 须匹配 |
| `revision` | 记录器去重（现行与 `state_seq` 同值） |
| `operation_mode` | `MODE_AUTO=0` / `PAUSED=1` / `MAINTENANCE=2` |
| `batch_state` | `WAITING_READY=0` 等开批 → `DISCOVERY=1` 首巡 → `RUNNING=2` → `PAUSE_PENDING=3` / `PAUSED=4` / `MAINTENANCE=5` / `COMPLETED=6` / `RECOVERY_REQUIRED=7` / `INTERRUPTED=8` / `NAVIGATING=9`（预留，现行不进） |
| `target_phase` | `TARGET_IDLE=0` … `SELECTING` / `OBSERVING` / `FINALIZING` / `VALIDATING` / `APPROACHING` / `TOOL_ACTION` / `RETREATING` / `COMPLETING` / `TARGET_SUCCEEDED|SKIPPED|FAILED` |
| `action_active` | 技能或重建动作进行中 |
| `auto_start_enabled` | 现行恒 false |
| `execution_enabled` / `grasp_enabled` / `tool_enabled` | 只扫不派 / 允许接触规划 / 允许 SetIO（默认刀关） |
| `recovery_required` | 接触恢复待 ACK |
| `progress` | 已尝试 / max(发现数, 已尝试, 1) |
| `permissions[]` | 当前允许的 `ControlTask.command` |
| `navigation_enabled` | 预留，固定座恒 false |
| `blockers[]` | 预留，现行恒空 |

**`CanonicalEvent`** — 线上事件流（不要用预留的 `HarvestEvent`）。

| 字段 | 含义 |
|------|------|
| `severity` | `INFO=0` / `WARNING=1` / `ERROR=2` / `AUDIT=3` |
| `code` | 稳定事件码，监控按此聚合，不要只解析 `message` |
| `message` | 人读；调度常填 JSON（终局带 `failure_code`） |
| `request_id` / `run_id` / `target_id` / `state_seq` | 对齐批次与快照 |
| `sequence` | 预留，现行恒 0 |
| `details[]` | 键值（失败码、阶段、过滤原因） |

事件码：`target_dispatched` / `target_succeeded` / `target_skipped` / `target_failed` / `target_canceled` / `target_operator_skipped`；过程 `photo_pose_reached` / `round_locked` / `survey_failed`；过滤 `targets_filtered`（`out_of_reach_window` / `out_of_depth_window` / `ik_no_solution`）；审计 `batch_paused` / `batch_resumed` / `recovery_required` / `recovery_acknowledged`；`observe_build_view_race`。

**`SceneSnapshot`**：`snapshot_id`、`scene_epoch`（须已 Begin）、`scene_key`、`degraded`、`observation_count`、`target_ids[]`、`message`。

**`HarvestSummary`**：`discovered` / `attempted` / `succeeded` / `skipped_quality` / `skipped_unreachable` / `failed` / `canceled`、`elapsed`、`outcomes[]`。

**`TargetOutcome`**：单颗入账。`outcome` 与 `ExecuteTarget.Result` 同一组常量。`reason` 是稳定原因码（`observe_failed:…` 等），不是自由文本。

**`JobIntent`**：枚举消息；独立话题未接线。调度从 `RunHarvest.goal.intent` 读。字段 `intent` + `target_ids[]`。

### 观测 / 单帧几何

**`PeachTargetObservationArray`** — 选果与对齐的入口。

| 字段 | 含义 |
|------|------|
| `snapshot_id` | 锁定集世代；锁定后不变 |
| `scene_epoch` | 最近一次被接受的 Begin；0=尚未 Begin。SELECT / WAIT_LOCK 必须与调度一致 |
| `harvest_run_id` | 所属批次；空=尚未开批 |
| `target_set_locked` | true 后 `observations` 为固定 ID 集 |
| `target_count` | 锁定后=集大小；锁定前可为 0 |
| `selected_target_id` | 须与 `HarvestState.target_id` 调和 |
| `collecting_count` / `pending_count` | 锁定前进度（锁定前 `observations` 仍为空数组） |
| `observations[]` | 见下 |

**`PeachTargetObservation`**：

| 字段 | 含义 |
|------|------|
| `target_id` | 稳定身份 |
| `priority` | 越小越先 |
| `confirmed` | 已过确认窗 |
| `selected` | 是否当前作业目标 |
| `harvest_status` | `PLANNED` / `WAITING_QUALITY` / `SELECTED` / `HARVESTED` |
| `tracking_status` | `OBSERVED=0` / `OCCLUDED=1` / `LOST=2` / `INVALID=3` / `OUT_OF_VIEW=4` / `DEPTH_VOID=5` |
| `camera_distance_m` / `confidence` | 距离与综合置信度 |
| `candidate` / `candidate_2d` / `fitting` / `mask` | 3D 候选、像素参考、拟合诊断、SAM 掩膜（空图=本帧无掩膜） |
| `diagnostic_flags[]` | 如 `tf_stale`、`mask_unavailable` |

**`BagGraspCandidate`** — `initial_pose` 与 `refined_pose` 共用。坐标系 `header.frame_id`（一般为 `base_link`）。无有效几何时数组话题发**空数组**盖掉旧值。

| 字段 | 含义 |
|------|------|
| `entry_pose` | 袋外预入口 |
| `bag_bottom` / `bag_neck` | 袋底 / 袋颈 [m] |
| `translation_direction` | 插入方向（bottom→neck 单位向量） |
| `bag_diameter_upper_m` / `suggested_travel_m` / `confidence` | 上径、建议行程、0..1 |
| `status` | `ACCEPT=0` 才可当几何；`REOBSERVE=1` 需再观察；`REJECT=2`。感知侧 ACCEPT **仍不授权运动** |
| `match_status` | 见 `MatchStatus` |
| `diagnostic_flags[]` / `strategy_id` / `*_version` | 门控与版本 |
| `position_covariance` / `direction_covariance` | 3×3 行优先；未知填 0 |

**`MatchStatus`**：`OK=0` 唯一匹配；`NEW=1` 新 ID；`AMBIGUOUS=2` 不强制合并；`REJECTED=3` 不入库。

**`BagGrasp2D`**：像素框 `bbox_*`；`bottom_px` / `neck_px` / `grasp_px` / `travel_end_px`（`z=0`）及对应 `has_*`；`status` 同 ACCEPT/REOBSERVE/REJECT。

**`BagFitting`**：拟合诊断，无效标量填 **-1**。含轴向误差、径向净空、有效深度比、圆柱/球 RMS 与内点比、袋长/行程、`status` 同上。`axis_polarity_corrected` 表示是否校正过底↔口。

### 重建 / 许可

**`GraspDecision`** — 套入/剪切的唯一权威。

| 字段 | 含义 |
|------|------|
| `allowed` | **只拦套入和 SetIO**。false 时仍可有几何，预抓取可以走，禁止降级接触 |
| `reason` / `failure_code` | 原因；`FailureCode.*`，0=无失败 |
| `entry` / `pregrasp` / `cut_pose` / `axis` | base 系入口、预抓取、剪切参考、袋轴。无融合时入口/轴填零 |
| `diameter_m` / `d95_m` / `travel_m` / `cut_travel_m` | 直径、D95、插入/剪切行程 |
| `radial_margin_m` / `axial_margin_m` | 径向/轴向余量；负值不可套/剪 |
| `corridor_clear` / `rmse_m` / `inlier_ratio` | 走廊、拟合误差、内点比 |
| `harvest_run_id` / `target_id` / `model_revision` / `tool_profile_id` | 身份与剖面 |

**`ReconstructionStatus`**：`state`（`IDLE` / `COLLECTING` / `READY` / …）；`target_id` 当前绑定（空=未绑）；`target_center_base` 未绑填 `[-1,-1,-1]`；`captured_views` / `rejected_views` / `tf_failures`；`tf_latency_ms` 未测 -1；`valid_depth_ratio`；`max_baseline_deg` / `mean_nearest_baseline_deg`；`view_directions[]` 给自适应视点。无效标量 &lt;0 视为无数据。

**`PregraspVerification`**：`frames_consistent`、`correction_count`、`axis_angle_deg`、`lateral_error_m`、`axial_error_m`、余量、`needs_correction` / `passed`、`failure_code`。话题侧仅重建发；技能 Result 里是自己用工具 TF 填的。

**`ShapeHypothesis`**：`center` / `axis` / `diameter_m` / `length_m` / 协方差 / `confidence` / `model_kind`（`cylinder` / `sphere` / `bag`）。不要单独凭本消息发运动。

**`TargetModel`**：Build 终局。`shape` + `quality`；接触几何 `bag_bottom` / `bag_neck` / `cut_plane_point` / `bag_axis` / `cut_normal` / `d95_m`；`fruit_prior_radius_m` 只作果体禁切包络；`accepted`、余量、`corridor_clear`、`occlusion_class`、源时间范围。

**`TargetQuality`**：`level` `LOW=0` … `HIGH_CONFIDENCE=3`；`score` 通常 0..1；`reason`。

**`GraspHypothesis`**：技能侧 `entry_pose`、`standoff_m`、`travel_m`、`envelope_clearance_m`、`rank_score`（精化约 1，降级锚点约 0.5）、`diagnostic_flags`（如 `degraded_anchor`）。不是运动指令。

### 周期结果块

**`HarvestResult`**：`grasped` 仅切断且撤退确认；`commanded` / `confirmed`；`completion_level` 与 ExecuteTarget 同一组。规划占位不得置 `grasped`。

**`DepositResult`**：`deposited`；卸果站未标定时最高到 G9，`deposited=false`，`reason` 如 `pending_m8_unload_pose`。

**`Verification`**：本周期 `passed`；`harvest_confirmed` 才表示果已带走；`failure_code`。

### 失败码 `FailureCode`

跨包稳定数值，事件与账本写常量，不得解析自由文本 `reason`。

| 值 | 名 | 含义 |
|----|----|------|
| 0 | `NONE` | 无失败 |
| 1 | `OBSERVE_FAILED` | 观察失败 |
| 2 | `BUILD_FAILED` | 建模失败 |
| 3 | `DECISION_REJECTED` | 抓取许可拒绝 |
| 4 | `PREGRASP_RESIDUAL` | 预抓取残差未过 |
| 5 | `SLEEVE_PLAN_FAILED` | 套入规划失败 |
| 6 | `CUT_COMMAND_FAILED` | 刀指令失败 |
| 7 | `CUT_FEEDBACK_TIMEOUT` | 切断反馈超时 |
| 8 | `RETREAT_FAILED` | 撤退失败 |
| 9 | `RECOVERY_REQUIRED` | 须人工 ACK |
| 10 | `VEHICLE_NOT_STATIONARY` | 车未静止（预留） |
| 11 | `EXACT_TF_MISSING` | 缺精确 stamp TF |
| 12 | `DYNAMIC_BUDGET_NEGATIVE` | 动态余量为负 |
| 13 | `UNBAGGED_NOT_IN_SCOPE` | 未套袋不在范围 |
| 14 | `DEGRADED_CONTACT_FORBIDDEN` | 禁止降级接触 |
| 15 | `MODEL_STALE` | 模型过期 |
| 16 | `CORRIDOR_BLOCKED` | 走廊被挡 |

---

## 预留导航（`reserved_interfaces`）

包体在 `_archive/parked_2026-09/peach_navigation`，不进构建。IDL 留名，清单无生产方。调度到位一步直通 `NAV_OK`。

| 名字 | 类型 | 含义 |
|------|------|------|
| `/peach_navigation_node/navigate_to_worksite` | action `NavigateToWorksite` | 底盘走到 `pose` / `site_id` |
| `/peach/navigation/target_report` | `HarvestTargetReport` | 向导航报当前目标位姿/袋轴/`IDLE..ABANDONED` |
| `/peach/navigation/vehicle_state` | `VehicleState` | 车是否到位且静止；`fresh` = 在 `freshness_s` 内 |
| `/peach/navigation/arm_status` | `HarvestOperationStatus` | 臂占用；`resume_allowed`；剪切/撤退完成前不得让底盘 resume |

契约预留、节点未全部接线的类型：`JobIntent`（只作枚举）、`HarvestEvent`（请用 `CanonicalEvent`）、`ShapeHypothesis` / `GraspHypothesis` 话题（已发但不是批次门）。

旁路视觉抓取走 `ivg_interfaces`，不进本清单、不进 `harvest_system`。
