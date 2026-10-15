# peach_interfaces

采摘各能力包之间**唯一允许互发的类型**。无节点、无 launch、**无 ROS 运行参数**。感知 / 重建 / 技能 / 调度 / 监控只依赖本包消息，禁止互相 import 业务结构体。

| 你要查的 | 在哪 |
|----------|------|
| 管子叫什么、谁发谁订、QoS | 下面「接线」+ [`config/interface_manifest.yaml`](config/interface_manifest.yaml) |
| 管子里每个字段什么意思 | `msg/` `srv/` `action/` 字段定义与注释；图名、生产消费方和QoS以manifest为准 |
| 算法阈值、后撤距离、选果窗 | **不在本包**。见文末「运行参数在哪」 |
| 一批谁先叫谁 | [doc/io.md](doc/io.md)（本包）与 [系统 io](../peach_bringup/doc/io.md) §1；架构 [doc/architecture.md](doc/architecture.md) 与 [系统架构](../peach_bringup/doc/architecture.md) §3 |

改字段：先改本包 IDL → 本 README 与 manifest → [doc/io.md](doc/io.md) 与 [系统 io](../peach_bringup/doc/io.md) → 各包 pub/sub。先编本包再编下游。漂移检查：

```bash
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

---

## 本包没有运行参数

没有 `*_parameters.yaml`，节点也不会 `declare_parameter`。清单里的 `qos:` 是**跨包必须对齐的传输约定**，不是可调阈值。

真正的运行参数在能力包 `config/<节点>.yaml`（部署事实源）+ `peach_common.yaml_params.attach` 与包内规则模块。C++ 执行合同另有 generate_parameter_library yaml。

| 文件 | 管什么 |
|------|--------|
| `peach_harvester/config/scene_perception.yaml` | 检测、锁定窗、深度窗、袋/果管线 |
| `peach_harvester/config/target_reconstruction.yaml` | 采帧门、TSDF、机位数 |
| `peach_harvester/config/scene_obstacles.yaml` | 场景障碍快照（体素/胶囊余量；0035） |
| `peach_harvester/config/grasp_standoffs.yaml` | 入口相对袋底、预抓取后撤（launch 注入各节点已声明参数） |
| `peach_arm/config/peach_arm.yaml` | 视点、MTC、接触、刀具 IO |
| `peach_arm/src/arm_parameters.yaml` | 执行合同（GPL 单源，2c 回迁） |
| `peach_harvester/config/peach_supervisor.yaml` | 批次、选果、`execute_pregrasp_only` |
| `peach_bringup/launch/harvest_system.launch.py` | Nav2 lifecycle 名单与顺序 |
| `peach_observability/config/observability.yaml` | 监控 Web + 启动自检；`debug.motion_enabled` 默认关（W11 起随包走） |

---

## 接线（图上的名字）

每一行 = 一根管子。`consumers: []` = 有人发、现行核内无人订（预留、只给 RViz 手订、或仅动作 Result 携带同类型）。

QoS 缩写：`R` reliable；`TL` transient_local（晚订户仍拿得到最后一帧）；`V` volatile；数字是 depth。未写 qos 的动作/服务走 ROS 默认。

### 开批 / 控批（调度）

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_supervisor/run_harvest` | action | `RunHarvest` | **唯一开批开关。** 采摘意图至少成功一颗才 `success`；`SURVEY_ONLY` 正常扫场结算可为 true（零目标不算失败）；autostart 部署参数可自动发（默认关，且硬等 `/peach/observability/selfcheck_passed`） | 调度 | （人工/客户端/autostart） | — |
| `/peach_supervisor/control` | service | `ControlTask` | 暂停/恢复/取消/跳过/ACK 恢复。`expected_state_seq` 须对上当前快照 | 调度 | （人工；监控默认不发） | — |
| `/peach_supervisor/state` | topic | `HarvestState` | **批次唯一快照。** 能力包只认其中的 `target_id` 当当前作业 | 调度 | 感知、重建、监控；调度自订 | R/TL/1 |
| `/peach_supervisor/events` | topic | `CanonicalEvent` | 可检索事件流（派发/成功/失败/暂停/ACK/过滤） | 调度 | 监控 | R/TL/50 |
| `/peach_supervisor/scene_snapshot` | topic | `SceneSnapshot` | WAIT_LOCK 结束或回访后的锁定集快照 | 调度 | 监控（bag 专用，不镜像） | R/TL/1 |
| `/peach_supervisor/set_enables` | service | `SetEnables` | 操作台使能开关；服务后广播 `/peach/batch/enables` + 审计 | 调度 | （人工；8090 转发未接，`ros2 service call` 可用） | — |
| `/peach_supervisor/set_batch_policy` | service | `SetBatchPolicy` | 运行期改批次策略（当前批或下批默认） | 调度 | （人工；8090 转发未接） | — |
| `/peach_supervisor/fire_step` | service | `FireStep` | 操作台单步；现行仅 PHOTO 已接线（MoveTo KIND_NAMED 到拍照位），VIEWPOINT/BUILD/APPROACH/PREVIEW/TOOL_DEBUG 占位调用即拒；未来运动类档位过臂侧命令门 | 调度 | （人工；8090 转发未接） | — |
| `/peach/batch/enables` | topic | `Enables` | 使能广播（Active 且 override 非空时 1Hz 心跳重发）；臂侧收到过即覆盖本地参数，缺心跳回落本地参数权威 | 调度 | 技能 | R/TL/1 |

### 生命周期

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_lifecycle_manager/manage_nodes` | service | `nav2_msgs/ManageLifecycleNodes` | 整栈 configure/activate/拆除；PAUSE 是节点 Inactive，不是批次暂停 | nav2_lifecycle_manager（外部） | observability / 人工 | — |
| `/peach_lifecycle_manager/is_active` | service | `std_srvs/Trigger` | nav2_lifecycle_manager 自带就绪查询（外部节点）；闩锁桥输入 | （nav2_lifecycle_manager，外部） | bringup | — |
| `/peach/lifecycle/managed_nodes_activated` | topic | `std_msgs/Bool` | 名单都 Active 后才允许接开批 | lifecycle 闩锁桥（`peach_lifecycle_flag_bridge`，bringup） | 调度 | R/TL/1 |

### 看场景

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_scene_perception_node/begin_scene` | service | `BeginScene` | 重启收齐窗，`scene_epoch+1`。同场保留身份；换 `scene_key` 才清表。仅首巡 Survey 到位后调一次 | 感知 | 调度 | — |
| `/peach/perception/target_observations` | topic | `PeachTargetObservationArray` | **园子里有哪些桃。** 调度**只订这一根**选果（须 `scene_epoch` 对齐且已锁定） | 感知 | 调度、重建、技能、监控 | R/V/10 |
| `/peach/perception/initial_pose` | topic | `BagGraspCandidateArray` | 单帧袋入口/轴**初值**。只给重建当起点；`ACCEPT` 也不授权运动 | 感知 | **仅重建** | R/TL/1 |
| `/peach/perception/diagnostics` | topic | `BagFittingArray` | 单帧拟合诊断（直径/RMSE/内点） | 感知 | 重建 | — |
| `/peach/perception/harvest_state` | topic | `std_msgs/String` | 感知侧锁定集 JSON，给监控看 | 感知 | 监控 | R/TL/1 |
| `/peach/perception/debug_image` | topic | `sensor_msgs/Image` | 检/分割叠加（confirmed-only） | 感知 | 监控 | R/V/10 |
| `/peach/scene/obstacles_refresh` | topic | `std_msgs/Empty` | Survey 快照式建图触发（Survey 成功=拍照位刚拍完，点云帧即障碍快照源；新批次自动重建） | 调度 | 场景障碍 | R/TL/1 |

### 建当前目标

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach_target_reconstruction_node/build_target_model` | action | `BuildTargetModel` | 绑 `target_id`，收够机位后 finalize。与 OBSERVE_ONLY **并行**。只用图像时刻精确 TF | 重建 | 调度 | — |
| `/peach/reconstruction/grasp_decision` | topic | `GraspDecision` | **融合入口/轴/预抓取/剪切 + `allowed`。** `allowed` 只拦套入和刀，不拦预抓取 | 重建 | 技能、调度（闩锁许可令牌缓存）、监控 | R/TL/1 |
| `/peach/reconstruction/refined_pose` | topic | `BagGraspCandidateArray` | 融合后袋位姿 | 重建 | 技能、场景障碍、监控 | R/TL/1 |
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
| `/peach_arm/survey_scene` | action | `SurveyScene` | 去全局拍照位并确认关节已静止。失败整批 `survey_failed`，不 Begin | 技能 | 调度 | — |
| `/peach_arm/execute_target` | action | `ExecuteTarget` | 对当前 `target_id` 跑一周期：预览 / 补视角 / 停预抓取 / 套入 | 技能 | 调度 | — |
| `/peach_arm/check_reachability` | service | `CheckReachability` | 选果：预抓取停位 IK；FULL 时再检套入终点。不规划、不动臂 | 技能 | 调度 | — |
| `/peach_arm/move_to` | action | `MoveTo` | 视点 / 拍照位 / 命名位 / 位姿转移移动（KIND_JOINTS 预留未实现）；取消=透传 abort + `RobotMoveStop`；与接触周期互斥 | 技能 | 调度（fast 档观察补视 / 回拍照位） | — |
| `/peach_arm/acknowledge_recovery` | service | `std_srvs/Trigger` | 接触或预抓取停住后，调度转发人工 ACK | 技能 | 调度 | — |
| `/peach/manipulation/grasp_hypothesis` | topic | `GraspHypothesis` | 本周期技能打算怎么抓。**只有监控订**，FSM 不靠它做决定 | 技能 | 监控 | R/TL/1 |
| `/peach_arm/status` | topic | `std_msgs/String` | 技能短状态 | 技能 | 监控 | R/TL/1 |
| `/peach_arm/tool_state` | topic | `ToolState` | 三轴工具状态（刀/保持/载荷），on-change + 1Hz 心跳 | 技能 | （核内无订；会话 bag 落证据） | R/TL/1 |

### 监控自产

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach/observability/tcp_path` | topic | `nav_msgs/Path` | TCP 轨迹 | 监控 | （核内无订） | R/TL/1 |
| `/peach/observability/markers` | topic | `MarkerArray` | 监控叠加 | 监控 | （核内无订） | R/TL/1 |
| `/peach/observability/job` | topic | `std_msgs/String` | 8090 任务侧 JSON | 监控 | （核内无订） | R/V/10 |
| `/peach/observability/metrics` | topic | `std_msgs/String` | 指标 JSON | 监控 | （核内无订） | R/V/10 |
| `/peach/observability/selfcheck_passed` | topic | `std_msgs/Bool` | 启动自检通过（autostart 硬门） | 监控 | bringup | R/TL/1 |

### 枝/叶掩膜（`peach_vegetation`，不进 harvest_system）

| 名字 | 种类 | 类型 | 含义 | 发 | 订 | QoS |
|------|------|------|------|----|----|-----|
| `/peach/vegetation/leaf_mask` | topic | `sensor_msgs/Image` | 叶掩膜 mono8 | 植被 | （核内无订） | R/V/10 |
| `/peach/vegetation/branch_mask` | topic | `sensor_msgs/Image` | 木质脊掩膜 mono8 | 植被 | （核内无订） | R/V/10 |
| `/peach/vegetation/overlay` | topic | `sensor_msgs/Image` | 绿叶红枝叠加 | 植被 | （核内无订） | R/V/10 |
| `/peach/vegetation/status` | topic | `std_msgs/String` | JSON 耗时/覆盖率 | 植被 | 监控（自检） | R/V/10 |

清单故意不收的可视化话题（源码仍发，RViz 用手订）：`/peach/perception/{axis,debug_image_raw,detections,markers,masks,single_cloud}`、`/peach/reconstruction/local_cloud`、`/peach_arm/planned_views`。

未列表内的其余现行 manifest 条目（`std_srvs` 调试/运维入口，核内无订户）：`/peach_arm/{go_to_photo_pose,preview_approach_insert,preview_full_contact,set_execution_armed,cancel_cycle}`、`/peach_target_reconstruction_node/{reset_reconstruction,finalize_reconstruction,save_session,query_reconstruction_state}`。

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
| `intent` | 见本 action `INTENT_*` 常量（`JobIntent` 已删，此处唯一权威）：`INTENT_PICK_ALL=0` 锁定集逐颗；`INTENT_PICK_SELECTED=1` 只采 `target_ids`；`INTENT_SURVEY_ONLY=2` 只扫不派 |
| `selection_mode` | 预留 `AUTO/MANUAL`；现行：`target_ids` 非空即优先按列表 |
| `target_ids` | 可选允许列表；空=不限定 |

Result：采摘意图（PICK_ALL / PICK_SELECTED）`success` = 至少成功一颗——空场景、全过滤或全失败均 false（`termination_reason=no_targets_succeeded`）；`SURVEY_ONLY` 正常扫场结算可为 true，零目标不算失败；取消或中止保持 false。批次态 `COMPLETED` / action `SUCCEEDED` 只证明批次正常结算，请求是否达成读本字段。`termination_reason`（`completed` / `canceled` / `survey_failed` / `begin_scene_failed` / `no_targets_succeeded` 等）；`summary` 见 `HarvestSummary`。

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
| `model_revision` / `tool_profile_id` / `calibration_revision` / `config_revision` | 模型身份；FULL/PREGRASP_ONLY 空版本拒执行 |
| `plan_id` / `generation` | 预览=执行同一计划；迟到结果丢弃 |
| `profile` | `0` 是 IDL 默认 HOLD，不覆盖 `mode=FULL`；`PROFILE_FULL=1` 强制套入 |

Result 终局 `outcome`：`SUCCEEDED=0` / `SKIPPED_QUALITY=1` / `SKIPPED_UNREACHABLE=2` / `FAILED=3` / `CANCELED=4`。

`completion_level`：`LEVEL_NONE` → `LEVEL_PREGRASP_REACHED`（已到位）→ `LEVEL_PREGRASP_VERIFIED`（残差过门且有新鲜观测）→ 套入 → 刀指令受理 → 切断确认 → 撤退确认 → `LEVEL_HARVEST_CONFIRMED`。`pregrasp.passed` 与 `completion_level` 不得互相抬级。产品成功看 `harvest.grasped`：仅切断证据∧撤退证据。`recovery_required` 为真时须人工 ACK 才允许下一颗。

其余 Result 块：`harvest` / `verification` / `outcome_record` / `pregrasp`（技能自填的残差，不是订重建话题）。cut/retreat/harvest 证据单源这些块（W7 起顶层 bool 镜像与 `deposit` 已删）。

Feedback：`HarvestState`。

### `NavigateToWorksite` — 预留

Goal：`pose`、`site_id`。现行固定座调度直通 `NAV_OK`，**不发送**本动作，无服务端。

---

## 服务字段

### `BeginScene`

请求：`request_id`、`scene_key`。响应：`accepted`（非 Active / 忙则 false）、`scene_epoch`（接受后从 1 递增）、`message`。

### `CheckReachability`

请求：`header`；`tcp_poses[]` = 感知入口（位置=袋底，姿态 Z=袋轴，滚转任意）；`timeout_s`（≤0 用服务端 0.1 s）；`require_sleeve`（默认 false=只预抓取；true=再检套入终点）；`suggested_travel_m[]` 与入口并行（空或 ≤0 用服务端 `minimum_travel_m`）。

服务端预抓取停位再 IK：沿 −Z 后撤 `mtc_approach_along_axis_m`（`grasp_standoffs.yaml`，现行 0.03 m）；姿态 `alignFrameZ(当前 TCP, 袋轴)`，不抄感知滚转。`require_sleeve` 时套入终点 = 入口沿袋轴 + clamp(travel, grasp min/max)，同姿态滚转扫描，再从预抓取 IK 解沿轴笛卡尔插值（步长/完成比与接触 LIN 同参）。

响应：`reachable[]` 与请求等长；`error_codes[]`（`no_ik` / `sleeve_no_ik` / `sleeve_no_cartesian` / `invalid_frame` / `moveit_unavailable`）；`message`。服务不可用时调度回退半径窗（FULL 另卡套入终点半径）。旧客户端不填新字段则行为与只检预抓取相同。

### `ControlTask`

`command`：`PAUSE=0`（接触段标 PAUSE_PENDING）/ `RESUME=1` / `ENTER_MAINTENANCE=2` / `EXIT_MAINTENANCE=3` / `CANCEL_NOW=4` / `SKIP_TARGET=5` / `ACKNOWLEDGE_RECOVERY=6`。

`expected_state_seq` 须等于当前 `HarvestState.state_seq`（0=不检查）。命令还须出现在快照 `permissions[]` 里。ACK 失败不消耗序号。

响应：`accepted`、`message`、`state_seq`、完整 `state`。

### 标准生命周期服务

`/peach_lifecycle_manager/manage_nodes` 使用 `nav2_msgs/srv/ManageLifecycleNodes`，本包不生成重复 IDL。8090 调试桥保留五条现行命令；响应仅有 `success`，没有 `message`。PAUSE 是节点生命周期停用，不是批次暂停。

## 消息字段

### 批次

**`HarvestState`** — 批次唯一快照。`batch_state` / `target_phase` 只由 `harvest_fsm.react` / domain reducer 推导，节点禁止手写。`PAUSE_PENDING` 是历史 IDL 值，编排器不得再把作业 phase 写成此值（暂停走 `operation_mode`）。`action_generation` / `transaction_id` / `scene_epoch` 绑定动作事务。

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
| `blockers[]` | 软门阻塞原因（2026-09-29 起填充；词表 `stack_not_ready` / `recovery_required` / `mode_paused` / `mode_maintenance` / `ledger_write_failed`，源 `harvest_fsm.blockers_for`） |

**`CanonicalEvent`** — 线上唯一事件流（`HarvestEvent` 已删，勿再造）。

| 字段 | 含义 |
|------|------|
| `severity` | `INFO=0` / `WARNING=1` / `ERROR=2` / `AUDIT=3` |
| `code` | 稳定事件码，监控按此聚合，不要只解析 `message` |
| `message` | 人读；调度常填 JSON（终局带 `failure_code`） |
| `request_id` / `run_id` / `target_id` / `state_seq` | 对齐批次与快照 |
| `sequence` | 预留，现行恒 0 |
| `details[]` | 键值（失败码、阶段、过滤原因） |

事件码：终局 `target_dispatched` / `target_succeeded` / `target_skipped` / `target_failed` / `target_canceled` / `target_operator_skipped`；批次中断 `survey_failed` / `begin_scene_failed` / `navigate_failed`；过程 `photo_pose_reached` / `round_locked`；单果时限 `target_timeout`（WARNING 级）；过滤 `targets_filtered`（`out_of_reach_window` / `out_of_depth_window` / `ik_no_solution`）；审计 `batch_paused` / `batch_resumed` / `recovery_required` / `recovery_acknowledged` / `enables_changed` / `batch_policy_updated` / `fire_step` / `ledger_restored`；派发异常 `observe_build_view_race`。

**`SceneSnapshot`**：`snapshot_id`、`scene_epoch`（须已 Begin）、`scene_key`、`degraded`、`observation_count`、`target_ids[]`、`message`。

**`HarvestSummary`**：`discovered` / `attempted` / `succeeded` / `skipped_quality` / `skipped_unreachable` / `failed` / `canceled`、`elapsed`、`outcomes[]`。

**`TargetOutcome`**：单颗入账。`outcome` 与 `ExecuteTarget.Result` 同一组常量。`reason` 是稳定原因码（`observe_failed:…` 等），不是自由文本。

**`JobIntent`（已删）**：枚举消息已随 e2f37ff 删除；`intent` 常量并入 `RunHarvest.action` 的 `INTENT_*`（唯一权威），调度从 `RunHarvest.goal.intent` 读。

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
| `match_status` | 身份匹配状态（`MatchStatus` 常量已删，IDL 不再定义取值） |
| `diagnostic_flags[]` / `strategy_id` / `*_version` | 门控与版本 |
| `position_covariance` / `direction_covariance` | 3×3 行优先；未知填 0 |

**`MatchStatus`（已删）**：原 `OK=0` / `NEW=1` / `AMBIGUOUS=2` / `REJECTED=3` 枚举消息已随 e2f37ff 删除（零引用）；`BagGraspCandidate.match_status` 字段保留，IDL 现不定义取值常量。

**`BagGrasp2D`**：像素框 `bbox_*`；`bottom_px` / `neck_px` / `grasp_px` / `travel_end_px`（`z=0`）及对应 `has_*`；`status` 同 ACCEPT/REOBSERVE/REJECT。

**`BagFitting`**：拟合诊断，无效标量填 **-1**。含轴向误差、径向净空、有效深度比、圆柱/球 RMS 与内点比、袋长/行程、`status` 同上。`axis_polarity_corrected` 表示是否校正过底↔口。

### 重建 / 许可

**`GraspDecision`** — 套入/剪切的唯一权威。

| 字段 | 含义 |
|------|------|
| `allowed` | **汇总位**=geometry∧sleeve∧cut（pregrasp 不进），只拦套入和 SetIO。果距与径向余量不进这三位，只作数据参考。false 时仍可有几何，预抓取可以走，禁止降级接触。TOOL 另要预备位残差与 `tool.enabled` |
| `reason` / `failure_code` | 原因；`FailureCode.*`，0=无失败 |
| `entry` / `pregrasp` / `cut_pose` / `axis` | base 系入口、预抓取、剪切参考、袋轴。无融合时入口/轴填零 |
| `diameter_m` / `d95_m` / `travel_m` / `cut_travel_m` | 直径、D95、插入/剪切行程 |
| `radial_margin_m` / `axial_margin_m` | 径向余量是静态开口相对袋径的初步查看标记，和轴向余量一样只作数据参考。接触时袋和刀具有弹性、会动，目前不构成约束；以后若启用径向余量，须另立动态判据。负值不拦套入或剪切 |
| `corridor_clear` / `rmse_m` / `inlier_ratio` | 走廊、拟合误差、内点比 |
| `harvest_run_id` / `target_id` / `model_revision` / `tool_profile_id` | 身份与剖面（`tool_profile_id` 由重建按节点 `tool.profile_id` 填，整栈=launch `tool_profile` 档案） |

**`ReconstructionStatus`**：`state`（`IDLE` / `COLLECTING` / `READY` / …）；`target_id` 当前绑定（空=未绑）；`target_center_base` 未绑填 `[-1,-1,-1]`；`captured_views` / `rejected_views` / `tf_failures`；`tf_latency_ms` 未测 -1；`valid_depth_ratio`；`max_baseline_deg` / `mean_nearest_baseline_deg`；`view_directions[]` 给自适应视点。无效标量 &lt;0 视为无数据。

**`PregraspVerification`**：`frames_consistent`、`correction_count`、`axis_angle_deg`、`lateral_error_m`、`axial_error_m`、余量、`needs_correction` / `passed`、`failure_code`。话题侧仅重建发；技能 Result 里是自己用工具 TF 填的。

**`ShapeHypothesis`**：`center` / `axis` / `diameter_m` / `length_m` / 协方差 / `confidence` / `model_kind`（`cylinder` / `sphere` / `bag`）。不要单独凭本消息发运动。

**`TargetModel`**：Build 终局。身份元组 `run_id + scene_epoch + target_id + model_revision + tool_profile_id + calibration_revision + config_revision`；`generated_at` / `valid_until`（心跳不得续签）；能力三态 geometry/pregrasp/sleeve/cut。`shape` + `quality`；接触几何 `bag_bottom` / `bag_neck` / `cut_plane_point` / `bag_axis` / `cut_normal` / `d95_m`；`fruit_prior_radius_m` 只作果体禁切包络；`accepted`、余量、`corridor_clear`、`occlusion_class`、源时间范围。空版本不得执行（预览除外）。

**`TargetQuality`**：`level` `LOW=0` … `HIGH_CONFIDENCE=3`；`score` 通常 0..1；`reason`。

**`GraspHypothesis`**：技能侧 `entry_pose`、`standoff_m`、`travel_m`、`envelope_clearance_m`、`rank_score`（精化约 1，降级锚点约 0.5）、`diagnostic_flags`（如 `degraded_anchor`）。不是运动指令。

### 周期结果块

**`HarvestResult`**：`grasped` 仅切断且撤退确认；`commanded` / `confirmed`；`completion_level` 与 ExecuteTarget 同一组。规划占位不得置 `grasped`。

**`DepositResult`**：`deposited`；卸果站未标定时最高到 G9，`deposited=false`，`reason` 如 `pending_m8_unload_pose`。W7 起不再随 `ExecuteTarget.Result` 携带（消息保留，卸果站预留）。

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
| 17 | `MODEL_IDENTITY_INCOMPLETE` | 身份元组缺版本 |
| 18 | `MODEL_EXPIRED` | `valid_until` 已过 |
| 19 | `TOOL_STATE_UNKNOWN` | SetIO 超时/无反馈 |
| 20 | `PLAN_MISMATCH` | 预览/执行计划不一致 |
| 21 | `TRANSIT_FAILED` | MoveTo 转移移动失败（2026-09-29 补，不再挪用 5） |
| 22 | `START_NOT_READY` | 周期启动门拒绝（MoveIt 未初始化/占用/未 arm） |
| 23 | `CANCELED` | 动作取消终局（BuildTargetModel 等） |

---

## 预留导航（`reserved_interfaces`）

包体在 `_archive/parked_2026-09/peach_navigation`，不进构建。IDL 留名，清单无生产方。调度到位一步直通 `NAV_OK`。

| 名字 | 类型 | 含义 |
|------|------|------|
| `/peach_navigation_node/navigate_to_worksite` | action `NavigateToWorksite` | 底盘走到 `pose` / `site_id` |
| `/peach/navigation/target_report` | `HarvestTargetReport` | 向导航报当前目标位姿/袋轴/`IDLE..ABANDONED` |
| `/peach/navigation/vehicle_state` | `VehicleState` | 车是否到位且静止；`fresh` = 在 `freshness_s` 内 |
| `/peach/navigation/arm_status` | `HarvestOperationStatus` | 臂占用；`resume_allowed`；剪切/撤退完成前不得让底盘 resume |

已删的预留类型：`JobIntent` / `HarvestEvent` / `MatchStatus`（e2f37ff 删除；`intent` 常量以 `RunHarvest.action` 的 `INTENT_*` 为唯一权威，事件流用 `CanonicalEvent`）。契约预留、已发但不是批次门的话题类型：`ShapeHypothesis` / `GraspHypothesis`。

## 清洁重写轮契约接线状态（阶段 2b/3 已全部落码）

`MoveTo.action`（`/peach_arm/move_to`，已接线 2b；supervisor 为消费方——fast 档观察补视/回拍照位，stow 仍走接触周期内 PTP）、`Clearance.msg`（接触许可令牌，随 `ExecuteTarget` goal）、`SetEnables.srv` / `SetBatchPolicy.srv`（`/peach_supervisor/*`，已接线 3；8090 页面转发未接——人工 `ros2 service call` 可用）、`FireStep.srv`（服务端已接线；现行仅 PHOTO 档实现走 MoveTo，其余档位占位调用即拒）。`RunHarvest` 已附加 `INTENT_*` 常量与批次策略字段（`target_harvest_ratio` / `per_target_timeout_s` / `sector_timeout_s` / `view_policy`）；`ExecuteTarget` 已附加 `clearance` / `profile` 与 `CK_*` 检查点常量（AT_STAGING→…→CUT_CONFIRMED→RETAINED→RETREATED→STOWED）。均已登记 manifest 现行区；状态与语义详见 [doc/io.md](doc/io.md) 的「清洁重写轮契约接线状态」表。

旁路视觉抓取走 `ivg_interfaces`，不进本清单、不进 `harvest_system`。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。

同revision模型时间约定：GraspDecision.header.stamp与TargetModel.generated_at/header.stamp共用首次生成时间，valid_until冻结，心跳不续签。独立单机位成功最多MEDIUM，至少2独立机位才可HIGH；READY/accepted不替代四能力或臂侧运行授权。
