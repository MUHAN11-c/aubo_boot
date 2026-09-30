# peach2_task

Peach v2 批次编排（方案 §2.2 / §8 / §10.2 / §11 / §13）：BehaviorTree.CPP v4 采摘树、选果、批次策略、
操作员使能、run 账本。**不发关节命令、不做 IK、不写 SetIO**——所有运动与 IO 只经 peach2_manipulation 命令门。

- 纯核（零 ROS，gtest 覆盖）：`core/selection`、`core/batch_policy`、`core/enables_policy`、`core/safety_gate`、
  `core/ledger`、`core/batch_session`（整批状态，BT 节点共享）。
- BT 逻辑节点（零 ROS）：`bt/logic_nodes`；ROS 叶子（薄异步适配）：`ros/ros_nodes`；节点：`task_node`。
- BehaviorTree.ROS2 未安装，未使用：`ros/async_leaves.hpp` 自写非阻塞 action/service 叶子
  （onStart 发送、onRunning 轮询回调写入的状态、onHalted 取消；tick 内永不阻塞）。

## 公有 API

C++ 公有头只供本包与测试使用（无下游包依赖）。图上的公有面：

| 名字 | 类型 | 方向 | QoS / 说明 |
|------|------|------|-----------|
| `/peach/task/run_batch` | action `RunBatch` | server | execute 只起树；取消 = haltTree + 各叶子取消在途 goal |
| `/peach/task/set_enables` | srv `SetEnables` | server | 链校验 tool⇒grasp⇒execution；非 Active 拒绝 |
| `/peach/task/acknowledge_recovery` | srv `std_srvs/Trigger` | server | 操作员 ACK：转发臂侧幂等 ACK 并清任务侧待 ACK（见「WaitForAck 机制」） |
| `/peach/enables` | `Enables` | pub | reliable, transient_local, 1；1 Hz 心跳，每次发布 `seq` +1；默认全 false |
| `/peach/task/state` | `BatchState` | pub | reliable, transient_local, 1；变化即发（≤10 Hz）+ 1 Hz 心跳；执行中同时作 RunBatch feedback |
| `/peach/perception/observations` | `TargetObservationArray` | sub | reliable, volatile, 10 |
| `/peach/end_effector/tool_state` | `ToolState` | sub | reliable, transient_local, 1 |
| `/peach/manipulation/recovery_required` | `std_msgs/Bool` | sub | reliable, transient_local, 1；臂侧锁存，对齐任务侧 recovery 状态 |
| `/aubo_io_controller/robot_status` | `aubo_msgs/RobotStatus` | sub | best_effort, 1；无 header，年龄按接收时刻 |
| `/peach/manipulation/move_to` | action `MoveTo` | client | Survey 回拍照位 |
| `/peach/scene/build_snapshot` | srv `BuildSceneSnapshot` | client | 首巡 `clear_previous=true`，重勘 `false`（合并） |
| `/peach/perception/begin_scene` | srv `BeginScene` | client | 首巡在快照之后调用，取本批 `scene_epoch` |
| `/peach/manipulation/check_reachability` | srv `CheckReachability` | client | SelectTarget；`reasons` 写进账本 |
| `/peach/target_model/observe` | action `ObserveTarget` | client | |
| `/peach/target_model/get_decision` | srv `GetDecision` | client | CheckDecision |
| `/peach/manipulation/harvest_target` | action `HarvestTarget` | client | |
| `/peach/manipulation/acknowledge_recovery` | srv `std_srvs/Trigger` | client | ACK 转发 |
| `/diagnostics` | diagnostic_updater | pub | 任务 `batch` / `servers` / `safety` |
| `/bond` | bondcpp | | on_activate 起、on_deactivate 断 |

话题名固定，不是参数。launch **永不**发 RunBatch；批次只能由操作员发起。

## 行为树

`trees/harvest_batch.xml`（方案 §8.2 骨架）。50 Hz wall timer 调 `Tree::tickOnce()`；单线程 executor，
叶子回调与 tick 同线程。

```text
HarvestBatch
└─ ReactiveSequence                      # CheckSafety 每 tick 重评；失败即 halt 整棵树（在途 goal 取消）
   ├─ CheckSafety
   └─ Sequence
      ├─ SubTree Survey                                # 首巡
      │  └─ Sequence: MoveToNamed({photo_pose}) → BuildSceneSnapshot(clear_previous=true)
      │               → BeginScene → WaitTargetSetLocked
      └─ Fallback
         ├─ Sequence: IntentIs(SURVEY_ONLY) → SettleBatch("survey_only")
         └─ Fallback
            ├─ KeepRunningUntilFailure
            │  └─ SubTree NextTarget
            │     └─ Sequence
            │        ├─ IfThenElse: IsRecoveryRequired → WaitForAck | AlwaysSuccess   # 先清待 ACK
            │        ├─ BatchGate                      # max_targets / 采收率 / 显式名单完成 → settle
            │        └─ IfThenElse
            │           ├─ SelectTarget(tid)           # 仅 locked_target_ids；CheckReachability + 纯核
            │           ├─ Fallback                    # 每颗果失败都记 skip，不中断批次
            │           │  ├─ Sequence: SubTree HarvestOne → RecordResult
            │           │  └─ RecordSkip               # STOP_BATCH 策略时 FAILURE → 批次中止
            │           └─ Sequence: NoTargetRound → SubTree Resurvey   # 空轮才回拍照位重勘
            └─ BatchSettled                            # 正常收批 SUCCESS，中止 FAILURE

Resurvey                                               # 同 epoch（id 不变），快照合并
└─ Sequence: MoveToNamed({photo_pose}) → BuildSceneSnapshot(clear_previous=false) → WaitTargetSetLocked

HarvestOne
└─ RetryOnPolicy(num_attempts={retry_attempts})       # RETRY_VIEW / WAIT 重试；REMEASURE_NECK 见下
   └─ Sequence
      ├─ IfThenElse
      │  ├─ NeckRemeasurePending                       # 上一尝试返回 NECK_REMEASURE_PENDING（仅 FULL）
      │  ├─ Sequence: ObserveTarget(max_views=1, neck_remeasure=true) → CheckDecision(level=cut)
      │  └─ WithinTargetDeadline                       # per_target_timeout_s，只罩 observe+decision
      │     └─ Sequence: ObserveTarget({max_views}) → CheckDecision(level=approach)
      ├─ NoRecoveryPending                             # 臂侧锁存中途拉起 → 不下发 HarvestTarget
      └─ HarvestTarget(intent → mode)
```

黑板（节点在 createTree 前写入根黑板）：`intent`(int, RunBatch.INTENT_*)、`tool_id`、`photo_pose`、
`max_views`(unsigned)、`retry_attempts`(unsigned)；子树 `_autoremap="true"`，`tid` 与 `model_revision`
在子树内产生。

### 节点与端口

| 节点 | 类型 | 端口 | 行为 |
|------|------|------|------|
| `CheckSafety` | Condition | — | 缓存的 robot_status（drives_powered ∧ ¬e_stopped ∧ ¬in_error ∧ 年龄<0.3 s；mock 可关）+ enables（每个 intent 要 execution，FULL 另要 grasp+tool）+ FULL 时 ToolState≠FAULT；失败 → abort `safety:<blockers>` |
| `MoveToNamed` | 异步 action | in `target`(string)、in `velocity_scaling`(double, 0) | MoveTo；失败/超时 → Survey 失败，批次中止 |
| `BuildSceneSnapshot` | 异步 srv | in `clear_previous`(bool, true) | 失败 → 批次中止（无障碍快照不接触）；日志带 `truncated` / `n_frames` |
| `BeginScene` | 异步 srv | — | 首巡调用 `/peach/perception/begin_scene`(request_id)；`accepted=false` 或服务失败 → 批次中止；成功把 `scene_epoch` 记进 session 与账本 |
| `WaitTargetSetLocked` | 异步 | — | 等 `target_set_locked`、`scene_epoch` = BeginScene 返回值、`header.stamp` ≥ 本节点开始时刻（stamp=0 则用接收时刻）；epoch 更大 → `scene_epoch_changed` 中止；缓存 `locked_target_ids` 与观测进 session；超时 `lock_wait_s` → 批次中止 |
| `SelectTarget` | 异步 srv | out `target_id`(string) | 只在 `locked_target_ids` 里做资格过滤 → CheckReachability(候选, tool, mode) → selection 纯核排序并 claim；每个 id 的 reachable/code/`reasons[i]` 写账本 `reachability`；无候选 FAILURE（空轮）；服务缺失/超时在 `require_reachability` 下中止批次（**不回退半径窗**） |
| `ObserveTarget` | 异步 action | in `target_id`、in `max_views`(unsigned)、in `neck_remeasure`(bool, false)、out `model_revision`(uint64) | 未收敛 → MODEL_NOT_CONVERGED |
| `CheckDecision` | 异步 srv | in `target_id`、in `tool_id`、in `level`(approach\|sleeve\|cut, approach)、in `min_model_revision`(uint64, 0) | GetDecision；not found → MODEL_STALE，valid_until 过期 → MODEL_EXPIRED；级别累进（cut 要求 approach∧sleeve∧cut） |
| `HarvestTarget` | 异步 action | in `target_id`、in `tool_id`、in `intent`(int) | intent=FULL → MODE_FULL，否则 MODE_PREGRASP_ONLY；超时取消并记 EXEC_TIMEOUT+需 ACK；结果 target_id 不符 → EXEC_FAILED+需 ACK；结果 `plan_only` 粘在本颗果上 |
| `RecordResult` | Sync | in `target_id` | 记成功；PREGRASP_ONLY 且 `ack_each_pregrasp` → 置待 ACK（`pregrasp_checkpoint`，plan-only 除外）；臂结果带 recovery_required → 置待 ACK |
| `RecordSkip` | Sync | in `target_id` | 按失败策略记 SKIPPED/FAILED/CANCELED + rework；RECOVER → 待 ACK（plan-only 只在臂结果 recovery_required 时）；STOP_BATCH → FAILURE |
| `WaitForAck` | Stateful | — | phase=WAITING_ACK，RUNNING 直到任务侧 ACK 已授予**且**臂侧 `recovery_required=false`（整棵树停在此处，不派下一颗） |
| `NeckRemeasurePending` | Condition | — | 本次尝试是颈部复测轮 → SUCCESS |
| `NoRecoveryPending` | Condition | — | 有待 ACK（如臂侧锁存中途拉起）→ 记 RECOVERY_REQUIRED 并 FAILURE（不重试，下一轮 WaitForAck） |
| `IsRecoveryRequired` / `BatchSettled` / `BatchGate` | Condition | — | 见树注释 |
| `IntentIs` | Condition | in `intent`(string) | |
| `SettleBatch` | Sync | in `reason`(string) | |
| `NoTargetRound` | Sync | — | 连续空轮 +1；达 `empty_survey_limit` → settle `no_targets` 并 FAILURE |
| `RetryOnPolicy` | Decorator | in `num_attempts`(unsigned, 2) | 子树失败按 session 失败策略：RETRY_VIEW 立即重试、WAIT 等 `wait_retry_s` 后重试、REMEASURE_NECK 立即进颈部复测轮（不计次数、不看单果时限）、其余/次数尽/超时/待 ACK 放弃 |
| `WithinTargetDeadline` | Decorator | — | 单果时限到 → halt 子树，记 TARGET_TIMEOUT（SKIP，rework `timeout`） |

### WaitForAck 机制

1. 故障（RECOVER 策略）、PREGRASP_ONLY 每颗检查点、或批次中臂侧 `/peach/manipulation/recovery_required`
   变 true → session 置 `recovery_required`，树在下一次循环开头停在 `WaitForAck`（在途果由 `NoRecoveryPending`
   截住，不下发 HarvestTarget，记 FAILED/`recovery`），`BatchState.phase=WAITING_ACK`、`blockers` 含
   `recovery_required`（臂侧锁存另加 `manipulation_recovery_required`）。
2. 操作员人工确认现场后调用 `/peach/task/acknowledge_recovery`（std_srvs/Trigger）。
3. 任务节点**异步转发**到 `/peach/manipulation/acknowledge_recovery`（幂等；延迟应答，不阻塞 executor；
   超时 `timeouts.ack_forward_s`）。只有臂侧返回 `success=true` 才授予任务侧 ACK；`WaitForAck` 还要等臂侧锁存
   回落为 false 才放行。锁存单独回落**不**放行（任务侧待 ACK 只由操作员清）。
   臂侧服务不在/失败/超时 → 原样把失败回给操作员，**不消耗任何状态**。
4. 无批次在等 ACK 时只转发（空闲时也能解除臂侧锁存）。臂侧锁存为 true 时拒绝 RunBatch。ACK 永不自动产生；
   取消批次可退出等待。

### plan-only

`HarvestResult.plan_only=true`（臂侧执行关闭，只规划）的果：不计 `attempted/succeeded/skipped/failed`，
计 `counts.plan_only`；账本 `targets[].plan_only=true`，rework kind `plan_only`（attempted=false）；
不触发 PREGRASP 检查点；只有臂结果 `recovery_required` 才要 ACK。`max_targets` 按 attempted+plan_only 计。
RunBatch.Result 无该计数字段，看 `results[].plan_only` 或账本。

### 失败策略（与 `FailureCode.msg` 注释一一对应，`core/batch_policy.cpp`）

| 策略 | FailureCode | 树内处理 | rework kind |
|------|-------------|----------|-------------|
| RETRY_VIEW | 11 EXACT_TF_MISSING, 12 LOW_QUALITY, 21 MODEL_STALE, 22 MODEL_EXPIRED, 32 PLAN_FAILED, 43 PREGRASP_RESIDUAL, 53 CUT_NOT_CONFIRMED | 整个 HarvestOne 重来（≤ retry_attempts），尽则 SKIPPED | perception / model / planning / contact_failed / tool_fault |
| SKIP | 10 NO_TARGET, 13 OUT_OF_SCOPE, 20 NOT_CONVERGED, 30 NO_IK, 31 COLLISION, 33 CARTESIAN_INCOMPLETE, 42 CONTACT_ABORT | SKIPPED，下一颗 | perception / model / unreachable / contact_failed |
| SKIP_TOOL | 23 BUDGET_RADIAL_NEGATIVE, 55 TOOL_NOT_FEASIBLE | SKIPPED，下一颗 | tool |
| APPROACH_ONLY | 24 BUDGET_AXIAL_NEGATIVE, 25 BUDGET_STRUCTURAL, 26 NECK_REMEASURE_MISMATCH | SKIPPED（FULL 不下刀） | approach_only |
| WAIT | 27 SWING_TOO_LARGE, 64 ENVIRONMENT_UNSAFE | 等 wait_retry_s 后重试，尽则 SKIPPED | swing / environment |
| REMEASURE_NECK | 28 NECK_REMEASURE_PENDING | 仅 FULL：同一 HarvestOne 内 ObserveTarget(neck_remeasure=true) → CheckDecision(cut) → HarvestTarget；每次常规尝试后至多一次，不计 retry 次数；再次 28 或 PREGRASP_ONLY → SKIPPED | neck_remeasure |
| RECOVER | 40 EXEC_FAILED, 41 EXEC_TIMEOUT, 44 RETREAT_FAILED, 50–52, 54 TOOL_FAULT, 63 RECOVERY_REQUIRED；或结果 recovery_required=true | FAILED，树停在 WaitForAck | recovery |
| SKIP（任务） | 45 TARGET_TIMEOUT | 单果时限到（WithinTargetDeadline），SKIPPED | timeout |
| STOP_BATCH | 60 SAFETY_GATE_CLOSED, 61 ROBOT_NOT_READY, 62 CANCELED, 65 DEPENDENCY_UNAVAILABLE | FAILED，批次中止 | safety / canceled / infrastructure |

未列出的码按组兜底：4x/5x → RECOVER（臂/刀状态未知），6x → STOP_BATCH，其余 → SKIP。
对端服务缺失/拒绝/无结果/请求非法一律记 65 DEPENDENCY_UNAVAILABLE。

### 选果（`core/selection`）

资格（显式 target_ids 也必须通过）：集合已锁定、id 在 `locked_target_ids` 内（不在 → `not_in_locked_set`；
锁定但无观测 → `locked_not_observed`）、未 claim、confirmed、CATEGORY_BAG、有 bottom/neck 几何、
不贴图像边、mask_quality / depth_coverage ≥ 阈值、相机距离在深度窗内（0=未知不过滤）。
排序键（首键优先）：显式名单位置 → 已知可达先于未知 → 高度分档（`height_band_m`）低者先（套筒自下沿袋轴接近，
下方袋在上方袋的接近走廊里，先摘下方清走廊）→ 相机距离近者先（深度 σ∝z²）→ ROI 面积大者先
（近距双检取大框，旧 batch.py:196-198）→ id。已知不可达永不选；`require_reachability=true` 时未知也不选。

### 账本

`<runs_dir>/<request_id>/ledger.json` 与 `rework.json`，写 `<file>.tmp` → fsync → rename。空 request_id →
`auto_<UTC yyyymmddThhmmss_微秒>Z`；非法字符拒绝（从不改写）；目录已存在则拒绝 goal（不续跑、不合并）。
`runs_dir` 空 → `$PEACH_RUNS_DIR`，再空 → `<cwd>/runs`。每颗果落定、开批、BeginScene、可达性查询、收批都整份重写 ledger。
schema `peach2_task/ledger/2`：`scene_epoch`、`counts.plan_only`、`targets[].plan_only`、
`reachability.<id>.{reachable, failure_code, failure_name, reason}`（最后一次查询）；不可达未尝试的果 rework reason
`check_reachability:<NAME>[:<reason>]`。

## 参数（generate_parameter_library：`params/peach2_task_parameters.yaml`；部署值 `config/peach2_task.param.yaml`）

`tree_file`、`tick_hz`(50)、`runs_dir`、`default_tool_id`（launch 传 tool_profile；goal 给了不同工具 → 拒绝）、
`empty_survey_limit`(2)、`per_target_timeout_s`(60，goal 0 时用)、`observe_max_views`(3)、`retry_attempts`(2)、
`wait_retry_s`(5)、`ack_each_pregrasp`(true)、`photo_pose`(global_photo_pose)、
`safety.{require_robot_status, robot_status_max_age_s}`、
`selection.{require_reachability, min_mask_quality, min_depth_coverage, depth_min_m, depth_max_m, height_band_m}`、
`timeouts.{server_wait_s, move_to_s, snapshot_s, begin_scene_s, lock_wait_s, reachability_s, observe_s, decision_s, harvest_s, ack_forward_s}`、
`bond_heartbeat_timeout_s`。批次开始时快照参数，批中改参下一批生效。

RunBatch 拒绝条件：节点非 Active、已有批次、臂侧 `recovery_required` 锁存中、intent 非法、request_id 非法或目录已存在、target_ids 含空串、
非 SURVEY_ONLY 且无工具、工具与挂载工具不符、限值非法（ratio∉[0,1]、timeout<0）。拒绝原因写日志与
`BatchState.message`。

## 用法（手工；launch 不自动发）

```bash
ros2 launch peach2_task peach2_task.launch.py hardware_mode:=mock tool_profile:=adaptive_shear_v1
# lifecycle 由整栈 lifecycle manager 驱动；单独调试可手动：
ros2 lifecycle set /peach2_task configure && ros2 lifecycle set /peach2_task activate

# 使能（默认全 false；PREGRASP_ONLY 只需 execution）
ros2 service call /peach/task/set_enables peach2_interfaces/srv/SetEnables \
  "{execution: true, grasp: false, tool: false}"

# 发批次（intent 0=SURVEY_ONLY 1=PREGRASP_ONLY 2=FULL）
ros2 action send_goal --feedback /peach/task/run_batch peach2_interfaces/action/RunBatch \
  "{request_id: 'field_pregrasp_001', intent: 1, tool_id: '', target_ids: [], max_targets: 3, \
    per_target_timeout_s: 0.0, target_harvest_ratio: 0.0}"

# 等 ACK 时（BatchState.phase=5 WAITING_ACK），人工确认现场后
ros2 service call /peach/task/acknowledge_recovery std_srvs/srv/Trigger
```

取消：`send_goal` 终端 Ctrl+C（action cancel）→ haltTree、在途子 goal 取消、账本记 CANCELED。
这是应用停轨，**不是急停**；硬件急停在示教器/柜，不经 ROS。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --base-paths src/peach2 --packages-select peach2_task \
  --packages-skip peach2_interfaces peach2_core --build-base build/v2/peach2_task --install-base build/v2/peach2_task_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --base-paths src/peach2 --packages-select peach2_task \
  --packages-skip peach2_interfaces peach2_core --build-base build/v2/peach2_task --install-base build/v2/peach2_task_install
colcon test-result --test-result-base build/v2/peach2_task --verbose
```

gtest：`test_selection`、`test_batch_policy`、`test_enables_policy`（含 safety gate）、`test_ledger`、
`test_batch_session`、`test_msg_mirror`（纯核镜像常量对 IDL + 消息转换）、`test_harvest_batch_tree`
（真 `harvest_batch.xml` + 真逻辑节点 + 纯 C++ mock 叶子 + 假时钟：全成功、max_targets/采收率/名单收批、
skip、RETRY_VIEW、WAIT、PREGRASP 检查点 ACK、RECOVER ACK、取消、失去使能 halt、空勘察上限、重勘新目标、
SURVEY_ONLY、STOP_BATCH、不可达+reason、单果超时 TARGET_TIMEOUT、BeginScene 仅首巡/clear_previous 真→假、
BeginScene 拒绝中止、锁定集限选、颈部复测→cut 成功、复测不受已耗重试限制、复测 MISMATCH/再 PENDING 跳过、
PREGRASP_ONLY 不复测、plan-only 不计 attempted、臂侧锁存阻断直到 ACK+回落、DEPENDENCY_UNAVAILABLE 中止）。
xmllint 需联网（colcon test 用 full_network）。不起任何 ROS 进程。

文档：本 README；无 rosdoc2 配置。许可：BSD-3-Clause。

## 接口需求（已由 peach2_interfaces 变更 01 解决）

acknowledge_recovery 两侧与幂等、`recovery_required` 锁存话题、TARGET_TIMEOUT / DEPENDENCY_UNAVAILABLE /
NECK_REMEASURE_PENDING、`locked_target_ids`、`CheckReachability.reasons`、`HarvestResult.plan_only`、
`BeginScene`、TargetObservation frame=base_link 均已进 IDL。仍开放：`/aubo_io_controller/robot_status`
（驱动话题）未列入 `interfaces.yaml`，本包按 spec §1 订阅。

## 已知限制与 TODO

- TODO(M0) 真机标定：`selection.*` 阈值、`height_band_m`、各 `timeouts.*`、`per_target_timeout_s`。
- `VerifyHarvest`（§8.2 回拍照位确认袋消失）未实现；当前成功以 HarvestTarget 结果为准。
- 暂停（PAUSED）未实现：BatchState.PAUSED 不会出现；停批用 action cancel。
- 风/光环境检查无传感器：CheckSafety 不判 ENVIRONMENT_UNSAFE（只作为下游返回码的 WAIT 策略）。
- 重勘不再 BeginScene（同 epoch），target_id 稳定性仍依赖 perception 跟踪器在同一 epoch 内不换 id；
  若他方中途 BeginScene（epoch 变大），本批以 `scene_epoch_changed` 中止而不是用新 id 续摘。
- 臂侧锁存中途拉起时在途果记 FAILED/`recovery`（即使尚未下发 HarvestTarget），作返工而非当成功或跳过。
- 颈部复测轮不受单果时限约束（臂停在 pregrasp，只有一次 observe+decision）；靠 `timeouts.observe_s` /
  `decision_s` 兜底。
- ObserveTarget 被 halt 时只发 cancel 不等待服务端结束；target_model 若是单槽服务，紧接的下一 goal 可能被拒
  （记 PERCEPTION_NO_TARGET 跳过）。
- 单果时限只罩 observe+decision，不抢占 HarvestTarget（接触中途放弃比完成更危险，由 `timeouts.harvest_s` 兜底）。
- bond 与 nav2 lifecycle_manager 的接线在整栈 bringup 包里做；本包只在 activate 起 bond。
