# 桃子自动采摘联动、Web 调试与接口手册

## 软件边界与启动

`peach_harvest_orchestrator` 是批次唯一所有者；`peach_approach_grasp` 只执行一个稳定
`target_id`；感知负责身份与全局目标收齐锁定；重建负责有序会话；Web 只代理类型化命令。
业务栈不包含 AUBO 上电、松刹车或安全恢复，也未修改冻结驱动。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch peach_harvest_orchestrator harvest_system.launch.py
```

Web 默认 `http://127.0.0.1:8090`。真机上电由现场人员操作；首次运动速度/加速度缩放不高于
0.1。

## 真机作业检查单（批次启动前逐项确认）

1. 上电/松刹车由现场人员手动完成（软件栈不含上电入口，`auto_power_on=false` 不变）。
2. 急停、限位、碰撞等级、低速模式现场确认就位；首次运动速度/加速度缩放 ≤0.1。
3. 速度纪律（能力端双档，勿随意调低）：
   - 接触段 MTC `moveit.velocity_scaling=0.05`；自由空间转移
     `moveit.transit_velocity_scaling=0.10`；
   - 低速档不得让高重力矩姿态持续过久——0.01 蠕行曾在高重力矩姿态持续过久而
     关节过流；若姿态不得不久留，优先改路径而非继续压速度。
4. 每次开机执行网卡/频率治理（不持久）：
   `sudo ethtool -K enp130s0 gro off gso off tso off` +
   `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`。
5. 启动前查旧进程防设备独占：`pgrep -af 'ros2 launch|component_container|ros2 run'`。

## 碰撞环境现状与静态碰撞体评估（阶段 F2 结论）

当前 MoveIt 规划环境为空（无 octomap、无静态碰撞体），依赖视点侧剔除
（`scan.min_camera_height_m` 地面平面 + `scan.protected_zones` 轴对齐盒）与
MTC 入口保护区拒绝兜底。评估结论与可选路线：

- **不动冻结栈可实现**：新增独立「场景对象发布节点」（非冻结包，如放
  `peach_approach_grasp` 或 bringup 侧新小节点），启动时经 PlanningScene 差分
  消息注入静态盒（树干/支柱/地面台架）；move_group 的 planning scene monitor
  已开 `publish_geometry_updates`，无需改 `aubo_e5_moveit_config` 任何文件。
- **不建议**：改 URDF/SRDF 加环境几何——`aubo_description` 属驱动栈契约面，
  环境属于部署差异而非机器人本体，混进描述包会让仿真/真机/不同果园共用
  同一模型时互相污染。
- 待授权项：若后续要 octomap 点云在线避障，需评估 move_group 传感器插件与
  感知点云带宽，另行立项；本阶段不实施。

## 批次流程

```
WAITING_READY
  → 四路就绪门通过 → DISCOVERY
  → 拍照前置（execution 使能且 photo_pose.enabled 时；否则按策略跳过）：
      go_to_photo_pose 移动到 SRDF 命名状态 global_photo_pose
      → reset_global_targets 重置感知收齐窗口
  → 感知收齐锁定：窗口内累积全部确认目标（含 REOBSERVE），窗口关闭一次性锁定
    （空集也锁定，target_count=0）
  → RUNNING：按固定优先级逐目标派发 RunTargetCycle goal
      · SUCCEEDED / SKIPPED_QUALITY / SKIPPED_UNREACHABLE / FAILED → 记账并自动推进
      · 操作员“跳过当前”（SKIP_TARGET）→ 取消活动 goal，记 CANCELED 并推进
      · 接触段 recovery_required → RECOVERY_REQUIRED，批次停等人工确认
  → 本轮锁定集合耗尽 → 复扫判定（decide_round 纯函数）：
      · 本轮摘到目标且未达 max_rounds → 回拍照位姿开启新一轮（物理递减集）
      · 锁定空集 / 达到 max_rounds / 关闭复扫 → complete_batch
  → COMPLETED →（photo_pose.return_on_complete）best-effort 再回一次拍照位姿
  → RunHarvest goal 携带 HarvestSummary 返回（succeed）
```

自动开始（`auto_start_enabled=true`）下，无 RunHarvest goal 也会按同一流程走批次；
RunHarvest 只是带结账返回的长时入口，不改变状态机走向。

## 自动流程与状态

四路就绪门（`readiness.timeout_s` 内新鲜为通）：

- **perception**：感知观测流新鲜即可，**不要求已锁定**——锁定由派发前置保证，收齐
  窗口期不会卡回 WAITING_READY；
- **reconstruction**：重建诊断流新鲜；
- **motion**：单目标 action server 在线，且（`readiness.require_robot_status=true` 时）
  RobotStatus 未急停、未故障、已上电、可运动；
- **web**：静态开关 `readiness.web`。

全就绪且 `auto_start_enabled` 时自动进入 DISCOVERY。目标派发另有五项前置
（`allow_dispatch` 纯函数）：拍照前置完成、感知 selected 非空、非重复派发、无活动
目标、能力端 action server 就绪。

失败或接触结果不确定进入 `RECOVERY_REQUIRED`，不自动恢复：现场人工撤离确认后
ACKNOWLEDGE_RECOVERY（保持暂停）再 RESUME。安全暂停在检查点生效（PAUSE_PENDING →
PAUSED）；立即取消（CANCEL_NOW）独立，直接取消活动 goal 并进 INTERRUPTED。维护模式
释放自动所有权，Web 才允许调用旧 Trigger 调试服务。

## 收齐锁定与优先级

感知侧 `GlobalHarvestPlan` 采用**收齐式窗口锁定**：`reset_global_targets`（或启动）后
进入收齐窗口，逐帧把已确认目标（含 REOBSERVE，不限于可选择目标）并入累积集，同 ID
后者覆盖。窗口关闭条件（先到先关）：

- 累计 ≥ `harvest.min_collect_frames` 帧 **且** 连续 `harvest.lock_settle_frames` 帧
  无新增确认 ID（集合静止）；
- 超过 `harvest.max_collect_s` 秒（超时强制关闭兜底）。

窗口关闭时对累积集一次性排序、按 `target_memory.max_targets` 截断并锁定；**空集也
锁定**（`target_set_locked=true`、`target_count=0`，编排器据此判定批次完成）。锁定后
新出现的 ID 不再入集，直到下一轮 reset。

优先级排序键（升序最优先）：

```
status（ACCEPT 先于 REOBSERVE，REJECT 垫底）
  → camera_distance_m（先近后远）
  → base_height_m（先低后高，harvest.priority_prefer_lower_first=false 时不参与）
  → -confidence（同距同高置信度高者优先）
  → target_id（字典序 tie-break，保证跨帧稳定）
```

设计依据：先近后远——先清外围目标，减少深入冠层时的碰枝与自遮挡；先低后高——避免
摘高处目标时碰落低处果实。该采收顺序经 Xiong 草莓采摘机与 SWEEPER 甜椒采摘项目实证。

## 单目标终态分级

能力端（`peach_approach_grasp`）内部以 `CycleState` 枚举流转周期状态，action Result
按终局分级返回 outcome；编排器按 outcome 记账并路由：

| outcome | 语义（产生点） | 编排器行为 |
|---|---|---|
| `SUCCEEDED` | 周期圆满：含只规划档 `PLAN_READY` 与未使能抓取档 `READY_FOR_GRASP` | 记 `succeeded`，自动推进下一目标 |
| `SKIPPED_QUALITY` | finalize 后最终质量门失败（RMSE/inlier/ID 不一致等） | 记 `skipped_quality`，自动推进 |
| `SKIPPED_UNREACHABLE` | 视角均不可达、扫描上限未收敛、MTC 规划阶段失败（未启动执行） | 记 `skipped_unreachable`，自动推进 |
| `FAILED` | 执行失败（含 MTC 执行已启动后的失败） | 记 `failed`，自动推进 |
| `CANCELED` | 操作员 SKIP_TARGET 主动跳过（goal 取消落地后记账） | 记 `canceled`，自动推进 |

`recovery_required=true`（接触轨迹已下发后取消/撤离失败等）不进账：批次直接进
`RECOVERY_REQUIRED` 等人工，恢复确认前保留派发锁防止无限重派同一故障目标。

周期推进权归属：action 驱动的周期由编排器在终态后统一调
`complete_selected_target` 推进感知计划（锁外 RPC）；手动 Trigger 周期没有编排器，
仍由能力端自推进兜底，两条路径不会双写。

## 控制命令

`/peach_harvest_orchestrator/control`（`ControlHarvest`，带幂等 request_id 与
expected_revision 乐观锁）：

| 命令 | 行为 |
|---|---|
| `PAUSE` | 目标周期运行中先 PAUSE_PENDING，到安全检查点落 PAUSED；recovery 期间拒绝 |
| `RESUME` | 恢复 AUTO，按就绪度回 DISCOVERY/WAITING_READY；recovery 期间拒绝 |
| `ENTER_MAINTENANCE` / `EXIT_MAINTENANCE` | 释放/收回自动所有权；退出维护保持暂停 |
| `CANCEL_NOW` | 状态机清活动目标与运行标志、进 INTERRUPTED 转 PAUSED；节点层向能力端传播取消活动 goal（真取消，非仅置标志） |
| `SKIP_TARGET` | 仅活动目标时接受：登记跳过意图并取消当前 goal；取消落地后记 `CANCELED`（“操作员跳过”）并推进感知计划 |
| `ACKNOWLEDGE_RECOVERY` | 现场人工确认已安全撤离后解除 recovery 锁定，保持 PAUSED |

## 复扫轮次

本轮锁定集合全部处理完（或锁定空集）后，纯函数 `decide_round` 判定去向：

- 未锁定或本轮已处理数 < 锁定数：`WAIT`（继续等，selected 暂空可能只是质量待恢复）；
- 本轮锁定空集：`COMPLETE`（“本轮未锁定到目标，批次完成”）；
- 本轮摘到目标、`harvest.rescan_until_empty=true` 且 `round < harvest.max_rounds`：
  `RESCAN`——回拍照位姿、重置感知、收齐锁定新一轮；
- 达到 `max_rounds` 或复扫关闭：`COMPLETE`（message 注明原因）。

**残局抬质量（协议 2.8，阶段 E3）**：进入上述 RESCAN/COMPLETE 判定与停滞计时
之前，若锁定集中还有最新终局账为 `SKIPPED_QUALITY`、抬质量次数未达
`harvest.observe_retry_max` 且本轮未抬过的目标（显式批次限清单内目标），编排器
按优先级逐个派发 `mode=OBSERVE_ONLY` 的 RunTargetCycle（只观察+精化验证不抓
取）。抬质量期间不算"停滞"（stall 计时挂起）；终局只进审计账（reason 标
observe_retry，不动 counters/attempted）、不推进感知计划；目标因质量改善重新
可选后回正常派发链以 FULL 重试。候选耗尽仍无可选目标才恢复停滞计时与轮次判定。

`round` 从 1 起计，`max_rounds` 为总轮次上限（含首轮，默认 3）。复扫是**物理递减
集**：已摘目标不在树上，新一轮收齐自然不再包含；因遮挡/质量未入集的目标则有机会
在新一轮被锁定。复扫轮即使无需移动机械臂（execution 关），也必须重置感知以开启新
一轮收齐。

## RunHarvest 与批次摘要

`/peach_harvest_orchestrator/run_harvest`（`RunHarvest` Action）为真实现：

- **单 goal 守卫**：recovery 未确认、blockers 非空、目标周期运行中、非 AUTO 模式或
  已有活动 RunHarvest goal 时拒绝新 goal；
- **终结三路径**（均携带 HarvestSummary）：批次 COMPLETED → `succeed`
  （termination_reason=“批次完成”）；recovery_required → `abort`；客户端取消 →
  级联取消活动单目标 goal（最多等 5s 落安全检查点）→ `canceled`（“批次已取消”）；
- COMPLETED/INTERRUPTED 后接受新 goal 会先 `reset_batch` 清账重开一轮；反馈每 200ms
  携带最新 HarvestState。

`HarvestSummary` 字段：

| 字段 | 含义 |
|---|---|
| `run_id` | 批次 ID（goal request_id，缺省自动生成 `run-<ns>`；无 goal 的自动批次为 `auto-<ns>`，首个拍照前置启动时生成） |
| `discovered` | 本批次累计发现目标数（各轮锁定沿累加；首轮 reset 清零重计） |
| `attempted` | 已处理目标数（全部终态合计） |
| `succeeded` / `skipped_quality` / `skipped_unreachable` / `failed` / `canceled` | 各终态计数 |
| `elapsed` | 批次墙钟时长（RunHarvest 批次自 goal 受理起计；auto_start 自动批次自首个拍照前置启动起计） |
| `outcomes[]` | 逐目标记录（`target_id`、`outcome`、`reason`、单目标 `elapsed`=派发→终局墙钟、`quality_score` 质量占位分：成功=1.0/降级=0.5/其余=0，待阶段 E 接真实质量分） |

批次吞吐 targets/hour = attempted/elapsed_hours 随批次终局事件
（`batch_completed`/`batch_canceled`/`batch_aborted`）的 message 透出（HarvestSummary
无吞吐字段且消息契约冻结）；单目标阶段耗时分解（能力端 Result
`stage_names`/`stage_durations`）拼进 `target_succeeded` 等终局事件 message。

状态话题的 `progress` = attempted / discovered（0 除保护，超界钳到 1），Web 据此渲染
批次进度条。

## 全局拍照位姿

真机记录的全局拍照位姿已固化为 SRDF 命名状态 `global_photo_pose`
（`aubo_e5_moveit_config/config/aubo_e5.srdf`）；`orchestrator.yaml` 中的
`global_photo_joints` 仅存档对照，实际移动走命名状态。

能力端服务 `~/go_to_photo_pose`（Trigger，编排器拍照前置的唯一运动入口）：

- 守卫：MoveIt 已初始化、无接触段 recovery、无运行中周期（running_ 原子占位）、
  安全门（机器人状态新鲜且无急停/错误/断电）；
- 规划：`setNamedTarget(photo_pose_named_target)` → Pilz `PTP`，失败回退 OMPL；
- `execution.enabled=false` 时仅规划即返回成功（不发送运动）；执行前再次复核安全门
  （规划耗时数秒，期间现场可能拍急停）。

编排器侧 PhotoStep（节点侧状态，不进状态机枚举）：
`NOT_STARTED → MOVE_PENDING → MOVING → RESET_PENDING → RESETTING → DONE`，两步均为
异步调用由定时器轮询，锁内绝不阻塞等 RPC。任一步失败冷却
`photo_pose.retry_cooldown_s` 后重试；连续 `photo_pose.max_retries` 次失败进入
`RECOVERY_REQUIRED`（确认恢复后从头走完整拍照流程）。单次调用超时
`photo_pose.service_timeout_s`（默认 90s——真机移动到拍照位姿耗时数十秒，不得按普通
服务超时设置）。首轮且无需移动时拍照前置直接完成（不打扰感知）；批次完成后
`photo_pose.return_on_complete=true` 会 best-effort 再回一次拍照位姿，结果仅记事件，
不影响批次结论。

## Web 操作关系

- 批次：安全暂停、恢复自动、进入/退出维护、**跳过当前**（SKIP_TARGET，二次确认，
  仅目标周期运行中可用）、立即取消、确认恢复。
- 渲染：批次状态与 `target_phase` 中文名、批次进度条（`state.progress`）、批次
  COMPLETED 高亮、blockers 健康门标签。
- 策略：自动开始、规划执行、抓取动作、工具 IO；依赖固定为
  `execution → grasp → tool`，周期中拒绝修改。
- 感知：查询计划、完成目标、重置目标。
- 重建：开始、采集、移除末帧、完成、保存、查询、重置。
- 靠近抓取：查询、预览、完整接触、启动、取消、恢复确认、Arm/Disarm。

写请求校验同源、HttpOnly SameSite Cookie、业务 revision 和维护模式；重置、立即取消、
Arm、接触动作及开启运动策略需要二次确认，审计缓存保留最近 200 条。网关已拆分为
`state.py`（线程安全最新值缓存）与 `http_server.py`（零 ROS 的 HTTP 层，Handler
窄接口 + 安全门禁路由）；监听非 loopback 地址时打醒目 warning——服务无登录鉴权，
只应暴露在可信采摘局域网。

## 类型化接口

| 名称 | 类型 | 含义 |
|---|---|---|
| `/peach_harvest_orchestrator/state` | `HarvestState` | 状态、阶段、阻塞、使能、revision、run/cycle id、progress |
| `/peach_harvest_orchestrator/events` | `HarvestEvent` | 过程与审计事件（severity + run_id/cycle_id/target_id） |
| `/peach_harvest_orchestrator/run_harvest` | `RunHarvest` Action | 长时批次入口、反馈、取消与 HarvestSummary |
| `/peach_approach_grasp_node/run_target_cycle` | `RunTargetCycle` Action | 单目标运动能力（outcome 分级） |
| `/peach_harvest_orchestrator/control` | `ControlHarvest` | 暂停、恢复、维护、取消、跳过、恢复确认 |
| `/peach_harvest_orchestrator/set_operation_policy` | `SetOperationPolicy` | 原子三级使能 |
| `/peach_approach_grasp_node/go_to_photo_pose` | `Trigger` | 移动到全局拍照位姿（编排器拍照前置调用） |
| `/peach_pose_node/reset_global_targets` | `Trigger` | 重置感知收齐窗口（每轮拍照后置调用） |
| `/peach_pose_node/clear_target_memory` | `Trigger` | 清空感知世界系身份记忆（`harvest.fresh_scene=true` 时批次首轮在重置收齐之前调用） |
| `/peach_pose_node/complete_selected_target` | `Trigger` | 推进感知固定优先级计划（目标终态后调用） |
| `/peach_pose_node/reopen_target` | `ReopenTarget` | 重开已终局目标恢复可选（E3 残局抬质量：OBSERVE_ONLY 派发前调用，使重建绑定精化；仅自动批次，best-effort 不熔断） |

跨包接口名由编排器参数化（`target_cycle_action_name`、`approach_node_name`、
`complete_target_service_name`、`reset_targets_service_name`、`photo_pose_service_name`、
`clear_memory_service_name`、`reopen_target_service_name`），
默认值即现网名，launch 不需要改。旧 Trigger 与 JSON 话题保留一个迁移周期；新客户端
使用类型化接口。

## 参数

编排器（`peach_harvest_orchestrator/config/orchestrator.yaml`）：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `auto_start_enabled` | `true` | 四路全就绪后自动进入 DISCOVERY |
| `execution_enabled` / `grasp_enabled` / `tool_enabled` | `false` | 三级操作策略初始值（交付默认全关） |
| `readiness.web` | `true` | Web 就绪门（静态开关） |
| `readiness.timeout_s` | `2.0` | 四路输入的新鲜度配置下限（s）；运行期阈值自适应为 max(本值, 2.5×实测发布周期EMA)（2.11，节点真死时超 2.5×EMA 判失连） |
| `readiness.require_robot_status` | `true` | motion 路是否要求 RobotStatus 正常 |
| `global_photo_joints` | 见 YAML | 拍照位姿关节角存档（仅对照，运动走 SRDF 命名状态） |
| `photo_pose.enabled` | `true` | 拍照前置开关；execution 关时自动跳过移动 |
| `photo_pose.max_retries` | `3` | 拍照前置连续失败此次数后进 RECOVERY_REQUIRED |
| `photo_pose.retry_cooldown_s` | `5.0` | 拍照前置失败重试冷却（s） |
| `photo_pose.service_timeout_s` | `90.0` | 单次 go_to_photo_pose / reset_global_targets 调用超时（s） |
| `photo_pose.return_on_complete` | `true` | 批次完成后 best-effort 再回一次拍照位姿 |
| `harvest.preflight_check` | `true` | 批次启动就位自校（F3）：首轮进入拍照前置前检查 TF 外参链（base_link→camera_link）可查 + robot_status 已收且无故障/急停（robot 检查跟随 `readiness.require_robot_status`）；未通过则挂起拍照前置、发 `preflight_failed` ERROR 事件（列清失败项，签名变化或每 5s 节流）、state blockers 计 `preflight`；幂等门——refresh 每拍重试不熔断，就位后自动放行（`preflight_passed` 事件）；复扫轮次不重复自校；`false` 整体跳过（旧行为） |
| `harvest.rescan_until_empty` | `true` | 复扫递减集循环开关 |
| `harvest.max_rounds` | `3` | 总轮次上限（round 从 1 起计，含首轮） |
| `harvest.collect_timeout_s` | `40.0` | 收齐窗口编排侧总超时配置下限（s）：拍照前置完成后迟迟不锁定超限则发 round_stall 并按本轮处理完重判轮次；运行期上限自适应 max(本值, 1.5×实测上次窗口时长) |
| `harvest.fresh_scene` | `false` | 换场景/换果树批次置 true：首轮开窗前（reset_global_targets 之前）先调 clear_target_memory 清感知世界系身份记忆；仅首轮生效，失败与 reset 同级走拍照前置重试链 |
| `harvest.observe_retry_enabled` | `true` | 残局抬质量开关（2.8/E3）：RESCAN/停滞判定前对 SKIPPED_QUALITY 残局目标发 OBSERVE_ONLY 周期抬质量；抬质量期间不算停滞；成功重新可选后回正常链 FULL 重试 |
| `harvest.observe_retry_max` | `2` | 每目标每批次抬质量次数上限（派发时消耗，拒绝/取消不归还；每轮每目标至多一次） |
| `target_cycle_action_name` 等 6 个接口名 | 现网名 | 跨包 action/service/节点名参数化 |

能力端（`peach_approach_grasp/config/approach_grasp.yaml`）新增：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `photo_pose_named_target` | `'global_photo_pose'` | `~/go_to_photo_pose` 的 SRDF 命名状态 |
| `scan.protected_zones` | `[]` | 环境几何保护区（F1）：base_link 系轴对齐盒列表，stride-6 扁平编码 `[xmin,ymin,zmin,xmax,ymax,zmax]*N`（米）；视点生成剔除相机位置落入任一盒（闭区间含表面）的候选，MTC 接触段入口点（含降级链入口）落入即拒规划，终局 SKIPPED_UNREACHABLE；畸形盒（长度残余组/非有限分量/min>=max）装载时告警并逐盒丢弃；与 `scan.min_camera_height_m`（“z<下限半空间”特例 shortcut）并存各自生效 |

感知（`peach_pose_ros2/config/peach_pose.yaml`）新增：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `harvest.min_collect_frames` | `10` | 收齐窗口最少累积帧数：达到后才允许按静止条件关闭窗口 |
| `harvest.lock_settle_frames` | `5` | 连续无新增确认 ID 的帧数，与最少帧数联合判定集合稳定 |
| `harvest.max_collect_s` | `8.0` | 收齐窗口最长时长（s），超时强制关闭兜底（空集也锁定） |
| `harvest.priority_prefer_lower_first` | `true` | 优先级启用高度键（先低后高）；false 则高度不参与排序 |

## 并发、档案与交付默认值

感知使用容量 1 的最新帧 worker；全局计划由节点级 RLock 统一保护（worker/executor
双线程写路径单写者化）。重建使用容量 3 的串行单写者 worker，满载拒绝新帧，节点级
RLock 把 TSDF 积分收敛为单线程，保证 ICP/TSDF 不并发。编排器锁内不阻塞：action 回调
锁内改状态、锁外推进感知 RPC；策略两阶段提交不持锁跨 RPC；拍照/复扫在途调用只
轮询不等待。命名档案位于 `peach_profiles/`，原子写入且防路径穿越。自带
`sim_safe` 与 `production_disabled` 均关闭运动、抓取和工具 IO；档案不能操作上电或安全
回路。
