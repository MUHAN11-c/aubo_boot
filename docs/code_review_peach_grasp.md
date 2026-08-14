# peach 自动抓取链路代码审查报告

> 审查日期：2026-08-13；审查人：AI 编码代理（deepseek-v4-pro）。
> 范围：`peach_harvest_orchestrator`、`peach_approach_grasp`、
> `peach_pose_ros2`、`peach_reconstruction_ros2`、
> `peach_perception_web` 的源码、参数与测试（只读审查）。
> 维度：逻辑正确性 / 状态机 / ROS 2 最佳实践 / 可扩展性 / 可读性 / 现代设计标准。
> 结论分级：**总体良好**——纯核分层与状态机设计属本项目最高水准；主要短板集中在
> 编排器**节点层**的边角路径、一条跨包状态通道缺失与跨包 JSON 字符串契约。

## 0. 总评

**良好（B+，架构 A- / 节点层 B）**。值得肯定的核心设计：C++/Python 两侧一致的
"纯核零 ROS + 薄壳接线"分层、终局判定只认枚举、RPC 绝不持锁、transient_local 闩锁
+ 1Hz 心跳解决启动顺序、参数 YAML 权威。纯核单测扎实（编排器状态机 185 断言）。
主要问题集中在：编排器节点层（1110 行单文件）存在 1 个可复现死锁路径、1 个状态
通道缺失（target_phase 反馈），以及机器人失能时批次继续派发的设计缺口；跨包
`std_msgs/String` JSON 契约无 schema 是全链最脆弱的接口。

---

## 1. 严重问题（3）

### R1. 能力端连续拒绝 → 恢复确认后批次永久停摆（死锁）

**位置**：`src/peach_harvest_orchestrator/src/harvest_orchestrator_node.cpp`
goal_response_callback（约 L390-410）、dispatch_target_locked（约 L370-390）；
`src/peach_harvest_orchestrator/src/state_machine.cpp` allow_dispatch (L68-75)。

**证据与推演**：
1. goal 被能力端拒绝且 `consecutive_rejections_ >= 6`（硬编码）时，
   节点调 `machine_.require_recovery(...)`（L401），但**不清
   `last_dispatched_target_`、不动 `photo_step_`**；
2. 现场 `ACKNOWLEDGE_RECOVERY`（状态机 → PAUSED）→ `RESUME`
   （→ DISCOVERY）。此时 `photo_step_ == DONE`、感知 selected 未推进、
   `last_dispatched_target_ == selected_target_id_`；
3. `allow_dispatch` 的第三项 `selected != last_dispatched` 恒 false，
   dispatch_target_locked 永不通过；没有任何节点路径会再清这个锁（只有
   targets_callback 的 selected 变化、goal 拒绝重试分支、accepted_callback 三处清）。
4. 无 RunHarvest goal 的自动批次因此**永久卡在 DISCOVERY**；唯一出口是重新发
   RunHarvest goal（accepted_callback L953 复位）。

**影响**：自动运行场景（交付默认 auto_start=true）下，能力端连续拒绝 6 次目标
（如能力端缓存持续不同步/重启后半同步状态）后，批次需要人工 CANCEL_NOW +
新 goal 才能继续；期间不产生任何告警事件。

**建议**：require_recovery 路径（或 ACKNOWLEDGE/RESUME 节点侧处理）复位
`last_dispatched_target_` 与 `dispatch_retries_`；把 6 提为参数
`dispatch.max_consecutive_rejections`；并为该路径补一个节点层回归测试。

### R2. RunTargetCycle 反馈从不填写 target_phase：目标阶段管线整体失效

**位置**：`src/peach_approach_grasp/src/cycle_action.cpp` executeAction
反馈循环（约 L95-115）；`harvest_orchestrator_node.cpp` feedback_callback
（L398-402）；Web `peach_perception_web/web/app.js` L24/92/93。

**证据**：能力端反馈只填 `target_id/action_active/三个使能/message`，
`feedback->state.target_phase` 恒为 0（TARGET_IDLE）——grep
`target_phase` 在 peach_approach_grasp/src 下 0 命中。编排器
feedback_callback 把 phase 直接映射进状态机，于是 begin_target 设置的 SELECTING
在首个反馈（200ms）即被打回 IDLE，此后 OBSERVING/FINALIZING/APPROACHING/
TOOL_ACTION/RETREATING 阶段永远不会出现在 `~/state` 中。

**影响**：Web 批次过程线（观察规划→质量验证→靠近抓取→工具动作→撤离收尾）只显示
"IDLE"；任何依赖 target_phase 的自动化/监控均失效。无运动安全影响，属可观测性
功能缺陷，但它是交付文档宣称的核心能力之一。

**建议**：能力端按 CycleState→TargetPhase 映射填反馈（MOVE_TO_VIEW→OBSERVING、
FINALIZE→FINALIZING、MTC_APPROACH_INSERT→APPROACHING、ACTUATE_TOOL→TOOL_ACTION、
MTC_RETREAT→RETREATING、PLAN_OBSERVATION/PREVIEW*→VALIDATING 等），并加单测锁定映射。

### R3. 机器人急停/失能期间批次继续派发，剩余目标被批量误记为"不可达"

**位置**：`allow_dispatch`（state_machine.cpp L68-75，五项前置不含
robot 状态）、`begin_target`（同上 L111-127）、能力端 `onActionGoal`
（cycle_action.cpp L37-48，不查机器人状态）、`planOrMoveTip` 执行前安全门
（motion_interface.cpp L149-156）。

**推演**：急停/断电后：四路就绪门的 motion 路失能（blockers 更新，但状态机**不**因
就绪度下降而退批/暂停）；当前目标结束后 refresh 继续派发下一目标——goal 被能力端
接受（不查机器人状态），行为树进入视点扫描，每个候选先规划成功、再在执行前安全门
被 `robot_status_not_motion_ready` 拒绝，8 段耗尽 → outcome
SKIPPED_UNREACHABLE → 编排器记账并推进感知。于是**急停期间每个剩余目标都先被浪费
数十秒规划、再被错误标记为不可达**，直到复扫拍照前置连续失败 3 次才进入
RECOVERY_REQUIRED。

**影响**：无物理风险（能力端绝不发运动），但批次账目被污染（skipped_unreachable
虚高）、耗时（每目标 ≤8 次规划）、恢复路径绕远。

**建议**：① dispatch_target_locked/allow_dispatch 增加 motion readiness 前置
（robot_status 新鲜且可运动）；② 能力端 onActionGoal 增加同款预检直接 REJECT；
③ 或编排器在 motion 路失能时自动 PAUSE 并记事件。三者取其一 + 节点层测试。

---

## 2. 中等问题（7）

### M1. 策略两阶段提交无回滚：revision 漂移时两侧使能分歧

policy_callback（harvest_orchestrator_node.cpp L810-860）：第二阶段 RPC 已把
execution/grasp/tool 写入能力端后，第三阶段若 revision 漂移则返回拒绝——**能力端
参数已生效，编排器侧却认为未生效**，两侧使能状态分歧直到下一次成功的
set_operation_policy。建议：漂移时回滚能力端参数（再发一次旧值），或接受分歧但
显式记录告警事件；同时把该语义写进测试。

### M2. CANCEL_NOW→RESUME 后挂新 RunHarvest goal：旧批次账目不清理

accepted_callback 仅对 COMPLETED/INTERRUPTED 调 reset_batch（L918-921）。若操作员
CANCEL_NOW（→INTERRUPTED）后 RESUME（→DISCOVERY）再挂新 goal，batch 已是 DISCOVERY
不触发 reset：counters/outcomes 延续上一段被中断的账，HarvestSummary 混入两段
批次的目标记录，run_id 却已更换。建议：reset_batch 条件改为"新 goal 到达时只要
非 RUNNING 中活动目标即清账"，或为 run_id 变化增设记账边界。

### M3. skip_requested_ 单标志跨周期残留的误记账窗口

SKIP_TARGET 置位（L837）后只在结果回调末尾复位（L508）。若 SKIP 落地前操作员又发
CANCEL_NOW，下一个目标被取消落地时仍带 skip 标志 → 被记为"操作员跳过"并推进感知
（未摘目标变 HARVESTED）。窗口小但污染审计与计划。建议：skip_requested_ 绑定
cycle_id 并随派发清零。

### M4. 跨包诊断契约用裸 std_msgs/String JSON，无 schema

`/peach/reconstruction/{diagnostics,grasp_decision,status}`、
`/peach/perception/harvest_state`、能力端 `~/status` 均为
字符串 JSON；消费端（orchestrator/grasp node）用 `value(key, default)` 静默
吞掉字段漂移/缺失。refined_pose/refined_diagnostics/target_observations 已类型化，
风格不一致。已验证字段名目前一致（view_coverage 等），但这是全链最脆弱的隐式契约。
建议：迁移到 peach_harvest_msgs 类型化消息（带 schema 演进说明），或至少加 JSON
schema 校验 + 缺失字段告警。

### M5. 重建节点在单线程 executor 上执行秒级重计算

`_on_finalize` → `_finalize_now` → `_run_tsdf`
（marching-cubes + 统计滤波）与 `_run_refit`（RANSAC）以及
`save_session` 的磁盘 IO 都跑在唯一 executor 线程上（rclpy.spin 默认
单线程），期间 1Hz 心跳与全部订阅被阻塞。编排器重建就绪门按 2s 新鲜度判活——
若 finalize 恰好发生在复扫轮开始前的就绪窗口，会瞬时判定重建失能并卡回
WAITING_READY。建议：rclpy MultiThreadedExecutor + MutuallyExclusive 组，或把
refit/session IO 移到 worker 线程。

### M6. MoveGroupInterface 跨线程访问缺乏显式互斥契约

能力端 4 线程 MultiThreadedExecutor + 周期 worker + action 线程共享同一
`move_group_`；串行化完全依赖 `running_` CAS 与
`GraspTask::task_mutex_`，而 MoveIt 官方明确 MoveGroupInterface 非线程安全。
当前时序恰好安全（周期运行中 go_to_photo_pose/preview 均被 CAS 挡住；cancel 的
stop() 属官方约定用法），但没有集中注释或锁来固化该契约，后续改动极易引入并发
plan()。建议：新增 `move_group_mutex_`（或明确文档化 running_ 即互斥锁）
并在 plan/execute/stop 入口断言。

### M7. 取消的 action Result 携带 outcome=FAILED

RunTargetCycle.action 的 Result 只有 SUCCEEDED/SKIPPED_QUALITY/SKIPPED_UNREACHABLE/
FAILED 四个常量；canceled 路径只能填 FAILED（cycle_action.cpp 终局分支）。编排器
靠 result.code 判别所以无实际后果，但任何其它客户端读到"取消但 outcome=FAILED"。
建议：Result 增加 CANCELED=4，或 reason 前缀标注，并同步 Web/文档。

---

## 3. 轻微问题 / 风格（Nit）

| # | 位置 | 说明 |
|---|---|---|
| N1 | harvest_orchestrator_node.cpp L400 | `consecutive_rejections_ >= 6` 魔法数未参数化 |
| N2 | harvest_orchestrator_node.cpp | RunHarvest abort 后继续跑的无 goal 批次沿用已结束 goal 的 run_id；文档称自动批次 run_id 为 "auto" |
| N3 | frame_collector.py L46 / target_registry.py 构造默认 / harvest_plan.py 构造默认 | 纯核构造默认值与 YAML 不一致（max_views 8 vs 24、recovery_scale 1.0 vs 3.5、max_targets 20 vs 50 等）——运行路径靠 params 注入一致，但离线直接使用默认值会得到不同行为，易误导 |
| N4 | harvest_plan.py update() 锁定分支 | selected 为空时按**当前帧** selectable 重选；目标瞬时闪烁会打乱执行顺序（不丢目标，仅顺序扰动） |
| N5 | harvest_orchestrator_node.cpp on_configure | `~/events` 用 QoS(50) volatile：订阅者晚启动会丢审计事件；关键审计可考虑 transient_local 或事件序列号缺口检测 |
| N6 | refresh()/result_callback 内 | 持 mutex_ 调 publish_state/publish_event（rclcpp publish 非阻塞，风险低；洁癖建议锁内拼装、锁外发布） |
| N7 | peach_pose_node.py | `~/` 与 `/peach/perception/*` 双话题并行发布同一消息对象（迁移期冗余，已注释声明，建议给出下线时间） |
| N8 | 两 Python 节点 | typing.List/Tuple 旧式泛型（3.12 可用内建泛型）；capacity=3、0.8×slop 告警阈值等散落魔法数 |
| N9 | cycle_state.hpp / state_json_ | 状态 JSON 字符串仍是对外契约（Web 依赖），枚举权威但字符串投影需与历史兼容——建议把投影映射纳入单测锁定 |
| N10 | grasp node onParameters | 任意参数修改（含与运动无关的）在周期运行中一律拒绝，粒度偏粗 |

---

## 4. 测试覆盖评估

**强**：
- 编排器纯核 `test_state_machine.cpp`（185 断言）：状态转移/幂等
  request_id/revision 乐观锁/decide_round/allow_dispatch/策略终审全覆盖；
- 抓取纯核：target_cache 43、action_contract 38、safety_gate 23、view_planner 14 断言；
- 重建/感知纯核（frame_collector、capture_gate、icp_refiner、tsdf_volume、overlap、
  registry、pipeline、fitting、params）测试齐备，含 ValueError 边界与脏数据防御。

**缺口（与 bug 分布正相关）**：
- 编排器**节点层**零测试：photo step 轮询、result 回调路由、派发重试、R1 死锁、
  M2/M3 边角全部裸奔——正是本次发现问题的层；
- 抓取 BT **节点体**零测试（test_behavior_tree.cpp 4 断言仅验证 XML 可解析 +
  打桩节点 tickOnce，不测 btAcquireViews/Finalize/MTC 分支）；
- quality_gate 仅 8 断言，未覆盖各失败分支的 reason 字符串；
- 无任何跨包集成/契约测试（R2 的 phase 映射缺失若有契约测试立刻可查）。

---

## 5. 最佳实践符合项（正面清单）

- **生命周期与所有权**：编排器 LifecycleNode + on_activate/deactivate 完整；批次唯一
  所有者语义清晰；析构/停用先置取消标志再 join 线程，无 use-after-free。
- **纯核分层**（C++/Python 一致）：状态机/视点/质量门/安全门/缓存/注册表/帧收集
  全部零 ROS、时钟注入，可单测、可离线复测。
- **并发纪律**：RPC 绝不持锁（future 轮询 500ms、两阶段提交、锁外推进感知）；
  Python 侧 RLock 单写者收敛 + BoundedWorker 背压策略显式。
- **启动顺序鲁棒性**：transient_local 闩锁 + 重建 1Hz 心跳 + 启动即首发，解决了
  感知/重建/编排器任意启动顺序下的就绪门死锁（注释记录了这个教训）。
- **终局判定只认枚举**：CycleState 枚举权威、字符串仅发布层投影，修复过字符串
  误分类问题。
- **参数治理**：YAML 权威源 + declare_parameter 逐一对齐（主体对齐，N3 例外）；
  速度双档、三级使能、5 个跨包接口名全部参数化。
- **可观测性**：事件流（severity + run/cycle/target id）、状态话题 revision 乐观锁、
  诊断 JSON 与 RViz Marker 齐全。
- **文档**：中文注释密度与准确性高，设计决策（为什么这么选）大多有注释记录。

## 6. 可扩展性与可读性评价

- **可扩展性良**：注册表模式（POSE_ESTIMATORS / FRAME_STORES / CLOUD_BUILDERS /
  GEOMETRY_REFINERS / POSE_REFINERS）加新拟合器/帧存储改动面小；行为树拓扑
  XML 可调；view_planner/quality_gate/safety_gate 全参数化；类型化 msg/srv/action
  集中管理。短板在**跨包契约**：JSON 字符串通道（M4）、feedback 映射缺失（R2）
  都是"改一处牵三处"且无法静态发现的风险点。
- **可读性良**：命名一致、职责划分清晰（薄壳回调 vs 纯核）、每个并发/状态决策都有
  注释解释"为什么"。可改进：编排器节点 1110 行单文件偏大，photo step 状态机与
  派发/回位逻辑可拆分为独立纯核（便于像状态机一样单测）；能力端
  approach_grasp_node.cpp + 4 个编译单元分工可保留但建议补类图注释。

## 7. 审查环境与并发写入披露（重要更正）

审查过程中曾把工作区里一处未提交改动（`tentative_ttl_sec` → 帧计数重构）误归因于
后台审查子代理，并据此中断了它。事后经 git diff 与文件 mtime 复核，**该归因不成立**：

- 该重构的 mtime（2026-08-14 15:00–15:04）早于子代理启动时间（约 15:20）；同一
  时段还有编排器派发重试、Web metrics/recorder 等一批未提交改动，均属工作区
  既有/并行的开发工作；
- 进程列表显示同一工作区上另有并行的 Cursor/Codex 会话（codegraph MCP 指向本
  工作区），且在本报告撰写期间（16:11–16:15）仍在持续修改
  `peach_approach_grasp` 与 `peach_pose.yaml` 等文件（帧率自适应超时功能，
  与本文 N3/M5 类时间假设问题同源）；
- 无任何证据表明审查子代理修改过文件；对其执行中断属误伤。

因此：工作树中全部 src/ 未提交改动**非本审查会话所为**，本文不作任何回退建议。
本文结论基于约 15:1x 读取的代码版本；并发改动可能已部分影响结论（如抓取端帧率
自适应超时已落地；R2 的 target_phase 反馈截至核对时仍未实现）。多写入者并存期间，
任何对 src/ 的修改或回退都应先与用户确认归属。

## 8. 修复建议优先级

| 优先级 | 项 | 性质 |
|---|---|---|
| P0 | R1 死锁（恢复路径清派发锁） | 逻辑缺陷，可复现 |
| P0 | R2 反馈 target_phase 映射 | 功能缺失（可观测性） |
| P1 | R3 派发前置加 motion readiness | 设计缺口（账目污染） |
| P1 | M1 策略回滚 / M2 新 goal 清账 | 一致性问题 |
| P2 | M3-M7、N1-N10 | 契约/并发卫生 |
| 持续 | 编排器节点层 + BT 节点体 + 跨包契约测试 | 测试补强（先补 R1/R2 回归） |

> 声明：本报告为静态审查结论；R1/R3 的推演基于代码路径，未在运行环境复现
> （审查期间 bash 工具故障，无法跑集成验证）。建议按 P0/P1 逐项补回归测试后再
> 修，避免修复引入新状态错误。
