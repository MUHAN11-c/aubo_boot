# peach_arm 架构

## 职责

机械臂执行能力。Active 后接动作，运动与工具经 `authorizeTransit` /
`authorizeStage` 复检；不选果、不写 ledger、不调用重建 Trigger。

## 源码地图

`src/main.cpp`、`manipulation_skills_node.cpp`：executor/生命周期/ROS外壳；`target_observers.cpp`：观测与决策订阅；`status.cpp`：状态与诊断；`cycle.cpp`：受理、取消、周期上下文与授权；`stages.cpp`：executeCycle阶段序列；`motion.cpp`、`grasp_task.cpp`：MoveIt/MTC；`move_to.cpp`：转移；`tool_actuator.cpp`：工具；`quality_gate.cpp`、`safety_gate.cpp`、`target_cache.cpp`、`view_planner.cpp`：门与纯核。私有头 `joint_line_ptp.hpp` / `joint_line_retiming.hpp` 负责同路径 PTP 重定时，`trajectory_audit.hpp` 记录 DEBUG 出站轨迹；`execution_guard.hpp` 负责有界执行与弃等资源。

## 入口、输入输出与参数

整栈 include `launch/peach_arm.launch.py`。提供 SurveyScene、ExecuteTarget、MoveTo、CheckReachability 和恢复 ACK；订阅模型、许可、观测、运行使能及机器人状态。图名/模式见 [io.md](io.md)。ExecuteTarget/接近/授权实现见 [系统源码设计](../../peach_bringup/doc/design.md)。部署参数 config/peach_arm.yaml；GPL 声明/默认/校验 src/arm_parameters.yaml，经 params_bridge 转换到纯核配置。工具几何来自 aubo_description；`harvest_system` 默认 `bite_shear_v1`。独立 `peach_arm.launch.py` 声明默认仍 `adaptive_shear_v1`，整栈入口会覆盖。

## 生命周期、并发与失败

MoveTo、SurveyScene 和 ExecuteTarget 受理后持有同一个运动席位，覆盖规划、执行、等反馈与终局处理。手动拍照、接触预览与 CheckReachability 查询也需取得该席位。查询占用策略与对象寿命，不要求 execution 使能、不发送运动；席位忙或节点非 Active 时返回逐目标 `motion_policy_busy` / `node_inactive`。运行中拒绝运动策略改参；手动服务若在参数验证与提交之间抢先取得席位，新请求等策略重载完成后再受理。空闲时调用 `cancel_cycle` 仍向 MoveIt 和 MTC 发停轨，在途请求的取消保留至本次退场。

Lifecycle 加 bond，MoveIt伴随节点同 executor；动作有工作线程与互斥/取消控制，执行等待经 execution_guard 有界处理。deactivate/cleanup/shutdown/error 先关闭输出并取消；线程未在各自原有 2s 窗口内退场时，转换返回 FAILURE 并保留线程及资源，输出保持关闭。工作体实际退场后才可重试收口。PREVIEW只规划；OBSERVE只观察；PREGRASP_ONLY停轨迹净空点等ACK；FULL执行套入/工具/撤退并回拍照位。接近由planToPregrasp先试几何允许的tryRolledApproach完整LIN，再按安全条件回退tryStagingTransit的直接 PTP 到净空点 A，不复制旧兜底方案。tool=false跳过SetIO；SetIO ACK不等于切断，切断与撤退确认才业务成功。当前切断确认输入是成功 ACK 与命令后刀具 DI 新闭合沿，尚无独立果柄分离证据。许可身份/时效或规划失败须带失败原因，取消不得复活旧轨迹。

## 验证与限制

PTP 一旦下发后失败，立即保留失败及 `execution_started` 证据终结接近，不再开始后续套入。套入的最终执行门在本次规划完成后重新调用同一周期的 CONTACT 授权，核对冻结目标身份、能力和期限；这里不改变已有模型版本接纳规则。有界执行的停止回调即使抛异常，也继续原有 5s 宽限并 join 已完成线程，未完成线程移交 retire 桶，结果为 FAILURE。

`tool_state` 使用生命周期发布器，随节点显式 activate/deactivate；cleanup 同时释放工具发布器与 IO 订阅。离线探针只证明这些函数与 Jazzy 生命周期 API 的边界行为，不代表完整采摘链或跨节点 DDS 投递验收。

`colcon test --packages-select peach_arm --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 决策快照与执行门

FULL 与 PREGRASP_ONLY 将请求的场景世代和修订绑定到目标锚点。模型的运行 ID 来自锁定观测的 `harvest_run_id`，批次反馈保留调度的 `run_id`。普通请求等到同一感知轮、场景、目标、工具、模型修订、标定与配置的决策后，在缓存锁内取得几何。并行 Build 的完整 `pending:<epoch>:<target_id>` 占位只延后绑定三项修订。显式 `unrefined` 请求还需要部署允许并且观测属于本轮本场景。

普通转移在 MoveIt 规划完成后重新核对执行使能、取消、生命周期与机器人状态。手动拍照位 Trigger 在执行时消费一次 `set_execution_armed`；Survey 动作直接走本次请求的授权。刀的开合在等待 SetIO 服务后、发请求前复核 TOOL 授权。有界执行一旦超时或收到取消，宽限期只等待收尾，迟到成功不会启动后续阶段。客户端取消的 ROS 终态为 CANCELED；服务或节点停用打断 ExecuteTarget 与 MoveTo 时用 ABORTED 返回业务取消结果。

操作台任一运行使能由 true 撤为 false 时，立即走现有 `requestCancelAll` 停在途运动；相同 false 心跳不重复停轨。外部使能心跳超时同样先取消，再回落本地参数；从未收到广播时仍由本地参数决定。沿轴撤退只复核 Active、机器人状态、取消与 execution/grasp 使能，不因视觉模型过期或撤销而拒绝退出接触区。

接触电流守护的 timer、monitor 创建/更新/释放共用互斥锁。timer 回调先锁定节点弱引用，在锁外调用取消级联，避免工作线程停止守护时释放正在 update 的 monitor。

场景障碍 ACM 每轮同时写入豁免连杆的 true 与 guard 连杆的当前布尔策略；从关闭切回开启时显式撤销相机×受管障碍的豁免，保留其他对象/SRDF 条目。接触令牌若提供正 `valid_until`，以冻结截止为准；仅未提供截止时才用原 `model_stamp` 新鲜度窗兜底。未来超过 0.5s 的模型戳始终拒绝，模型身份、能力与缓存有效期复检保持生效。

指定目标 ACM 在每次规划同步时，对该目标 × 当前 `tool.contact_links` 显式写入阶段策略：Sleeve/Cut 为 true，Transit/Pregrasp/Retreat 为 false，撤销同目标遗留的接触豁免。阶段由周期 worker 在套入/撤退前设置；空目标不写该域，换目标不批量清理其它目标。此局部更新保留现有场景障碍、相机 guard 和其它对象条目；不提供跨 writer 事务。完整接触预检在任务的局部场景内明确切换 Sleeve/Retreat，实际全局策略仍由周期阶段设置。

激活后的 ACM 后台重试只捕获独立共享 generation 值；改参或生命周期关闭时递增撤销旧轮。每轮开始、100ms 分片等待和读取场景服务返回后的写入前都会复检，旧配置不再开始后续写入/重试。已派发的 DDS 服务请求不能撤回；正常规划仍按当前参数重写 ACM。

等帧与再确认的自适应下限为 `min(2.0s, 配置上限)`；小于 2s 的合法配置仍遵守原上限，不构造逆序 clamp 区间。

工具状态、事务和三轴诊断在同一把锁下取快照，SetIO 的发送与响应等待在锁外。关刀确认需要成功 ACK 和发送之后采样的新闭合沿，两者可先后到达。按驱动 `IOState.stamp` 的纳秒时间过滤延迟与乱序样本，持续高电平或失败的 ACK 均无法确认闭合。

驱动 IO 戳来自同机 controller update 的 ROS 时钟；订阅边界在严格解析后拒绝大于臂节点 `now()` 的戳，不增加容差。未来样本既不能确认闭合，也不能污染后续正常样本的单调戳基准。

FULL 在任何接近运动（包括先回拍照位）前，先以当前场景的局部副本、拍照命名关节起点验证转移→套入→撤退；该命名起点只用于规划，不下发位置跳变。分类、IK 种子和局部场景使用同一命名状态，避免把观察位直达失败等同拍照位直达失败。实际回拍照位后，每个候选在首次转移下发前再次完整预检。每段分别保留 5 s 门，超时或碰撞失败即在动前拒绝。预览用局部 ModifyPlanningScene 切换同一目标×接触连杆的 Sleeve=true、Retreat=false；转移保持 Pregrasp=false，未给转移借用接触豁免。近果速度默认 0.10，新增净空会加长套入；默认值不保证任意目标通过时限，不能以提高时限或缩短几何兜底。

Reachability、接触预览、纠偏及 PREGRASP_ONLY 共用 `trajectoryApproachGeometry` 的 A；原 GraspDecision/假设 entry、视觉 N、质量与令牌身份不改。当前 TF 仅决定 keep-roll，不改变后撤位置。SKIP 必须在 A 的笛卡尔精度内且对轴，不能把原 N 或 E 当到位。实际套入以绝对 F、撤退以绝对 A 为位置目标，MTC PointStamped 保持实际起点刀口姿态；FK 逐段核对最终位置，拒绝虽过最小 fraction 却不到终点的路径。套入前还复核当前起点位于 A 的精度内。轴向残差仍仅记录，净空停位位置检查独立于视觉几何；纠偏从最新同身份 E 重算 A，C 只加一次。PREGRASP_ONLY 只停 A，不套入或切刀。

拍照位原路返程保留实际执行成功的接近段边界，不把多个已过预算的命令合成一条超预算命令。返程先核全历史的关节顺序、时间、有限数、接缝与每段原执行预算，再逆序逐段倒放；每段核实际起点、当前场景路径点、执行授权/取消与实际终点，最后仍验证命名位及静止。缓存端点不符或中途任一失败即清除并失败，不改走新规划路径；没有返程缓存时保留原拍照位规划流程。

场景复检复用节点现有 `GetPlanningScene` client，默认回调组由四线程 executor 处理，与规划服务组和周期 worker 独立。服务必须已 ready；每段响应预算500ms、20ms轮询取消，超时/取消清理 pending request，future就绪后仍复检截止和取消。500ms是响应接受期限，不是操作系统调度的硬实时保证。完整场景快照保留现行 ACM，`PlanningScene::isPathValid` 检查本段既有离散路径点，记录读取与检查总耗时；不写场景、不重新规划，也不宣称连续碰撞检测或运动中的动态避障。读取后再核起点与授权；每段真实执行仍使用原 `execute_timeout_s`。

预览起点关节比对要求双方样本有限、容差有限且非负；轨迹行程门逐点检查有限关节位置，单点轨迹也不能借助无差分绕过检查。空集/维度拒绝语义和既有时长门关闭值保持不变。

onDecision从同一GraspDecision原子提交完整模型身份、entry/cut/axis、质量及四项能力到TargetCache，避免分离话题到达顺序把新许可配到旧几何。非观察 FULL/PREGRASP 接近前有界waitForDecision（提交含精化几何，不再先 waitForRefined），缺失/过期则decision_snapshot_unavailable；取消可结束等待。Snapshot READY仅表示同目标几何完整且有效，不代表sleeve/cut能力VALID，也不代表运行使能或硬件就绪。预抓取与CONTACT/TOOL分档授权，明确的套入/剪切拒绝不得误报成“模型未到”。模型年龄来自冻结生成时间，诊断心跳不刷新年龄；同revision截止时间只收紧，不续签。源码及离线用例已更新，真实相机+mock+RViz联合验收仍未据此宣称通过。

## ExecuteTarget 阶段与接触链日志

当前部署的 photo/stow 命名目标同为 `global_photo_pose`，所以 FULL 返程一跳。
`harvestStowNamedHops` 仍支持显式不同 stow 时先回 photo、再到 stow；阶段函数
`stageReturnHarvestStow` 的兼容名称不代表默认仍去旧 `harvest_stow`。

`stages.cpp::executeCycle` 按顺序走。默认 `reconstruct_in_trajectory` 周期 `skip_observation=true`，开场后直接 Finalize 等 Build，不先环绕观察。阶段走 `peach_arm.cycle`（`[FINALIZE]` 等）；预检/滚转/失败码走 `peach_arm.contact`。失败经 `failStage` 打 `failure_code=` 再 `[FAILED]`。状态 JSON 的 `state` 与下表一致。rqt_console 过滤这两个 logger；文件在 `$ROS_HOME/log`。

| 阶段 | 何时出现 | logger / 消息 |
|------|----------|---------------|
| FINALIZE | 非观察周期先等 GraspDecision（含精化几何）；用精化行程再跑 CheckReachability。合取分项写 `geometry_ready` / `gate` / `decision_target` | `cycle` `[FINALIZE]`；`contact` `caller=FINALIZE` `CheckReachability` |
| READY_FOR_GRASP | Finalize 过了但 `grasp.enabled=false` | `cycle` `[READY_FOR_GRASP]`「未执行接触动作」 |
| RECONFIRM | 抓取使能开：新鲜观测与锚点漂移（默认 3 cm / 最多 3 次） | `cycle` `[RECONFIRM]` |
| MTC_APPROACH_INSERT | FULL 先以拍照命名起点预验证，再实际回拍照位、直接转移到 A | `cycle` `[MTC_APPROACH_INSERT]`；`contact` `接近筛滚转` `跳过接近滚转` |
| PREGRASP_ONLY 停住 | 残差验证后停轨迹净空点 A 等 ACK，不 SetIO、不回 harvest_stow | `cycle` `[RECONFIRM]`「轨迹净空停位验证」随后 `[MTC_APPROACH_INSERT]`「PREGRASP_ONLY：停在轨迹净空点」 |
| 套入/刀/撤 | 仅 FULL；`tool.enabled` 默认关则跳过 SetIO | `cycle` `[ACTUATE_TOOL]` `[MTC_RETREAT]` |

CheckReachability 与接近共用刀口滚转梯子 {0,±15°,±30°,±45°}：第一档沿轴笛卡尔 ≥ `mtc_cartesian_min_fraction`（0.95）才算套入可达。keep-roll 两端 IK 通过但完成比不足时换下一档，全失败才 `sleeve_no_cartesian`。行程夹在 `grasp.minimum_travel_m`–`maximum_travel_m`（0.02–0.20 m）。接近段另有果实胶囊、绕行比 1.5 / 0.12 m / 0.08 m、整体 TCP 姿态 110° 门；45° 只限制工具 Z 滚转。笛卡尔 LIN 连续两档规划失败即停滚转梯子改走 PTP；PTP 大弧被拒后试下一 IK 候选。

常见失败码（`peach_arm.contact` 的 `failure_code=`）：3=`DECISION_REJECTED`（许可未到齐/行程非法），5=`SLEEVE_PLAN_FAILED`（精化可达性或到预抓取失败），11=`EXACT_TF_MISSING`，14=`DEGRADED_CONTACT_FORBIDDEN`（接近未就绪：日志写 `geometry_missing` / `gate=` / `decision_target=` 分项），15=`MODEL_STALE`。完整表见 `FailureCode.msg`。

## 线程收尾与工具滚转选择

`thread_join.hpp` 通过 packaged_task 的 future 确认执行结束，再调用 std::thread::join。生命周期回收与 action 创建周期 worker 共用线程槽互斥；槽正被使用时回收直接失败，不释放资源或增加等待预算。各 action 与 cycle 工作体持节点强引用，在 callable 入口移入栈，避免 future 反向持有节点形成环。超时保留 joinable 线程；仅当最后一个强引用在工作体完成节点访问后释放、析构恰在该线程发生时，允许解离当前线程自身。底层有界执行仍保留原停止/宽限逻辑，离线验证不代替整栈关停验收。

工作体强引用会推迟析构，因此退出取消显式触发：main 注册 Jazzy Context 的 weak pre-shutdown 回调，在 context 仍有效时调用 `prepareForShutdown()`；正常或异常离开 executor.spin 也走同一入口，异常返回非零。入口 noexcept、只执行一次原有有界收口，先关闭输出并禁止再 activate。信号线程与资源重建/释放只在取消所读的 MoveIt/MTC 槽上共用短锁，构造和析构在锁外，避免与 Admission 重载锁形成反序。回调不重入 Context shutdown 或修改回调名单。

工具Z轴滚转候选为0、±15°、±30°、±45°。执行路径按相同绝对滚转量分成四组，惰性请求下一组；组内正负滚转各保留当前种子与最多3次重启，完整验证关节边界、自碰和完整轴向路径（FULL 含套入终点）后，仍按腕轴加权关节代价排序。较小组的全部候选未运动失败后才求下一组；取消、许可拒绝或已下发运动立即收口。PREVIEW 与执行使用相同分组搜索。仅IK有解不能宣称轨迹可用，以减少拧腕。较小绝对滚转候选的完整轨迹通过后才停止；完整PTP轨迹守卫失败且尚未运动时可继续下一候选。FULL 套入预检与接近共用同一规则：第一档沿轴笛卡尔完成比达标的刀口滚转才算可达，仅两端 IK 不够。45°仅限制工具Z轴滚转；110°仍为倾斜与滚转共同产生的整体TCP姿态测地线门。该排序不放宽碰撞或轨迹门。

接近仅两段：直接转移至轨迹净空点 A，再单条轴向套入到 F。令 E 为原入口、u 为单位套入轴、S 为共享视觉后撤、C 为 `moveit.approach_clearance_m`、L 为原切断行程，则视觉预抓取 N=E−uS 保持不变，A=N−uC，F=E+uL；套入/撤退名义长度为 S+C+L。N 只在轴向路径上经过，不另停。C 默认 0.05 m，有限且在 [0,0.20]；C=0 回到视觉停位。远处用受护栏约束的直接 PTP，已对轴且几何允许时可单条 LIN；无解明确拒绝，没有中间走廊或三段回退。只有尚未运动、未取消且未因授权拒绝时才继续候选搜索。

## MTC任务生命周期

`grasp_task.cpp::makeTaskShell`直接以`Task(task_name, false)`构造，从创建时关闭introspection。不能先默认构造再`enableIntrospection(false)`：MTC IntrospectionExecutor线程可能尚未spin便被cancel并join，而Jazzy executor启动spin会重新置运行标志，导致销毁等待拖至ROS shutdown。该修正针对任务生命周期，不改变规划、安全门或轨迹形状；候选可视化使用实际TCP/RobotState，不把遭拒的候选发布成执行轨迹。

候选日志分别记录任务建立、plan及reset耗时；每次滚转迭代检查cancel_generation，取消后不继续搜索更大滚转。独立重复创建/销毁探针验证了构造方式差异，证据解释与联合测试边界见测试指导；它不能替代真实相机抓取成功或全路径线程安全验收。

## Survey观测新鲜度（2026-10-23）

`SurveyScene` 到位后等待图像曝光stamp严格晚于到位ROS时刻、且scene_epoch保持一致的观测；`snapshot_id`仅表示锁定集世代，不作为帧计数。截止使用steady clock与公共frame_wait_s预算，超时abort并置degraded，不再成功返回旧帧。首巡未Begin/未锁定可在收到新曝光后success+degraded，随后由调度Begin/WAIT_LOCK完成锁定。


2026-11-05 净空点 IK 邻近搜索：每个滚转的第一个当前种子给 KDL 传入逐变量一致性界 `max(|v_min|, |v_max|) × moveit.velocity_scaling × moveit.execute_timeout_s`；变量顺序使用 MoveIt group 的 `getVariableNames()`，速度缩放与 `makePtpSolver` 相同。KDL 在当前种子局部求解失败后仍可能内部随机重启，一致性界让第一次搜索优先寻找执行时限内可能到达的解，避免首解落入另一肩肘构型。后续三次全空间随机重启、每次 0.2 s、原有关节/自碰/完整轴向套入/轨迹/5 s 执行门均保留；该界只是必要位移条件，没有加入加速度假设或替代轨迹规划。仅单自由度有限转动关节且位置/速度界与缩放/超时均有效时启用；其他关节、缺速度或无效参数整组回退原无一致性界搜索。本机六轴 ±3.05 rad，不通过 ±2π 改写关节位置。

2026-11-05 规划装配优化：每个 `GraspTask` 独立持有一次加载的 `RobotModelLoader`，各 MTC Task 通过 `setRobotModel` 复用该模型。它不与 MGI/CheckReachability 共用可变 KDL 求解器；任务仍各自创建 CurrentState 并获取当前规划场景、应用现有 ACM 和全部轨迹门。模型通过别名 `shared_ptr` 与 loader 同属资源控制块，普通释放、空闲参数重建、以及原有执行弃等 Task 移交均保留 loader 生命周期；模型先于 loader 析构。没有跨周期缓存场景、目标许可或轨迹授权。

2026-11-05 直连执行预算预检：实际执行、tip 是模型中的 URDF link，且当前管线明确为单个 Pilz `CommandPlanner`、原语为 `LIN` 时，用同一 TCP 快照到预抓取的弦长除以同节点 `robot_description_planning.cartesian_limits.max_trans_vel × velocity_scaling`，得到不含加减速/对轴的必要时间下界。MTC MoveTo 对 URDF link 原点建立约束，Pilz LIN 使用同一 link；不把 attached-body/subframe 到母 link 的固定偏置当作零。弦长额外减去 1 mm 让剪枝保守；不改变目标或轨迹验收容差。下界严格超过 `execute_timeout_s` 才跳过整组直连滚转，进入净空点候选搜索。缺失/非法参数、非 URDF link 的 tip、不同管线/插件/原语、无效几何和不做执行预算预验证的仅规划模式均保留原搜索；可满足预算的直连仍优先。碰撞、完整套入/返程、已运动后禁止换路、执行前授权和取消门不变。

同路径 PTP 重定时仅接入明确的单 Pilz `CommandPlanner` 关节目标 PTP。原计划必须是静止到静止、有限有界单自由度转动关节的同步单调直线；每轴从实际 RobotModel 取位置、速度、加速度、jerk 界，速度/加速度再乘当前 solver 实际缩放。Ruckig 以单标量走同一关节直线，jerk 同时不超过模型界及至少 0.1 s 加速度爬升对应的界；v/a 另留 0.1%、jerk 留 1% 数值余量。原全部 q 点精确保留，额外加入相位边界；v/a 按最终整数纳秒时间计算，首尾 v/a 为零。发送 q/v/a 保持 JTC quintic 的 C2 连续，jerk 有界但不要求连续。

每个最终消息区间的 quintic 多项式转换到 Bernstein 基，使用控制值凸包及舍入余量保守核对全区间位置、速度、加速度、jerk；这不是只验采样点。RobotTrajectory 以 double 保存相邻时间，重新序列化可产生 1 ns 截断，因此还须对 set/getRobotTrajectoryMsg 后的最终消息重新过全区间门。没有收益、输入/模型未知、量化时间冲突、动态界、当前场景碰撞/路径约束或原 PTP 几何/5 s 门失败时，保留原路径及时间。通过后在 solver 交给 MTC 前替换真实 RobotTrajectory，后续原 MTC 全套几何/预算、执行授权/取消仍复检，实际执行成功的同一轨迹才进入完整返程缓存。返程用整数纳秒相减，q/a 原样、v 取反，避免 double 倒放再次截断时间。此优化不修改目标、候选排序、原段界、控制器配置或默认速度参数；`ruckig` 为 Jazzy apt 显式依赖。

专用 `peach_arm.trajectory_audit` logger 默认 INFO；仅 DEBUG 时记录实际出站关节轨迹及模型物理界。MTC 使用只读 SolutionBase 委托，从 Task::execute 同一次 toMsg 的 appendTo 记录子轨迹；基类默认拷贝保持原状态指针，appendTo/computeCost 转发原解，不注册新状态或更改场景/解。MGI 记录同一 boundedExecute 闭包中的 plan_copy。审计标记 `arm_outbound_not_dds_wire`，不将其称为 DDS 线缆或 JTC 实际接收截获。默认路径不构造审计 JSON，也不增加 toMsg；不启用 service introspection 或新的运动接口。

同值 Enables 心跳只更新接收时钟与状态；状态、原因或来源变化仍记 INFO，撤权取消及原有异常日志保持生效，没有改变消息接口。
