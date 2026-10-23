# peach_arm 架构

## 职责

机械臂执行能力。Active 后接动作，运动与工具经 ExecutionAuthority 复检；不选果、不写 ledger、不调用重建 Trigger。

## 源码地图

`src/main.cpp`、`manipulation_skills_node.cpp`：executor/生命周期/ROS外壳；`target_observers.cpp`：观测与决策订阅；`status.cpp`：状态与诊断；`cycle.cpp`：受理、取消、周期上下文与授权；`stages.cpp`：executeCycle阶段序列；`motion.cpp`、`grasp_task.cpp`：MoveIt/MTC；`move_to.cpp`：转移；`tool_actuator.cpp`：工具；`quality_gate.cpp`、`safety_gate.cpp`、`target_cache.cpp`、`view_planner.cpp`：门与纯核。

## 入口、输入输出与参数

整栈 include `launch/peach_arm.launch.py`。提供 SurveyScene、ExecuteTarget、MoveTo、CheckReachability 和恢复 ACK；订阅模型、许可、观测、运行使能及机器人状态。图名/模式见 [io.md](io.md)。ExecuteTarget/接近/授权实现见 [系统源码设计](../../peach_bringup/doc/design.md)。部署参数 config/peach_arm.yaml；GPL 声明/默认/校验 src/arm_parameters.yaml，经 params_bridge 转换到纯核配置。工具几何来自 aubo_description；`harvest_system` 默认 `bite_shear_v1`。独立 `peach_arm.launch.py` 声明默认仍 `adaptive_shear_v1`，整栈入口会覆盖。

## 生命周期、并发与失败

Lifecycle 加 bond，MoveIt伴随节点同 executor；动作有工作线程与互斥/取消控制，执行等待经 execution_guard 有界处理。deactivate/cleanup 应取消并回收资源；历史存在关停/滞留线程风险，不能仅因有超时便宣称清理完全。PREVIEW只规划；OBSERVE只观察；PREGRASP_ONLY停预抓取等ACK；FULL执行套入/工具/撤退。接近由planToPregrasp先试几何允许的tryRolledApproach完整LIN，再按安全条件回退tryStagingTransit的PTP+垂直LIN+沿轴LIN，不复制旧兜底方案。tool=false跳过SetIO；SetIO ACK不等于切断，切断与撤退确认才业务成功。许可身份/时效或规划失败须带失败原因，取消不得复活旧轨迹。

## 验证与限制

`colcon test --packages-select peach_arm --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 决策快照与执行门

onDecision从同一GraspDecision原子提交完整模型身份、entry/cut/axis、质量及四项能力到TargetCache，避免分离话题到达顺序把新许可配到旧几何。非观察 FULL/PREGRASP 接近前有界waitForDecision（提交含精化几何，不再先 waitForRefined），缺失/过期则decision_snapshot_unavailable；取消可结束等待。Snapshot READY仅表示同目标几何完整且有效，不代表sleeve/cut能力VALID，也不代表运行使能或硬件就绪。预抓取与CONTACT/TOOL分档授权，明确的套入/剪切拒绝不得误报成“模型未到”。模型年龄来自冻结生成时间，诊断心跳不刷新年龄；同revision截止时间只收紧，不续签。源码及离线用例已更新，真实相机+mock+RViz联合验收仍未据此宣称通过。

## ExecuteTarget 阶段与接触链日志

`stages.cpp::executeCycle` 按顺序走。默认 `reconstruct_in_trajectory` 周期 `skip_observation=true`，开场后直接 Finalize 等 Build，不先环绕观察。阶段走 `peach_arm.cycle`（`[FINALIZE]` 等）；预检/滚转/失败码走 `peach_arm.contact`。失败经 `failStage` 打 `failure_code=` 再 `[FAILED]`。状态 JSON 的 `state` 与下表一致。rqt_console 过滤这两个 logger；文件在 `$ROS_HOME/log`。

| 阶段 | 何时出现 | logger / 消息 |
|------|----------|---------------|
| FINALIZE | 非观察周期先等 GraspDecision（含精化几何）；用精化行程再跑 CheckReachability。合取分项写 `geometry_ready` / `gate` / `decision_target` | `cycle` `[FINALIZE]`；`contact` `caller=FINALIZE` `CheckReachability` |
| READY_FOR_GRASP | Finalize 过了但 `grasp.enabled=false` | `cycle` `[READY_FOR_GRASP]`「未执行接触动作」 |
| RECONFIRM | 抓取使能开：新鲜观测与锚点漂移（默认 3 cm / 最多 3 次） | `cycle` `[RECONFIRM]` |
| MTC_APPROACH_INSERT | 先回拍照位再接近预抓取；FULL 用插入行程筛滚转 | `cycle` `[MTC_APPROACH_INSERT]`；`contact` `接近筛滚转` `跳过接近/入冠滚转` |
| PREGRASP_ONLY 停住 | 残差验证后停预抓取等 ACK，不 SetIO、不回 harvest_stow | `cycle` `[RECONFIRM]`「预抓取停稳验证」随后 `[MTC_APPROACH_INSERT]`「PREGRASP_ONLY：停在预抓取」 |
| 套入/刀/撤 | 仅 FULL；`tool.enabled` 默认关则跳过 SetIO | `cycle` `[ACTUATE_TOOL]` `[MTC_RETREAT]` |

CheckReachability 与接近共用刀口滚转梯子 {0,±15°,±30°,±45°}：第一档沿轴笛卡尔 ≥ `mtc_cartesian_min_fraction`（0.95）才算套入可达。keep-roll 两端 IK 通过但完成比不足时换下一档，全失败才 `sleeve_no_cartesian`。行程夹在 `grasp.minimum_travel_m`–`maximum_travel_m`（0.02–0.20 m）。接近段另有果实胶囊、绕行比 1.5 / 0.12 m / 0.08 m、整体 TCP 姿态 110° 门；45° 只限制工具 Z 滚转。笛卡尔 LIN 连续两档规划失败即停滚转梯子改走 PTP；PTP 大弧被拒后试下一 IK 候选。

常见失败码（`peach_arm.contact` 的 `failure_code=`）：3=`DECISION_REJECTED`（许可未到齐/行程非法），5=`SLEEVE_PLAN_FAILED`（精化可达性或到预抓取失败），11=`EXACT_TF_MISSING`，14=`DEGRADED_CONTACT_FORBIDDEN`（接近未就绪：日志写 `geometry_missing` / `gate=` / `decision_target=` 分项），15=`MODEL_STALE`。完整表见 `FailureCode.msg`。

## 线程收尾与工具滚转选择

`thread_join.hpp`通过packaged_task的future先确认执行结束，再调用std::thread::join，避免混用pthread原生join与C++线程对象状态造成E1退出−6。超时仍采用detach兜底，滞留线程的历史风险尚在，不能由短测退出成功推定所有挂死路径安全。

工具Z轴滚转候选为0、±15°、±30°、±45°。IK候选列表先按绝对滚转、再按腕轴加权关节代价排序，逐候选验证完整接近走廊；仅IK有解不能宣称轨迹可用，以减少拧腕。较小绝对滚转候选的完整轨迹通过后才停止；完整PTP轨迹守卫失败且尚未运动时可继续下一候选。FULL 套入预检与接近共用同一规则：第一档沿轴笛卡尔完成比达标的刀口滚转才算可达，仅两端 IK 不够。45°仅限制工具Z轴滚转；110°仍为倾斜与滚转共同产生的整体TCP姿态测地线门。该排序不放宽碰撞或轨迹门。

接近轨迹优先完整直线，必要时仅使用原有冠内两段LIN之间2cm blend平滑连接；没有新增CIRC或OMPL接触兜底。PTP是关节空间路径，不能声称TCP走精确圆弧。fallback只允许在尚未运动、未取消且未因授权拒绝时继续。

## MTC任务生命周期

`grasp_task.cpp::makeTaskShell`直接以`Task(task_name, false)`构造，从创建时关闭introspection。不能先默认构造再`enableIntrospection(false)`：MTC IntrospectionExecutor线程可能尚未spin便被cancel并join，而Jazzy executor启动spin会重新置运行标志，导致销毁等待拖至ROS shutdown。该修正针对任务生命周期，不改变规划、安全门或轨迹形状；候选可视化使用实际TCP/RobotState，不把遭拒的候选发布成执行轨迹。

候选日志分别记录任务建立、plan及reset耗时；每次滚转迭代检查cancel_generation，取消后不继续搜索更大滚转。独立重复创建/销毁探针验证了构造方式差异，证据解释与联合测试边界见测试指导；它不能替代真实相机抓取成功或全路径线程安全验收。

## Survey观测新鲜度（2026-10-23）

`SurveyScene` 到位后等待图像曝光stamp严格晚于到位ROS时刻、且scene_epoch保持一致的观测；`snapshot_id`仅表示锁定集世代，不作为帧计数。截止使用steady clock与公共frame_wait_s预算，超时abort并置degraded，不再成功返回旧帧。首巡未Begin/未锁定可在收到新曝光后success+degraded，随后由调度Begin/WAIT_LOCK完成锁定。
