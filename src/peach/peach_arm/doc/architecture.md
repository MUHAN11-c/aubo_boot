# peach_arm 架构

## 职责

机械臂执行能力。Active 后接动作，运动与工具经 ExecutionAuthority 复检；不选果、不写 ledger、不调用重建 Trigger。

## 源码地图

`src/main.cpp`、`manipulation_skills_node.cpp`：executor/生命周期/ROS外壳；`target_observers.cpp`：观测与决策订阅；`status.cpp`：状态与诊断；`cycle.cpp`：受理、取消、周期上下文与授权；`stages.cpp`：executeCycle阶段序列；`motion.cpp`、`grasp_task.cpp`：MoveIt/MTC；`move_to.cpp`：转移；`tool_actuator.cpp`：工具；`quality_gate.cpp`、`safety_gate.cpp`、`target_cache.cpp`、`view_planner.cpp`：门与纯核。

## 入口、输入输出与参数

整栈 include `launch/peach_arm.launch.py`。提供 SurveyScene、ExecuteTarget、MoveTo、CheckReachability 和恢复 ACK；订阅模型、许可、观测、运行使能及机器人状态。图名/模式见 [io.md](io.md)。部署参数 config/peach_arm.yaml；GPL 声明/默认/校验 src/arm_parameters.yaml，经 params_bridge 转换到纯核配置。工具几何来自 aubo_description，整栈默认 bite_shear_v1。

## 生命周期、并发与失败

Lifecycle 加 bond，MoveIt伴随节点同 executor；动作有工作线程与互斥/取消控制，执行等待经 execution_guard 有界处理。deactivate/cleanup 应取消并回收资源；历史存在关停/滞留线程风险，不能仅因有超时便宣称清理完全。PREVIEW只规划；OBSERVE只观察；PREGRASP_ONLY停预抓取等ACK；FULL执行套入/工具/撤退。接近由planToPregrasp先试几何允许的tryRolledApproach完整LIN，再按安全条件回退tryStagingTransit的PTP+垂直LIN+沿轴LIN，不复制旧兜底方案。tool=false跳过SetIO；SetIO ACK不等于切断，切断与撤退确认才业务成功。许可身份/时效或规划失败须带失败原因，取消不得复活旧轨迹。

## 验证与限制

`colcon test --packages-select peach_arm --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 决策快照与执行门

onDecision从同一GraspDecision原子提交完整模型身份、entry/cut/axis、质量及四项能力到TargetCache，避免分离话题到达顺序把新许可配到旧几何。FULL接近前有界waitForDecision，缺失/过期则decision_snapshot_unavailable；取消可结束等待。Snapshot READY仅表示同目标几何完整且有效，不代表sleeve/cut能力VALID，也不代表运行使能或硬件就绪。预抓取与CONTACT/TOOL分档授权，明确的套入/剪切拒绝不得误报成“模型未到”。模型年龄来自冻结生成时间，诊断心跳不刷新年龄；同revision截止时间只收紧，不续签。源码及离线用例已更新，真实相机+mock+RViz联合验收仍未据此宣称通过。

## 线程收尾与工具滚转选择

`thread_join.hpp`通过packaged_task的future先确认执行结束，再调用std::thread::join，避免混用pthread原生join与C++线程对象状态造成E1退出−6。超时仍采用detach兜底，滞留线程的历史风险尚在，不能由短测退出成功推定所有挂死路径安全。

工具Z轴滚转候选为0、±15°、±30°、±45°。IK候选列表先按绝对滚转、再按腕轴加权关节代价排序，逐候选验证完整接近走廊；仅IK有解不能宣称轨迹可用，以减少拧腕。较小绝对滚转候选的完整轨迹通过后才停止；完整PTP轨迹守卫失败且尚未运动时可继续下一候选。45°仅限制工具Z轴滚转；110°仍为倾斜与滚转共同产生的整体TCP姿态测地线门。该排序不放宽碰撞或轨迹门。

接近轨迹优先完整直线，必要时仅使用原有冠内两段LIN之间2cm blend平滑连接；没有新增CIRC或OMPL接触兜底。PTP是关节空间路径，不能声称TCP走精确圆弧。fallback只允许在尚未运动、未取消且未因授权拒绝时继续。
