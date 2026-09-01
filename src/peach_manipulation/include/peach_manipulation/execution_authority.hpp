// 功能：运动阶段枚举与授权矩阵（authorizeStage 的单一事实源）。
#ifndef PEACH_MANIPULATION__EXECUTION_AUTHORITY_HPP_
#define PEACH_MANIPULATION__EXECUTION_AUTHORITY_HPP_

namespace peach_manipulation
{

// 运动阶段。授权矩阵（authorizeStage 逐级叠加，节点原子成员/安全门为输入）：
//   公共（全部阶段）  ：motion_output_permitted_（Lifecycle Active）
//                       ∧ SafetyGate::robotReady ∧ !cancel_requested_
//   TRANSIT / PREGRASP：公共 ∧ execution_enabled_
//   CONTACT（套入）    ：公共 ∧ grasp_enabled_ ∧ GraspDecision 复检
//                        （graspDecisionTargetSnapshot()==ctx.target_id 且
//                         qualitySnapshot().grasp_allowed）
//   TOOL：CONTACT 条件 ∧ tool_enabled_
// 撤离（撤退/回 stow 的撤退段）不依赖视觉：插入后目标常被工具遮挡、收割后
// GraspDecision 可能翻转，决策复检只在套入/剪切入口（requireStageAuthority
// CONTACT/TOOL）判定；撤退段授权经 GraspTask retreat 门（公共 + execution +
// grasp，无决策复检）。原 motion 注入 safety_gate lambda（Active ∧ robotReady）
// 保留为 TRANSIT 级底座，plan-only 路径不经其执行段。
enum class MotionStage { TRANSIT, PREGRASP, CONTACT, TOOL };

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__EXECUTION_AUTHORITY_HPP_
