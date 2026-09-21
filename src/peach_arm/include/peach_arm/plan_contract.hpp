// 功能：预览与执行绑定同一 plan_id + 模型元组 + 起始关节容差。
// G2 语义修正：预览绑定只由 PREVIEW 模式 goal 写入——OBSERVE_ONLY（及
// PREGRASP_ONLY/FULL 等执行类模式）一律不写（观察是采数据不是计划预览；
// observe goal 在模型建好前本就带不了三修订）。执行侧消费经 executePlanGate。
#ifndef PEACH_MANIPULATION__PLAN_CONTRACT_HPP_
#define PEACH_MANIPULATION__PLAN_CONTRACT_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "peach_arm/model_contract.hpp"

namespace peach_arm
{

/// 接触计划绑定契约：预览与执行共用 plan_id + 模型元组 + 起始关节。
struct ContactPlan
{
  std::string plan_id;              ///< 计划标识（空=未绑定，执行侧拒）。
  ModelIdentity model;              ///< 生成该计划的模型身份元组。
  std::vector<double> start_joints; ///< 计划起始关节（require_start_joints 时比对容差）。
  std::uint32_t scene_epoch{0};     ///< 生成时的场景世代（跨世代计划作废）。
  bool require_start_joints{true};  ///< true=执行前核对当前关节仍在起点容差内。
};

/// 逐轴关节差是否全在容差内（维度不符/空集恒 false——宁拒不猜）。
inline bool jointsWithinTolerance(
  const std::vector<double> & left, const std::vector<double> & right, double tol_rad)
{
  if (left.size() != right.size() || left.empty()) {
    return false;
  }
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (std::abs(left[i] - right[i]) > tol_rad) {
      return false;
    }
  }
  return true;
}

/// 预览计划能否绑定本次执行：plan_id、场景世代、模型元组三重一致，
/// 再按 require_start_joints 比对起始关节容差。
inline bool previewMatchesExecute(
  const ContactPlan & preview, const ContactPlan & execute, double joint_tol_rad)
{
  if (preview.plan_id.empty() || preview.plan_id != execute.plan_id) {
    return false;
  }
  if (preview.scene_epoch != execute.scene_epoch) {
    return false;
  }
  if (!identitiesMatch(preview.model, execute.model)) {
    return false;
  }
  if (!preview.require_start_joints) {
    return true;
  }
  return jointsWithinTolerance(preview.start_joints, execute.start_joints, joint_tol_rad);
}

/// 执行侧计划契约门结果：pass=false 时 failure_code=20（peach_interfaces
/// FailureCode.PLAN_MISMATCH；纯核头不引 ROS 消息，数值钉死，cycle.cpp
/// static_assert 与 IDL 双向锁定），放行时 failure_code=0（NONE）。
struct ExecutePlanGate
{
  bool pass{true};
  std::uint32_t failure_code{0};
};

/// 执行 goal 的计划绑定门（G2 语义修正）：绑定只在「上一受理 goal 是
/// PREVIEW 模式」时存在（写侧仅 PREVIEW；observe/执行模式不写绑定）。
/// 无绑定或 goal 无 plan_id → 放行（契约未启用，非拒单——fast 档不发
/// PREVIEW 即整条不生效，是有意的）；否则全字段比对（plan_id/场景世代/
/// 模型元组/按需起始关节），不一致拒单并给 PLAN_MISMATCH。
inline ExecutePlanGate executePlanGate(
  bool preview_valid, bool goal_has_plan_id, const ContactPlan & preview,
  const ContactPlan & execute, double joint_tol_rad)
{
  if (!preview_valid || !goal_has_plan_id) {
    return ExecutePlanGate{};
  }
  if (previewMatchesExecute(preview, execute, joint_tol_rad)) {
    return ExecutePlanGate{};
  }
  return ExecutePlanGate{false, 20u};
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__PLAN_CONTRACT_HPP_
