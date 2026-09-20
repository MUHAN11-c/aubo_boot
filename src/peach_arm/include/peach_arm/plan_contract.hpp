// 功能：预览与执行绑定同一 plan_id + 模型元组 + 起始关节容差。
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

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__PLAN_CONTRACT_HPP_
