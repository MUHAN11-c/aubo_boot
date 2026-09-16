// 功能：预览与执行绑定同一 plan_id + 模型元组 + 起始关节容差。
#ifndef PEACH_MANIPULATION__PLAN_CONTRACT_HPP_
#define PEACH_MANIPULATION__PLAN_CONTRACT_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "peach_manipulation/model_contract.hpp"

namespace peach_manipulation
{

struct ContactPlan
{
  std::string plan_id;
  ModelIdentity model;
  std::vector<double> start_joints;
  std::uint32_t scene_epoch{0};
  bool require_start_joints{true};
};

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

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__PLAN_CONTRACT_HPP_
