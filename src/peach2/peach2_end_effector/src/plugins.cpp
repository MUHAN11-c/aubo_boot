#include "peach2_end_effector/plugins.hpp"

#include <cmath>
#include <optional>

namespace peach2_end_effector
{

RollConstraint ShearV1::roll_constraint(const TargetGeometry & target) const
{
  RollConstraint rc;
  std::optional<double> avoid;
  if (target.branch_direction) {
    avoid = roll_of_direction(target.axis, *target.branch_direction);
  }
  if (!avoid && target.avoid_direction) {
    avoid = roll_of_direction(target.axis, *target.avoid_direction);
  }
  if (!avoid) {
    return rc;
  }
  rc.center_rad = wrap_angle(*avoid + M_PI);
  rc.half_width_rad = kRollHalfWidthRad;
  rc.period_rad = 2.0 * M_PI;
  return rc;
}

InsertPlan ShearV1::insert(const TargetGeometry & /*target*/) const
{
  InsertPlan p;
  p.mode = InsertMode::LINEAR;
  p.criterion = ArrivalCriterion::TRAVEL_COMPLETE;
  p.speed_mps = 0.02;
  p.dwell_s = 0.25;
  return p;
}

RollConstraint BiteShearV1::roll_constraint(const TargetGeometry & target) const
{
  RollConstraint rc;
  rc.period_rad = M_PI;
  rc.half_width_rad = M_PI / 2.0;
  if (!target.branch_direction) {
    return rc;
  }
  const auto branch = roll_of_direction(target.axis, *target.branch_direction);
  if (!branch) {
    return rc;
  }
  // TCP +X along the branch => jaws (closing along +Y) perpendicular to it.
  rc.center_rad = wrap_angle(*branch);
  rc.half_width_rad = kRollHalfWidthRad;
  return rc;
}

InsertPlan BiteShearV1::insert(const TargetGeometry & /*target*/) const
{
  InsertPlan p;
  p.mode = InsertMode::LINEAR;
  p.criterion = ArrivalCriterion::TRAVEL_AND_THROAT_CONTACT;
  p.speed_mps = 0.02;
  p.dwell_s = 0.25;
  p.requires_force_sensing = true;
  p.linear_fallback = true;
  return p;
}

RollConstraint AdaptiveShearV1::roll_constraint(const TargetGeometry & /*target*/) const
{
  return RollConstraint{};
}

InsertPlan AdaptiveShearV1::insert(const TargetGeometry & /*target*/) const
{
  InsertPlan p;
  p.mode = InsertMode::ADMITTANCE;
  p.criterion = ArrivalCriterion::CONTACT_FORCE_STABLE;
  p.speed_mps = 0.02;
  p.dwell_s = 0.3;
  p.contact_force_n = 2.0;
  p.force_stable_s = 0.3;
  p.requires_force_sensing = true;
  p.linear_fallback = true;
  return p;
}

}  // namespace peach2_end_effector
