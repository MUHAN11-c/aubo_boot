#include "peach2_manipulation/motion_backend.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "peach2_end_effector/failure_codes.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;

PlanResult finalize_plan(
  JointTrajectory trajectory, const std::optional<std::vector<double>> & start, PlanKind kind,
  const std::string & label, double at_goal_tolerance_rad, bool collapse_at_goal)
{
  PlanResult result;
  if (collapse_at_goal && !trajectory.points.empty()) {
    const std::vector<double> reference = start ? *start : trajectory.points.front().positions;
    bool at_goal = true;
    for (const auto & p : trajectory.points) {
      if (!(max_joint_deviation(p.positions, reference) < at_goal_tolerance_rad)) {
        at_goal = false;
        break;
      }
    }
    if (at_goal) {
      const std::size_t n = reference.size();
      TrajectoryPoint point;
      point.positions = reference;
      point.velocities.assign(n, 0.0);
      point.accelerations.assign(n, 0.0);
      trajectory.points.assign(1U, std::move(point));
      result.ok = true;
      result.reason = label + ":at_goal";
      result.trajectory = std::move(trajectory);
      return result;
    }
  }
  if (trajectory.points.size() < 2U) {
    result.failure_code = kind == PlanKind::LINEAR ?
      failure::PLAN_CARTESIAN_INCOMPLETE : failure::PLAN_FAILED;
    result.reason = label + ":empty_trajectory";
    return result;
  }
  result.ok = true;
  result.trajectory = std::move(trajectory);
  return result;
}

}  // namespace peach2_manipulation
