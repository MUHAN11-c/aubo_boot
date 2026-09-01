// 功能：接近轨迹护栏。绕行只看关节行程（与速度无关）；时长仅记录，
// max_duration_s<=0 时不按时长拒发。
#ifndef PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_
#define PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace peach_manipulation
{

struct TrajectoryGuardLimits
{
  // <=0：不按时长拒发（时长随速度变，不能当绕行判据）。
  double max_duration_s{0.0};
  double max_total_joint_travel_rad{12.0};
  double max_single_joint_travel_rad{6.1};
};

struct TrajectoryGuardReport
{
  bool allowed{false};
  double duration_s{0.0};
  double total_joint_travel_rad{0.0};
  double max_single_joint_travel_rad{0.0};
  std::size_t point_count{0U};
  std::string reason;
};

inline TrajectoryGuardReport inspectApproachTrajectories(
  const std::vector<trajectory_msgs::msg::JointTrajectory> & parts,
  const TrajectoryGuardLimits & limits)
{
  TrajectoryGuardReport report;
  if (parts.empty()) {
    report.reason = "接近轨迹为空";
    return report;
  }
  std::vector<std::string> names;
  std::vector<double> per_joint;
  std::vector<double> previous_end;
  for (const auto & trajectory : parts) {
    if (trajectory.points.empty() || trajectory.joint_names.empty()) {
      report.reason = "接近轨迹为空";
      return report;
    }
    const auto joint_count = trajectory.joint_names.size();
    if (names.empty()) {
      names = trajectory.joint_names;
      per_joint.assign(joint_count, 0.0);
    } else if (trajectory.joint_names != names) {
      report.reason = "接近轨迹关节名不一致";
      return report;
    }
    for (const auto & point : trajectory.points) {
      if (point.positions.size() != joint_count) {
        report.reason = "接近轨迹关节维度不一致";
        return report;
      }
    }
    // 段间接缝也计入行程：过渡点边界处腕部 ±π 翻转不会出现在任一段内部。
    if (!previous_end.empty()) {
      const auto & next_start = trajectory.points.front().positions;
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const double delta = std::abs(next_start[joint] - previous_end[joint]);
        if (!std::isfinite(delta)) {
          report.reason = "接近轨迹含非有限关节值";
          return report;
        }
        per_joint[joint] += delta;
        report.total_joint_travel_rad += delta;
      }
    }
    for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto & previous = trajectory.points[i - 1].positions;
      const auto & current = trajectory.points[i].positions;
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const double delta = std::abs(current[joint] - previous[joint]);
        if (!std::isfinite(delta)) {
          report.reason = "接近轨迹含非有限关节值";
          return report;
        }
        per_joint[joint] += delta;
        report.total_joint_travel_rad += delta;
      }
    }
    const auto & end = trajectory.points.back().time_from_start;
    report.duration_s += static_cast<double>(end.sec) +
      static_cast<double>(end.nanosec) * 1e-9;
    report.point_count += trajectory.points.size();
    previous_end = trajectory.points.back().positions;
  }
  report.max_single_joint_travel_rad =
    *std::max_element(per_joint.begin(), per_joint.end());

  std::ostringstream reason;
  if (limits.max_duration_s > 0.0 && report.duration_s > limits.max_duration_s) {
    reason << "预计时长 " << report.duration_s << "s > " <<
      limits.max_duration_s << "s";
    report.reason = reason.str();
    return report;
  }
  if (report.total_joint_travel_rad > limits.max_total_joint_travel_rad) {
    reason << "累计关节行程 " << report.total_joint_travel_rad << "rad > " <<
      limits.max_total_joint_travel_rad << "rad";
    report.reason = reason.str();
    return report;
  }
  if (
    report.max_single_joint_travel_rad > limits.max_single_joint_travel_rad)
  {
    reason << "单轴累计行程 " << report.max_single_joint_travel_rad << "rad > " <<
      limits.max_single_joint_travel_rad << "rad";
    report.reason = reason.str();
    return report;
  }
  report.allowed = true;
  reason << "短路径门通过";
  report.reason = reason.str();
  return report;
}

inline TrajectoryGuardReport inspectApproachTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const TrajectoryGuardLimits & limits)
{
  return inspectApproachTrajectories({trajectory}, limits);
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__TRAJECTORY_GUARD_HPP_
