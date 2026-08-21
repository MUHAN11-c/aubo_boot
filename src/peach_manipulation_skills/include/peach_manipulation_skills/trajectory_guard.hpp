// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef PEACH_MANIPULATION_SKILLS__TRAJECTORY_GUARD_HPP_
#define PEACH_MANIPULATION_SKILLS__TRAJECTORY_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace peach_manipulation_skills
{

struct TrajectoryGuardLimits
{
  double max_duration_s{12.0};
  double max_total_joint_travel_rad{4.0};
  double max_single_joint_travel_rad{2.1};
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

inline TrajectoryGuardReport inspectApproachTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const TrajectoryGuardLimits & limits)
{
  TrajectoryGuardReport report;
  report.point_count = trajectory.points.size();
  if (trajectory.points.empty() || trajectory.joint_names.empty()) {
    report.reason = "接近轨迹为空";
    return report;
  }
  const auto joint_count = trajectory.joint_names.size();
  std::vector<double> per_joint(joint_count, 0.0);
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != joint_count) {
      report.reason = "接近轨迹关节维度不一致";
      return report;
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
  report.max_single_joint_travel_rad =
    *std::max_element(per_joint.begin(), per_joint.end());
  const auto & end = trajectory.points.back().time_from_start;
  report.duration_s = static_cast<double>(end.sec) +
    static_cast<double>(end.nanosec) * 1e-9;

  std::ostringstream reason;
  if (report.duration_s > limits.max_duration_s) {
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

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__TRAJECTORY_GUARD_HPP_
