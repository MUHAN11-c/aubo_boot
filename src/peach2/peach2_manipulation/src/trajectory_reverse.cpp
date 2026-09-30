#include "peach2_manipulation/trajectory_reverse.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace peach2_manipulation
{

JointTrajectory reverse_trajectory(const JointTrajectory & forward)
{
  JointTrajectory out;
  out.joint_names = forward.joint_names;
  if (forward.points.empty()) {
    return out;
  }
  const double total = forward.duration_s();
  out.points.reserve(forward.points.size());
  for (auto it = forward.points.rbegin(); it != forward.points.rend(); ++it) {
    TrajectoryPoint p;
    p.time_from_start_s = total - it->time_from_start_s;
    p.positions = it->positions;
    p.velocities.reserve(it->velocities.size());
    for (double v : it->velocities) {
      p.velocities.push_back(-v);
    }
    p.accelerations = it->accelerations;
    out.points.push_back(std::move(p));
  }
  return out;
}

JointTrajectory reverse_path(const std::vector<JointTrajectory> & segments)
{
  JointTrajectory out;
  bool first = true;
  for (auto it = segments.rbegin(); it != segments.rend(); ++it) {
    if (it->points.empty()) {
      continue;
    }
    if (first) {
      out.joint_names = it->joint_names;
    } else if (it->joint_names != out.joint_names) {
      throw std::invalid_argument("reverse_path: joint names differ between segments");
    }
    const JointTrajectory rev = reverse_trajectory(*it);
    const double offset = out.duration_s();
    size_t start = 0;
    if (!first && !out.points.empty() &&
      max_joint_deviation(out.points.back().positions, rev.points.front().positions) < 1e-9)
    {
      start = 1;
    }
    for (size_t i = start; i < rev.points.size(); ++i) {
      TrajectoryPoint p = rev.points[i];
      p.time_from_start_s += offset;
      out.points.push_back(std::move(p));
    }
    first = false;
  }
  return out;
}

double joint_path_length(const JointTrajectory & trajectory)
{
  double total = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & a = trajectory.points[i - 1].positions;
    const auto & b = trajectory.points[i].positions;
    double sq = 0.0;
    for (size_t j = 0; j < std::min(a.size(), b.size()); ++j) {
      sq += (b[j] - a[j]) * (b[j] - a[j]);
    }
    total += std::sqrt(sq);
  }
  return total;
}

double max_joint_deviation(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size() || a.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double m = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    m = std::max(m, std::fabs(a[i] - b[i]));
  }
  return m;
}

}  // namespace peach2_manipulation
