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
#include "peach_manipulation_skills/view_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace peach_manipulation_skills
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

Eigen::Vector3d safeUnit(const Eigen::Vector3d & value, const Eigen::Vector3d & fallback)
{
  if (!value.allFinite() || value.norm() < 1.0e-9) {
    return fallback;
  }
  return value.normalized();
}
}  // namespace

double angleDegrees(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const Eigen::Vector3d a = safeUnit(first, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d b = safeUnit(second, Eigen::Vector3d::UnitX());
  const double dot = std::clamp(a.dot(b), -1.0, 1.0);
  return std::acos(dot) * 180.0 / kPi;
}

ViewPlanner::ViewPlanner(ViewPlannerConfig config)
: config_(std::move(config))
{
}

Eigen::Matrix3d ViewPlanner::lookAtOptical(
  const Eigen::Vector3d & camera_position,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & world_up)
{
  const Eigen::Vector3d optical_z = safeUnit(
    target - camera_position, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d down = -safeUnit(world_up, Eigen::Vector3d::UnitZ());
  if (std::abs(down.dot(optical_z)) > 0.97) {
    down = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d optical_x = safeUnit(
    down.cross(optical_z), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d optical_y = safeUnit(
    optical_z.cross(optical_x), Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rotation;
  rotation.col(0) = optical_x;
  rotation.col(1) = optical_y;
  rotation.col(2) = optical_z;
  return rotation;
}

Eigen::Matrix3d ViewPlanner::toolOrientation(
  const Eigen::Vector3d & approach_axis,
  const Eigen::Vector3d & preferred_x)
{
  const Eigen::Vector3d z_axis = safeUnit(approach_axis, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d x_axis = preferred_x - preferred_x.dot(z_axis) * z_axis;
  if (x_axis.norm() < 1.0e-6) {
    const Eigen::Vector3d fallback =
      std::abs(z_axis.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    x_axis = fallback - fallback.dot(z_axis) * z_axis;
  }
  x_axis.normalize();
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return rotation;
}

std::vector<ViewCandidate> ViewPlanner::generate(
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & current_camera_position,
  const std::vector<Eigen::Vector3d> & observed_directions) const
{
  ViewContext context;
  context.target = target;
  context.current_camera_position = current_camera_position;
  context.observed_directions = observed_directions;
  return generate(context);
}

std::vector<ViewCandidate> ViewPlanner::generate(const ViewContext & context) const
{
  const Eigen::Vector3d & target = context.target;
  const Eigen::Vector3d & current_camera_position = context.current_camera_position;
  const std::vector<Eigen::Vector3d> & observed_directions =
    context.observed_directions;
  const Eigen::Vector3d front = safeUnit(
    current_camera_position - target, Eigen::Vector3d::UnitX());
  Eigen::Vector3d side = Eigen::Vector3d::UnitZ().cross(front);
  if (side.norm() < 1.0e-6) {
    side = Eigen::Vector3d::UnitY();
  }
  side.normalize();
  const Eigen::Vector3d up = front.cross(side).normalized();
  std::vector<Eigen::Vector3d> observed = observed_directions;
  if (observed.empty()) {
    observed.push_back(front);
  }

  std::vector<ViewCandidate> result;
  const double radial_progress = std::clamp(
    static_cast<double>(observed_directions.size()) /
    std::max(1, config_.views_to_minimum_radius), 0.0, 1.0);
  const double desired_layer = radial_progress *
    std::max(0, config_.candidate_layers - 1);
  const int azimuth_steps = static_cast<int>(
    std::floor(config_.azimuth_limit_deg / config_.azimuth_step_deg));
  const int elevation_steps = static_cast<int>(
    std::floor(config_.elevation_limit_deg / config_.elevation_step_deg));
  for (int layer = 0; layer < config_.candidate_layers; ++layer) {
    const double radius = std::max(
      config_.minimum_radius_m,
      config_.observation_radius_m - layer * config_.radial_step_m);
    for (int azimuth_index = -azimuth_steps;
      azimuth_index <= azimuth_steps; ++azimuth_index)
    {
      for (int elevation_index = -elevation_steps;
        elevation_index <= elevation_steps; ++elevation_index)
      {
        if (azimuth_index == 0 && elevation_index == 0 && layer > 0) {
          continue;
        }
        const double azimuth_deg = azimuth_index * config_.azimuth_step_deg;
        const double elevation_deg = elevation_index * config_.elevation_step_deg;
        const double azimuth = radians(azimuth_deg);
        const double elevation = radians(elevation_deg);
        Eigen::Vector3d direction =
          std::cos(elevation) * std::cos(azimuth) * front +
          std::cos(elevation) * std::sin(azimuth) * side +
          std::sin(elevation) * up;
        direction.normalize();
        const Eigen::Vector3d camera_position = target + radius * direction;
        // 桌面保护平面：低于 z 下限的视点规划必败，不生成（省时且防撞桌）。
        if (camera_position.z() < config_.min_camera_height_m) {
          continue;
        }
        // 环境几何保护区（阶段 F1）：相机位置落入任一保护盒（闭区间，含盒
        // 表面）的候选不生成——盒内视点必然碰撞，同平面过滤一样省时且防撞。
        if (protectedZoneHit(camera_position, config_.protected_zones)) {
          continue;
        }

        double nearest = std::numeric_limits<double>::max();
        for (const auto & previous : observed) {
          nearest = std::min(nearest, angleDegrees(direction, previous));
        }
        const double motion = angleDegrees(direction, front);
        const double baseline_error =
          (nearest - config_.preferred_baseline_deg) /
          std::max(1.0, config_.preferred_baseline_deg * 0.7);
        const double overlap_score = std::exp(-0.5 * baseline_error * baseline_error);
        const double novelty_score = std::clamp(
          nearest / std::max(1.0, config_.azimuth_limit_deg), 0.0, 1.0);
        const double motion_score = 1.0 - std::clamp(
          motion / std::max(1.0, config_.azimuth_limit_deg +
          config_.elevation_limit_deg), 0.0, 1.0);
        const double radial_score = std::exp(
          -std::abs(static_cast<double>(layer) - desired_layer));

        ViewCandidate candidate;
        candidate.direction_target_to_camera = direction;
        candidate.radius_m = radius;
        candidate.azimuth_deg = azimuth_deg;
        candidate.elevation_deg = elevation_deg;
        candidate.nearest_baseline_deg = nearest;
        candidate.motion_angle_deg = motion;
        candidate.score = 0.45 * overlap_score + 0.30 * novelty_score +
          0.15 * motion_score + 0.10 * radial_score;
        candidate.camera_pose.translation() = camera_position;
        candidate.camera_pose.linear() = lookAtOptical(
          candidate.camera_pose.translation(), target);
        std::ostringstream label;
        label << "orbit_a" << azimuth_index << "_e" << elevation_index <<
          "_r" << layer;
        candidate.label = label.str();
        result.push_back(candidate);
      }
    }
  }
  std::stable_sort(
    result.begin(), result.end(),
    [](const ViewCandidate & first, const ViewCandidate & second) {
      return first.score > second.score;
    });
  return result;
}

}  // namespace peach_manipulation_skills
