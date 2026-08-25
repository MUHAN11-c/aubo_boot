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

#include <Eigen/Core>

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
Eigen::Vector2d visibilityDesired(
  const ViewContext & context,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & side,
  const Eigen::Vector3d & up)
{
  Eigen::Vector2d desired = Eigen::Vector2d::Zero();
  const int width = std::max(1, context.image_width);
  const int height = std::max(1, context.image_height);
  if (context.bbox_valid && context.bbox_w > 0 && context.bbox_h > 0) {
    const double cx = static_cast<double>(context.bbox_x) +
      0.5 * static_cast<double>(context.bbox_w);
    const double cy = static_cast<double>(context.bbox_y) +
      0.5 * static_cast<double>(context.bbox_h);
    desired.x() += (cx - 0.5 * width) / (0.5 * width);
    desired.y() += (cy - 0.5 * height) / (0.5 * height);
    constexpr double kMarginPx = 8.0;
    const double clip_left = std::max(
      0.0, kMarginPx - static_cast<double>(context.bbox_x));
    const double clip_right = std::max(
      0.0, static_cast<double>(context.bbox_x + context.bbox_w) -
      (width - kMarginPx));
    const double clip_top = std::max(
      0.0, kMarginPx - static_cast<double>(context.bbox_y));
    const double clip_bottom = std::max(
      0.0, static_cast<double>(context.bbox_y + context.bbox_h) -
      (height - kMarginPx));
    // 框贴边说明袋/果被裁切：相机沿光学 +X/+Y 移动才能把裁掉的一侧纳入画面。
    desired.x() += 1.5 * (clip_right - clip_left) / width;
    desired.y() += 1.5 * (clip_bottom - clip_top) / height;
  }
  for (const auto & neighbor : context.neighbor_centers) {
    const Eigen::Vector3d rel = neighbor - target;
    desired.x() += 0.35 * rel.dot(side);
    desired.y() += 0.35 * rel.dot(up);
  }
  if (desired.norm() < 1.0e-6) {
    desired.x() = 1.0;
  }
  return desired.normalized();
}

bool bboxTooSmall(const ViewContext & context)
{
  if (!context.bbox_valid || context.bbox_w <= 0 || context.bbox_h <= 0) {
    return false;
  }
  const double area = static_cast<double>(context.bbox_w) *
    static_cast<double>(context.bbox_h);
  const double image = static_cast<double>(
    std::max(1, context.image_width) * std::max(1, context.image_height));
  return area / image < 0.04;
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

  const double current_radius = (current_camera_position - target).norm();
  const double radius0 = std::clamp(
    current_radius, config_.minimum_radius_m, 2.0);
  const bool want_closer = bboxTooSmall(context) &&
    radius0 > config_.minimum_radius_m + 0.5 * config_.radial_step_m;
  const Eigen::Vector2d desired = visibilityDesired(context, target, side, up);

  std::vector<ViewCandidate> result;
  const int azimuth_steps = 1;
  const int elevation_steps = config_.elevation_limit_deg > 1.0e-6 ? 1 : 0;
  const int layer_count = want_closer ? 2 : 1;
  for (int layer = 0; layer < layer_count; ++layer) {
    const double radius = std::max(
      config_.minimum_radius_m, radius0 - layer * config_.radial_step_m);
    for (int azimuth_index = -azimuth_steps;
      azimuth_index <= azimuth_steps; ++azimuth_index)
    {
      for (int elevation_index = -elevation_steps;
        elevation_index <= elevation_steps; ++elevation_index)
      {
        if (azimuth_index == 0 && elevation_index == 0) {
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
        if (camera_position.z() < config_.min_camera_height_m) {
          continue;
        }
        if (protectedZoneHit(camera_position, config_.protected_zones)) {
          continue;
        }

        const double motion = angleDegrees(direction, front);
        if (motion < 3.0) {
          continue;
        }
        double nearest = std::numeric_limits<double>::max();
        for (const auto & previous : observed) {
          nearest = std::min(nearest, angleDegrees(direction, previous));
        }
        const double move_side = direction.dot(side);
        const double move_up = direction.dot(up);
        Eigen::Vector2d move(move_side, move_up);
        if (move.norm() > 1.0e-9) {
          move.normalize();
        }
        const double align = 0.5 * (move.dot(desired) + 1.0);
        const double baseline_error =
          (nearest - config_.preferred_baseline_deg) /
          std::max(1.0, config_.preferred_baseline_deg * 0.7);
        const double overlap_score = std::exp(-0.5 * baseline_error * baseline_error);
        const double motion_score = 1.0 - std::clamp(
          motion / std::max(1.0, config_.azimuth_step_deg +
          std::max(1.0, config_.elevation_step_deg)), 0.0, 1.0);

        ViewCandidate candidate;
        candidate.direction_target_to_camera = direction;
        candidate.radius_m = radius;
        candidate.azimuth_deg = azimuth_deg;
        candidate.elevation_deg = elevation_deg;
        candidate.nearest_baseline_deg = nearest;
        candidate.motion_angle_deg = motion;
        candidate.score = 0.55 * align + 0.25 * overlap_score + 0.20 * motion_score;
        candidate.camera_pose.translation() = camera_position;
        candidate.camera_pose.linear() = lookAtOptical(
          candidate.camera_pose.translation(), target);
        std::ostringstream label;
        label << "see_a" << azimuth_index << "_e" << elevation_index <<
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
