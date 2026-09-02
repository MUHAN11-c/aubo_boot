// 功能：观察视点候选生成与评分（spherical_adaptive）。从当前相机沿直线截到
// max_camera_step_m，评分以行程最短为主；禁止绕球面、不对侧兜圈。纯核，零 ROS。
#include "peach_manipulation/view_planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "peach_manipulation/math_utils.hpp"

namespace peach_manipulation
{
namespace
{

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
    if (context.foreground_ratio >= 0.0 && context.foreground_ratio < 0.40) {
      break;
    }
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

bool maskFillLow(const ViewContext & context)
{
  return context.foreground_ratio >= 0.0 && context.foreground_ratio < 0.40;
}

Eigen::Vector3d clampToStep(
  const Eigen::Vector3d & current,
  const Eigen::Vector3d & goal,
  double max_step)
{
  const Eigen::Vector3d delta = goal - current;
  const double norm = delta.norm();
  if (norm <= 1.0e-9 || max_step <= 0.0 || norm <= max_step) {
    return goal;
  }
  return current + (max_step / norm) * delta;
}

// 视线（目标→相机）与可达球的最近交点：沿该射线收进，不绕行。
Eigen::Vector3d projectLookRayIntoReach(
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & camera,
  double min_radius,
  double reach)
{
  const Eigen::Vector3d offset = camera - target;
  const double radius = offset.norm();
  if (radius < 1.0e-9 || reach <= 0.0 || camera.norm() <= reach) {
    return camera;
  }
  const Eigen::Vector3d dir = offset / radius;
  const double b = 2.0 * target.dot(dir);
  const double c = target.squaredNorm() - reach * reach;
  const double disc = b * b - 4.0 * c;
  if (disc < 0.0) {
    return target + std::max(min_radius, 0.5 * radius) * dir;
  }
  const double root = std::sqrt(disc);
  const double t_lo = 0.5 * (-b - root);
  const double t_hi = 0.5 * (-b + root);
  double t = std::clamp(radius, t_lo, t_hi);
  t = std::max(t, min_radius);
  return target + t * dir;
}
}  // namespace

double angleDegrees(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  return angleBetweenDeg(
    safeUnit(first, Eigen::Vector3d::UnitX()),
    safeUnit(second, Eigen::Vector3d::UnitX()));
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
  const bool want_closer = (bboxTooSmall(context) || maskFillLow(context)) &&
    radius0 > config_.minimum_radius_m + 0.5 * config_.radial_step_m;
  const bool outside_reach =
    current_camera_position.norm() > config_.workspace_max_reach_m;
  const Eigen::Vector2d desired = visibilityDesired(context, target, side, up);
  const double max_step = std::max(0.01, config_.max_camera_step_m);

  std::vector<ViewCandidate> result;
  auto consider = [&](
    Eigen::Vector3d goal,
    double azimuth_deg,
    double elevation_deg,
    int azimuth_index,
    int elevation_index,
    int layer,
    const char * tag)
    {
      goal = clampToStep(current_camera_position, goal, max_step);
      if (goal.z() < config_.min_camera_height_m) {
        return;
      }
      if (protectedZoneHit(goal, config_.protected_zones)) {
        return;
      }
      const double travel = (goal - current_camera_position).norm();
      Eigen::Vector3d direction = safeUnit(goal - target, front);
      const double motion = angleDegrees(direction, front);
      if (travel < 0.012 && motion < 3.0 && !outside_reach) {
        return;
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
      const double near_score = 1.0 - std::clamp(travel / max_step, 0.0, 1.0);
      const double radius = (goal - target).norm();
      const double standoff_score = 1.0 - std::clamp(
        (radius - config_.minimum_radius_m) /
      std::max(0.05, config_.observation_radius_m), 0.0, 1.0);
      const double align_w = maskFillLow(context) ? 0.22 : 0.18;
      const double overlap_w = maskFillLow(context) ? 0.08 : 0.12;
      const double standoff_w = want_closer || outside_reach ? 0.20 : 0.10;
      const double near_w = 1.0 - align_w - overlap_w - standoff_w;
      ViewCandidate candidate;
      candidate.direction_target_to_camera = direction;
      candidate.radius_m = radius;
      candidate.azimuth_deg = azimuth_deg;
      candidate.elevation_deg = elevation_deg;
      candidate.nearest_baseline_deg = nearest;
      candidate.motion_angle_deg = motion;
      candidate.travel_m = travel;
      candidate.score = near_w * near_score + standoff_w * standoff_score +
        align_w * align + overlap_w * overlap_score;
      candidate.camera_pose.translation() = goal;
      candidate.camera_pose.linear() = lookAtOptical(goal, target);
      std::ostringstream label;
      label << tag << "_a" << azimuth_index << "_e" << elevation_index <<
        "_r" << layer;
      candidate.label = label.str();
      result.push_back(std::move(candidate));
    };

  if (outside_reach || current_camera_position.norm() >
    0.92 * config_.workspace_max_reach_m)
  {
    consider(
      projectLookRayIntoReach(
        target, current_camera_position, config_.minimum_radius_m,
        config_.workspace_max_reach_m),
      0.0, 0.0, 0, 0, 0, "see_in");
  }

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
        consider(
          target + radius * direction,
          azimuth_deg, elevation_deg,
          azimuth_index, elevation_index, layer, "see");
      }
    }
  }
  if (result.empty()) {
    consider(
      target + radius0 * (
        std::cos(radians(config_.azimuth_step_deg)) * front +
        std::sin(radians(config_.azimuth_step_deg)) * side).normalized(),
      config_.azimuth_step_deg, 0.0, 1, 0, 0, "see_fb");
  }
  std::stable_sort(
    result.begin(), result.end(),
    [](const ViewCandidate & first, const ViewCandidate & second) {
      if (std::abs(first.score - second.score) > 1.0e-9) {
        return first.score > second.score;
      }
      return first.travel_m < second.travel_m;
    });
  return result;
}

}  // namespace peach_manipulation
