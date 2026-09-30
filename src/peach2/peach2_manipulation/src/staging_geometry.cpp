#include "peach2_manipulation/staging_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "peach2_end_effector/types.hpp"

namespace peach2_manipulation
{

Eigen::Isometry3d staging_pose(
  const Eigen::Isometry3d & pregrasp, const Eigen::Vector3d & bag_bottom, double distance_m)
{
  const Eigen::Vector3d axis = pregrasp.linear().col(2).normalized();
  const double d = std::clamp(distance_m, kStagingMinM, kStagingMaxM);
  const double pregrasp_above_bottom = (pregrasp.translation() - bag_bottom).dot(axis);
  const double back = d + std::max(0.0, pregrasp_above_bottom);
  Eigen::Isometry3d out = pregrasp;
  out.translation() = pregrasp.translation() - back * axis;
  return out;
}

Eigen::Isometry3d tcp_for_blade(
  const Eigen::Vector3d & blade_target, const Eigen::Matrix3d & rotation,
  const Eigen::Isometry3d & blade_in_tcp)
{
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  tcp.linear() = rotation;
  tcp.translation() = blade_target - rotation * blade_in_tcp.translation();
  return tcp;
}

Eigen::Isometry3d pose_with_roll(
  const Eigen::Vector3d & position, const Eigen::Vector3d & axis, double roll_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = peach2_end_effector::tcp_rotation(axis, roll_rad);
  pose.translation() = position;
  return pose;
}

Residual pregrasp_residual(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & target,
  const ResidualTolerance & tolerance)
{
  Residual r;
  const Eigen::Vector3d z = target.linear().col(2).normalized();
  const Eigen::Vector3d dp = actual.translation() - target.translation();
  r.axial_m = std::fabs(dp.dot(z));
  r.lateral_m = (dp - dp.dot(z) * z).norm();
  const double c = std::clamp(actual.linear().col(2).normalized().dot(z), -1.0, 1.0);
  r.angle_rad = std::acos(c);
  if (!std::isfinite(r.lateral_m) || !std::isfinite(r.axial_m) || !std::isfinite(r.angle_rad)) {
    r.reason = "residual_not_finite";
    return r;
  }
  if (r.lateral_m > tolerance.lateral_m) {
    r.reason = "lateral_residual";
  } else if (r.angle_rad > tolerance.angle_rad) {
    r.reason = "angle_residual";
  } else if (r.axial_m > tolerance.axial_m) {
    r.reason = "axial_residual";
  } else {
    r.ok = true;
    r.reason = "ok";
  }
  return r;
}

InsertGeometry insert_geometry(const Eigen::Isometry3d & pregrasp, const Eigen::Vector3d & tcp_goal)
{
  const Eigen::Vector3d z = pregrasp.linear().col(2).normalized();
  const Eigen::Vector3d dp = tcp_goal - pregrasp.translation();
  InsertGeometry g;
  g.travel_m = dp.dot(z);
  g.lateral_m = (dp - g.travel_m * z).norm();
  return g;
}

}  // namespace peach2_manipulation
