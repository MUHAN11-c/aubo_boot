#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <string>

namespace peach2_manipulation
{

/// Allowed staging distance below the bag bottom [m] (方案 §12: 0.15-0.25 m, outside canopy).
constexpr double kStagingMinM = 0.15;
constexpr double kStagingMaxM = 0.25;

/// Staging pose: same orientation as the pregrasp, on the pregrasp line (so staging ->
/// pregrasp is a pure axial LIN), `distance_m` (clamped to [0.15, 0.25]) below whichever of
/// bag bottom / pregrasp is lower along the axis. Axis = pregrasp +Z (bag bottom -> neck).
Eigen::Isometry3d staging_pose(
  const Eigen::Isometry3d & pregrasp, const Eigen::Vector3d & bag_bottom, double distance_m);

/// TCP pose with rotation `rotation` whose blade frame lands on `blade_target`.
Eigen::Isometry3d tcp_for_blade(
  const Eigen::Vector3d & blade_target, const Eigen::Matrix3d & rotation,
  const Eigen::Isometry3d & blade_in_tcp);

/// Pregrasp pose at `position` with +Z = axis and roll `roll_rad` (peach2_end_effector
/// roll_frame convention).
Eigen::Isometry3d pose_with_roll(
  const Eigen::Vector3d & position, const Eigen::Vector3d & axis, double roll_rad);

struct ResidualTolerance
{
  double lateral_m{0.003};
  double axial_m{0.005};
  double angle_rad{2.0 * M_PI / 180.0};
};

/// Pregrasp residual in the target frame: lateral = offset normal to the target +Z, axial =
/// along it, angle = tilt between +Z axes (roll is ignored; plugins own roll).
struct Residual
{
  double lateral_m{0.0};
  double axial_m{0.0};
  double angle_rad{0.0};
  bool ok{false};
  std::string reason;
};

Residual pregrasp_residual(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & target,
  const ResidualTolerance & tolerance);

/// Signed TCP travel along the pregrasp +Z from pregrasp to `tcp_goal`, and the lateral
/// offset of the goal from the pregrasp line.
struct InsertGeometry
{
  double travel_m{0.0};
  double lateral_m{0.0};
};

InsertGeometry insert_geometry(
  const Eigen::Isometry3d & pregrasp,
  const Eigen::Vector3d & tcp_goal);

}  // namespace peach2_manipulation
