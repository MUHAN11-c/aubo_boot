#pragma once

#include <string>
#include <vector>

namespace peach2_manipulation
{

/// ROS-free mirror of trajectory_msgs/JointTrajectory (single start time, joint space).
struct TrajectoryPoint
{
  double time_from_start_s{0.0};
  std::vector<double> positions;      ///< [rad]
  std::vector<double> velocities;     ///< [rad/s], may be empty
  std::vector<double> accelerations;  ///< [rad/s^2], may be empty
};

struct JointTrajectory
{
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPoint> points;

  bool empty() const {return points.empty();}
  double duration_s() const {return points.empty() ? 0.0 : points.back().time_from_start_s;}
};

/// Time-reversed trajectory: x_r(t) = x(T - t). Positions reversed, velocities negated,
/// accelerations unchanged (x_r'' = x''(T - t); the old stack negated them, which inverts the
/// dynamics the controller feeds forward).
JointTrajectory reverse_trajectory(const JointTrajectory & forward);

/// Reverse of executing `segments` in order: last segment reversed first. Duplicate junction
/// points are merged. Throws std::invalid_argument when joint names differ between segments.
JointTrajectory reverse_path(const std::vector<JointTrajectory> & segments);

/// Sum of Euclidean joint-space step lengths [rad].
double joint_path_length(const JointTrajectory & trajectory);

/// Max |q_a - q_b| over joints; +inf on size mismatch.
double max_joint_deviation(const std::vector<double> & a, const std::vector<double> & b);

/// Single-point trajectory = the start already is the goal. Executors report it reached
/// without sending it (a one-point goal is rejected by execute_trajectory / JTC).
inline bool is_null_motion(const JointTrajectory & trajectory)
{
  return trajectory.points.size() == 1U;
}

}  // namespace peach2_manipulation
