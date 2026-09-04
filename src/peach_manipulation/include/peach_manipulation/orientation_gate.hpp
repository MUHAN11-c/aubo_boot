// 功能：MoveIt 姿态保持门（OrientationConstraint）组装。包内私用。
// 只挂已齐 LIN（分档要求起点对轴，拦笛卡尔插值中途侧翻）。
// 未齐第一段 LIN-align 与 CIRC 不挂：Jazzy ValidateSolution 验每个路点含起点，
// 起点相对目标 >20° 会 INVALID_MOTION_PLAN。接触不用 PTP。
#ifndef PEACH_MANIPULATION__ORIENTATION_GATE_HPP_
#define PEACH_MANIPULATION__ORIENTATION_GATE_HPP_

#include <Eigen/Geometry>

#include <string>

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_msgs/msg/constraints.hpp>

#include "peach_manipulation/math_utils.hpp"

namespace peach_manipulation
{

// tip 姿态不偏离 target_pose 超过 tol_deg 的三轴等宽容差约束集。
inline moveit_msgs::msg::Constraints makeOrientationGate(
  const std::string & link_name, const std::string & frame_id,
  const Eigen::Isometry3d & target_pose, double tol_deg,
  const std::string & name)
{
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.link_name = link_name;
  orientation.header.frame_id = frame_id;
  orientation.orientation = tf2::toMsg(Eigen::Quaterniond(target_pose.linear()));
  const double tol = tol_deg * kPi / 180.0;
  orientation.absolute_x_axis_tolerance = tol;
  orientation.absolute_y_axis_tolerance = tol;
  orientation.absolute_z_axis_tolerance = tol;
  orientation.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  constraints.name = name;
  constraints.orientation_constraints.push_back(orientation);
  return constraints;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__ORIENTATION_GATE_HPP_
