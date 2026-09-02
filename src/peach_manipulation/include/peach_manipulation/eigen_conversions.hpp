// 功能：geometry_msgs ↔ Eigen 转换（内部走 tf2_eigen 官方实现）。包内私用。
#ifndef PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_
#define PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace peach_manipulation
{

// 非法四元数（非有限/近零范数）回退为单位旋转、仅取平移，
// 避免污染下游规划；tf2::fromMsg 本身不检查四元数有效性。
inline Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (quaternion.coeffs().allFinite() && quaternion.norm() >= 1.0e-9) {
    tf2::fromMsg(pose, transform);
  } else {
    transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  }
  return transform;
}

inline geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  return tf2::toMsg(transform);
}

inline Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point & point)
{
  Eigen::Vector3d out;
  tf2::fromMsg(point, out);
  return out;
}

inline Eigen::Vector3d vectorToEigen(const geometry_msgs::msg::Vector3 & vector)
{
  Eigen::Vector3d out;
  tf2::fromMsg(vector, out);
  return out;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_
