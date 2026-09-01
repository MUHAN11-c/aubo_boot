// 功能：geometry_msgs ↔ Eigen 转换。包内私用。
#ifndef PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_
#define PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace peach_manipulation
{

// 非法四元数（非有限/近零范数）回退为单位旋转，避免污染下游规划。
inline Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

inline geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

inline Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y, point.z};
}

inline Eigen::Vector3d vectorToEigen(const geometry_msgs::msg::Vector3 & vector)
{
  return {vector.x, vector.y, vector.z};
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__EIGEN_CONVERSIONS_HPP_
