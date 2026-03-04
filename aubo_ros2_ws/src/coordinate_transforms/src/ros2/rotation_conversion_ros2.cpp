#include "coordinate_transforms/ros2/rotation_conversion_ros2.hpp"

namespace coordinate_transforms {
namespace ros2 {

core::Quaternion to_core_quaternion(const geometry_msgs::msg::Quaternion& q) {
  return {q.x, q.y, q.z, q.w};
}

geometry_msgs::msg::Quaternion to_ros_quaternion(const core::Quaternion& q) {
  geometry_msgs::msg::Quaternion out;
  out.x = q[0];
  out.y = q[1];
  out.z = q[2];
  out.w = q[3];
  return out;
}

void pose_to_origin_rotation(const geometry_msgs::msg::Pose& pose,
                             core::Point3& origin, core::Matrix3& R) {
  origin[0] = pose.position.x;
  origin[1] = pose.position.y;
  origin[2] = pose.position.z;
  core::Quaternion q = to_core_quaternion(pose.orientation);
  R = core::quaternion_to_rotation_matrix(q);
}

geometry_msgs::msg::Pose origin_rotation_to_pose(const core::Point3& origin, const core::Matrix3& R) {
  geometry_msgs::msg::Pose out;
  out.position.x = origin[0];
  out.position.y = origin[1];
  out.position.z = origin[2];
  core::Quaternion q = core::rotation_matrix_to_quaternion(R);
  out.orientation = to_ros_quaternion(q);
  return out;
}

geometry_msgs::msg::Quaternion rpy_to_ros_quaternion(double roll, double pitch, double yaw) {
  core::Quaternion q = core::rpy_to_quaternion(roll, pitch, yaw);
  return to_ros_quaternion(q);
}

core::RPY ros_quaternion_to_rpy(const geometry_msgs::msg::Quaternion& q) {
  core::Quaternion cq = to_core_quaternion(q);
  return core::quaternion_to_rpy(cq);
}

}  // namespace ros2
}  // namespace coordinate_transforms
