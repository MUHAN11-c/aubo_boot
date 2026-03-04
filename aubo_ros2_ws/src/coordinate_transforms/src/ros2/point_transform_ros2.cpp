#include "coordinate_transforms/ros2/point_transform_ros2.hpp"
#include "coordinate_transforms/ros2/rotation_conversion_ros2.hpp"

namespace coordinate_transforms {
namespace ros2 {

core::Matrix4 transform_stamped_to_matrix4(const geometry_msgs::msg::TransformStamped& t) {
  const auto& tr = t.transform.translation;
  const auto& r = t.transform.rotation;
  core::Quaternion q = {r.x, r.y, r.z, r.w};
  core::Matrix3 R = core::quaternion_to_rotation_matrix(q);
  core::Matrix4 T;
  T[0] = R[0]; T[1] = R[1]; T[2] = R[2];  T[3] = tr.x;
  T[4] = R[3]; T[5] = R[4]; T[6] = R[5];  T[7] = tr.y;
  T[8] = R[6]; T[9] = R[7]; T[10] = R[8]; T[11] = tr.z;
  T[12] = 0;   T[13] = 0;   T[14] = 0;   T[15] = 1;
  return T;
}

geometry_msgs::msg::Point transform_point_ros2(
    const geometry_msgs::msg::Point& p, const geometry_msgs::msg::TransformStamped& t) {
  core::Matrix4 T = transform_stamped_to_matrix4(t);
  core::Point3 pc = {p.x, p.y, p.z};
  core::Point3 out = core::transform_point(pc, T);
  geometry_msgs::msg::Point result;
  result.x = out[0];
  result.y = out[1];
  result.z = out[2];
  return result;
}

geometry_msgs::msg::Pose transform_pose_ros2(
    const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::TransformStamped& t) {
  core::Point3 origin;
  core::Matrix3 R;
  pose_to_origin_rotation(pose, origin, R);
  core::Matrix4 T = transform_stamped_to_matrix4(t);
  core::Point3 new_origin;
  core::Matrix3 new_R;
  core::transform_pose(origin, R, T, new_origin, new_R);
  return origin_rotation_to_pose(new_origin, new_R);
}

}  // namespace ros2
}  // namespace coordinate_transforms
