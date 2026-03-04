#ifndef COORDINATE_TRANSFORMS_ROS2_POINT_TRANSFORM_ROS2_HPP
#define COORDINATE_TRANSFORMS_ROS2_POINT_TRANSFORM_ROS2_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "coordinate_transforms/core/point_transform.hpp"
#include "coordinate_transforms/core/rotation_conversion.hpp"

namespace coordinate_transforms {
namespace ros2 {

/** TransformStamped -> 4x4 matrix (row-major). */
core::Matrix4 transform_stamped_to_matrix4(const geometry_msgs::msg::TransformStamped& t);

/** Transform point using TransformStamped. */
geometry_msgs::msg::Point transform_point_ros2(
  const geometry_msgs::msg::Point& p, const geometry_msgs::msg::TransformStamped& t);

/** Transform pose: result_pose = t * pose (in same frame sense as TF). */
geometry_msgs::msg::Pose transform_pose_ros2(
  const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::TransformStamped& t);

}  // namespace ros2
}  // namespace coordinate_transforms

#endif
