#ifndef COORDINATE_TRANSFORMS_ROS2_ROTATION_CONVERSION_ROS2_HPP
#define COORDINATE_TRANSFORMS_ROS2_ROTATION_CONVERSION_ROS2_HPP

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "coordinate_transforms/core/rotation_conversion.hpp"
#include "coordinate_transforms/core/coordinate_systems.hpp"
#include <array>

namespace coordinate_transforms {
namespace ros2 {

/** geometry_msgs::Quaternion (x,y,z,w) -> core Quaternion. */
core::Quaternion to_core_quaternion(const geometry_msgs::msg::Quaternion& q);

/** core Quaternion -> geometry_msgs::Quaternion. */
geometry_msgs::msg::Quaternion to_ros_quaternion(const core::Quaternion& q);

/** Pose -> origin (Point3) + rotation matrix (3x3). */
void pose_to_origin_rotation(const geometry_msgs::msg::Pose& pose,
                             core::Point3& origin, core::Matrix3& R);

/** Origin + R -> Pose. */
geometry_msgs::msg::Pose origin_rotation_to_pose(const core::Point3& origin, const core::Matrix3& R);

/** RPY (rad) -> geometry_msgs::Quaternion. */
geometry_msgs::msg::Quaternion rpy_to_ros_quaternion(double roll, double pitch, double yaw);

/** geometry_msgs::Quaternion -> RPY (rad). */
core::RPY ros_quaternion_to_rpy(const geometry_msgs::msg::Quaternion& q);

}  // namespace ros2
}  // namespace coordinate_transforms

#endif
