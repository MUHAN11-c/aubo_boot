#ifndef COORDINATE_TRANSFORMS_ROS2_COORDINATE_SYSTEMS_ROS2_HPP
#define COORDINATE_TRANSFORMS_ROS2_COORDINATE_SYSTEMS_ROS2_HPP

#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "coordinate_transforms/core/coordinate_systems.hpp"
#include <array>
#include <vector>

namespace coordinate_transforms {
namespace ros2 {

/** Build CameraIntrinsics from sensor_msgs/CameraInfo (K matrix). */
core::CameraIntrinsics camera_info_to_intrinsics(const sensor_msgs::msg::CameraInfo& info);

/** project_3d_to_2d: core call, result as geometry_msgs/Point (x=u, y=v, z=0). */
geometry_msgs::msg::Point project_3d_to_2d_point(
  const geometry_msgs::msg::Point& point_3d, const sensor_msgs::msg::CameraInfo& info);

/** unproject: pixel (u,v) + depth -> geometry_msgs/Point (camera frame). */
geometry_msgs::msg::Point unproject_2d_to_3d_point(
  double u, double v, double depth, const sensor_msgs::msg::CameraInfo& info);

/** Reprojection error using CameraInfo. */
double reprojection_error_ros2(
  const geometry_msgs::msg::Point& point_2d_obs_uv,
  const geometry_msgs::msg::Point& point_3d,
  const sensor_msgs::msg::CameraInfo& info,
  bool squared = false);

}  // namespace ros2
}  // namespace coordinate_transforms

#endif
