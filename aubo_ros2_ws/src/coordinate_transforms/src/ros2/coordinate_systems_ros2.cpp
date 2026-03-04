#include "coordinate_transforms/ros2/coordinate_systems_ros2.hpp"

namespace coordinate_transforms {
namespace ros2 {

core::CameraIntrinsics camera_info_to_intrinsics(const sensor_msgs::msg::CameraInfo& info) {
  core::CameraIntrinsics K;
  if (info.k.size() >= 9) {
    K.fx = info.k[0];
    K.fy = info.k[4];
    K.cx = info.k[2];
    K.cy = info.k[5];
  }
  return K;
}

geometry_msgs::msg::Point project_3d_to_2d_point(
    const geometry_msgs::msg::Point& point_3d, const sensor_msgs::msg::CameraInfo& info) {
  core::CameraIntrinsics K = camera_info_to_intrinsics(info);
  core::Point3 p = {point_3d.x, point_3d.y, point_3d.z};
  core::Point2 uv = core::project_3d_to_2d(p, K);
  geometry_msgs::msg::Point out;
  out.x = uv[0];
  out.y = uv[1];
  out.z = 0.0;
  return out;
}

geometry_msgs::msg::Point unproject_2d_to_3d_point(
    double u, double v, double depth, const sensor_msgs::msg::CameraInfo& info) {
  core::CameraIntrinsics K = camera_info_to_intrinsics(info);
  core::Point3 p = core::unproject_2d_to_3d(u, v, depth, K);
  geometry_msgs::msg::Point out;
  out.x = p[0];
  out.y = p[1];
  out.z = p[2];
  return out;
}

double reprojection_error_ros2(
    const geometry_msgs::msg::Point& point_2d_obs_uv,
    const geometry_msgs::msg::Point& point_3d,
    const sensor_msgs::msg::CameraInfo& info,
    bool squared) {
  core::CameraIntrinsics K = camera_info_to_intrinsics(info);
  core::Point2 uv = {point_2d_obs_uv.x, point_2d_obs_uv.y};
  core::Point3 p = {point_3d.x, point_3d.y, point_3d.z};
  return core::reprojection_error(uv, p, K, squared);
}

}  // namespace ros2
}  // namespace coordinate_transforms
