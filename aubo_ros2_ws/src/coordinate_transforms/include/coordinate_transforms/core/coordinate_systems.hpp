#ifndef COORDINATE_TRANSFORMS_CORE_COORDINATE_SYSTEMS_HPP
#define COORDINATE_TRANSFORMS_CORE_COORDINATE_SYSTEMS_HPP

#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace coordinate_transforms {
namespace core {

// Simple 3D point
using Point3 = std::array<double, 3>;
// 2D point (u, v) pixel
using Point2 = std::array<double, 2>;
// 3x3 matrix row-major
using Matrix3 = std::array<double, 9>;
// 4x4 matrix row-major
using Matrix4 = std::array<double, 16>;

/** Camera intrinsics: fx, fy, cx, cy. K = [fx 0 cx; 0 fy cy; 0 0 1] */
struct CameraIntrinsics {
  double fx{1.0}, fy{1.0}, cx{0.0}, cy{0.0};
};

// --- 3D -> 2D (project) ---
/** Project single 3D point in camera frame to pixel (u,v). Returns invalid (e.g. negative u) if Z<=0. */
Point2 project_3d_to_2d(const Point3& point_3d, const CameraIntrinsics& K);
/** Batch: P_3d (N x 3) -> uv (N x 2). Z<=0 rows get invalid marker (e.g. -1,-1). */
std::vector<Point2> project_3d_to_2d_batch(const std::vector<Point3>& P_3d, const CameraIntrinsics& K);

// --- 2D + depth -> 3D (unproject) ---
/** Unproject pixel (u,v) with depth Z to camera 3D point (X,Y,Z). */
Point3 unproject_2d_to_3d(double u, double v, double depth, const CameraIntrinsics& K);
/** Batch: uv (N x 2), depths (N) -> P_3d (N x 3). */
std::vector<Point3> unproject_2d_to_3d_batch(
  const std::vector<Point2>& uv, const std::vector<double>& depths, const CameraIntrinsics& K);

// --- Reprojection error ---
/** Reprojection error (pixel distance) between observed 2D and projected 3D. Returns NaN if Z<=0. */
double reprojection_error(const Point2& point_2d_obs, const Point3& point_3d, const CameraIntrinsics& K, bool squared = false);
/** Per-point reprojection errors. */
std::vector<double> reprojection_error_batch(
  const std::vector<Point2>& uv_obs, const std::vector<Point3>& P_3d, const CameraIntrinsics& K, bool squared = false);
/** RMS of reprojection errors. */
double reprojection_error_rms(const std::vector<Point2>& uv_obs, const std::vector<Point3>& P_3d, const CameraIntrinsics& K);

// --- Camera <-> World (4x4 transform) ---
/** Transform point from camera to world: p_w = T_w_c * p_c (4x4 applied to [x,y,z,1]). */
Point3 transform_camera_to_world(const Point3& p_c, const Matrix4& T_w_c);
/** Transform point from world to camera. */
Point3 transform_world_to_camera(const Point3& p_w, const Matrix4& T_w_c);

}  // namespace core
}  // namespace coordinate_transforms

#endif
