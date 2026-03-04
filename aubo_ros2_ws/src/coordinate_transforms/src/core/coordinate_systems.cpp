#include "coordinate_transforms/core/coordinate_systems.hpp"
#include <limits>

namespace coordinate_transforms {
namespace core {

namespace {
constexpr double kInvalidPixel = -1.0;
}

// 针孔投影：相机系 3D (X,Y,Z) → 像素 (u,v)。内参 K = [fx 0 cx; 0 fy cy; 0 0 1]。
// 公式: u = fx*X/Z + cx,  v = fy*Y/Z + cy。Z≤0 视为无效（点在相机后方）。
Point2 project_3d_to_2d(const Point3& point_3d, const CameraIntrinsics& K) {
  const double Z = point_3d[2];
  if (Z <= 0.0) return {kInvalidPixel, kInvalidPixel};
  const double u = K.fx * point_3d[0] / Z + K.cx;
  const double v = K.fy * point_3d[1] / Z + K.cy;
  return {u, v};
}

std::vector<Point2> project_3d_to_2d_batch(const std::vector<Point3>& P_3d, const CameraIntrinsics& K) {
  std::vector<Point2> out(P_3d.size());
  for (size_t i = 0; i < P_3d.size(); ++i)
    out[i] = project_3d_to_2d(P_3d[i], K);
  return out;
}

// 反投影：像素 (u,v) + 深度 Z → 相机系 3D (X,Y,Z)。
// 公式: X = (u - cx)*Z/fx,  Y = (v - cy)*Z/fy,  Z = depth（与投影公式逆推）。
Point3 unproject_2d_to_3d(double u, double v, double depth, const CameraIntrinsics& K) {
  const double Z = depth;
  const double X = (u - K.cx) * Z / K.fx;
  const double Y = (v - K.cy) * Z / K.fy;
  return {X, Y, Z};
}

std::vector<Point3> unproject_2d_to_3d_batch(
    const std::vector<Point2>& uv, const std::vector<double>& depths, const CameraIntrinsics& K) {
  if (uv.size() != depths.size()) throw std::invalid_argument("uv and depths size mismatch");
  std::vector<Point3> out(uv.size());
  for (size_t i = 0; i < uv.size(); ++i)
    out[i] = unproject_2d_to_3d(uv[i][0], uv[i][1], depths[i], K);
  return out;
}

// 重投影误差：观测像素 (u_obs,v_obs) 与将 3D 点投影得到的 (u_proj,v_proj) 的像素距离。
// 公式: e² = (u_obs - u_proj)² + (v_obs - v_proj)²；返回 e（或 squared 时返回 e²）。用于标定/PnP/BA。
double reprojection_error(const Point2& point_2d_obs, const Point3& point_3d, const CameraIntrinsics& K, bool squared) {
  Point2 proj = project_3d_to_2d(point_3d, K);
  if (proj[0] == kInvalidPixel) return std::numeric_limits<double>::quiet_NaN();
  const double du = point_2d_obs[0] - proj[0];
  const double dv = point_2d_obs[1] - proj[1];
  const double e2 = du * du + dv * dv;
  return squared ? e2 : std::sqrt(e2);
}

std::vector<double> reprojection_error_batch(
    const std::vector<Point2>& uv_obs, const std::vector<Point3>& P_3d, const CameraIntrinsics& K, bool squared) {
  if (uv_obs.size() != P_3d.size()) throw std::invalid_argument("uv_obs and P_3d size mismatch");
  std::vector<double> out(uv_obs.size());
  for (size_t i = 0; i < uv_obs.size(); ++i)
    out[i] = reprojection_error(uv_obs[i], P_3d[i], K, squared);
  return out;
}

// 重投影误差 RMS。公式: RMS = sqrt( (1/N) * sum_i e_i² )，N 为有效点数。
double reprojection_error_rms(const std::vector<Point2>& uv_obs, const std::vector<Point3>& P_3d, const CameraIntrinsics& K) {
  if (uv_obs.empty()) return 0.0;
  double sum_sq = 0.0;
  int count = 0;
  for (size_t i = 0; i < uv_obs.size(); ++i) {
    double e = reprojection_error(uv_obs[i], P_3d[i], K, true);
    if (std::isfinite(e)) { sum_sq += e; count++; }
  }
  if (count == 0) return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(sum_sq / count);
}

// 齐次变换: p_out = T * [p; 1]，取前三维。即 p' = R*p + t（行优先存储 T = [R|t; 0 0 0 1]）。
static Point3 apply_transform_4x4(const Point3& p, const Matrix4& T) {
  double x = T[0]*p[0] + T[1]*p[1] + T[2]*p[2] + T[3];
  double y = T[4]*p[0] + T[5]*p[1] + T[6]*p[2] + T[7];
  double z = T[8]*p[0] + T[9]*p[1] + T[10]*p[2] + T[11];
  return {x, y, z};
}

// 相机系 → 世界系: p_w = T_w_c * p_c（齐次）。
Point3 transform_camera_to_world(const Point3& p_c, const Matrix4& T_w_c) {
  return apply_transform_4x4(p_c, T_w_c);
}

// 刚体逆: T^{-1} = [R^T | -R^T*t; 0 0 0 1]。用于世界→相机: p_c = T_c_w * p_w。
static Matrix4 invert_4x4_rigid(const Matrix4& T) {
  Matrix4 inv{};
  inv[0] = T[0]; inv[1] = T[4]; inv[2] = T[8];
  inv[4] = T[1]; inv[5] = T[5]; inv[6] = T[9];
  inv[8] = T[2]; inv[9] = T[6]; inv[10] = T[10];
  inv[3] = -(inv[0]*T[3] + inv[1]*T[7] + inv[2]*T[11]);
  inv[7] = -(inv[4]*T[3] + inv[5]*T[7] + inv[6]*T[11]);
  inv[11] = -(inv[8]*T[3] + inv[9]*T[7] + inv[10]*T[11]);
  inv[12] = 0; inv[13] = 0; inv[14] = 0; inv[15] = 1;
  return inv;
}

Point3 transform_world_to_camera(const Point3& p_w, const Matrix4& T_w_c) {
  Matrix4 T_c_w = invert_4x4_rigid(T_w_c);
  return apply_transform_4x4(p_w, T_c_w);
}

}  // namespace core
}  // namespace coordinate_transforms
