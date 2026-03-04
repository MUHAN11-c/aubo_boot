#ifndef COORDINATE_TRANSFORMS_CORE_ROTATION_CONVERSION_HPP
#define COORDINATE_TRANSFORMS_CORE_ROTATION_CONVERSION_HPP

#include <array>
#include <cmath>

namespace coordinate_transforms {
namespace core {

// Quaternion (x, y, z, w) - ROS convention
using Quaternion = std::array<double, 4>;
// RPY in radians: (roll, pitch, yaw)
using RPY = std::array<double, 3>;
// 3x3 row-major
using Matrix3 = std::array<double, 9>;

// --- Quaternion <-> Rotation matrix ---
Matrix3 quaternion_to_rotation_matrix(const Quaternion& q);
Quaternion rotation_matrix_to_quaternion(const Matrix3& R);

// --- Quaternion <-> RPY (roll-pitch-yaw, radians) ---
Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
Quaternion rpy_to_quaternion(const RPY& rpy);
RPY quaternion_to_rpy(const Quaternion& q);

// --- Rotation matrix <-> RPY ---
Matrix3 rpy_to_rotation_matrix(double roll, double pitch, double yaw);
Matrix3 rpy_to_rotation_matrix(const RPY& rpy);
RPY rotation_matrix_to_rpy(const Matrix3& R);

// --- Helpers ---
// 归一化: q := q / |q|，范数 |q| = sqrt(x²+y²+z²+w²)，保证单位四元数用于旋转表示。
inline void quaternion_normalize(Quaternion& q) {
  double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-10) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}

}  // namespace core
}  // namespace coordinate_transforms

#endif
