/**
 * 旋转表示转换（无 Eigen 分支，手写实现）
 * 当未找到 Eigen3 时编译此文件；否则使用 rotation_conversion_eigen.cpp。
 * 四元数 q = (x, y, z, w)，单位四元数表示旋转，与 ROS/tf2 约定一致。
 */
#include "coordinate_transforms/core/rotation_conversion.hpp"
#include <cmath>

namespace coordinate_transforms {
namespace core {

// clamp: x 限制在 [lo, hi]，用于 asin 前保证 |sinp|<=1 避免数值溢出
static double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 四元数 → 旋转矩阵 R（行优先 3×3）
// 公式: R = [ 1-2(y²+z²)   2(xy-zw)   2(xz+yw)
//            2(xy+zw)   1-2(x²+z²)  2(yz-xw)
//            2(xz-yw)   2(yz+xw)   1-2(x²+y²) ]
// 即 R_ij 由 q 的二次型得到，|q|=1 时 R 为正交阵。
Matrix3 quaternion_to_rotation_matrix(const Quaternion& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  Matrix3 R;
  R[0] = 1 - 2*(y*y + z*z);  R[1] = 2*(x*y - z*w);     R[2] = 2*(x*z + y*w);
  R[3] = 2*(x*y + z*w);      R[4] = 1 - 2*(x*x + z*z); R[5] = 2*(y*z - x*w);
  R[6] = 2*(x*z - y*w);      R[7] = 2*(y*z + x*w);     R[8] = 1 - 2*(x*x + y*y);
  return R;
}

// 旋转矩阵 R → 单位四元数 q。采用 Shepperd 方法，按 trace 与对角元选最大分支保证数值稳定。
// 公式: w = (1/4s)(trace+1), x = (R[2,1]-R[1,2])/(4s), y = (R[0,2]-R[2,0])/(4s), z = (R[1,0]-R[0,1])/(4s)
// 其中 s = sqrt(trace+1)，或从其他分支用 R 元素解出 q 分量，最后归一化 |q|=1。
Quaternion rotation_matrix_to_quaternion(const Matrix3& R) {
  const double trace = R[0] + R[4] + R[8];
  Quaternion q;
  if (trace > 0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    q[3] = 0.25 / s;
    q[0] = (R[7] - R[5]) * s;
    q[1] = (R[2] - R[6]) * s;
    q[2] = (R[3] - R[1]) * s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
    q[3] = (R[7] - R[5]) / s;
    q[0] = 0.25 * s;
    q[1] = (R[1] + R[3]) / s;
    q[2] = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
    q[3] = (R[2] - R[6]) / s;
    q[0] = (R[1] + R[3]) / s;
    q[1] = 0.25 * s;
    q[2] = (R[5] + R[7]) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
    q[3] = (R[3] - R[1]) / s;
    q[0] = (R[2] + R[6]) / s;
    q[1] = (R[5] + R[7]) / s;
    q[2] = 0.25 * s;
  }
  quaternion_normalize(q);
  return q;
}

// 欧拉角 RPY (roll, pitch, yaw，弧度) → 四元数。内旋顺序: 绕 X 转 roll，再绕 Y 转 pitch，再绕 Z 转 yaw。
// 公式: q = q_roll * q_pitch * q_yaw，其中各轴半角: q_roll = (sin(r/2),0,0,cos(r/2)) 等，
// 展开得: qx = sr*cp*cy - cr*sp*sy, qy = cr*sp*cy + sr*cp*sy, qz = cr*cp*sy - sr*sp*cy, qw = cr*cp*cy + sr*sp*sy。
Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll*0.5), sr = std::sin(roll*0.5);
  const double cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
  const double cy = std::cos(yaw*0.5), sy = std::sin(yaw*0.5);
  Quaternion q;
  q[0] = sr*cp*cy - cr*sp*sy;
  q[1] = cr*sp*cy + sr*cp*sy;
  q[2] = cr*cp*sy - sr*sp*cy;
  q[3] = cr*cp*cy + sr*sp*sy;
  quaternion_normalize(q);
  return q;
}

Quaternion rpy_to_quaternion(const RPY& rpy) {
  return rpy_to_quaternion(rpy[0], rpy[1], rpy[2]);
}

// 四元数 → RPY (弧度)。从四元数提取欧拉角，与内旋 X→Y→Z 对应。
// 公式: sin(pitch) = 2(w*y - z*x); roll = atan2(2(w*x+y*z), 1-2(x²+y²)); yaw = atan2(2(w*z+x*y), 1-2(y²+z²))。
// 万向锁时 pitch = ±π/2，仅 roll+yaw 组合有定义。
RPY quaternion_to_rpy(const Quaternion& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  RPY rpy;
  const double sinp = 2.0 * (w*y - z*x);
  if (std::abs(sinp) >= 1)
    rpy[1] = std::copysign(M_PI / 2, sinp);
  else
    rpy[1] = std::asin(clamp(sinp, -1.0, 1.0));
  rpy[0] = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
  rpy[2] = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
  return rpy;
}

// RPY → 旋转矩阵 R。R = Rz(yaw) * Ry(pitch) * Rx(roll)，固定轴（外旋）等价于内旋 Z→Y→X。
// 公式: R = [ cp*cy   -cr*sy+sr*sp*cy   sr*sy+cr*sp*cy
//            cp*sy    cr*cy+sr*sp*sy   -sr*cy+cr*sp*sy
//            -sp      sr*cp             cr*cp ]
// 其中 cr=cos(roll), sr=sin(roll), cp=cos(pitch), sp=sin(pitch), cy=cos(yaw), sy=sin(yaw)。
Matrix3 rpy_to_rotation_matrix(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Matrix3 R;
  R[0] = cp*cy;  R[1] = -cr*sy + sr*sp*cy;  R[2] = sr*sy + cr*sp*cy;
  R[3] = cp*sy;  R[4] = cr*cy + sr*sp*sy;    R[5] = -sr*cy + cr*sp*sy;
  R[6] = -sp;    R[7] = sr*cp;               R[8] = cr*cp;
  return R;
}

Matrix3 rpy_to_rotation_matrix(const RPY& rpy) {
  return rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2]);
}

RPY rotation_matrix_to_rpy(const Matrix3& R) {
  Quaternion q = rotation_matrix_to_quaternion(R);
  return quaternion_to_rpy(q);
}

}  // namespace core
}  // namespace coordinate_transforms
