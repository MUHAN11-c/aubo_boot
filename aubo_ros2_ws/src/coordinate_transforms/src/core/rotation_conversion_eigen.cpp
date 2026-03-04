/**
 * 旋转表示转换（Eigen 实现分支）
 * 当 find_package(Eigen3) 成功时编译此文件替代 rotation_conversion.cpp。
 * 使用 Eigen 进行四元数、旋转矩阵、RPY 的转换，数值稳定且与 ROS/tf2 约定一致。
 */
#include "coordinate_transforms/core/rotation_conversion.hpp"
#include <Eigen/Geometry>
#include <cmath>

namespace coordinate_transforms {
namespace core {

namespace {
// 本文件内 RowMajor 3x3 与 std::array<double,9> 互转
inline Eigen::Matrix3d toEigen3(const Matrix3& R) {
  return Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R.data());
}
inline void fromEigen3(const Eigen::Matrix3d& M, Matrix3& R) {
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R.data()) = M;
}
// 四元数：ROS/tf2 为 (x,y,z,w)，Eigen 构造为 Quaterniond(w,x,y,z)
inline Eigen::Quaterniond toEigenQuat(const Quaternion& q) {
  return Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
}
inline void fromEigenQuat(const Eigen::Quaterniond& qe, Quaternion& q) {
  q[0] = qe.x(); q[1] = qe.y(); q[2] = qe.z(); q[3] = qe.w();
}
}

// 四元数 → 旋转矩阵 R（行优先 3×3）
// 公式: R = 2*q*q^T - (q·q)I + 2*q_w*[q_v]_× + 2*[q_v]_×^2
// 或展开: R_ij 由单位四元数 q=(x,y,z,w) 得
//   R = [ 1-2(y²+z²)   2(xy-zw)   2(xz+yw)
//         2(xy+zw)   1-2(x²+z²)  2(yz-xw)
//         2(xz-yw)   2(yz+xw)   1-2(x²+y²) ]
// Eigen: R = q.toRotationMatrix()
Matrix3 quaternion_to_rotation_matrix(const Quaternion& q) {
  Eigen::Quaterniond qe = toEigenQuat(q);
  qe.normalize();
  Matrix3 R;
  fromEigen3(qe.toRotationMatrix(), R);
  return R;
}

// 旋转矩阵 R → 单位四元数 q
// 公式: 从 R 恢复 q，常用 Shepperd 法按 trace 与对角元选分支:
//   trace = R00+R11+R22, s = sqrt(trace+1)
//   w = (1/4s)(trace+1), x = (R21-R12)/(4s), y = (R02-R20)/(4s), z = (R10-R01)/(4s)
// 数值不稳定时从 R 的其它分量解出并归一化 |q|=1。
// Eigen: Quaterniond::FromRotationMatrix(R)
Quaternion rotation_matrix_to_quaternion(const Matrix3& R) {
  Eigen::Matrix3d Me = toEigen3(R);
  Eigen::Quaterniond qe(Me);
  qe.normalize();
  Quaternion q;
  fromEigenQuat(qe, q);
  return q;
}

// 欧拉角 RPY (roll, pitch, yaw，弧度) → 四元数。内旋顺序: X(roll) → Y(pitch) → Z(yaw)
// 公式: q = q_roll * q_pitch * q_yaw
//   q_roll  = (sin(r/2),0,0,cos(r/2)), q_pitch = (0,sin(p/2),0,cos(p/2)), q_yaw = (0,0,sin(y/2),cos(y/2))
// 展开: qx = sr*cp*cy - cr*sp*sy, qy = cr*sp*cy + sr*cp*sy, qz = cr*cp*sy - sr*sp*cy, qw = cr*cp*cy + sr*sp*sy
// Eigen: AngleAxisd(yaw,Z)*AngleAxisd(pitch,Y)*AngleAxisd(roll,X)，注意应用顺序与 RPY 约定一致
Quaternion rpy_to_quaternion(double roll, double pitch, double yaw) {
  Eigen::Quaterniond qe =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Quaternion q;
  fromEigenQuat(qe, q);
  quaternion_normalize(q);
  return q;
}

Quaternion rpy_to_quaternion(const RPY& rpy) {
  return rpy_to_quaternion(rpy[0], rpy[1], rpy[2]);
}

// 四元数 → RPY (弧度)。与内旋 X→Y→Z 对应
// 公式: sin(pitch) = 2(w*y - z*x)
//       roll = atan2(2(w*x+y*z), 1-2(x²+y²))
//       yaw  = atan2(2(w*z+x*y), 1-2(y²+z²))
// 万向锁时 pitch = ±π/2，仅 roll+yaw 组合有定义。（与无 Eigen 分支公式一致，保证行为一致）
RPY quaternion_to_rpy(const Quaternion& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  RPY rpy;
  const double sinp = 2.0 * (w * y - z * x);
  auto clamp = [](double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  };
  if (std::abs(sinp) >= 1)
    rpy[1] = std::copysign(M_PI / 2.0, sinp);
  else
    rpy[1] = std::asin(clamp(sinp, -1.0, 1.0));
  rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return rpy;
}

// RPY → 旋转矩阵。R = Rz(yaw)*Ry(pitch)*Rx(roll)，固定轴（外旋）等价于内旋 Z→Y→X
// 公式: R = [ cp*cy   -cr*sy+sr*sp*cy   sr*sy+cr*sp*cy
//            cp*sy    cr*cy+sr*sp*sy   -sr*cy+cr*sp*sy
//            -sp      sr*cp             cr*cp ]
// 其中 cr=cos(roll), sr=sin(roll), cp=cos(pitch), sp=sin(pitch), cy=cos(yaw), sy=sin(yaw)
// Eigen: R = AngleAxisd(yaw,Z)*AngleAxisd(pitch,Y)*AngleAxisd(roll,X).toRotationMatrix()
Matrix3 rpy_to_rotation_matrix(double roll, double pitch, double yaw) {
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  Matrix3 out;
  fromEigen3(R, out);
  return out;
}

Matrix3 rpy_to_rotation_matrix(const RPY& rpy) {
  return rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2]);
}

// 旋转矩阵 → RPY。先转为四元数再提 RPY，保证与 quaternion_to_rpy 一致
// 公式: R → q (见上) → roll,pitch,yaw
RPY rotation_matrix_to_rpy(const Matrix3& R) {
  Quaternion q = rotation_matrix_to_quaternion(R);
  return quaternion_to_rpy(q);
}

}  // namespace core
}  // namespace coordinate_transforms
