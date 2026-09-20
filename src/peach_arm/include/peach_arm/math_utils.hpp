// 功能：跨编译单元共享的向量夹角原语（纯函数，仅依赖 Eigen）。包内私用。
// 弧度↔度转换用 Eigen 的 EIGEN_PI（原仓内 kPi 常量已删，W5-7）；退化
// 输入的兜底包装统一在 angles.hpp。
#ifndef PEACH_MANIPULATION__MATH_UTILS_HPP_
#define PEACH_MANIPULATION__MATH_UTILS_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace peach_arm
{

// 两向量夹角 [deg]；不做退化输入兜底——各调用方按自身语义处理
// 近零/非有限（回退 180°、-1 或 safeUnit），见各处包装函数。
inline double angleBetweenDeg(
  const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double na = first.norm();
  const double nb = second.norm();
  const double cosine = std::clamp(first.dot(second) / (na * nb), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / static_cast<double>(EIGEN_PI);
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__MATH_UTILS_HPP_
