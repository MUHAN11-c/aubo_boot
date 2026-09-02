// 功能：跨编译单元共享的向量夹角原语（纯函数，仅依赖 Eigen）。包内私用。
#ifndef PEACH_MANIPULATION__MATH_UTILS_HPP_
#define PEACH_MANIPULATION__MATH_UTILS_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace peach_manipulation
{

constexpr double kPi = 3.14159265358979323846;

// 两向量夹角 [deg]；不做退化输入兜底——各调用方按自身语义处理
// 近零/非有限（回退 180°、-1 或 safeUnit），见各处包装函数。
inline double angleBetweenDeg(
  const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double na = first.norm();
  const double nb = second.norm();
  const double cosine = std::clamp(first.dot(second) / (na * nb), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / kPi;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__MATH_UTILS_HPP_
