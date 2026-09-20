// 功能：夹角包装统一（W5-7）。统一前 math_utils / stages / target_cache /
// view_planner 四处内联包装各自兜底；此处把退化输入的兜底语义显式成参数，
// 非退化路径一律委托 math_utils::angleBetweenDeg（公式不变）。
#ifndef PEACH_MANIPULATION__ANGLES_HPP_
#define PEACH_MANIPULATION__ANGLES_HPP_

#include <Eigen/Geometry>

#include "peach_arm/math_utils.hpp"

namespace peach_arm
{

// 退化输入（零/非有限向量）的兜底语义（钉死，不得混用）：
enum class AngleDegenerate
{
  MaxMismatch,   ///< 退化→180°：最大不对轴，让门判定走拒绝侧而非误放行
                 /// （原 stages.cpp 残差/对轴门包装）。
  Invalid,       ///< 退化→-1：不可判，不与真实夹角区间 [0,180] 混淆
                 /// （原 target_cache.cpp 诊断投影包装）。
  FallbackUnitX  ///< 退化→+X 兜底归一化：结果有限且落在 0–180°
                 /// （原 view_planner.cpp 评分包装）。
};

/// 两向量夹角 [deg]；退化输入按 fallback 处理，非退化语义同 angleBetweenDeg。
/// nonzero_eps：模长不大于该值视为退化（原各处包装分别取 1e-9 / 1e-6，
/// 由调用方按自身语义传入；非有限分量任何模式下都按退化处理）。
inline double angleDeg(
  const Eigen::Vector3d & first, const Eigen::Vector3d & second,
  AngleDegenerate fallback, double nonzero_eps = 1.0e-9)
{
  const bool first_ok = first.allFinite() && first.norm() > nonzero_eps;
  const bool second_ok = second.allFinite() && second.norm() > nonzero_eps;
  if (first_ok && second_ok) {
    return angleBetweenDeg(first, second);
  }
  switch (fallback) {
    case AngleDegenerate::MaxMismatch:
      return 180.0;
    case AngleDegenerate::Invalid:
      return -1.0;
    case AngleDegenerate::FallbackUnitX:
      return angleBetweenDeg(
        first_ok ? first : Eigen::Vector3d::UnitX(),
        second_ok ? second : Eigen::Vector3d::UnitX());
  }
  return -1.0;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__ANGLES_HPP_
