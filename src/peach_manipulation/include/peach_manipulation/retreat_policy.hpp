// 功能：部分套入后从实际位姿规划撤退，不逆播完整名义轨迹。
#ifndef PEACH_MANIPULATION__RETREAT_POLICY_HPP_
#define PEACH_MANIPULATION__RETREAT_POLICY_HPP_

namespace peach_manipulation
{

enum class RetreatMode
{
  FromActual = 0,
  ReverseNominal = 1
};

inline RetreatMode sleeveRetreatMode(bool sleeve_partial)
{
  return sleeve_partial ? RetreatMode::FromActual : RetreatMode::ReverseNominal;
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__RETREAT_POLICY_HPP_
