// 功能：运动阶段授权拒因分类与终局分级（M3c）。纯核零 ROS——
// ExecuteTarget.Result 常量按 pregrasp_level.hpp 同款镜像钉死，
// cycle.cpp static_assert 与 IDL 双向锁定。
#ifndef PEACH_MANIPULATION__STAGE_DENIAL_HPP_
#define PEACH_MANIPULATION__STAGE_DENIAL_HPP_

#include <cstdint>

namespace peach_arm
{

/// authorizeStage 的拒因分类：EXPIRED 专指接触许可令牌/许可「过期」
/// （valid_until 超时或 model_stamp 超新鲜度窗）——可重派（重建后令牌
/// 换新即可再执行）；DENIED 覆盖其余一切拒因（权限/安全/取消/使能/
/// 许可明确不允许/目标不符）。区分二者是 M3c 的分级依据：过期≠不允许。
enum class StageDenial
{
  ALLOWED,  ///< 授权通过。
  EXPIRED,  ///< 令牌/许可过期（可重派）。
  DENIED    ///< 其余拒因（许可明确不允许等）。
};

/// ExecuteTarget.Result.FAILED
constexpr std::uint8_t kOutcomeFailed = 3;
/// ExecuteTarget.Result.SKIPPED_QUALITY
constexpr std::uint8_t kOutcomeSkippedQuality = 1;

/// requireStageAuthority 的拒因 → 终局分级：令牌/许可过期与 GraspDecision
/// 复检未通过同归 SKIPPED_QUALITY（可重派）；其余拒因保持 FAILED。
inline std::uint8_t stageDenialOutcome(StageDenial denial, bool decision_recheck_failed)
{
  if (denial == StageDenial::EXPIRED || decision_recheck_failed) {
    return kOutcomeSkippedQuality;
  }
  return kOutcomeFailed;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__STAGE_DENIAL_HPP_
