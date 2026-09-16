// 功能：预抓取完成等级。REACHED ≠ VERIFIED。
#ifndef PEACH_MANIPULATION__PREGRASP_LEVEL_HPP_
#define PEACH_MANIPULATION__PREGRASP_LEVEL_HPP_

#include <cstdint>

namespace peach_arm
{

/// ExecuteTarget.Result.LEVEL_NONE
constexpr std::uint8_t kLevelNone = 0;
/// ExecuteTarget.Result.LEVEL_PREGRASP_REACHED
constexpr std::uint8_t kLevelPregraspReached = 1;
/// ExecuteTarget.Result.LEVEL_PREGRASP_VERIFIED
constexpr std::uint8_t kLevelPregraspVerified = 2;

/// 残差过门且有新鲜观测才 VERIFIED；已到位未过门为 REACHED。
inline std::uint8_t completion_level_after_pregrasp_verify(
  bool residual_passed, bool reached_pose = true, bool fresh_observation = true)
{
  if (!reached_pose) {
    return kLevelNone;
  }
  if (residual_passed && fresh_observation) {
    return kLevelPregraspVerified;
  }
  return kLevelPregraspReached;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__PREGRASP_LEVEL_HPP_
