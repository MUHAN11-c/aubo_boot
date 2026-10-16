// 功能：入冠候选的确定性工艺排序，以及未运动时的重试边界。
#ifndef PEACH_ARM__STAGING_CANDIDATES_HPP_
#define PEACH_ARM__STAGING_CANDIDATES_HPP_

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace peach_arm
{
struct StagingCandidate
{
  std::map<std::string, double> joints;
  double roll_rad{0.0};
  double joint_cost{0.0};
};

inline void sortStagingCandidates(std::vector<StagingCandidate> & candidates)
{
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      const double roll_a = std::abs(a.roll_rad);
      const double roll_b = std::abs(b.roll_rad);
      return roll_a != roll_b ? roll_a < roll_b : a.joint_cost < b.joint_cost;
    });
}

inline bool retryStagingCandidate(
  bool success, bool execution_started, bool canceled, bool retry_forbidden)
{
  return !success && !execution_started && !canceled && !retry_forbidden;
}

/// 同一笛卡尔 LIN 连续两档滚转都规划失败则停梯子。拍照位到预抓取的
/// LIN 超速失败与刀口滚转无关（同一笛卡尔弦），继续扫只会重复关节速度极限。
inline bool abandonRolledLinSearch(int consecutive_plan_failures)
{
  return consecutive_plan_failures >= 2;
}
}  // namespace peach_arm
#endif  // PEACH_ARM__STAGING_CANDIDATES_HPP_
