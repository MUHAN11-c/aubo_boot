// 功能：ACM 只对指定目标对象 × 指定工具链接 × 明确接触阶段放行。
#ifndef PEACH_MANIPULATION__ACM_POLICY_HPP_
#define PEACH_MANIPULATION__ACM_POLICY_HPP_

#include <string>

namespace peach_manipulation
{

enum class ContactAcmStage
{
  Transit = 0,
  Pregrasp = 1,
  Sleeve = 2,
  Cut = 3,
  Retreat = 4
};

/// 整张 <octomap> 工具豁免已撤销（F10）。
inline bool allowToolVersusWholeOctomap()
{
  return false;
}

inline bool acmAllows(
  const std::string & target_object_id,
  const std::string & tool_link,
  ContactAcmStage stage)
{
  if (target_object_id.empty() || tool_link.empty()) {
    return false;
  }
  if (stage != ContactAcmStage::Sleeve && stage != ContactAcmStage::Cut) {
    return false;
  }
  return tool_link == "sleeve_mouth" || tool_link == "tcp" ||
         tool_link == "tool_axis" || tool_link == "cutting_plane";
}

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__ACM_POLICY_HPP_
