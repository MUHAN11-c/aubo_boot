// 功能：ACM 只对指定目标对象 × 指定工具链接 × 明确接触阶段放行。
#ifndef PEACH_MANIPULATION__ACM_POLICY_HPP_
#define PEACH_MANIPULATION__ACM_POLICY_HPP_

#include <string>

namespace peach_arm
{

enum class ContactAcmStage
{
  Transit = 0,
  Pregrasp = 1,
  Sleeve = 2,
  Cut = 3,
  Retreat = 4
};

/// 整张 <octomap> 工具豁免：F10 曾撤销，09-17 真机轮回退撤销——眼在手上
/// 时工具永远在相机视野正下方，octomap updater 的 self-filter 漏收工具
/// 点云，工具×地图检查会与自家工具的幽灵体素自碰死锁（Survey 回拍照位
/// PTP/OMPL 全灭的实锤根因）。防撞主力是臂连杆与 camera_body（保持受查，
/// 不在本豁免范围）。updater self-filter 修复后可再收紧回 false。
inline bool allowToolVersusWholeOctomap()
{
  return true;
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

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__ACM_POLICY_HPP_
