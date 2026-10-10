// 功能：ACM 只对指定目标对象 × 指定工具链接 × 明确接触阶段放行。
// 工具连杆清单自工具档案参数注入（W5-6，GPL yaml tool.links /
// tool.contact_links；默认=原三处硬编码），本文件只保留阶段策略判定。
#ifndef PEACH_MANIPULATION__ACM_POLICY_HPP_
#define PEACH_MANIPULATION__ACM_POLICY_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

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

/// 接触阶段（套入/剪切）目标对象 × 工具连杆放行判定：目标/连杆非空、
/// 阶段为 Sleeve/Cut、且连杆在接触豁免清单（tool.contact_links）内。
inline bool acmAllows(
  const std::string & target_object_id,
  const std::string & tool_link,
  ContactAcmStage stage,
  const std::vector<std::string> & contact_tool_links)
{
  if (target_object_id.empty() || tool_link.empty()) {
    return false;
  }
  if (stage != ContactAcmStage::Sleeve && stage != ContactAcmStage::Cut) {
    return false;
  }
  return std::find(
    contact_tool_links.begin(), contact_tool_links.end(),
    tool_link) != contact_tool_links.end();
}

/// 场景障碍豁免描述（GPL moveit.obstacle_* 参数的纯核载体）。
struct ObstacleExemptionSpec
{
  std::vector<std::string> exempt_links;      ///< 豁免连杆（tool.links 部署值）。
  std::vector<std::string> guard_links;       ///< guard 开时唯一受查连杆。
  std::vector<std::string> obstacle_object_ids;  ///< 场景障碍对象 id。
  bool guard_enabled{true};                   ///< false=相机一并豁免。
};

/// 场景障碍豁免条目（2026-09-29 避障只为保护相机）：返回应写 ACM
/// allowed=true 的 (link, object) 对——exempt_links × ({保留名 <octomap>} ∪
/// obstacle_object_ids)；guard_enabled=false 时 guard_links（相机）一并
/// 豁免（全机器人×障碍放行，障碍拦路到不了位时恢复可达性）。纯核可测。
inline std::vector<std::pair<std::string, std::string>>
obstacleExemptionEntries(const ObstacleExemptionSpec & spec)
{
  std::vector<std::string> objects;
  objects.reserve(spec.obstacle_object_ids.size() + 1U);
  objects.push_back("<octomap>");  // 保留名；updater 关闭时该条目无害
  for (const auto & id : spec.obstacle_object_ids) {
    if (!id.empty()) {
      objects.push_back(id);
    }
  }
  std::vector<std::string> links = spec.exempt_links;
  if (!spec.guard_enabled) {
    links.insert(links.end(), spec.guard_links.begin(), spec.guard_links.end());
  }
  std::vector<std::pair<std::string, std::string>> entries;
  entries.reserve(links.size() * objects.size());
  for (const auto & link : links) {
    if (link.empty()) {
      continue;
    }
    for (const auto & object : objects) {
      entries.emplace_back(link, object);
    }
  }
  return entries;
}

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__ACM_POLICY_HPP_
