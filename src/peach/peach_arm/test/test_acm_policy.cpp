// obstacleExemptionEntries 纯核单测（2026-09-29 避障只为保护相机）：
// 豁免条目组装两态（guard 开=仅相机受查；guard 关=相机一并豁免）与
// 部署语义锚定（<octomap> 保留名常驻、空 id/空 link 跳过）。
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "peach_arm/acm_policy.hpp"

namespace
{

using peach_arm::ObstacleExemptionSpec;
using peach_arm::obstacleExemptionEntries;

bool hasEntry(
  const std::vector<std::pair<std::string, std::string>> & entries,
  const std::string & link, const std::string & object)
{
  return std::find(entries.begin(), entries.end(),
           std::make_pair(link, object)) != entries.end();
}

ObstacleExemptionSpec makeSpec(bool guard_enabled)
{
  return ObstacleExemptionSpec{
    {"wrist2_Link", "tool_body_link"},  // exempt_links（部署=全机器人−相机）
    {"camera_body_link"},               // guard_links（唯一受查）
    {"peach_scene_obstacles"},          // obstacle_object_ids
    guard_enabled};
}

TEST(ObstacleExemption, GuardOnExemptsLinksVersusAllObjectsNotCamera)
{
  const auto entries = obstacleExemptionEntries(makeSpec(true));
  // 豁免连杆 × {<octomap>, peach_scene_obstacles} 全部成对
  EXPECT_TRUE(hasEntry(entries, "wrist2_Link", "<octomap>"));
  EXPECT_TRUE(hasEntry(entries, "wrist2_Link", "peach_scene_obstacles"));
  EXPECT_TRUE(hasEntry(entries, "tool_body_link", "<octomap>"));
  EXPECT_TRUE(hasEntry(entries, "tool_body_link", "peach_scene_obstacles"));
  // guard 开=相机不出现在任何豁免条目（唯一受查对）
  for (const auto & [link, object] : entries) {
    EXPECT_NE(link, "camera_body_link")
      << "guard 开时相机不得豁免（避障目标失效）: " << link << " x " << object;
  }
  EXPECT_EQ(entries.size(), 4U);
}

TEST(ObstacleExemption, GuardOffExemptsCameraToo)
{
  const auto entries = obstacleExemptionEntries(makeSpec(false));
  // guard 关=相机一并豁免（恢复可达性；对象仍在场景）
  EXPECT_TRUE(hasEntry(entries, "camera_body_link", "<octomap>"));
  EXPECT_TRUE(hasEntry(entries, "camera_body_link", "peach_scene_obstacles"));
  EXPECT_EQ(entries.size(), 6U);
}

TEST(ObstacleExemption, EmptyIdsAndLinksSkipped)
{
  const ObstacleExemptionSpec spec{
    {"wrist2_Link", ""}, {"camera_body_link"}, {"", "peach_scene_obstacles"},
    true};
  const auto entries = obstacleExemptionEntries(spec);
  // 空 link / 空 object id 不产生条目；非空对象 id 与 <octomap> 保留名并存
  EXPECT_EQ(entries.size(), 2U);
  EXPECT_TRUE(hasEntry(entries, "wrist2_Link", "<octomap>"));
  EXPECT_TRUE(hasEntry(entries, "wrist2_Link", "peach_scene_obstacles"));
}

}  // namespace
