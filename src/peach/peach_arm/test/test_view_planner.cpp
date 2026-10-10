// 功能：视点规划器 generate 不变量测试（候选有限、半径收缩、保护区剔除、排序）。
#include "peach_arm/view_planner.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

constexpr double kEps = 1.0e-9;

// 基准场景：目标在原点、相机在 (0.5, 0, 0.4)（半径 0.6403，未超可达域）。
// 放开单步上限避免截步干扰，使半径语义可以精确断言。
peach_arm::ViewPlannerConfig baseConfig()
{
  peach_arm::ViewPlannerConfig config;
  config.observation_radius_m = 0.70;
  config.minimum_radius_m = 0.32;
  config.max_camera_step_m = 0.5;
  config.workspace_max_reach_m = 0.78;
  return config;
}

peach_arm::ViewContext baseContext()
{
  peach_arm::ViewContext context;
  context.target = Eigen::Vector3d::Zero();
  context.current_camera_position = Eigen::Vector3d(0.5, 0.0, 0.4);
  context.image_width = 640;
  context.image_height = 480;
  return context;
}

}  // namespace

// ---------- generate：候选有限、半径带内、相机姿态一致 ----------

TEST(ViewPlanner, GenerateProducesBoundedCandidatesOnRadiusBand)
{
  const peach_arm::ViewPlanner planner(baseConfig());
  const std::vector<peach_arm::ViewCandidate> candidates =
    planner.generate(baseContext());
  // elevation_limit 默认 0 → 仅方位 ±1 档，候选数确定有限。
  ASSERT_EQ(candidates.size(), 2U);
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.radius_m, baseConfig().minimum_radius_m - kEps);
    EXPECT_LE(candidate.radius_m, baseConfig().observation_radius_m + kEps);
    // radius_m 与相机位姿到目标的实际距离一致；姿态仍正交。
    EXPECT_NEAR(
      (candidate.camera_pose.translation() - baseContext().target).norm(),
      candidate.radius_m, kEps);
    EXPECT_TRUE(
      (candidate.camera_pose.linear().transpose() * candidate.camera_pose.linear())
      .isApprox(Eigen::Matrix3d::Identity(), 1e-9));
    EXPECT_GE(candidate.camera_pose.translation().z(), 0.06 - kEps);
    EXPECT_LE(candidate.travel_m, baseConfig().max_camera_step_m + kEps);
  }
  // 评分 stable_sort 后非增（同分按行程升序）。
  for (std::size_t i = 1; i < candidates.size(); ++i) {
    EXPECT_GE(candidates[i - 1U].score, candidates[i].score - 1e-9);
    if (std::abs(candidates[i - 1U].score - candidates[i].score) <= 1e-9) {
      EXPECT_LE(candidates[i - 1U].travel_m, candidates[i].travel_m + 1e-9);
    }
  }
}

TEST(ViewPlanner, GenerateShrinksRadiusWhenMaskFillLow)
{
  // 分割占比不足 → want_closer：双层候选，内层半径按 radial_step 收缩。
  peach_arm::ViewContext context = baseContext();
  context.foreground_ratio = 0.2;
  const peach_arm::ViewPlanner planner(baseConfig());
  const std::vector<peach_arm::ViewCandidate> candidates =
    planner.generate(context);
  ASSERT_EQ(candidates.size(), 4U);
  const double outer = (context.current_camera_position - context.target).norm();
  double min_radius = candidates.front().radius_m;
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.radius_m, baseConfig().minimum_radius_m - kEps);
    EXPECT_LE(candidate.radius_m, baseConfig().observation_radius_m + kEps);
    min_radius = std::min(min_radius, candidate.radius_m);
  }
  EXPECT_NEAR(min_radius, outer - baseConfig().radial_step_m, 1e-6);
}

TEST(ViewPlanner, GenerateConvenienceOverloadMatchesContext)
{
  const peach_arm::ViewPlanner planner(baseConfig());
  const std::vector<peach_arm::ViewCandidate> via_context =
    planner.generate(baseContext());
  const std::vector<peach_arm::ViewCandidate> via_args = planner.generate(
    baseContext().target, baseContext().current_camera_position,
    baseContext().observed_directions);
  ASSERT_EQ(via_context.size(), via_args.size());
  ASSERT_EQ(via_args.size(), 2U);
  for (std::size_t i = 0; i < via_args.size(); ++i) {
    EXPECT_NEAR(via_args[i].radius_m, via_context[i].radius_m, kEps);
    EXPECT_NEAR(via_args[i].score, via_context[i].score, kEps);
  }
}

// ---------- 保护区：盒内不产生候选 ----------

TEST(ViewPlanner, GenerateDropsCandidatesInsideProtectedZone)
{
  // 全包盒：一切候选（含兜底）都在盒内 → 空列表表达「无可用视点」。
  peach_arm::ViewPlannerConfig config = baseConfig();
  peach_arm::ProtectedZone everything;
  everything.min = Eigen::Vector3d(-10.0, -10.0, -10.0);
  everything.max = Eigen::Vector3d(10.0, 10.0, 10.0);
  config.protected_zones = {everything};
  EXPECT_TRUE(
    peach_arm::ViewPlanner(config).generate(baseContext()).empty());

  // +y 半区盒：方位 +12°（y≈0.13）被剔除，−12°（y≈−0.13）保留。
  peach_arm::ViewPlannerConfig half = baseConfig();
  peach_arm::ProtectedZone plus_y;
  plus_y.min = Eigen::Vector3d(-10.0, 0.05, -10.0);
  plus_y.max = Eigen::Vector3d(10.0, 10.0, 10.0);
  half.protected_zones = {plus_y};
  const std::vector<peach_arm::ViewCandidate> remaining =
    peach_arm::ViewPlanner(half).generate(baseContext());
  ASSERT_EQ(remaining.size(), 1U);
  EXPECT_LE(remaining.front().camera_pose.translation().y(), 0.05);
  EXPECT_NEAR(remaining.front().azimuth_deg, -12.0, 1e-6);
}
