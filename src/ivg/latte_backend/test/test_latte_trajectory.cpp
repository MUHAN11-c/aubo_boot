// 心形轨迹纯核 gtest：阶段点数分配、画圈半径、动态 roll 渐变、
// spout→TCP 转换（零偏移恒等、非零偏移方向）。
#include <gtest/gtest.h>

#include <cmath>
#include "latte_backend/latte_trajectory.hpp"

namespace
{

latte_backend::LatteTrajectoryParams makeParams()
{
  latte_backend::LatteTrajectoryParams p;
  p.cup_x = -0.63;
  p.cup_y = -0.308;
  p.cup_z = 0.198;
  p.spout_offset_x = 0.0;
  p.spout_offset_y = 0.0;
  p.spout_offset_z = 0.0;
  p.tcp_orientation.x = 0.0;
  p.tcp_orientation.y = 0.0;
  p.tcp_orientation.z = 0.0;
  p.tcp_orientation.w = 1.0;
  return p;
}

}  // namespace

TEST(LatteTrajectory, StagePointAllocation)
{
  auto p = makeParams();
  p.heart.total_points = 200;
  latte_backend::LatteTrajectoryGenerator gen(p);
  // 融合 25% / 成形 55% / 收尾 20%
  EXPECT_EQ(gen.stageMix().waypoints.size(), 50u);
  EXPECT_EQ(gen.stageDraw().waypoints.size(), 110u);
  EXPECT_EQ(gen.stageFinish().waypoints.size(), 40u);
  // 纯过渡阶段无连续轨迹
  EXPECT_TRUE(gen.stageApproach().waypoints.empty());
  EXPECT_TRUE(gen.stageHome().waypoints.empty());
}

TEST(LatteTrajectory, MixCircleRadius)
{
  auto p = makeParams();
  p.heart.mix_circle_r = 0.01;
  p.heart.sway_offset_y = 0.01;
  latte_backend::LatteTrajectoryGenerator gen(p);
  const auto mix = gen.stageMix();
  ASSERT_FALSE(mix.waypoints.empty());
  for (const auto & wp : mix.waypoints) {
    const double dx = wp.position.x - p.cup_x;
    const double dy = wp.position.y - (p.cup_y + p.heart.sway_offset_y);
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), p.heart.mix_circle_r, 1e-9);
    EXPECT_NEAR(wp.position.z, p.cup_z + p.heart.mix_height, 1e-9);
  }
}

TEST(LatteTrajectory, DrawDynamicRollGradient)
{
  auto p = makeParams();
  p.heart.roll_draw_dynamic = true;
  latte_backend::LatteTrajectoryGenerator gen(p);
  const auto draw = gen.stageDraw();
  ASSERT_GT(draw.waypoints.size(), 2u);
  // 首点 roll=60°（绕 X 四元数 x=sin30°），末点 roll=45°（x=sin22.5°）
  const auto & first = draw.waypoints.front().orientation;
  const auto & last = draw.waypoints.back().orientation;
  EXPECT_NEAR(first.x, std::sin(60.0 * M_PI / 360.0), 1e-9);
  EXPECT_NEAR(last.x, std::sin(45.0 * M_PI / 360.0), 1e-9);
}

TEST(LatteTrajectory, FinishPushY)
{
  auto p = makeParams();
  latte_backend::LatteTrajectoryGenerator gen(p);
  const auto fin = gen.stageFinish();
  ASSERT_FALSE(fin.waypoints.empty());
  const double y0 = fin.waypoints.front().position.y;
  const double y1 = fin.waypoints.back().position.y;
  EXPECT_NEAR(y1 - y0, p.heart.push_y, 1e-9);
}

TEST(LatteTrajectory, SpoutToTcpIdentityWhenZeroOffset)
{
  auto p = makeParams();  // 偏移全 0
  latte_backend::LatteTrajectoryGenerator gen(p);
  const auto mix = gen.stageMix();
  const auto tcp = gen.spoutToTcp(mix);
  ASSERT_EQ(tcp.waypoints.size(), mix.waypoints.size());
  for (size_t i = 0; i < mix.waypoints.size(); ++i) {
    EXPECT_NEAR(
      tcp.waypoints[i].position.x, mix.waypoints[i].position.x, 1e-12);
    EXPECT_NEAR(
      tcp.waypoints[i].position.y, mix.waypoints[i].position.y, 1e-12);
    EXPECT_NEAR(
      tcp.waypoints[i].position.z, mix.waypoints[i].position.z, 1e-12);
  }
}

TEST(LatteTrajectory, SpoutToTcpOffsetAlongLocalZ)
{
  auto p = makeParams();
  p.spout_offset_z = 0.2;  // TCP 前方（局部 Z）0.2 m
  latte_backend::LatteTrajectoryGenerator gen(p);
  const auto approach = gen.stageApproach();
  const auto tcp = gen.spoutToTcp(approach);
  // TCP 朝向=identity（世界系）：spout 世界位置 = TCP + (0,0,0.2)
  // 因此 TCP = spout - (0,0,0.2)
  EXPECT_NEAR(
    tcp.transition_target.position.z,
    approach.transition_target.position.z - 0.2, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
