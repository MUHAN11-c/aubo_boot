// 运动纯核 gtest：slerp / interpolateCartesian / buildApproachWaypoints /
// buildSegmentWaypoints / quatSameHemisphere / applyGraspZOffset。
// 零 ROS 图依赖（只构造消息结构体），对应 AGENTS 单测金字塔 L1。
#include <gtest/gtest.h>

#include "ivg_demo_services/motion_utils.hpp"

#include <cmath>

namespace
{

geometry_msgs::msg::Quaternion quat(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::msg::Pose pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = quat(0.0, 0.0, 0.0, 1.0);
  return p;
}

}  // namespace

TEST(MotionUtils, SlerpEndpoints)
{
  const auto q0 = quat(0.0, 0.0, 0.0, 1.0);
  const auto q1 = quat(0.0, 0.0, 1.0, 0.0);  // 绕 Z 180°
  EXPECT_NEAR(ivg_demo_services::slerp(q0, q1, 0.0).w, 1.0, 1e-9);
  EXPECT_NEAR(ivg_demo_services::slerp(q0, q1, 1.0).z, 1.0, 1e-9);
  // 中点：绕 Z 90°，四元数 w=cos(45°), z=sin(45°)
  const auto mid = ivg_demo_services::slerp(q0, q1, 0.5);
  EXPECT_NEAR(mid.w, std::cos(M_PI / 4), 1e-9);
  EXPECT_NEAR(mid.z, std::sin(M_PI / 4), 1e-9);
}

TEST(MotionUtils, SlerpSameHemisphereShortestPath)
{
  // q 与 -q 同旋转；dot<0 时 slerp 翻转 q1 取短弧，两种写法中点应一致
  const auto q0 = quat(0.0, 0.0, 0.0, 1.0);
  const auto q1 = quat(0.0, 0.0, 0.6, 0.8);
  const auto q1_neg = quat(0.0, 0.0, -0.6, -0.8);
  const auto mid = ivg_demo_services::slerp(q0, q1, 0.5);
  const auto mid_neg = ivg_demo_services::slerp(q0, q1_neg, 0.5);
  EXPECT_NEAR(mid.w, mid_neg.w, 1e-9);
  EXPECT_NEAR(mid.z, mid_neg.z, 1e-9);
}

TEST(MotionUtils, InterpolateCartesianCountAndLinearity)
{
  const auto from = pose(0.0, 0.0, 0.0);
  const auto to = pose(1.0, 2.0, 3.0);
  const auto wps = ivg_demo_services::interpolateCartesian(from, to, 10);
  ASSERT_EQ(wps.size(), 10u);
  EXPECT_NEAR(wps.back().position.x, 1.0, 1e-9);
  EXPECT_NEAR(wps[4].position.y, 1.0, 1e-9);  // t=0.5
}

TEST(MotionUtils, QuatSameHemisphere)
{
  const auto ref = quat(0.0, 0.0, 0.0, 1.0);
  const auto same = ivg_demo_services::quatSameHemisphere(ref, ref);
  EXPECT_DOUBLE_EQ(same.w, 1.0);
  const auto flipped = ivg_demo_services::quatSameHemisphere(
    ref, quat(0.0, 0.0, 0.0, -1.0));
  EXPECT_DOUBLE_EQ(flipped.w, 1.0);
}

TEST(MotionUtils, ApplyGraspZOffset)
{
  const auto g = pose(0.1, 0.2, 0.3);
  const auto ee = ivg_demo_services::applyGraspZOffset(g, 0.05);
  EXPECT_NEAR(ee.position.z, 0.35, 1e-12);
  EXPECT_NEAR(ee.position.x, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(ee.orientation.w, g.orientation.w);
}

TEST(MotionUtils, ApproachWaypointsOrderAndZLimit)
{
  const auto cur = pose(0.5, 0.5, 0.3);
  auto target = pose(0.6, 0.4, 0.01);  // 低于 z_min_limit
  target.orientation = quat(0.0, 0.0, 0.0, 1.0);
  const auto wps = ivg_demo_services::buildApproachWaypoints(cur, target, 0.05, 0.05);
  ASSERT_EQ(wps.size(), 5u);
  // X→Y→抬升→旋转→下降：前三点保持当前 Z，下降点被 z 限位抬到 0.05
  EXPECT_NEAR(wps[0].position.x, 0.6, 1e-9);
  EXPECT_NEAR(wps[0].position.y, 0.5, 1e-9);
  EXPECT_NEAR(wps[1].position.y, 0.4, 1e-9);
  EXPECT_NEAR(wps[1].position.z, 0.3, 1e-9);
  EXPECT_NEAR(wps[2].position.z, 0.05 + 0.05, 1e-9);  // gz 限位 0.05 + height_above
  EXPECT_NEAR(wps[4].position.z, 0.05, 1e-9);
  // 旋转点与下降点姿态一致，且与当前姿态同半球
  EXPECT_DOUBLE_EQ(wps[3].orientation.w, wps[4].orientation.w);
}

TEST(MotionUtils, SegmentWaypointsZFloor)
{
  const auto start = pose(0.0, 0.0, 0.2);
  const std::vector<ivg_demo_services::CartesianSegment> segs = {
    {'z', -0.3},  // 会触底
    {'x', 0.1},
  };
  const auto wps = ivg_demo_services::buildSegmentWaypoints(start, segs, 0.05);
  ASSERT_EQ(wps.size(), 2u);
  EXPECT_NEAR(wps[0].position.z, 0.05, 1e-9);
  EXPECT_NEAR(wps[1].position.x, 0.1, 1e-9);
  EXPECT_NEAR(wps[1].position.z, 0.05, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
