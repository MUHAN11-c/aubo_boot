// 功能：接近轨迹护栏测试（拼接、原路返程、关节行程门、笛卡尔绕行门、姿态行程）。
#include "peach_arm/trajectory_guard.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

using peach_arm::CartesianDetourLimits;
using peach_arm::CartesianWaypoint;
using peach_arm::TrajectoryGuardLimits;

// 便捷造点：位置 + 绕 Z 旋转 rot_deg 的四元数（xyzw）。
CartesianWaypoint wp(double x, double y, double z, double rot_deg = 0.0)
{
  constexpr double kHalfRad = 3.14159265358979323846 / 360.0;
  const double half = rot_deg * kHalfRad;
  CartesianWaypoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.qz = std::sin(half);
  point.qw = std::cos(half);
  return point;
}

trajectory_msgs::msg::JointTrajectoryPoint trajectoryPoint(
  double t_s, const std::vector<double> & positions,
  const std::vector<double> & velocities = {},
  const std::vector<double> & accelerations = {})
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = peach_arm::secToDuration(t_s);
  point.positions = positions;
  point.velocities = velocities;
  point.accelerations = accelerations;
  return point;
}

trajectory_msgs::msg::JointTrajectory singleJointTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"j1"};
  trajectory.points = {
    trajectoryPoint(0.0, {0.0}, {1.0}, {0.5}),
    trajectoryPoint(1.0, {1.0}, {2.0}, {-0.25}),
    trajectoryPoint(3.0, {3.0}, {0.5}, {0.125})};
  return trajectory;
}

}  // namespace

// ---------- concatJointTrajectories：时间轴相接、接缝去重 ----------

TEST(TrajectoryGuard, ConcatStitchesTimeAxisAndDedupsSeam)
{
  trajectory_msgs::msg::JointTrajectory first;
  first.joint_names = {"j1", "j2"};
  first.points = {trajectoryPoint(0.0, {0.0, 0.0}),
    trajectoryPoint(2.5, {1.0, 1.0})};
  trajectory_msgs::msg::JointTrajectory second;
  second.joint_names = {"j1", "j2"};
  // 首点与上一段末点重复（接缝），拼接时必须去掉。
  second.points = {trajectoryPoint(0.0, {1.0, 1.0}),
    trajectoryPoint(3.0, {2.0, 0.0})};
  const trajectory_msgs::msg::JointTrajectory merged =
    peach_arm::concatJointTrajectories({first, second});
  ASSERT_EQ(merged.points.size(), 3U);
  EXPECT_EQ(merged.joint_names.size(), 2U);
  double previous = -1.0;
  for (const auto & point : merged.points) {
    const double t = peach_arm::durationToSec(point.time_from_start);
    EXPECT_GT(t, previous);
    previous = t;
  }
  EXPECT_NEAR(peach_arm::durationToSec(merged.points[0].time_from_start), 0.0, 1e-9);
  EXPECT_NEAR(peach_arm::durationToSec(merged.points[1].time_from_start), 2.5, 1e-9);
  EXPECT_NEAR(peach_arm::durationToSec(merged.points[2].time_from_start), 5.5, 1e-9);
  EXPECT_NEAR(merged.points[2].positions[0], 2.0, 1e-12);
  EXPECT_NEAR(merged.points[2].positions[1], 0.0, 1e-12);
}

TEST(TrajectoryGuard, ConcatRejectsNameMismatchAndEmpty)
{
  trajectory_msgs::msg::JointTrajectory first;
  first.joint_names = {"j1"};
  first.points = {trajectoryPoint(0.0, {0.0}), trajectoryPoint(1.0, {1.0})};
  trajectory_msgs::msg::JointTrajectory mismatched;
  mismatched.joint_names = {"j2"};
  mismatched.points = {trajectoryPoint(0.0, {1.0})};
  const trajectory_msgs::msg::JointTrajectory bad =
    peach_arm::concatJointTrajectories({first, mismatched});
  EXPECT_TRUE(bad.points.empty());
  EXPECT_TRUE(bad.joint_names.empty());
  // 空入参 / 空段：空轨迹。
  EXPECT_TRUE(peach_arm::concatJointTrajectories({}).points.empty());
  trajectory_msgs::msg::JointTrajectory no_points;
  no_points.joint_names = {"j1"};
  EXPECT_TRUE(
    peach_arm::concatJointTrajectories({first, no_points}).points.empty());
}

// ---------- reverseJointTrajectory：倒放、速度/加速度取反、端点清零 ----------

TEST(TrajectoryGuard, ReverseInvertsOrderSignsAndZeroesEndpoints)
{
  const trajectory_msgs::msg::JointTrajectory reversed =
    peach_arm::reverseJointTrajectory(singleJointTrajectory());
  ASSERT_EQ(reversed.points.size(), 3U);
  ASSERT_EQ(reversed.joint_names.size(), 1U);
  EXPECT_EQ(reversed.joint_names[0], "j1");
  // 时间轴从 0 起倒放：0 / 2 / 3。
  EXPECT_NEAR(
    peach_arm::durationToSec(reversed.points[0].time_from_start), 0.0, 1e-9);
  EXPECT_NEAR(
    peach_arm::durationToSec(reversed.points[1].time_from_start), 2.0, 1e-9);
  EXPECT_NEAR(
    peach_arm::durationToSec(reversed.points[2].time_from_start), 3.0, 1e-9);
  // 位置倒序：3 / 1 / 0。
  EXPECT_NEAR(reversed.points[0].positions[0], 3.0, 1e-12);
  EXPECT_NEAR(reversed.points[1].positions[0], 1.0, 1e-12);
  EXPECT_NEAR(reversed.points[2].positions[0], 0.0, 1e-12);
  // 速度取反后端点清零，中点取反。
  EXPECT_NEAR(reversed.points[0].velocities[0], 0.0, 1e-12);
  EXPECT_NEAR(reversed.points[1].velocities[0], -2.0, 1e-12);
  EXPECT_NEAR(reversed.points[2].velocities[0], 0.0, 1e-12);
  // 加速度全程取反（端点速度才清零，加速度保留符号语义）。
  EXPECT_NEAR(reversed.points[0].accelerations[0], -0.125, 1e-12);
  EXPECT_NEAR(reversed.points[1].accelerations[0], 0.25, 1e-12);
  EXPECT_NEAR(reversed.points[2].accelerations[0], -0.5, 1e-12);
}

TEST(TrajectoryGuard, ReverseShortTrajectoryKeepsNames)
{
  trajectory_msgs::msg::JointTrajectory single;
  single.joint_names = {"j1"};
  single.points = {trajectoryPoint(0.0, {0.0})};
  const trajectory_msgs::msg::JointTrajectory reversed =
    peach_arm::reverseJointTrajectory(single);
  EXPECT_TRUE(reversed.points.empty());
  EXPECT_EQ(reversed.joint_names.size(), 1U);
}

// ---------- inspectApproachTrajectories：行程/时长/非有限门 ----------

TEST(TrajectoryGuard, InspectApproachCountsTravelIncludingSeam)
{
  // 两段间 1 rad 的接缝行程也必须计入：合计 6.0（段内 4 + 接缝 2）。
  trajectory_msgs::msg::JointTrajectory first;
  first.joint_names = {"j1", "j2"};
  first.points = {trajectoryPoint(0.0, {0.0, 0.0}),
    trajectoryPoint(1.0, {1.0, 1.0})};
  trajectory_msgs::msg::JointTrajectory second;
  second.joint_names = {"j1", "j2"};
  second.points = {trajectoryPoint(0.0, {2.0, 2.0}),
    trajectoryPoint(1.0, {3.0, 3.0})};
  TrajectoryGuardLimits limits;
  limits.max_duration_s = 0.0;
  limits.max_total_joint_travel_rad = 10.0;
  limits.max_single_joint_travel_rad = 10.0;
  const peach_arm::TrajectoryGuardReport report =
    peach_arm::inspectApproachTrajectories({first, second}, limits);
  EXPECT_TRUE(report.allowed);
  EXPECT_NEAR(report.total_joint_travel_rad, 6.0, 1e-12);
  EXPECT_NEAR(report.max_single_joint_travel_rad, 3.0, 1e-12);
  EXPECT_NEAR(report.duration_s, 2.0, 1e-9);
  EXPECT_EQ(report.point_count, 4U);
}

TEST(TrajectoryGuard, InspectApproachRejectsTravelAndDuration)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"j1", "j2"};
  trajectory.points = {trajectoryPoint(0.0, {0.0, 0.0}),
    trajectoryPoint(1.0, {1.0, -1.0})};
  TrajectoryGuardLimits base;
  base.max_duration_s = 0.0;
  // 时长门关（<=0 跳过）时同一条轨迹过。
  EXPECT_TRUE(
    peach_arm::inspectApproachTrajectory(trajectory, base).allowed);
  // 时长门开且超限 → 拒，理由给预计时长。
  TrajectoryGuardLimits timed = base;
  timed.max_duration_s = 0.5;
  const auto duration_report =
    peach_arm::inspectApproachTrajectory(trajectory, timed);
  EXPECT_FALSE(duration_report.allowed);
  EXPECT_NE(duration_report.reason.find("预计时长"), std::string::npos);
  // 累计行程超限（行程 2.0 > 1.5）。
  TrajectoryGuardLimits total = base;
  total.max_total_joint_travel_rad = 1.5;
  const auto total_report =
    peach_arm::inspectApproachTrajectory(trajectory, total);
  EXPECT_FALSE(total_report.allowed);
  EXPECT_NE(total_report.reason.find("累计关节行程"), std::string::npos);
  // 单轴超限（单轴 1.0 > 0.5）。
  TrajectoryGuardLimits single = base;
  single.max_single_joint_travel_rad = 0.5;
  const auto single_report =
    peach_arm::inspectApproachTrajectory(trajectory, single);
  EXPECT_FALSE(single_report.allowed);
  EXPECT_NE(single_report.reason.find("单轴累计行程"), std::string::npos);
}

TEST(TrajectoryGuard, InspectApproachRejectsMalformed)
{
  TrajectoryGuardLimits limits;
  // 空段列 / 空点 / 关节名不一致 / 维度不一致 / 非有限值全拒。
  EXPECT_FALSE(peach_arm::inspectApproachTrajectories({}, limits).allowed);
  trajectory_msgs::msg::JointTrajectory empty_points;
  empty_points.joint_names = {"j1"};
  const auto empty_report =
    peach_arm::inspectApproachTrajectory(empty_points, limits);
  EXPECT_FALSE(empty_report.allowed);
  EXPECT_NE(empty_report.reason.find("接近轨迹为空"), std::string::npos);

  trajectory_msgs::msg::JointTrajectory first;
  first.joint_names = {"j1"};
  first.points = {trajectoryPoint(0.0, {0.0}), trajectoryPoint(1.0, {1.0})};
  trajectory_msgs::msg::JointTrajectory other_names;
  other_names.joint_names = {"j2"};
  other_names.points = {trajectoryPoint(0.0, {1.0})};
  const auto names_report =
    peach_arm::inspectApproachTrajectories({first, other_names}, limits);
  EXPECT_FALSE(names_report.allowed);
  EXPECT_NE(names_report.reason.find("关节名不一致"), std::string::npos);

  trajectory_msgs::msg::JointTrajectory bad_dim;
  bad_dim.joint_names = {"j1"};
  bad_dim.points = {trajectoryPoint(0.0, {0.0, 0.0})};
  const auto dim_report =
    peach_arm::inspectApproachTrajectory(bad_dim, limits);
  EXPECT_FALSE(dim_report.allowed);
  EXPECT_NE(dim_report.reason.find("维度不一致"), std::string::npos);

  trajectory_msgs::msg::JointTrajectory non_finite;
  non_finite.joint_names = {"j1"};
  non_finite.points = {trajectoryPoint(0.0, {0.0}),
    trajectoryPoint(1.0, {std::numeric_limits<double>::quiet_NaN()})};
  const auto finite_report =
    peach_arm::inspectApproachTrajectory(non_finite, limits);
  EXPECT_FALSE(finite_report.allowed);
  EXPECT_NE(finite_report.reason.find("非有限"), std::string::npos);
}

// ---------- inspectCartesianDetour：绕行比 / 弦偏离 / 回退 ----------

TEST(TrajectoryGuard, InspectCartesianDetourStraightLinePasses)
{
  const std::vector<CartesianWaypoint> straight = {
    wp(0.0, 0.0, 0.0), wp(0.1, 0.0, 0.0), wp(0.2, 0.0, 0.0)};
  const peach_arm::CartesianDetourReport report =
    peach_arm::inspectCartesianDetour(straight, CartesianDetourLimits{});
  EXPECT_TRUE(report.allowed);
  EXPECT_NEAR(report.chord_m, 0.2, 1e-9);
  EXPECT_NEAR(report.path_m, 0.2, 1e-9);
  EXPECT_NEAR(report.detour_ratio, 1.0, 1e-9);
  EXPECT_NEAR(report.max_dev_m, 0.0, 1e-9);
  EXPECT_NEAR(report.max_recede_m, 0.0, 1e-9);
}

TEST(TrajectoryGuard, InspectCartesianDetourRejectsDetourRatio)
{
  // 明显绕路：0.6 m 弦走出 6 m 折线路径（偏离 0.1、回退微小，
  // 用只开绕行比的门隔离语义，再用默认门整判一次）。
  std::vector<CartesianWaypoint> zigzag;
  zigzag.push_back(wp(0.0, 0.0, 0.0));
  for (int i = 1; i <= 30; ++i) {
    zigzag.push_back(wp(0.02 * i, i % 2 == 0 ? 0.1 : -0.1, 0.0));
  }
  CartesianDetourLimits ratio_only;
  ratio_only.max_chord_deviation_m = 0.0;
  ratio_only.max_recede_m = 0.0;
  const peach_arm::CartesianDetourReport isolated =
    peach_arm::inspectCartesianDetour(zigzag, ratio_only);
  EXPECT_FALSE(isolated.allowed);
  EXPECT_GT(isolated.detour_ratio, 2.2);
  EXPECT_NE(isolated.reason.find("绕行比"), std::string::npos);
  const peach_arm::CartesianDetourReport full =
    peach_arm::inspectCartesianDetour(zigzag, CartesianDetourLimits{});
  EXPECT_FALSE(full.allowed);
}

TEST(TrajectoryGuard, InspectCartesianDetourRejectsRecedeAndDeviation)
{
  // 回退：中途点比起点更远离目标 0.2 m。
  const std::vector<CartesianWaypoint> recede = {
    wp(0.0, 0.0, 0.0), wp(-0.2, 0.0, 0.0), wp(0.2, 0.0, 0.0)};
  CartesianDetourLimits recede_only;
  recede_only.max_chord_deviation_m = 0.0;
  recede_only.max_detour_ratio = 0.0;
  const peach_arm::CartesianDetourReport recede_report =
    peach_arm::inspectCartesianDetour(recede, recede_only);
  EXPECT_FALSE(recede_report.allowed);
  EXPECT_NEAR(recede_report.max_recede_m, 0.2, 1e-9);
  EXPECT_NE(recede_report.reason.find("回退"), std::string::npos);
  // 弦偏离：中途点离弦 0.3 m（关掉回退与绕行比隔离语义）。
  const std::vector<CartesianWaypoint> deviate = {
    wp(0.0, 0.0, 0.0), wp(0.1, 0.3, 0.0), wp(0.2, 0.0, 0.0)};
  CartesianDetourLimits dev_only;
  dev_only.max_recede_m = 0.0;
  dev_only.max_detour_ratio = 0.0;
  const peach_arm::CartesianDetourReport dev_report =
    peach_arm::inspectCartesianDetour(deviate, dev_only);
  EXPECT_FALSE(dev_report.allowed);
  EXPECT_NEAR(dev_report.max_dev_m, 0.3, 1e-9);
  EXPECT_NE(dev_report.reason.find("偏离"), std::string::npos);
  // 短弦（<0.02 m）不计算绕行比：折返也放行。
  const std::vector<CartesianWaypoint> tiny = {
    wp(0.0, 0.0, 0.0), wp(0.01, 0.0, 0.0), wp(0.005, 0.0, 0.0)};
  const peach_arm::CartesianDetourReport tiny_report =
    peach_arm::inspectCartesianDetour(tiny, CartesianDetourLimits{});
  EXPECT_TRUE(tiny_report.allowed);
  EXPECT_NEAR(tiny_report.detour_ratio, 0.0, 1e-12);
}

// ---------- quatGeodesicDeg + inspectTcpOrientationTravel ----------

TEST(TrajectoryGuard, QuatGeodesicHandlesIdentityAndSign)
{
  // 0° / 90° / 180° 手算值；q 与 -q 同一姿态。
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(0, 0, 0, 1, 0, 0, 0, 1), 0.0, 1e-6);
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(0, 0, 0, 1, 0, 0, 1, 1), 90.0, 1e-6);
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(1, 0, 0, 0, 0, 0, 0, 1), 180.0, 1e-6);
  // q 与 -q 同一姿态：取绝对点积后夹角为 0。单位对精确为 0；非单位对经
  // 归一化后有 √2² 的浮点残差（~1e-6 rad），容差放宽到 1e-4°。
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(0, 0, 0, 1, 0, 0, 0, -1), 0.0, 1e-12);
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(0, 0, 1, 1, 0, 0, -1, -1), 0.0, 1e-4);
  // 零四元数按不可判处理：0。
  EXPECT_NEAR(peach_arm::quatGeodesicDeg(0, 0, 0, 0, 0, 0, 0, 1), 0.0, 1e-6);
}

TEST(TrajectoryGuard, OrientationTravelCapAndSlack)
{
  // 0° 行程：过，最大行程 0。
  const std::vector<CartesianWaypoint> flat = {
    wp(0.0, 0.0, 0.0), wp(0.1, 0.0, 0.0)};
  const peach_arm::OrientationTravelReport flat_report =
    peach_arm::inspectTcpOrientationTravel(flat, 110.0);
  EXPECT_TRUE(flat_report.allowed);
  EXPECT_NEAR(flat_report.max_from_start_deg, 0.0, 1e-6);
  // 起止 90°、绝对上限 110°：过。
  const std::vector<CartesianWaypoint> quarter = {
    wp(0.0, 0.0, 0.0), wp(0.1, 0.0, 0.0, 90.0)};
  const peach_arm::OrientationTravelReport quarter_report =
    peach_arm::inspectTcpOrientationTravel(quarter, 110.0);
  EXPECT_TRUE(quarter_report.allowed);
  EXPECT_NEAR(quarter_report.max_from_start_deg, 90.0, 1e-6);
  // 中途拧到 180°：超绝对上限拒。
  const std::vector<CartesianWaypoint> flipped = {
    wp(0.0, 0.0, 0.0), wp(0.1, 0.0, 0.0, 180.0), wp(0.2, 0.0, 0.0, 90.0)};
  EXPECT_FALSE(peach_arm::inspectTcpOrientationTravel(flipped, 110.0).allowed);
  // slack 语义：cap = min(绝对上限, 起止测地线 + slack)。起止 20° + slack 10°
  // → cap 30°，中途 60° 虽在 110° 内仍拒；无 slack 同一点列过。
  const std::vector<CartesianWaypoint> wiggle = {
    wp(0.0, 0.0, 0.0), wp(0.05, 0.0, 0.0, 60.0), wp(0.1, 0.0, 0.0, 20.0)};
  EXPECT_TRUE(
    peach_arm::inspectTcpOrientationTravel(wiggle, 110.0).allowed);
  EXPECT_FALSE(
    peach_arm::inspectTcpOrientationTravel(wiggle, 110.0, 10.0).allowed);
  // 上限 <=0：审查关闭恒过。
  const peach_arm::OrientationTravelReport off =
    peach_arm::inspectTcpOrientationTravel(flipped, 0.0);
  EXPECT_TRUE(off.allowed);
}
