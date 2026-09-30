#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "peach2_manipulation/trajectory_reverse.hpp"

using peach2_manipulation::JointTrajectory;
using peach2_manipulation::TrajectoryPoint;

namespace
{

JointTrajectory make(double q0, double q1, double t)
{
  JointTrajectory tr;
  tr.joint_names = {"a", "b"};
  tr.points.push_back({0.0, {q0, 0.0}, {0.0, 0.0}, {1.0, 0.5}});
  tr.points.push_back({t / 2.0, {(q0 + q1) / 2.0, 0.1}, {0.4, 0.2}, {0.0, -0.3}});
  tr.points.push_back({t, {q1, 0.2}, {0.0, 0.0}, {-1.0, -0.5}});
  return tr;
}

}  // namespace

TEST(TrajectoryReverse, PositionsTimesVelocitiesAccelerations)
{
  const auto fwd = make(0.0, 1.0, 2.0);
  const auto rev = peach2_manipulation::reverse_trajectory(fwd);
  ASSERT_EQ(rev.points.size(), 3U);
  EXPECT_EQ(rev.joint_names, fwd.joint_names);
  EXPECT_DOUBLE_EQ(rev.points[0].time_from_start_s, 0.0);
  EXPECT_DOUBLE_EQ(rev.points[1].time_from_start_s, 1.0);
  EXPECT_DOUBLE_EQ(rev.points[2].time_from_start_s, 2.0);
  EXPECT_EQ(rev.points[0].positions, fwd.points[2].positions);
  EXPECT_EQ(rev.points[2].positions, fwd.points[0].positions);
  EXPECT_DOUBLE_EQ(rev.points[1].velocities[0], -0.4);
  EXPECT_DOUBLE_EQ(rev.points[1].velocities[1], -0.2);
  // x_r'' = x''(T - t): accelerations keep their sign (old stack negated them).
  EXPECT_DOUBLE_EQ(rev.points[0].accelerations[0], -1.0);
  EXPECT_DOUBLE_EQ(rev.points[1].accelerations[1], -0.3);
  EXPECT_DOUBLE_EQ(rev.points[2].accelerations[0], 1.0);
}

TEST(TrajectoryReverse, DoubleReverseIsIdentity)
{
  const auto fwd = make(0.3, -0.7, 1.5);
  const auto back = peach2_manipulation::reverse_trajectory(
    peach2_manipulation::reverse_trajectory(fwd));
  ASSERT_EQ(back.points.size(), fwd.points.size());
  for (size_t i = 0; i < fwd.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(back.points[i].time_from_start_s, fwd.points[i].time_from_start_s);
    EXPECT_EQ(back.points[i].positions, fwd.points[i].positions);
    EXPECT_EQ(back.points[i].velocities, fwd.points[i].velocities);
    EXPECT_EQ(back.points[i].accelerations, fwd.points[i].accelerations);
  }
}

TEST(TrajectoryReverse, EmptyAndMissingDerivatives)
{
  JointTrajectory empty;
  EXPECT_TRUE(peach2_manipulation::reverse_trajectory(empty).empty());
  JointTrajectory pos_only;
  pos_only.joint_names = {"a"};
  pos_only.points.push_back({0.0, {0.0}, {}, {}});
  pos_only.points.push_back({1.0, {1.0}, {}, {}});
  const auto rev = peach2_manipulation::reverse_trajectory(pos_only);
  EXPECT_TRUE(rev.points[0].velocities.empty());
  EXPECT_DOUBLE_EQ(rev.points[0].positions[0], 1.0);
}

TEST(TrajectoryReverse, PathReversesSegmentOrderAndMergesJunctions)
{
  const auto s1 = make(0.0, 1.0, 2.0);
  const auto s2 = make(1.0, 3.0, 1.0);   // starts where s1 ends (joint b differs: 0.2 vs 0.0)
  auto s2b = s2;
  s2b.points[0].positions = s1.points.back().positions;
  const auto path = peach2_manipulation::reverse_path({s1, s2b});
  // s2b reversed (3 points) + s1 reversed without its duplicated first point (2 points).
  ASSERT_EQ(path.points.size(), 5U);
  EXPECT_EQ(path.points.front().positions, s2b.points.back().positions);
  EXPECT_EQ(path.points.back().positions, s1.points.front().positions);
  EXPECT_DOUBLE_EQ(path.duration_s(), 3.0);
  for (size_t i = 1; i < path.points.size(); ++i) {
    EXPECT_GT(path.points[i].time_from_start_s, path.points[i - 1].time_from_start_s);
  }
}

TEST(TrajectoryReverse, PathRejectsMismatchedJoints)
{
  auto s1 = make(0.0, 1.0, 1.0);
  auto s2 = make(1.0, 2.0, 1.0);
  s2.joint_names = {"x", "y"};
  EXPECT_THROW(peach2_manipulation::reverse_path({s1, s2}), std::invalid_argument);
}

TEST(TrajectoryReverse, PathLengthAndDeviation)
{
  JointTrajectory t;
  t.joint_names = {"a", "b"};
  t.points.push_back({0.0, {0.0, 0.0}, {}, {}});
  t.points.push_back({1.0, {3.0, 4.0}, {}, {}});
  EXPECT_DOUBLE_EQ(peach2_manipulation::joint_path_length(t), 5.0);
  EXPECT_DOUBLE_EQ(peach2_manipulation::max_joint_deviation({0.0, 1.0}, {0.5, 0.9}), 0.5);
  EXPECT_TRUE(std::isinf(peach2_manipulation::max_joint_deviation({0.0}, {0.0, 1.0})));
}
