#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_manipulation/motion_backend.hpp"

namespace failure = peach2_end_effector::failure;
namespace pm = peach2_manipulation;

namespace
{

const std::vector<double> kStart{0.0, -0.5, 1.0, 0.0, 1.2, 0.0};

std::vector<double> offset(double dq)
{
  std::vector<double> q = kStart;
  q[2] += dq;
  return q;
}

pm::JointTrajectory trajectory(const std::vector<std::vector<double>> & points)
{
  pm::JointTrajectory t;
  t.joint_names = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint",
    "wrist2_joint", "wrist3_joint"};
  double time = 0.0;
  for (const auto & q : points) {
    t.points.push_back({time, q, std::vector<double>(6, 0.1), std::vector<double>(6, 0.2)});
    time += 1.0;
  }
  return t;
}

}  // namespace

TEST(FinalizePlan, MovingTrajectoryPassesThrough)
{
  const auto t = trajectory({kStart, offset(0.1), offset(0.2)});
  const auto r = pm::finalize_plan(t, kStart, pm::PlanKind::FREE, "move_to", 0.005);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.failure_code, failure::NONE);
  ASSERT_EQ(r.trajectory.points.size(), 3U);
  EXPECT_FALSE(pm::is_null_motion(r.trajectory));
  EXPECT_DOUBLE_EQ(r.trajectory.duration_s(), 2.0);
}

TEST(FinalizePlan, StationaryTrajectoryCollapsesToStart)
{
  // e.g. OMPL start == goal after time parameterization: several points, all on the start.
  const auto t = trajectory({kStart, offset(0.003), offset(-0.004)});
  const auto r = pm::finalize_plan(t, kStart, pm::PlanKind::FREE, "transit_staging", 0.005);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.failure_code, failure::NONE);
  EXPECT_EQ(r.reason, "transit_staging:at_goal");
  ASSERT_TRUE(pm::is_null_motion(r.trajectory));
  const auto & p = r.trajectory.points.front();
  EXPECT_EQ(p.positions, kStart);
  EXPECT_DOUBLE_EQ(p.time_from_start_s, 0.0);
  EXPECT_EQ(p.velocities, std::vector<double>(6, 0.0));
  EXPECT_EQ(p.accelerations, std::vector<double>(6, 0.0));
  EXPECT_EQ(r.trajectory.joint_names, t.joint_names);
}

TEST(FinalizePlan, SinglePointOnStartIsNullMotion)
{
  // Pilz LIN with identical start and goal returns one point.
  const auto t = trajectory({offset(0.001)});
  const auto r = pm::finalize_plan(t, kStart, pm::PlanKind::LINEAR, "retreat_fallback", 0.005);
  ASSERT_TRUE(r.ok);
  ASSERT_TRUE(pm::is_null_motion(r.trajectory));
  EXPECT_EQ(r.trajectory.points.front().positions, kStart);
}

TEST(FinalizePlan, SinglePointWithoutKnownStartUsesItself)
{
  const auto t = trajectory({kStart});
  const auto r = pm::finalize_plan(t, std::nullopt, pm::PlanKind::NAMED, "move_to", 0.005);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(pm::is_null_motion(r.trajectory));
}

TEST(FinalizePlan, SinglePointAwayFromStartFails)
{
  const auto t = trajectory({offset(0.01)});
  const auto free = pm::finalize_plan(t, kStart, pm::PlanKind::FREE, "move_to", 0.005);
  EXPECT_FALSE(free.ok);
  EXPECT_EQ(free.failure_code, failure::PLAN_FAILED);
  EXPECT_EQ(free.reason, "move_to:empty_trajectory");
  const auto lin = pm::finalize_plan(t, kStart, pm::PlanKind::LINEAR, "insert", 0.005);
  EXPECT_FALSE(lin.ok);
  EXPECT_EQ(lin.failure_code, failure::PLAN_CARTESIAN_INCOMPLETE);
}

TEST(FinalizePlan, EmptyTrajectoryFails)
{
  const pm::JointTrajectory t;
  const auto free = pm::finalize_plan(t, kStart, pm::PlanKind::FREE, "move_to", 0.005);
  EXPECT_FALSE(free.ok);
  EXPECT_EQ(free.failure_code, failure::PLAN_FAILED);
  EXPECT_EQ(free.reason, "move_to:empty_trajectory");
  const auto lin = pm::finalize_plan(t, kStart, pm::PlanKind::LINEAR, "approach", 0.005);
  EXPECT_EQ(lin.failure_code, failure::PLAN_CARTESIAN_INCOMPLETE);
}

TEST(FinalizePlan, AnyPointBeyondToleranceIsRealMotion)
{
  // Goal back on the start but the path leaves it: execute it.
  const auto t = trajectory({kStart, offset(0.006), kStart});
  const auto r = pm::finalize_plan(t, kStart, pm::PlanKind::FREE, "move_to", 0.005);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.trajectory.points.size(), 3U);
}

TEST(FinalizePlan, JointCountMismatchIsNotAtGoal)
{
  const auto t = trajectory({kStart});
  const std::vector<double> five(kStart.begin(), kStart.end() - 1);
  const auto r = pm::finalize_plan(t, five, pm::PlanKind::FREE, "move_to", 0.005);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.failure_code, failure::PLAN_FAILED);
}

TEST(FinalizePlan, ResidualCorrectionDoesNotCollapse)
{
  // 4 mm-class joint step sits inside the 0.005 rad transit band; correction must still execute.
  const auto t = trajectory({kStart, offset(0.003), offset(0.004)});
  const auto r = pm::finalize_plan(
    t, kStart, pm::PlanKind::LINEAR, "pregrasp_correction", 0.005, false);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.trajectory.points.size(), 3U);
  EXPECT_FALSE(pm::is_null_motion(r.trajectory));
}

TEST(FinalizePlan, ResidualCorrectionSinglePointFails)
{
  const auto t = trajectory({offset(0.001)});
  const auto r = pm::finalize_plan(
    t, kStart, pm::PlanKind::LINEAR, "pregrasp_correction", 0.005, false);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.failure_code, failure::PLAN_CARTESIAN_INCOMPLETE);
  EXPECT_EQ(r.reason, "pregrasp_correction:empty_trajectory");
}
