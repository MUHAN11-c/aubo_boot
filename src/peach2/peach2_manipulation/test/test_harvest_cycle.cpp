#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "fakes.hpp"

namespace failure = peach2_end_effector::failure;
namespace pm = peach2_manipulation;
namespace ee = peach2_end_effector;
using peach2_fakes::CycleRig;
using peach2_fakes::FakeMotion;
using Fault = ee::MockIoBackend::Fault;

namespace
{

ee::MockIoBackend::Config mock_with(Fault fault)
{
  ee::MockIoBackend::Config c;
  c.fault = fault;
  return c;
}

std::vector<std::string> v(std::initializer_list<const char *> l)
{
  return std::vector<std::string>(l.begin(), l.end());
}

}  // namespace

// ---------------------------------------------------------------- plan-only

TEST(HarvestCycle, PlanOnlyWhenExecutionDisabled)
{
  CycleRig rig;
  rig.execution = rig.grasp = rig.tool_enabled = false;
  const auto r = rig.run();
  // Never SUCCEEDED: a consumer that ignores plan_only must not count it as harvested.
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::NONE);
  EXPECT_EQ(r.reason, "planned");
  EXPECT_EQ(r.reached, pm::Reached::NONE);
  EXPECT_FALSE(r.recovery_required);
  EXPECT_TRUE(r.plan_only);
  EXPECT_TRUE(rig.motion.executed.empty());
  EXPECT_EQ(rig.io->write_count(), 0);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "transit_staging"), rig.config.roll_samples);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "insert"), 1);
  EXPECT_EQ(rig.decisions.calls(), 1U);
}

TEST(HarvestCycle, PlanOnlyWhenHeartbeatMissing)
{
  CycleRig rig;
  rig.heartbeat = false;
  const auto r = rig.run();
  EXPECT_TRUE(r.plan_only);
  EXPECT_TRUE(rig.motion.executed.empty());
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(HarvestCycle, PlanOnlyRequestNeverExecutes)
{
  CycleRig rig;
  const auto r = rig.run(pm::CycleMode::FULL, true);
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::NONE);
  EXPECT_EQ(r.reason, "planned");
  EXPECT_EQ(r.reached, pm::Reached::NONE);
  EXPECT_TRUE(r.plan_only);
  EXPECT_TRUE(rig.motion.executed.empty());
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(HarvestCycle, PlanOnlyPregraspSkipsInsertPlan)
{
  CycleRig rig;
  rig.execution = false;
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::NONE);
  EXPECT_EQ(r.reason, "planned");
  EXPECT_TRUE(r.plan_only);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "insert"), 0);
}

TEST(HarvestCycle, PlanOnlyFailureNeverCarriesNone)
{
  // A backend reporting ok=false with code 0 must not look reachable (plan_only && NONE).
  CycleRig rig;
  rig.execution = false;
  rig.motion.plan_fail["insert"] = failure::NONE;
  const auto r = rig.run();
  EXPECT_TRUE(r.plan_only);
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::PLAN_FAILED);
  EXPECT_NE(r.reason, "planned");
}

TEST(HarvestCycle, PlanOnlyFailureReportsPlanningCode)
{
  CycleRig rig;
  rig.execution = false;
  rig.motion.plan_fail["transit_staging"] = failure::PLAN_NO_IK;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::PLAN_NO_IK);
  EXPECT_TRUE(r.plan_only);
  EXPECT_NE(r.reason.find("transit_staging"), std::string::npos);
}

TEST(HarvestCycle, PlanOnlyInsertFailure)
{
  CycleRig rig;
  rig.execution = false;
  rig.motion.plan_fail["insert"] = failure::PLAN_COLLISION;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::PLAN_COLLISION);
  EXPECT_TRUE(r.plan_only);
}

TEST(HarvestCycle, ExecutedResultIsNotPlanOnly)
{
  CycleRig rig;
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED);
  EXPECT_FALSE(r.plan_only);
  EXPECT_EQ(r.reached, pm::Reached::PREGRASP);
}

// ---------------------------------------------------------------- scene phases

TEST(HarvestCycle, ScenePhasePerPlannedSegment)
{
  // Every approach-side plan sees the current bag as an obstacle; insert and later do not.
  CycleRig rig;
  std::vector<pm::ScenePhase> calls;
  std::optional<pm::ScenePhase> phase;
  rig.scene = [&](pm::ScenePhase p, const std::string & target_id, std::string *) {
      EXPECT_EQ(target_id, "t1");
      calls.push_back(p);
      phase = p;
      return true;
    };
  std::map<std::string, std::set<pm::ScenePhase>> seen;
  rig.motion.on_plan = [&](const pm::PlanRequest & r) {
      ASSERT_TRUE(phase.has_value()) << r.label << " planned before any scene update";
      seen[r.label].insert(*phase);
    };
  const auto r = rig.run();
  ASSERT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  ASSERT_GE(calls.size(), 2u);
  EXPECT_EQ(calls.front(), pm::ScenePhase::APPROACH);
  EXPECT_EQ(seen["transit_staging"], std::set<pm::ScenePhase>{pm::ScenePhase::APPROACH});
  EXPECT_EQ(seen["approach_pregrasp"], std::set<pm::ScenePhase>{pm::ScenePhase::APPROACH});
  EXPECT_EQ(seen["insert"], std::set<pm::ScenePhase>{pm::ScenePhase::CONTACT});
  EXPECT_EQ(seen["transit_release"], std::set<pm::ScenePhase>{pm::ScenePhase::CONTACT});
}

TEST(HarvestCycle, PlanOnlyChainSwitchesSceneBeforeInsert)
{
  CycleRig rig;
  rig.execution = false;
  std::vector<pm::ScenePhase> calls;
  rig.scene = [&](pm::ScenePhase p, const std::string &, std::string *) {
      calls.push_back(p);
      return true;
    };
  EXPECT_EQ(rig.run().reason, "planned");
  EXPECT_EQ(
    calls, (std::vector<pm::ScenePhase>{pm::ScenePhase::APPROACH, pm::ScenePhase::CONTACT}));

  calls.clear();
  rig.motion.planned.clear();
  EXPECT_EQ(rig.run(pm::CycleMode::PREGRASP_ONLY).reason, "planned");
  EXPECT_EQ(calls, std::vector<pm::ScenePhase>{pm::ScenePhase::APPROACH});
}

TEST(HarvestCycle, SceneFailureBeforeMotionSkips)
{
  CycleRig rig;
  rig.scene = [](pm::ScenePhase, const std::string &, std::string * why) {
      *why = "apply_planning_scene_unavailable";
      return false;
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::DEPENDENCY_UNAVAILABLE);
  EXPECT_NE(r.reason.find("scene_update_failed:approach"), std::string::npos);
  EXPECT_TRUE(rig.motion.planned.empty());
  EXPECT_TRUE(rig.motion.executed.empty());
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(HarvestCycle, SceneFailureAtInsertRetreatsWithoutInserting)
{
  CycleRig rig;
  rig.scene = [](pm::ScenePhase p, const std::string &, std::string *) {
      return p == pm::ScenePhase::APPROACH;
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::DEPENDENCY_UNAVAILABLE);
  EXPECT_NE(r.reason.find("scene_update_failed:contact"), std::string::npos);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "insert"), 0);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_FALSE(r.recovery_required);
  EXPECT_EQ(rig.tool->status().state, ee::ToolState::OPEN_CONFIRMED);
}

// ---------------------------------------------------------------- admission

TEST(HarvestCycle, FullRequiresGraspAndTool)
{
  CycleRig rig;
  rig.grasp = rig.tool_enabled = false;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_TRUE(rig.motion.executed.empty());
  EXPECT_EQ(rig.io->write_count(), 0);
}

TEST(HarvestCycle, RejectsBeforeMoving)
{
  {
    CycleRig rig;
    rig.targets.geometry.reset();
    EXPECT_EQ(rig.run().failure_code, failure::MODEL_EXPIRED);
  }
  {
    CycleRig rig;
    rig.decisions.script = {std::nullopt};
    EXPECT_EQ(rig.run().reason, "decision_missing");
  }
  {
    CycleRig rig;
    auto d = peach2_fakes::default_decision(rig.clock->now() - 200.0);
    rig.decisions.script = {d};
    EXPECT_EQ(rig.run().reason, "decision_expired");
  }
  {
    CycleRig rig;
    auto d = peach2_fakes::default_decision(rig.clock->now());
    d.approach_allowed = false;
    d.failure_code = failure::BUDGET_RADIAL_NEGATIVE;
    rig.decisions.script = {d};
    const auto r = rig.run();
    EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
    EXPECT_EQ(r.failure_code, failure::BUDGET_RADIAL_NEGATIVE);
  }
  {
    CycleRig rig;
    rig.targets.geometry->d95_m = 0.13;
    const auto r = rig.run();
    EXPECT_EQ(r.failure_code, failure::TOOL_NOT_FEASIBLE);
    EXPECT_TRUE(rig.motion.executed.empty());
    EXPECT_EQ(rig.io->write_count(), 0);
  }
}

// ---------------------------------------------------------------- happy paths

TEST(HarvestCycle, FullCycleSucceeds)
{
  CycleRig rig;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(r.reached, pm::Reached::RELEASED);
  EXPECT_EQ(r.failure_code, failure::NONE);
  EXPECT_FALSE(r.recovery_required);
  EXPECT_FALSE(r.plan_only);
  EXPECT_EQ(
    rig.motion.executed,
    v({"transit_staging", "approach_pregrasp", "insert", "reverse", "transit_release"}));
  // Decision asked at start, before INSERT and before CUT; later queries pin the revision.
  EXPECT_EQ(rig.decisions.calls(), 3U);
  EXPECT_EQ(rig.decisions.min_revisions, (std::vector<uint64_t>{0U, 1U, 1U}));
  EXPECT_EQ(rig.io->write_count(), 3);   // open, close, open (release)
  EXPECT_EQ(
    rig.stages,
    v({"PREPARE_TOOL", "TRANSIT_STAGING", "APPROACH_PREGRASP", "VERIFY_PREGRASP", "INSERT", "CUT",
      "CONFIRM", "RETREAT", "TRANSIT_RELEASE", "RELEASE", "DONE"}));
  EXPECT_EQ(r.stage_names.size(), r.stage_times_s.size());
  EXPECT_EQ(r.stage_names.size(), 11U);
  EXPECT_GT(r.cycle_time_s, 0.0);
  EXPECT_DOUBLE_EQ(r.radial_margin_m, 0.008);
}

TEST(HarvestCycle, InsertGoalPutsBladeOnNeck)
{
  CycleRig rig;
  Eigen::Isometry3d insert_goal = Eigen::Isometry3d::Identity();
  rig.motion.on_plan = [&](const pm::PlanRequest & r) {
      if (r.label == "insert") {
        insert_goal = r.tcp_goal;
      }
    };
  ASSERT_EQ(rig.run().outcome, pm::Outcome::SUCCEEDED);
  const Eigen::Vector3d blade = (insert_goal * rig.tool->blade_in_tcp()).translation();
  EXPECT_TRUE(blade.isApprox(Eigen::Vector3d(0.5, 0.0, 0.88), 1e-9));
  EXPECT_NEAR(insert_goal.translation().z(), 0.88 + 0.079, 1e-9);
}

TEST(HarvestCycle, StagingIsOutsideCanopy)
{
  CycleRig rig;
  std::vector<Eigen::Isometry3d> staging;
  rig.motion.on_plan = [&](const pm::PlanRequest & r) {
      if (r.label == "transit_staging") {
        staging.push_back(r.tcp_goal);
      }
    };
  rig.run();
  ASSERT_FALSE(staging.empty());
  for (const auto & s : staging) {
    EXPECT_LE(s.translation().z(), 0.80 - pm::kStagingMinM + 1e-9);
  }
}

TEST(HarvestCycle, PregraspOnlyWithExecutionOnly)
{
  CycleRig rig;
  rig.grasp = rig.tool_enabled = false;
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(r.failure_code, failure::NONE);
  // Ordered progress: the exit is not RETREATED (that implies a cut); it shows in the stages.
  EXPECT_EQ(r.reached, pm::Reached::PREGRASP);
  EXPECT_EQ(
    r.stage_names,
    v({"TRANSIT_STAGING", "APPROACH_PREGRASP", "VERIFY_PREGRASP", "RETREAT"}));
  EXPECT_EQ(r.stage_times_s.size(), r.stage_names.size());
  EXPECT_FALSE(r.recovery_required);
  EXPECT_EQ(rig.motion.executed, v({"transit_staging", "approach_pregrasp", "reverse"}));
  EXPECT_EQ(rig.io->write_count(), 0);
  EXPECT_EQ(rig.decisions.calls(), 1U);
}

TEST(HarvestCycle, PregraspOnlyRetreatFailureKeepsPregrasp)
{
  CycleRig rig;
  rig.grasp = rig.tool_enabled = false;
  rig.motion.exec_fail["reverse"] = {false, failure::EXEC_FAILED, "fake_exec_fail"};
  rig.motion.plan_fail["retreat_fallback"] = failure::PLAN_CARTESIAN_INCOMPLETE;
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::RETREAT_FAILED);
  EXPECT_TRUE(r.recovery_required);
  EXPECT_EQ(r.reached, pm::Reached::PREGRASP);
  ASSERT_FALSE(r.stage_names.empty());
  EXPECT_EQ(r.stage_names.back(), "RETREAT");
}

TEST(HarvestCycle, PregraspOnlyWithoutRetreat)
{
  CycleRig rig;
  rig.config.pregrasp_only_retreat = false;
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED);
  EXPECT_EQ(r.reached, pm::Reached::PREGRASP);
  EXPECT_FALSE(r.recovery_required);
  EXPECT_EQ(rig.motion.executed, v({"transit_staging", "approach_pregrasp"}));
}

// ---------------------------------------------------------------- null motion (start at goal)

TEST(HarvestCycle, TransitAlreadyAtStagingIsNotSent)
{
  CycleRig rig;
  rig.motion.at_goal.insert("transit_staging");
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(r.reached, pm::Reached::RELEASED);
  EXPECT_EQ(
    rig.motion.executed, v({"approach_pregrasp", "insert", "reverse", "transit_release"}));
  EXPECT_EQ(r.stage_names.front(), "PREPARE_TOOL");
  EXPECT_EQ(r.stage_names[1], "TRANSIT_STAGING");
}

TEST(HarvestCycle, NullMotionRetreatFallbackCountsAsExited)
{
  CycleRig rig;
  rig.motion.exec_fail["reverse"] = {false, failure::EXEC_FAILED, "fake_exec_fail"};
  rig.motion.at_goal.insert("retreat_fallback");
  const auto r = rig.run(pm::CycleMode::PREGRASP_ONLY);
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_FALSE(r.recovery_required);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "retreat_fallback"), 1);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "null_motion"), 0);
}

TEST(HarvestCycle, NullMotionTransitReleaseIsNotSent)
{
  CycleRig rig;
  rig.motion.at_goal.insert("transit_release");
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(r.reached, pm::Reached::RELEASED);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "transit_release"), 0);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "null_motion"), 0);
}

// ---------------------------------------------------------------- pregrasp residual

TEST(HarvestCycle, ResidualCorrectedOnce)
{
  CycleRig rig;
  rig.motion.tcp_error["approach_pregrasp"] = Eigen::Vector3d(0.006, 0.0, 0.0);
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "pregrasp_correction"), 1);
}

TEST(HarvestCycle, ResidualCorrectionNotCollapsedAsAtGoal)
{
  // 4 mm TCP residual is above the 3 mm gate and below 0.005 rad at-goal; must still move.
  CycleRig rig;
  rig.motion.at_goal_tolerance_rad = 0.005;
  rig.motion.tcp_error["approach_pregrasp"] = Eigen::Vector3d(0.004, 0.0, 0.0);
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "pregrasp_correction"), 1);
}

TEST(HarvestCycle, ResidualFailsAndRetreats)
{
  for (auto mode : {pm::CycleMode::FULL, pm::CycleMode::PREGRASP_ONLY}) {
    CycleRig rig;
    rig.motion.tcp_error["approach_pregrasp"] = Eigen::Vector3d(0.006, 0.0, 0.0);
    rig.motion.tcp_error["pregrasp_correction"] = Eigen::Vector3d(0.006, 0.0, 0.0);
    const auto r = rig.run(mode);
    EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
    EXPECT_EQ(r.failure_code, failure::PREGRASP_RESIDUAL);
    // FULL keeps its historical RETREATED; pregrasp-only never got past NONE.
    EXPECT_EQ(
      r.reached, mode == pm::CycleMode::FULL ? pm::Reached::RETREATED : pm::Reached::NONE);
    EXPECT_FALSE(r.recovery_required);
    EXPECT_EQ(rig.motion.executed.back(), "reverse");
    EXPECT_EQ(FakeMotion::count(rig.motion.planned, "insert"), 0);
  }
}

// ---------------------------------------------------------------- decision re-queries

TEST(HarvestCycle, SleeveDeniedBeforeInsert)
{
  CycleRig rig;
  auto d1 = peach2_fakes::default_decision(rig.clock->now());
  auto d2 = d1;
  d2.sleeve_allowed = false;
  d2.cut_allowed = false;
  d2.failure_code = failure::BUDGET_RADIAL_NEGATIVE;
  rig.decisions.script = {d1, d2};
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::BUDGET_RADIAL_NEGATIVE);
  EXPECT_EQ(rig.motion.executed, v({"transit_staging", "approach_pregrasp", "reverse"}));
  EXPECT_EQ(rig.io->write_count(), 1);   // prepare only, never closed
  EXPECT_FALSE(r.recovery_required);
}

TEST(HarvestCycle, CutDeniedAfterInsert)
{
  CycleRig rig;
  auto d1 = peach2_fakes::default_decision(rig.clock->now());
  auto d3 = d1;
  d3.cut_allowed = false;
  rig.decisions.script = {d1, d1, d3};
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::BUDGET_AXIAL_NEGATIVE);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_EQ(rig.io->write_count(), 1);
  EXPECT_EQ(rig.motion.executed.back(), "reverse");
}

TEST(HarvestCycle, ModelMovedIsStale)
{
  CycleRig rig;
  auto d1 = peach2_fakes::default_decision(rig.clock->now());
  auto d2 = d1;
  d2.revision = 2;
  d2.pregrasp_tcp.translation().x() += 0.05;
  rig.decisions.script = {d1, d2};
  const auto r = rig.run();
  EXPECT_EQ(r.failure_code, failure::MODEL_STALE);
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "insert"), 0);
}

TEST(HarvestCycle, SmallRevisionChangeAccepted)
{
  CycleRig rig;
  auto d1 = peach2_fakes::default_decision(rig.clock->now());
  auto d2 = d1;
  d2.revision = 2;
  d2.blade_target.z() += 0.002;
  rig.decisions.script = {d1, d2};
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(rig.decisions.min_revisions.back(), 2U);
}

TEST(HarvestCycle, DecisionExpiresBeforeCut)
{
  CycleRig rig;
  auto d1 = peach2_fakes::default_decision(rig.clock->now());
  auto d3 = d1;
  d3.valid_until_s = rig.clock->now() - 1.0;
  rig.decisions.script = {d1, d1, d3};
  const auto r = rig.run();
  EXPECT_EQ(r.failure_code, failure::MODEL_EXPIRED);
  EXPECT_EQ(rig.io->write_count(), 1);
}

// ---------------------------------------------------------------- tool failures

TEST(HarvestCycle, PrepareFailureStopsBeforeMotion)
{
  CycleRig rig(mock_with(Fault::STUCK_CLOSED));
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::TOOL_NOT_OPEN);
  EXPECT_TRUE(r.recovery_required);   // tool latched in FAULT
  EXPECT_TRUE(rig.motion.executed.empty());
}

TEST(HarvestCycle, CutNotConfirmedRetriesOnceThenSucceeds)
{
  ee::MockIoBackend::Config mock;
  mock.simulate_current = true;
  mock.current_cut_signature = false;
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  CycleRig rig(mock, cur);
  int cuts = 0;
  rig.motion.during_execute = [&](const std::string & label) {
      if (label == "reverse" && cuts == 0) {
        ++cuts;
        rig.io->set_current_cut_signature(true);
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "insert"), 2);
  EXPECT_EQ(
    rig.motion.executed,
    v({"transit_staging", "approach_pregrasp", "insert", "reverse", "insert", "reverse",
      "transit_release"}));
  EXPECT_EQ(rig.decisions.calls(), 5U);
}

TEST(HarvestCycle, CutNotConfirmedTwiceFails)
{
  ee::MockIoBackend::Config mock;
  mock.simulate_current = true;
  mock.current_cut_signature = false;
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  CycleRig rig(mock, cur);
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::CUT_NOT_CONFIRMED);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_FALSE(r.recovery_required);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "insert"), 2);
  EXPECT_EQ(rig.io->last_output(0), false);   // left open
}

TEST(HarvestCycle, CutRetryDisabled)
{
  ee::MockIoBackend::Config mock;
  mock.simulate_current = true;
  mock.current_cut_signature = false;
  ee::CurrentSignatureConfig cur;
  cur.enabled = true;
  CycleRig rig(mock, cur);
  rig.config.cut_retry_max = 0;
  const auto r = rig.run();
  EXPECT_EQ(r.failure_code, failure::CUT_NOT_CONFIRMED);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "insert"), 1);
}

TEST(HarvestCycle, FeedbackTimeoutNeedsRecovery)
{
  CycleRig rig(mock_with(Fault::STUCK_OPEN));
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::TOOL_FEEDBACK_TIMEOUT);
  EXPECT_TRUE(r.recovery_required);
  EXPECT_EQ(rig.motion.executed.back(), "reverse");
  EXPECT_EQ(rig.io->last_output(0), false);
  EXPECT_EQ(rig.tool->status().state, ee::ToolState::FAULT);
}

TEST(HarvestCycle, LoopbackIsToolFault)
{
  CycleRig rig(mock_with(Fault::LOOPBACK));
  const auto r = rig.run();
  EXPECT_EQ(r.failure_code, failure::TOOL_FAULT);
  EXPECT_TRUE(r.recovery_required);
  EXPECT_TRUE(rig.tool->status().suspected_loopback);
}

TEST(HarvestCycle, ReleaseFailure)
{
  CycleRig rig(mock_with(Fault::OPEN_STUCK));
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::TOOL_NOT_OPEN);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_TRUE(r.recovery_required);
}

// ---------------------------------------------------------------- gate / cancel

TEST(HarvestCycle, GateClosesDuringInsert)
{
  CycleRig rig;
  rig.motion.during_execute = [&](const std::string & label) {
      if (label == "insert") {
        rig.execution = false;
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_TRUE(r.recovery_required);
  EXPECT_GE(rig.motion.stop_calls, 1);
  EXPECT_EQ(rig.motion.executed.back(), "insert");   // no retreat without execution
  EXPECT_EQ(rig.io->last_output(0), false);
}

TEST(HarvestCycle, HeartbeatLostDuringApproach)
{
  CycleRig rig;
  rig.motion.during_execute = [&](const std::string & label) {
      if (label == "approach_pregrasp") {
        rig.heartbeat = false;
        rig.clock->sleep(5.0);
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.failure_code, failure::SAFETY_GATE_CLOSED);
  EXPECT_NE(r.reason.find("enables_heartbeat_lost"), std::string::npos);
  EXPECT_TRUE(r.recovery_required);
}

TEST(HarvestCycle, CancelDuringApproach)
{
  CycleRig rig;
  rig.motion.during_execute = [&](const std::string & label) {
      if (label == "approach_pregrasp") {
        rig.cancel = true;
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::CANCELED);
  EXPECT_EQ(r.failure_code, failure::CANCELED);
  EXPECT_TRUE(r.recovery_required);
  EXPECT_EQ(rig.motion.executed.back(), "approach_pregrasp");
}

TEST(HarvestCycle, CancelDuringTransitNoRecovery)
{
  CycleRig rig;
  rig.motion.during_execute = [&](const std::string & label) {
      if (label == "transit_staging") {
        rig.cancel = true;
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::CANCELED);
  EXPECT_FALSE(r.recovery_required);
}

// ---------------------------------------------------------------- execution / retreat

TEST(HarvestCycle, ExecFailureInCanopyRetreats)
{
  CycleRig rig;
  rig.motion.exec_fail["insert"] = {false, failure::EXEC_FAILED, "fake"};
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::EXEC_FAILED);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_FALSE(r.recovery_required);
}

TEST(HarvestCycle, RetreatFallsBackToAxialLin)
{
  CycleRig rig;
  rig.motion.validate_ok = false;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "retreat_fallback"), 1);
  EXPECT_EQ(FakeMotion::count(rig.motion.executed, "reverse"), 0);
}

TEST(HarvestCycle, RetreatFailureNeedsRecovery)
{
  CycleRig rig;
  rig.motion.validate_ok = false;
  rig.motion.plan_fail["retreat_fallback"] = failure::PLAN_CARTESIAN_INCOMPLETE;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::RETREAT_FAILED);
  EXPECT_TRUE(r.recovery_required);
}

TEST(HarvestCycle, StartStateMismatchReplans)
{
  CycleRig rig;
  int approach_plans = 0;
  rig.motion.on_plan = [&](const pm::PlanRequest & r) {
      if (r.label == "approach_pregrasp" && ++approach_plans == rig.config.roll_samples) {
        rig.motion.joints[0] += 0.1;   // arm moved after roll selection finished
      }
    };
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SUCCEEDED) << r.reason;
  EXPECT_EQ(FakeMotion::count(rig.motion.planned, "transit_staging"), rig.config.roll_samples + 1);
}

TEST(HarvestCycle, ApproachUnplannableForAllRolls)
{
  CycleRig rig;
  rig.motion.plan_fail["approach_pregrasp"] = failure::PLAN_CARTESIAN_INCOMPLETE;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::SKIPPED);
  EXPECT_EQ(r.failure_code, failure::PLAN_CARTESIAN_INCOMPLETE);
  EXPECT_TRUE(rig.motion.executed.empty());
}

TEST(HarvestCycle, TransitReleaseFailure)
{
  CycleRig rig;
  rig.motion.plan_fail["transit_release"] = failure::PLAN_FAILED;
  const auto r = rig.run();
  EXPECT_EQ(r.outcome, pm::Outcome::FAILED);
  EXPECT_EQ(r.failure_code, failure::PLAN_FAILED);
  EXPECT_EQ(r.reached, pm::Reached::RETREATED);
  EXPECT_FALSE(r.recovery_required);
}

TEST(HarvestCycle, ToolMismatch)
{
  CycleRig rig;
  pm::CycleDeps deps;
  deps.motion = &rig.motion;
  deps.ee = rig.tool.get();
  deps.decisions = &rig.decisions;
  deps.targets = &rig.targets;
  deps.gate = [&](pm::GateStage s, bool nt) {return rig.gate.check(s, rig.clock->now(), nt);};
  deps.enables = [&]() {return rig.gate.enables(rig.clock->now());};
  deps.cancel_requested = []() {return false;};
  deps.now_s = [&]() {return rig.clock->now();};
  deps.sleep_s = [&](double dt) {rig.clock->sleep(dt);};
  pm::HarvestCycle cycle(rig.config, deps);
  pm::CycleRequest req;
  req.target_id = "t1";
  req.tool_id = "shear_v1";
  const auto r = cycle.run(req);
  EXPECT_EQ(r.failure_code, failure::TOOL_NOT_FEASIBLE);
  EXPECT_TRUE(rig.motion.planned.empty());
}
