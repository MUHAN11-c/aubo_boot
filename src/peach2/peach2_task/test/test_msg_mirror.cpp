// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
// The pure core mirrors IDL constants to stay ROS-free; this test pins the mirror to the IDL.
#include <gtest/gtest.h>

#include <peach2_interfaces/action/harvest_target.hpp>
#include <peach2_interfaces/action/run_batch.hpp>
#include <peach2_interfaces/msg/batch_state.hpp>
#include <peach2_interfaces/msg/failure_code.hpp>
#include <peach2_interfaces/msg/harvest_result.hpp>
#include <peach2_interfaces/msg/target_observation.hpp>
#include <peach2_interfaces/msg/target_observation_array.hpp>
#include <peach2_interfaces/srv/check_reachability.hpp>

#include <string>
#include <vector>

#include "peach2_task/core/batch_policy.hpp"
#include "peach2_task/core/batch_session.hpp"
#include "peach2_task/ros/conversions.hpp"

namespace core = peach2_task::core;
namespace fc = peach2_task::core::fc;
namespace conv = peach2_task::ros;
using peach2_interfaces::msg::BatchState;
using peach2_interfaces::msg::FailureCode;
using peach2_interfaces::msg::HarvestResult;

TEST(MsgMirror, FailureCodes)
{
  EXPECT_EQ(fc::NONE, FailureCode::NONE);
  EXPECT_EQ(fc::PERCEPTION_NO_TARGET, FailureCode::PERCEPTION_NO_TARGET);
  EXPECT_EQ(fc::PERCEPTION_EXACT_TF_MISSING, FailureCode::PERCEPTION_EXACT_TF_MISSING);
  EXPECT_EQ(fc::PERCEPTION_LOW_QUALITY, FailureCode::PERCEPTION_LOW_QUALITY);
  EXPECT_EQ(fc::PERCEPTION_OUT_OF_SCOPE, FailureCode::PERCEPTION_OUT_OF_SCOPE);
  EXPECT_EQ(fc::MODEL_NOT_CONVERGED, FailureCode::MODEL_NOT_CONVERGED);
  EXPECT_EQ(fc::MODEL_STALE, FailureCode::MODEL_STALE);
  EXPECT_EQ(fc::MODEL_EXPIRED, FailureCode::MODEL_EXPIRED);
  EXPECT_EQ(fc::BUDGET_RADIAL_NEGATIVE, FailureCode::BUDGET_RADIAL_NEGATIVE);
  EXPECT_EQ(fc::BUDGET_AXIAL_NEGATIVE, FailureCode::BUDGET_AXIAL_NEGATIVE);
  EXPECT_EQ(fc::BUDGET_STRUCTURAL, FailureCode::BUDGET_STRUCTURAL);
  EXPECT_EQ(fc::NECK_REMEASURE_MISMATCH, FailureCode::NECK_REMEASURE_MISMATCH);
  EXPECT_EQ(fc::SWING_TOO_LARGE, FailureCode::SWING_TOO_LARGE);
  EXPECT_EQ(fc::NECK_REMEASURE_PENDING, FailureCode::NECK_REMEASURE_PENDING);
  EXPECT_EQ(fc::PLAN_NO_IK, FailureCode::PLAN_NO_IK);
  EXPECT_EQ(fc::PLAN_COLLISION, FailureCode::PLAN_COLLISION);
  EXPECT_EQ(fc::PLAN_FAILED, FailureCode::PLAN_FAILED);
  EXPECT_EQ(fc::PLAN_CARTESIAN_INCOMPLETE, FailureCode::PLAN_CARTESIAN_INCOMPLETE);
  EXPECT_EQ(fc::EXEC_FAILED, FailureCode::EXEC_FAILED);
  EXPECT_EQ(fc::EXEC_TIMEOUT, FailureCode::EXEC_TIMEOUT);
  EXPECT_EQ(fc::CONTACT_ABORT, FailureCode::CONTACT_ABORT);
  EXPECT_EQ(fc::PREGRASP_RESIDUAL, FailureCode::PREGRASP_RESIDUAL);
  EXPECT_EQ(fc::RETREAT_FAILED, FailureCode::RETREAT_FAILED);
  EXPECT_EQ(fc::TARGET_TIMEOUT, FailureCode::TARGET_TIMEOUT);
  EXPECT_EQ(fc::TOOL_NOT_OPEN, FailureCode::TOOL_NOT_OPEN);
  EXPECT_EQ(fc::TOOL_COMMAND_FAILED, FailureCode::TOOL_COMMAND_FAILED);
  EXPECT_EQ(fc::TOOL_FEEDBACK_TIMEOUT, FailureCode::TOOL_FEEDBACK_TIMEOUT);
  EXPECT_EQ(fc::CUT_NOT_CONFIRMED, FailureCode::CUT_NOT_CONFIRMED);
  EXPECT_EQ(fc::TOOL_FAULT, FailureCode::TOOL_FAULT);
  EXPECT_EQ(fc::TOOL_NOT_FEASIBLE, FailureCode::TOOL_NOT_FEASIBLE);
  EXPECT_EQ(fc::SAFETY_GATE_CLOSED, FailureCode::SAFETY_GATE_CLOSED);
  EXPECT_EQ(fc::ROBOT_NOT_READY, FailureCode::ROBOT_NOT_READY);
  EXPECT_EQ(fc::CANCELED, FailureCode::CANCELED);
  EXPECT_EQ(fc::RECOVERY_REQUIRED, FailureCode::RECOVERY_REQUIRED);
  EXPECT_EQ(fc::ENVIRONMENT_UNSAFE, FailureCode::ENVIRONMENT_UNSAFE);
  EXPECT_EQ(fc::DEPENDENCY_UNAVAILABLE, FailureCode::DEPENDENCY_UNAVAILABLE);
}

TEST(MsgMirror, PhasesOutcomesIntents)
{
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::IDLE), BatchState::IDLE);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::SURVEYING), BatchState::SURVEYING);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::SELECTING), BatchState::SELECTING);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::OBSERVING), BatchState::OBSERVING);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::HARVESTING), BatchState::HARVESTING);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::WAITING_ACK), BatchState::WAITING_ACK);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::PAUSED), BatchState::PAUSED);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::COMPLETED), BatchState::COMPLETED);
  EXPECT_EQ(static_cast<uint8_t>(core::Phase::ABORTED), BatchState::ABORTED);

  EXPECT_EQ(core::outcome::SUCCEEDED, HarvestResult::OUTCOME_SUCCEEDED);
  EXPECT_EQ(core::outcome::SKIPPED, HarvestResult::OUTCOME_SKIPPED);
  EXPECT_EQ(core::outcome::FAILED, HarvestResult::OUTCOME_FAILED);
  EXPECT_EQ(core::outcome::CANCELED, HarvestResult::OUTCOME_CANCELED);
  EXPECT_EQ(core::reached::NONE, HarvestResult::REACHED_NONE);
  EXPECT_EQ(core::reached::PREGRASP, HarvestResult::REACHED_PREGRASP);
  EXPECT_EQ(core::reached::INSERTED, HarvestResult::REACHED_INSERTED);
  EXPECT_EQ(core::reached::CUT_CONFIRMED, HarvestResult::REACHED_CUT_CONFIRMED);
  EXPECT_EQ(core::reached::RETREATED, HarvestResult::REACHED_RETREATED);
  EXPECT_EQ(core::reached::RELEASED, HarvestResult::REACHED_RELEASED);

  using Goal = peach2_interfaces::action::RunBatch::Goal;
  EXPECT_EQ(static_cast<uint8_t>(core::Intent::SURVEY_ONLY), Goal::INTENT_SURVEY_ONLY);
  EXPECT_EQ(static_cast<uint8_t>(core::Intent::PREGRASP_ONLY), Goal::INTENT_PREGRASP_ONLY);
  EXPECT_EQ(static_cast<uint8_t>(core::Intent::FULL), Goal::INTENT_FULL);
  EXPECT_EQ(core::kCategoryBag, peach2_interfaces::msg::TargetObservation::CATEGORY_BAG);
}

TEST(Conversions, CandidateHeightFallsBackToBottom)
{
  peach2_interfaces::msg::TargetObservation obs;
  obs.target_id = "a";
  obs.bottom.valid = true;
  obs.bottom.position.z = 1.1;
  obs.neck.position.z = 1.3;
  obs.roi.width = 10;
  obs.roi.height = 20;
  auto c = conv::to_candidate(obs);
  EXPECT_TRUE(c.has_geometry);
  EXPECT_DOUBLE_EQ(c.height_m, 1.1);
  EXPECT_DOUBLE_EQ(c.roi_area_px, 200.0);
  obs.neck.valid = true;
  c = conv::to_candidate(obs);
  EXPECT_DOUBLE_EQ(c.height_m, 1.3);
  obs.bottom.valid = obs.neck.valid = false;
  EXPECT_FALSE(conv::to_candidate(obs).has_geometry);
}

TEST(Conversions, ObservationSetCarriesLockedIds)
{
  peach2_interfaces::msg::TargetObservationArray array;
  array.target_set_locked = true;
  array.scene_epoch = 3;
  array.locked_target_ids = {"a", "b"};
  array.observations.resize(1);
  array.observations[0].target_id = "a";
  const auto set = conv::to_observation_set(array);
  EXPECT_TRUE(set.target_set_locked);
  EXPECT_EQ(set.scene_epoch, 3U);
  EXPECT_EQ(set.locked_target_ids, (std::vector<std::string>{"a", "b"}));
  ASSERT_EQ(set.candidates.size(), 1U);
}

TEST(Conversions, ReachMapRejectsMismatchedArrays)
{
  peach2_interfaces::srv::CheckReachability::Response r;
  r.target_ids = {"a", "b"};
  r.reachable = {true, false};
  r.failure_codes = {0, 30};
  core::ReachMap m;
  ASSERT_TRUE(conv::to_reach_map(r, &m));
  EXPECT_TRUE(m.at("a").reachable);
  EXPECT_EQ(m.at("b").failure_code, 30U);
  EXPECT_TRUE(m.at("b").reason.empty());
  r.reasons = {"", "no ik at pregrasp"};
  ASSERT_TRUE(conv::to_reach_map(r, &m));
  EXPECT_EQ(m.at("b").reason, "no ik at pregrasp");
  r.reasons = {"only one"};
  EXPECT_FALSE(conv::to_reach_map(r, &m));
  r.reasons.clear();
  r.failure_codes = {0};
  EXPECT_FALSE(conv::to_reach_map(r, &m));
}

TEST(Conversions, DecisionExpiry)
{
  peach2_interfaces::msg::GraspDecision d;
  d.approach_allowed = true;
  d.model_revision = 4;
  d.valid_until.sec = 100;
  EXPECT_FALSE(conv::to_decision(true, d, 99LL * 1000000000LL).expired);
  EXPECT_TRUE(conv::to_decision(true, d, 101LL * 1000000000LL).expired);
  d.valid_until.sec = 0;
  EXPECT_FALSE(conv::to_decision(true, d, 101LL * 1000000000LL).expired);
  const auto missing = conv::to_decision(false, d, 0);
  EXPECT_FALSE(missing.found);
  EXPECT_FALSE(missing.approach_allowed);
}

TEST(Conversions, HarvestResultRoundTripAndModes)
{
  core::HarvestOutcome o;
  o.target_id = "a";
  o.outcome = core::outcome::SKIPPED;
  o.failure_code = fc::TOOL_NOT_FEASIBLE;
  o.stage_names = {"approach"};
  o.stage_times_s = {1.0};
  const auto back = conv::to_outcome(conv::to_msg(o));
  EXPECT_EQ(back.target_id, "a");
  EXPECT_EQ(back.outcome, core::outcome::SKIPPED);
  EXPECT_EQ(back.failure_code, fc::TOOL_NOT_FEASIBLE);
  EXPECT_EQ(back.stage_names, o.stage_names);
  EXPECT_FALSE(back.plan_only);
  o.plan_only = true;
  EXPECT_TRUE(conv::to_msg(o).plan_only);
  EXPECT_TRUE(conv::to_outcome(conv::to_msg(o)).plan_only);

  using HGoal = peach2_interfaces::action::HarvestTarget::Goal;
  EXPECT_EQ(conv::harvest_mode_for(core::Intent::FULL), HGoal::MODE_FULL);
  EXPECT_EQ(conv::harvest_mode_for(core::Intent::PREGRASP_ONLY), HGoal::MODE_PREGRASP_ONLY);
  using RReq = peach2_interfaces::srv::CheckReachability::Request;
  EXPECT_EQ(conv::reachability_mode_for(core::Intent::FULL), RReq::MODE_FULL);
  EXPECT_EQ(conv::reachability_mode_for(core::Intent::SURVEY_ONLY), RReq::MODE_PREGRASP_ONLY);
}

TEST(Conversions, BatchStateFromSnapshot)
{
  core::StateSnapshot s;
  s.request_id = "r";
  s.phase = core::Phase::WAITING_ACK;
  s.blockers = {"recovery_required"};
  s.counts.attempted = 2;
  s.recovery_required = true;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;
  const auto m = conv::to_msg(s, stamp);
  EXPECT_EQ(m.phase, BatchState::WAITING_ACK);
  EXPECT_EQ(m.attempted, 2U);
  EXPECT_TRUE(m.recovery_required);
  EXPECT_EQ(m.header.stamp.sec, 5);
}
