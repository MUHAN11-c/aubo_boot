// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

#include "peach2_task/core/batch_policy.hpp"

namespace core = peach2_task::core;
namespace fc = peach2_task::core::fc;
using core::Policy;

TEST(BatchPolicy, EveryCodeHasTheDocumentedPolicy)
{
  // Mirrors the per-code comments of peach2_interfaces/msg/FailureCode.msg.
  const std::map<uint32_t, Policy> expected{
    {fc::PERCEPTION_NO_TARGET, Policy::SKIP},
    {fc::PERCEPTION_EXACT_TF_MISSING, Policy::RETRY_VIEW},
    {fc::PERCEPTION_LOW_QUALITY, Policy::RETRY_VIEW},
    {fc::PERCEPTION_OUT_OF_SCOPE, Policy::SKIP},
    {fc::MODEL_NOT_CONVERGED, Policy::SKIP},
    {fc::MODEL_STALE, Policy::RETRY_VIEW},
    {fc::MODEL_EXPIRED, Policy::RETRY_VIEW},
    {fc::BUDGET_RADIAL_NEGATIVE, Policy::SKIP_TOOL},
    {fc::BUDGET_AXIAL_NEGATIVE, Policy::APPROACH_ONLY},
    {fc::BUDGET_STRUCTURAL, Policy::APPROACH_ONLY},
    {fc::NECK_REMEASURE_MISMATCH, Policy::APPROACH_ONLY},
    {fc::SWING_TOO_LARGE, Policy::WAIT},
    {fc::NECK_REMEASURE_PENDING, Policy::REMEASURE_NECK},
    {fc::PLAN_NO_IK, Policy::SKIP},
    {fc::PLAN_COLLISION, Policy::SKIP},
    {fc::PLAN_FAILED, Policy::RETRY_VIEW},
    {fc::PLAN_CARTESIAN_INCOMPLETE, Policy::SKIP},
    {fc::EXEC_FAILED, Policy::RECOVER},
    {fc::EXEC_TIMEOUT, Policy::RECOVER},
    {fc::CONTACT_ABORT, Policy::SKIP},
    {fc::PREGRASP_RESIDUAL, Policy::RETRY_VIEW},
    {fc::RETREAT_FAILED, Policy::RECOVER},
    {fc::TARGET_TIMEOUT, Policy::SKIP},
    {fc::TOOL_NOT_OPEN, Policy::RECOVER},
    {fc::TOOL_COMMAND_FAILED, Policy::RECOVER},
    {fc::TOOL_FEEDBACK_TIMEOUT, Policy::RECOVER},
    {fc::CUT_NOT_CONFIRMED, Policy::RETRY_VIEW},
    {fc::TOOL_FAULT, Policy::RECOVER},
    {fc::TOOL_NOT_FEASIBLE, Policy::SKIP_TOOL},
    {fc::SAFETY_GATE_CLOSED, Policy::STOP_BATCH},
    {fc::ROBOT_NOT_READY, Policy::STOP_BATCH},
    {fc::CANCELED, Policy::STOP_BATCH},
    {fc::RECOVERY_REQUIRED, Policy::RECOVER},
    {fc::ENVIRONMENT_UNSAFE, Policy::WAIT},
    {fc::DEPENDENCY_UNAVAILABLE, Policy::STOP_BATCH},
  };
  for (const auto & [code, policy] : expected) {
    EXPECT_TRUE(core::is_known_code(code)) << code;
    EXPECT_EQ(core::policy_for(code), policy) << core::failure_name(code);
  }
  EXPECT_EQ(core::failure_name(fc::NONE), "NONE");
  EXPECT_EQ(core::failure_name(fc::TOOL_FAULT), "TOOL_FAULT");
  EXPECT_EQ(core::failure_name(fc::DEPENDENCY_UNAVAILABLE), "DEPENDENCY_UNAVAILABLE");
  EXPECT_STREQ(core::policy_name(Policy::REMEASURE_NECK), "REMEASURE_NECK");
}

TEST(BatchPolicy, UnknownCodesFallBackByGroup)
{
  EXPECT_FALSE(core::is_known_code(19));
  EXPECT_EQ(core::failure_name(19), "UNKNOWN_19");
  EXPECT_EQ(core::policy_for(19), Policy::SKIP);
  EXPECT_EQ(core::policy_for(49), Policy::RECOVER);
  EXPECT_EQ(core::policy_for(59), Policy::RECOVER);
  EXPECT_EQ(core::policy_for(69), Policy::STOP_BATCH);
  EXPECT_EQ(core::policy_for(999), Policy::SKIP);
}

TEST(BatchPolicy, ReworkKinds)
{
  auto kind = [](uint32_t code) {return core::rework_kind(code, core::policy_for(code));};
  EXPECT_EQ(kind(fc::CANCELED), "canceled");
  EXPECT_EQ(kind(fc::BUDGET_RADIAL_NEGATIVE), "tool");
  EXPECT_EQ(kind(fc::TOOL_NOT_FEASIBLE), "tool");
  EXPECT_EQ(kind(fc::BUDGET_AXIAL_NEGATIVE), "approach_only");
  EXPECT_EQ(kind(fc::EXEC_FAILED), "recovery");
  EXPECT_EQ(kind(fc::SAFETY_GATE_CLOSED), "safety");
  EXPECT_EQ(kind(fc::SWING_TOO_LARGE), "swing");
  EXPECT_EQ(kind(fc::ENVIRONMENT_UNSAFE), "environment");
  EXPECT_EQ(kind(fc::PERCEPTION_NO_TARGET), "perception");
  EXPECT_EQ(kind(fc::MODEL_NOT_CONVERGED), "model");
  EXPECT_EQ(kind(fc::PLAN_FAILED), "planning");
  EXPECT_EQ(kind(fc::PLAN_NO_IK), "unreachable");
  EXPECT_EQ(kind(fc::CONTACT_ABORT), "contact_failed");
  EXPECT_EQ(kind(fc::NECK_REMEASURE_PENDING), "neck_remeasure");
  EXPECT_EQ(kind(fc::TARGET_TIMEOUT), "timeout");
  EXPECT_EQ(kind(fc::DEPENDENCY_UNAVAILABLE), "infrastructure");
  EXPECT_EQ(core::rework_kind(fc::NONE, Policy::SKIP), "other");
}

TEST(BatchPolicy, ValidateLimits)
{
  core::BatchLimits l;
  EXPECT_TRUE(core::validate(l).empty());
  l.target_harvest_ratio = 1.5;
  EXPECT_FALSE(core::validate(l).empty());
  l.target_harvest_ratio = NAN;
  EXPECT_FALSE(core::validate(l).empty());
  l = core::BatchLimits{};
  l.per_target_timeout_s = -1.0;
  EXPECT_FALSE(core::validate(l).empty());
  l = core::BatchLimits{};
  l.empty_survey_limit = 0;
  EXPECT_FALSE(core::validate(l).empty());
}

TEST(BatchPolicy, GateMaxTargetsCountsAttempted)
{
  core::BatchLimits l;
  l.max_targets = 2;
  core::Counts c;
  c.attempted = 1;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::CONTINUE);
  c.attempted = 2;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::MAX_TARGETS);
  c.attempted = 1;
  c.plan_only = 1;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::MAX_TARGETS);
  EXPECT_STREQ(core::gate_reason(core::Gate::MAX_TARGETS), "max_targets");
}

TEST(BatchPolicy, GateRatioRoundsUp)
{
  core::BatchLimits l;
  l.target_harvest_ratio = 0.5;
  core::Counts c;
  c.discovered = 3;
  c.succeeded = 1;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::CONTINUE);
  c.succeeded = 2;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::RATIO_REACHED);
  c.discovered = 4;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::RATIO_REACHED);
  c.discovered = 0;
  c.succeeded = 0;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::CONTINUE);
  l.target_harvest_ratio = 0.3;
  c.discovered = 10;
  c.succeeded = 3;
  EXPECT_EQ(core::evaluate_gate(l, c), core::Gate::RATIO_REACHED);
}

TEST(BatchPolicy, EmptySurveyLimit)
{
  core::BatchLimits l;
  EXPECT_FALSE(core::empty_limit_reached(1, l));
  EXPECT_TRUE(core::empty_limit_reached(2, l));
  l.empty_survey_limit = 0;
  EXPECT_TRUE(core::empty_limit_reached(1, l));
}

TEST(BatchPolicy, TargetDeadline)
{
  core::TargetDeadline d;
  EXPECT_FALSE(d.armed());
  EXPECT_FALSE(d.exceeded(1e9));
  d.start(10.0, 5.0);
  EXPECT_TRUE(d.armed());
  EXPECT_FALSE(d.exceeded(14.9));
  EXPECT_NEAR(d.remaining_s(12.0), 3.0, 1e-9);
  EXPECT_TRUE(d.exceeded(15.0));
  d.start(10.0, 0.0);
  EXPECT_FALSE(d.armed());
  EXPECT_TRUE(std::isinf(d.remaining_s(100.0)));
  d.start(0.0, 1.0);
  d.clear();
  EXPECT_FALSE(d.exceeded(100.0));
}
