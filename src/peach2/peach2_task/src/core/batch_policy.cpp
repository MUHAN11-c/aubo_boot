// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/batch_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace peach2_task::core
{
namespace
{

struct CodeRow
{
  uint32_t code;
  const char * name;
  Policy policy;
};

// Policies follow the trailing comment of each constant in FailureCode.msg.
constexpr CodeRow kCodeTable[] = {
  {fc::NONE, "NONE", Policy::NONE},
  {fc::PERCEPTION_NO_TARGET, "PERCEPTION_NO_TARGET", Policy::SKIP},
  {fc::PERCEPTION_EXACT_TF_MISSING, "PERCEPTION_EXACT_TF_MISSING", Policy::RETRY_VIEW},
  {fc::PERCEPTION_LOW_QUALITY, "PERCEPTION_LOW_QUALITY", Policy::RETRY_VIEW},
  {fc::PERCEPTION_OUT_OF_SCOPE, "PERCEPTION_OUT_OF_SCOPE", Policy::SKIP},
  {fc::MODEL_NOT_CONVERGED, "MODEL_NOT_CONVERGED", Policy::SKIP},
  {fc::MODEL_STALE, "MODEL_STALE", Policy::RETRY_VIEW},
  {fc::MODEL_EXPIRED, "MODEL_EXPIRED", Policy::RETRY_VIEW},
  {fc::BUDGET_RADIAL_NEGATIVE, "BUDGET_RADIAL_NEGATIVE", Policy::SKIP_TOOL},
  {fc::BUDGET_AXIAL_NEGATIVE, "BUDGET_AXIAL_NEGATIVE", Policy::APPROACH_ONLY},
  {fc::BUDGET_STRUCTURAL, "BUDGET_STRUCTURAL", Policy::APPROACH_ONLY},
  {fc::NECK_REMEASURE_MISMATCH, "NECK_REMEASURE_MISMATCH", Policy::APPROACH_ONLY},
  {fc::SWING_TOO_LARGE, "SWING_TOO_LARGE", Policy::WAIT},
  {fc::NECK_REMEASURE_PENDING, "NECK_REMEASURE_PENDING", Policy::REMEASURE_NECK},
  {fc::PLAN_NO_IK, "PLAN_NO_IK", Policy::SKIP},
  {fc::PLAN_COLLISION, "PLAN_COLLISION", Policy::SKIP},
  {fc::PLAN_FAILED, "PLAN_FAILED", Policy::RETRY_VIEW},
  {fc::PLAN_CARTESIAN_INCOMPLETE, "PLAN_CARTESIAN_INCOMPLETE", Policy::SKIP},
  {fc::EXEC_FAILED, "EXEC_FAILED", Policy::RECOVER},
  {fc::EXEC_TIMEOUT, "EXEC_TIMEOUT", Policy::RECOVER},
  {fc::CONTACT_ABORT, "CONTACT_ABORT", Policy::SKIP},
  {fc::PREGRASP_RESIDUAL, "PREGRASP_RESIDUAL", Policy::RETRY_VIEW},
  {fc::RETREAT_FAILED, "RETREAT_FAILED", Policy::RECOVER},
  {fc::TARGET_TIMEOUT, "TARGET_TIMEOUT", Policy::SKIP},
  {fc::TOOL_NOT_OPEN, "TOOL_NOT_OPEN", Policy::RECOVER},
  {fc::TOOL_COMMAND_FAILED, "TOOL_COMMAND_FAILED", Policy::RECOVER},
  {fc::TOOL_FEEDBACK_TIMEOUT, "TOOL_FEEDBACK_TIMEOUT", Policy::RECOVER},
  {fc::CUT_NOT_CONFIRMED, "CUT_NOT_CONFIRMED", Policy::RETRY_VIEW},
  {fc::TOOL_FAULT, "TOOL_FAULT", Policy::RECOVER},
  {fc::TOOL_NOT_FEASIBLE, "TOOL_NOT_FEASIBLE", Policy::SKIP_TOOL},
  {fc::SAFETY_GATE_CLOSED, "SAFETY_GATE_CLOSED", Policy::STOP_BATCH},
  {fc::ROBOT_NOT_READY, "ROBOT_NOT_READY", Policy::STOP_BATCH},
  {fc::CANCELED, "CANCELED", Policy::STOP_BATCH},
  {fc::RECOVERY_REQUIRED, "RECOVERY_REQUIRED", Policy::RECOVER},
  {fc::ENVIRONMENT_UNSAFE, "ENVIRONMENT_UNSAFE", Policy::WAIT},
  {fc::DEPENDENCY_UNAVAILABLE, "DEPENDENCY_UNAVAILABLE", Policy::STOP_BATCH},
};

const CodeRow * find_row(uint32_t code)
{
  for (const auto & row : kCodeTable) {
    if (row.code == code) {
      return &row;
    }
  }
  return nullptr;
}

}  // namespace

Policy policy_for(uint32_t code)
{
  if (const auto * row = find_row(code)) {
    return row->policy;
  }
  // Unknown codes fall back by group; arm/tool groups assume the hardware state is unknown.
  const uint32_t group = code / 10;
  if (group == 4 || group == 5) {
    return Policy::RECOVER;
  }
  if (group == 6) {
    return Policy::STOP_BATCH;
  }
  return Policy::SKIP;
}

const char * policy_name(Policy policy)
{
  switch (policy) {
    case Policy::NONE: return "NONE";
    case Policy::RETRY_VIEW: return "RETRY_VIEW";
    case Policy::SKIP: return "SKIP";
    case Policy::SKIP_TOOL: return "SKIP_TOOL";
    case Policy::APPROACH_ONLY: return "APPROACH_ONLY";
    case Policy::RECOVER: return "RECOVER";
    case Policy::STOP_BATCH: return "STOP_BATCH";
    case Policy::WAIT: return "WAIT";
    case Policy::REMEASURE_NECK: return "REMEASURE_NECK";
  }
  return "UNKNOWN";
}

std::string failure_name(uint32_t code)
{
  if (const auto * row = find_row(code)) {
    return row->name;
  }
  return "UNKNOWN_" + std::to_string(code);
}

bool is_known_code(uint32_t code)
{
  return find_row(code) != nullptr;
}

std::string rework_kind(uint32_t code, Policy policy)
{
  if (code == fc::CANCELED) {
    return "canceled";
  }
  if (code == fc::TARGET_TIMEOUT) {
    return "timeout";
  }
  if (code == fc::DEPENDENCY_UNAVAILABLE) {
    return "infrastructure";
  }
  switch (policy) {
    case Policy::REMEASURE_NECK: return "neck_remeasure";
    case Policy::SKIP_TOOL: return "tool";
    case Policy::APPROACH_ONLY: return "approach_only";
    case Policy::RECOVER: return "recovery";
    case Policy::STOP_BATCH: return "safety";
    case Policy::WAIT: return code == fc::SWING_TOO_LARGE ? "swing" : "environment";
    default: break;
  }
  switch (code / 10) {
    case 1: return "perception";
    case 2: return "model";
    case 3: return code == fc::PLAN_FAILED ? "planning" : "unreachable";
    case 4: return "contact_failed";
    case 5: return "tool_fault";
    default: return "other";
  }
}

std::string validate(const BatchLimits & limits)
{
  std::ostringstream err;
  if (!std::isfinite(limits.target_harvest_ratio) || limits.target_harvest_ratio < 0.0 ||
    limits.target_harvest_ratio > 1.0)
  {
    err << "target_harvest_ratio must be in [0, 1], got " << limits.target_harvest_ratio;
  } else if (!std::isfinite(limits.per_target_timeout_s) || limits.per_target_timeout_s < 0.0) {
    err << "per_target_timeout_s must be >= 0, got " << limits.per_target_timeout_s;
  } else if (limits.empty_survey_limit < 1) {
    err << "empty_survey_limit must be >= 1";
  }
  return err.str();
}

Gate evaluate_gate(const BatchLimits & limits, const Counts & counts)
{
  if (limits.max_targets > 0 && counts.attempted + counts.plan_only >= limits.max_targets) {
    return Gate::MAX_TARGETS;
  }
  if (limits.target_harvest_ratio > 0.0 && counts.discovered > 0) {
    const double needed = std::ceil(limits.target_harvest_ratio * counts.discovered - 1e-9);
    if (static_cast<double>(counts.succeeded) >= needed) {
      return Gate::RATIO_REACHED;
    }
  }
  return Gate::CONTINUE;
}

const char * gate_reason(Gate gate)
{
  switch (gate) {
    case Gate::CONTINUE: return "";
    case Gate::MAX_TARGETS: return "max_targets";
    case Gate::RATIO_REACHED: return "ratio_reached";
  }
  return "";
}

bool empty_limit_reached(uint32_t consecutive_empty_rounds, const BatchLimits & limits)
{
  return consecutive_empty_rounds >= std::max<uint32_t>(1, limits.empty_survey_limit);
}

void TargetDeadline::start(double now_s, double timeout_s)
{
  armed_ = timeout_s > 0.0;
  started_s_ = now_s;
  timeout_s_ = timeout_s;
}

void TargetDeadline::clear()
{
  armed_ = false;
}

bool TargetDeadline::exceeded(double now_s) const
{
  return armed_ && now_s - started_s_ >= timeout_s_;
}

double TargetDeadline::remaining_s(double now_s) const
{
  if (!armed_) {
    return std::numeric_limits<double>::infinity();
  }
  return timeout_s_ - (now_s - started_s_);
}

}  // namespace peach2_task::core
