// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>

/// Batch limits, per-target deadline and the FailureCode -> handling policy table.
/// Zero ROS: failure codes are mirrored here and checked against the IDL by test_msg_mirror.
namespace peach2_task::core
{

/// Mirror of peach2_interfaces/msg/FailureCode.
namespace fc
{
constexpr uint32_t NONE = 0;
constexpr uint32_t PERCEPTION_NO_TARGET = 10;
constexpr uint32_t PERCEPTION_EXACT_TF_MISSING = 11;
constexpr uint32_t PERCEPTION_LOW_QUALITY = 12;
constexpr uint32_t PERCEPTION_OUT_OF_SCOPE = 13;
constexpr uint32_t MODEL_NOT_CONVERGED = 20;
constexpr uint32_t MODEL_STALE = 21;
constexpr uint32_t MODEL_EXPIRED = 22;
constexpr uint32_t BUDGET_RADIAL_NEGATIVE = 23;
constexpr uint32_t BUDGET_AXIAL_NEGATIVE = 24;
constexpr uint32_t BUDGET_STRUCTURAL = 25;
constexpr uint32_t NECK_REMEASURE_MISMATCH = 26;
constexpr uint32_t SWING_TOO_LARGE = 27;
constexpr uint32_t NECK_REMEASURE_PENDING = 28;
constexpr uint32_t PLAN_NO_IK = 30;
constexpr uint32_t PLAN_COLLISION = 31;
constexpr uint32_t PLAN_FAILED = 32;
constexpr uint32_t PLAN_CARTESIAN_INCOMPLETE = 33;
constexpr uint32_t EXEC_FAILED = 40;
constexpr uint32_t EXEC_TIMEOUT = 41;
constexpr uint32_t CONTACT_ABORT = 42;
constexpr uint32_t PREGRASP_RESIDUAL = 43;
constexpr uint32_t RETREAT_FAILED = 44;
constexpr uint32_t TARGET_TIMEOUT = 45;
constexpr uint32_t TOOL_NOT_OPEN = 50;
constexpr uint32_t TOOL_COMMAND_FAILED = 51;
constexpr uint32_t TOOL_FEEDBACK_TIMEOUT = 52;
constexpr uint32_t CUT_NOT_CONFIRMED = 53;
constexpr uint32_t TOOL_FAULT = 54;
constexpr uint32_t TOOL_NOT_FEASIBLE = 55;
constexpr uint32_t SAFETY_GATE_CLOSED = 60;
constexpr uint32_t ROBOT_NOT_READY = 61;
constexpr uint32_t CANCELED = 62;
constexpr uint32_t RECOVERY_REQUIRED = 63;
constexpr uint32_t ENVIRONMENT_UNSAFE = 64;
constexpr uint32_t DEPENDENCY_UNAVAILABLE = 65;
}  // namespace fc

/// One handling policy per failure code (FailureCode.msg header: "exactly one policy").
enum class Policy : uint8_t
{
  NONE = 0,       ///< not a failure
  RETRY_VIEW,     ///< re-observe and retry the whole target cycle (bounded by retry_attempts)
  SKIP,           ///< give up this target for this batch
  SKIP_TOOL,      ///< give up with this tool; another tool may work (rework kind "tool")
  APPROACH_ONLY,  ///< approach allowed but no cut; recorded as skipped for FULL
  RECOVER,        ///< arm/tool state unknown: block the whole tree until a human ACK
  STOP_BATCH,     ///< terminate the batch
  WAIT,           ///< wait wait_retry_s, then retry (bounded); give up as skip
  /// FULL only: close-range neck re-measure (ObserveTarget neck_remeasure) + cut decision,
  /// then the manipulation cycle again; once per normal attempt, not counted as an attempt.
  REMEASURE_NECK,
};

Policy policy_for(uint32_t code);
const char * policy_name(Policy policy);
/// Symbolic FailureCode name, "UNKNOWN_<n>" for codes not in the mirror.
std::string failure_name(uint32_t code);
bool is_known_code(uint32_t code);
/// rework.json category for a failed target.
std::string rework_kind(uint32_t code, Policy policy);

struct BatchLimits
{
  uint32_t max_targets = 0;           ///< attempted + plan-only targets; 0 = unlimited
  double target_harvest_ratio = 0.0;  ///< succeeded / discovered; 0 = disabled
  double per_target_timeout_s = 0.0;  ///< [s] observe+decision budget per target; 0 = unlimited
  uint32_t empty_survey_limit = 2;    ///< consecutive surveys with no selectable target
};

/// Empty string when valid, otherwise a human readable reason.
std::string validate(const BatchLimits & limits);

struct Counts
{
  uint32_t discovered = 0;
  uint32_t attempted = 0;  ///< physically attempted (plan-only targets excluded)
  uint32_t succeeded = 0;
  uint32_t skipped = 0;
  uint32_t failed = 0;
  uint32_t plan_only = 0;  ///< HarvestResult.plan_only: planned, nothing moved or actuated
};

enum class Gate : uint8_t { CONTINUE, MAX_TARGETS, RATIO_REACHED };

Gate evaluate_gate(const BatchLimits & limits, const Counts & counts);
const char * gate_reason(Gate gate);
bool empty_limit_reached(uint32_t consecutive_empty_rounds, const BatchLimits & limits);

/// Monotonic per-target budget. Times are caller supplied seconds (steady clock).
class TargetDeadline
{
public:
  void start(double now_s, double timeout_s);
  void clear();
  bool armed() const {return armed_;}
  bool exceeded(double now_s) const;
  double remaining_s(double now_s) const;

private:
  bool armed_ = false;
  double started_s_ = 0.0;
  double timeout_s_ = 0.0;
};

}  // namespace peach2_task::core
