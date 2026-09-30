#pragma once

#include <cstdint>

/// Zero-ROS mirror of peach2_interfaces/msg/FailureCode. Node code static_asserts equality
/// against the generated IDL constants, so a drift fails the build.
namespace peach2_end_effector::failure
{

constexpr uint32_t NONE = 0;

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

}  // namespace peach2_end_effector::failure
