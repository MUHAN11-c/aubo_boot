#include "peach2_manipulation/conversions.hpp"

#include <cmath>
#include <limits>

#include "peach2_end_effector/failure_codes.hpp"
#include "peach2_interfaces/msg/failure_code.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;
using FC = peach2_interfaces::msg::FailureCode;
using ToolStateMsg = peach2_interfaces::msg::ToolState;
using HarvestResultMsg = peach2_interfaces::msg::HarvestResult;

// The end-effector package mirrors FailureCode without depending on the IDL; keep them equal.
static_assert(failure::NONE == FC::NONE);
static_assert(failure::MODEL_NOT_CONVERGED == FC::MODEL_NOT_CONVERGED);
static_assert(failure::MODEL_STALE == FC::MODEL_STALE);
static_assert(failure::MODEL_EXPIRED == FC::MODEL_EXPIRED);
static_assert(failure::BUDGET_RADIAL_NEGATIVE == FC::BUDGET_RADIAL_NEGATIVE);
static_assert(failure::BUDGET_AXIAL_NEGATIVE == FC::BUDGET_AXIAL_NEGATIVE);
static_assert(failure::BUDGET_STRUCTURAL == FC::BUDGET_STRUCTURAL);
static_assert(failure::NECK_REMEASURE_MISMATCH == FC::NECK_REMEASURE_MISMATCH);
static_assert(failure::SWING_TOO_LARGE == FC::SWING_TOO_LARGE);
static_assert(failure::NECK_REMEASURE_PENDING == FC::NECK_REMEASURE_PENDING);
static_assert(failure::PLAN_NO_IK == FC::PLAN_NO_IK);
static_assert(failure::PLAN_COLLISION == FC::PLAN_COLLISION);
static_assert(failure::PLAN_FAILED == FC::PLAN_FAILED);
static_assert(failure::PLAN_CARTESIAN_INCOMPLETE == FC::PLAN_CARTESIAN_INCOMPLETE);
static_assert(failure::EXEC_FAILED == FC::EXEC_FAILED);
static_assert(failure::EXEC_TIMEOUT == FC::EXEC_TIMEOUT);
static_assert(failure::CONTACT_ABORT == FC::CONTACT_ABORT);
static_assert(failure::PREGRASP_RESIDUAL == FC::PREGRASP_RESIDUAL);
static_assert(failure::RETREAT_FAILED == FC::RETREAT_FAILED);
static_assert(failure::TARGET_TIMEOUT == FC::TARGET_TIMEOUT);
static_assert(failure::TOOL_NOT_OPEN == FC::TOOL_NOT_OPEN);
static_assert(failure::TOOL_COMMAND_FAILED == FC::TOOL_COMMAND_FAILED);
static_assert(failure::TOOL_FEEDBACK_TIMEOUT == FC::TOOL_FEEDBACK_TIMEOUT);
static_assert(failure::CUT_NOT_CONFIRMED == FC::CUT_NOT_CONFIRMED);
static_assert(failure::TOOL_FAULT == FC::TOOL_FAULT);
static_assert(failure::TOOL_NOT_FEASIBLE == FC::TOOL_NOT_FEASIBLE);
static_assert(failure::SAFETY_GATE_CLOSED == FC::SAFETY_GATE_CLOSED);
static_assert(failure::ROBOT_NOT_READY == FC::ROBOT_NOT_READY);
static_assert(failure::CANCELED == FC::CANCELED);
static_assert(failure::RECOVERY_REQUIRED == FC::RECOVERY_REQUIRED);
static_assert(failure::ENVIRONMENT_UNSAFE == FC::ENVIRONMENT_UNSAFE);
static_assert(failure::DEPENDENCY_UNAVAILABLE == FC::DEPENDENCY_UNAVAILABLE);

using peach2_end_effector::ToolState;
static_assert(static_cast<uint8_t>(ToolState::UNKNOWN) == ToolStateMsg::UNKNOWN);
static_assert(static_cast<uint8_t>(ToolState::OPEN_CONFIRMED) == ToolStateMsg::OPEN_CONFIRMED);
static_assert(static_cast<uint8_t>(ToolState::CLOSING) == ToolStateMsg::CLOSING);
static_assert(static_cast<uint8_t>(ToolState::CLOSED_CONFIRMED) == ToolStateMsg::CLOSED_CONFIRMED);
static_assert(static_cast<uint8_t>(ToolState::OPENING) == ToolStateMsg::OPENING);
static_assert(static_cast<uint8_t>(ToolState::FAULT) == ToolStateMsg::FAULT);

static_assert(static_cast<uint8_t>(Outcome::SUCCEEDED) == HarvestResultMsg::OUTCOME_SUCCEEDED);
static_assert(static_cast<uint8_t>(Outcome::SKIPPED) == HarvestResultMsg::OUTCOME_SKIPPED);
static_assert(static_cast<uint8_t>(Outcome::FAILED) == HarvestResultMsg::OUTCOME_FAILED);
static_assert(static_cast<uint8_t>(Outcome::CANCELED) == HarvestResultMsg::OUTCOME_CANCELED);
static_assert(static_cast<uint8_t>(Reached::PREGRASP) == HarvestResultMsg::REACHED_PREGRASP);
static_assert(static_cast<uint8_t>(Reached::INSERTED) == HarvestResultMsg::REACHED_INSERTED);
static_assert(
  static_cast<uint8_t>(Reached::CUT_CONFIRMED) == HarvestResultMsg::REACHED_CUT_CONFIRMED);
static_assert(static_cast<uint8_t>(Reached::RETREATED) == HarvestResultMsg::REACHED_RETREATED);
static_assert(static_cast<uint8_t>(Reached::RELEASED) == HarvestResultMsg::REACHED_RELEASED);

Eigen::Isometry3d pose_from_msg(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return out;
}

DecisionView decision_from_msg(const peach2_interfaces::msg::GraspDecision & msg)
{
  DecisionView d;
  d.target_id = msg.target_id;
  d.tool_id = msg.tool_id;
  d.revision = msg.model_revision;
  d.valid_until_s = static_cast<double>(msg.valid_until.sec) +
    static_cast<double>(msg.valid_until.nanosec) * 1e-9;
  d.approach_allowed = msg.approach_allowed;
  d.sleeve_allowed = msg.sleeve_allowed;
  d.cut_allowed = msg.cut_allowed;
  d.radial_margin_m = msg.radial_margin_m;
  d.axial_margin_m = msg.axial_margin_m;
  d.pregrasp_tcp = pose_from_msg(msg.pregrasp_tcp);
  d.blade_target = Eigen::Vector3d(msg.blade_target.x, msg.blade_target.y, msg.blade_target.z);
  d.insert_travel_m = msg.insert_travel_m;
  d.failure_code = msg.failure_code;
  d.reason = msg.reason;
  return d;
}

std::optional<peach2_end_effector::TargetGeometry> geometry_from_msg(
  const peach2_interfaces::msg::TargetModel & msg)
{
  if (!msg.bottom.valid || !msg.neck.valid) {
    return std::nullopt;
  }
  Eigen::Vector3d axis(msg.axis.x, msg.axis.y, msg.axis.z);
  if (!axis.allFinite() || axis.norm() < 1e-6) {
    return std::nullopt;
  }
  if (!std::isfinite(msg.d95_m) || !std::isfinite(msg.length_m) || msg.d95_m <= 0.0) {
    return std::nullopt;
  }
  peach2_end_effector::TargetGeometry g;
  g.target_id = msg.target_id;
  g.bottom = Eigen::Vector3d(
    msg.bottom.position.x, msg.bottom.position.y, msg.bottom.position.z);
  g.neck = Eigen::Vector3d(msg.neck.position.x, msg.neck.position.y, msg.neck.position.z);
  g.axis = axis.normalized();
  g.d95_m = msg.d95_m;
  g.length_m = msg.length_m;
  g.sigma_lateral95_m = msg.sigma_lateral95_m;
  g.sigma_axial95_m = msg.sigma_axial95_m;
  if (!g.bottom.allFinite() || !g.neck.allFinite()) {
    return std::nullopt;
  }
  if (msg.branch_direction_known) {
    const Eigen::Vector3d branch(
      msg.branch_direction.x, msg.branch_direction.y, msg.branch_direction.z);
    if (branch.allFinite() && branch.norm() > 1e-6) {
      g.branch_direction = branch.normalized();
    }
  }
  return g;
}

peach2_interfaces::msg::HarvestResult result_to_msg(const CycleResult & result)
{
  HarvestResultMsg msg;
  msg.target_id = result.target_id;
  msg.tool_id = result.tool_id;
  msg.outcome = static_cast<uint8_t>(result.outcome);
  msg.reached = static_cast<uint8_t>(result.reached);
  msg.failure_code = result.failure_code;
  msg.reason = result.reason;
  msg.recovery_required = result.recovery_required;
  msg.plan_only = result.plan_only;
  msg.cycle_time_s = result.cycle_time_s;
  msg.stage_names = result.stage_names;
  msg.stage_times_s = result.stage_times_s;
  msg.radial_margin_m = result.radial_margin_m;
  msg.axial_margin_m = result.axial_margin_m;
  return msg;
}

peach2_interfaces::msg::ToolState tool_state_to_msg(const peach2_end_effector::ToolStatus & s)
{
  ToolStateMsg msg;
  msg.tool_id = s.tool_id;
  msg.state = static_cast<uint8_t>(s.state);
  msg.command_closed = s.command_closed;
  if (!s.feedback_closed) {
    msg.feedback = ToolStateMsg::FEEDBACK_UNKNOWN;
  } else {
    msg.feedback = *s.feedback_closed ? ToolStateMsg::FEEDBACK_CLOSED : ToolStateMsg::FEEDBACK_OPEN;
  }
  msg.suspected_loopback = s.suspected_loopback;
  msg.actuator_current_a = s.current_a ?
    static_cast<float>(*s.current_a) : std::numeric_limits<float>::quiet_NaN();
  msg.fault_reason = s.fault_reason;
  return msg;
}

RobotStatusSample robot_status_from_msg(
  const aubo_msgs::msg::RobotStatus & msg, double received_s)
{
  RobotStatusSample s;
  s.mode = msg.mode;
  s.e_stopped = msg.e_stopped;
  s.drives_powered = msg.drives_powered;
  s.motion_possible = msg.motion_possible;
  s.in_motion = msg.in_motion;
  s.in_error = msg.in_error;
  s.error_code = msg.error_code;
  s.received_s = received_s;
  return s;
}

EnablesSample enables_from_msg(const peach2_interfaces::msg::Enables & msg, double received_s)
{
  EnablesSample s;
  s.seq = msg.seq;
  s.execution = msg.execution;
  s.grasp = msg.grasp;
  s.tool = msg.tool;
  s.received_s = received_s;
  return s;
}

}  // namespace peach2_manipulation
