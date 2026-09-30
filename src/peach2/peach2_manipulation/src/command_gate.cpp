#include "peach2_manipulation/command_gate.hpp"

#include <string>

#include "peach2_end_effector/failure_codes.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;

namespace
{

GateVerdict closed(uint32_t code, const std::string & reason) {return {false, code, reason};}
GateVerdict opened() {return {true, failure::NONE, "open"};}

}  // namespace

const char * to_string(GateStage stage)
{
  switch (stage) {
    case GateStage::TRANSIT: return "TRANSIT";
    case GateStage::APPROACH: return "APPROACH";
    case GateStage::CONTACT: return "CONTACT";
    case GateStage::TOOL: return "TOOL";
    case GateStage::RETREAT: return "RETREAT";
    case GateStage::RELEASE: return "RELEASE";
    case GateStage::TOOL_SAFE: return "TOOL_SAFE";
  }
  return "INVALID";
}

CommandGate::CommandGate(GateConfig config)
: config_(config)
{
}

EffectiveEnables CommandGate::enables(double now_s) const
{
  EffectiveEnables e;
  if (!enables_ || now_s - enables_->received_s > config_.enables_timeout_s ||
    now_s < enables_->received_s - 1.0)
  {
    return e;
  }
  e.heartbeat_ok = true;
  e.execution = enables_->execution;
  e.grasp = e.execution && enables_->grasp;
  e.tool = e.grasp && enables_->tool;
  return e;
}

GateVerdict CommandGate::robot_ready(double now_s, bool new_trajectory) const
{
  if (!config_.require_robot_status) {
    return opened();
  }
  if (!robot_) {
    return closed(failure::ROBOT_NOT_READY, "robot_status_missing");
  }
  if (now_s - robot_->received_s > config_.robot_status_max_age_s) {
    return closed(failure::ROBOT_NOT_READY, "robot_status_stale");
  }
  if (robot_->e_stopped != 0) {
    return closed(failure::ROBOT_NOT_READY, "e_stopped");
  }
  if (robot_->drives_powered != 1) {
    return closed(failure::ROBOT_NOT_READY, "drives_unpowered");
  }
  if (robot_->in_error != 0) {
    return closed(
      failure::ROBOT_NOT_READY, "robot_in_error:" + std::to_string(robot_->error_code));
  }
  if (new_trajectory && robot_->motion_possible != 1) {
    return closed(failure::ROBOT_NOT_READY, "motion_not_possible");
  }
  return opened();
}

GateVerdict CommandGate::check(GateStage stage, double now_s, bool new_trajectory) const
{
  if (!active_) {
    return closed(failure::SAFETY_GATE_CLOSED, "not_active");
  }
  if (stage == GateStage::TOOL_SAFE) {
    // Opening the blade reduces stored energy; allowed after cancel / disable as long as the
    // controller is alive and not in e-stop. Drives may be off (protective stop).
    if (!config_.require_robot_status) {
      return opened();
    }
    if (!robot_ || now_s - robot_->received_s > config_.robot_status_max_age_s) {
      return closed(failure::ROBOT_NOT_READY, "robot_status_stale");
    }
    if (robot_->e_stopped != 0) {
      return closed(failure::ROBOT_NOT_READY, "e_stopped");
    }
    return opened();
  }
  if (cancel_) {
    return closed(failure::CANCELED, "canceled");
  }
  const GateVerdict robot = robot_ready(now_s, new_trajectory);
  if (!robot.open) {
    return robot;
  }
  const EffectiveEnables e = enables(now_s);
  if (!e.heartbeat_ok) {
    return closed(failure::SAFETY_GATE_CLOSED, "enables_heartbeat_lost");
  }
  switch (stage) {
    case GateStage::TRANSIT:
    case GateStage::APPROACH:
    case GateStage::RETREAT:
      if (!e.execution) {
        return closed(failure::SAFETY_GATE_CLOSED, "execution_disabled");
      }
      break;
    case GateStage::CONTACT:
      if (!e.grasp) {
        return closed(
          failure::SAFETY_GATE_CLOSED, e.execution ? "grasp_disabled" : "execution_disabled");
      }
      break;
    case GateStage::TOOL:
    case GateStage::RELEASE:
      if (!e.tool) {
        return closed(
          failure::SAFETY_GATE_CLOSED,
          !e.execution ? "execution_disabled" : (!e.grasp ? "grasp_disabled" : "tool_disabled"));
      }
      break;
    case GateStage::TOOL_SAFE:
      break;
  }
  return opened();
}

}  // namespace peach2_manipulation
