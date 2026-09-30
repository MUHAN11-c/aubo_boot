#include "peach2_end_effector/tool_state_machine.hpp"

#include <string>

namespace peach2_end_effector
{

ToolStateMachine::ToolStateMachine(ToolStateMachineConfig config)
: config_(config)
{
}

CommandDecision ToolStateMachine::begin_command(bool close, double /*now_s*/)
{
  if (writing_) {
    return {false, "command_in_flight"};
  }
  if (state_ == ToolState::FAULT) {
    if (close) {
      return {false, "fault_locked:" + fault_reason_};
    }
    pending_close_ = false;
    writing_ = true;
    feedback_at_command_ = feedback_;
    return {true, "open_in_fault"};
  }
  if (close) {
    if (state_ != ToolState::OPEN_CONFIRMED) {
      return {false, std::string("not_open_confirmed:") + to_string(state_)};
    }
    if (feedback_.value_or(false)) {
      enter_fault("feedback_stuck_closed");
      return {false, "feedback_stuck_closed"};
    }
    state_ = ToolState::CLOSING;
  } else {
    state_ = ToolState::OPENING;
  }
  pending_close_ = close;
  writing_ = true;
  feedback_at_command_ = feedback_;
  return {true, close ? "closing" : "opening"};
}

void ToolStateMachine::end_command(bool write_ok, double now_s)
{
  if (!writing_) {
    return;
  }
  writing_ = false;
  if (!write_ok) {
    // Output level unchanged (or unknown); keep the last acknowledged command.
    pending_ = false;
    enter_fault("command_write_failed");
    return;
  }
  command_closed_ = pending_close_;
  command_s_ = now_s;
  pending_ = (state_ != ToolState::FAULT);
}

void ToolStateMachine::feedback(bool closed, double now_s)
{
  feedback_ = closed;
  if (writing_ || state_ == ToolState::FAULT) {
    return;
  }
  if (pending_) {
    if (now_s < command_s_) {
      return;
    }
    const double delay = now_s - command_s_;
    if (state_ == ToolState::CLOSING && closed) {
      if (delay < config_.min_actuation_s) {
        suspected_loopback_ = true;
        enter_fault("suspected_loopback");
        return;
      }
      state_ = ToolState::CLOSED_CONFIRMED;
      pending_ = false;
    } else if (state_ == ToolState::OPENING && !closed) {
      if (feedback_at_command_.value_or(false) && delay < config_.min_actuation_s) {
        suspected_loopback_ = true;
        enter_fault("suspected_loopback");
        return;
      }
      state_ = ToolState::OPEN_CONFIRMED;
      pending_ = false;
    }
    return;
  }
  if (state_ == ToolState::OPEN_CONFIRMED && closed) {
    enter_fault("unexpected_feedback_closed");
  } else if (state_ == ToolState::CLOSED_CONFIRMED && !closed) {
    enter_fault("unexpected_feedback_open");
  }
}

void ToolStateMachine::update(double now_s)
{
  if (state_ == ToolState::FAULT || writing_) {
    return;
  }
  if (pending_ && now_s - command_s_ > config_.feedback_timeout_s) {
    enter_fault(pending_close_ ? "close_feedback_timeout" : "open_feedback_timeout");
    return;
  }
  if (config_.max_close_energized_s > 0.0 && command_closed_ &&
    now_s - command_s_ > config_.max_close_energized_s)
  {
    enter_fault("close_energized_too_long");
  }
}

void ToolStateMachine::fault(const std::string & reason)
{
  enter_fault(reason);
}

void ToolStateMachine::reset_by_ack()
{
  state_ = ToolState::UNKNOWN;
  pending_ = false;
  writing_ = false;
  suspected_loopback_ = false;
  fault_reason_.clear();
}

void ToolStateMachine::mark_unknown()
{
  if (state_ == ToolState::FAULT) {
    return;
  }
  state_ = ToolState::UNKNOWN;
  pending_ = false;
}

void ToolStateMachine::enter_fault(const std::string & reason)
{
  if (state_ != ToolState::FAULT) {
    fault_reason_ = reason;
  }
  state_ = ToolState::FAULT;
  pending_ = false;
}

}  // namespace peach2_end_effector
