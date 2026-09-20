// 功能：刀具 GPIO 状态机。SetIO ACK 只表示柜侧接受命令，不等于切断确认。
#include "peach_arm/tool_actuator.hpp"

namespace peach_arm
{

void ToolActuator::setSendIo(SendIo send_io)
{
  send_io_ = std::move(send_io);
}

bool ToolActuator::arm(const ToolCommandContext & ctx, std::string & reason)
{
  if (cut_commanded_ && ctx.contact_transaction_id == ctx_.contact_transaction_id &&
    !ctx.contact_transaction_id.empty())
  {
    reason = "cut_already_commanded_this_transaction";
    return false;
  }
  ctx_ = ctx;
  state_ = ToolActuatorState::ARMED;
  reason.clear();
  return true;
}

bool ToolActuator::sendCut(std::string & reason)
{
  if (state_ != ToolActuatorState::ARMED) {
    reason = "tool_not_armed";
    return false;
  }
  if (cut_commanded_) {
    reason = "cut_already_commanded_this_transaction";
    return false;
  }
  if (!send_io_) {
    reason = "tool_io_not_wired";
    return false;
  }
  if (!send_io_(reason)) {
    state_ = ToolActuatorState::UNKNOWN;
    return false;
  }
  cut_commanded_ = true;
  state_ = ToolActuatorState::CUT_COMMAND_SENT;
  reason = "CUT_COMMAND_ACCEPTED";
  return true;
}

bool ToolActuator::confirmFeedback(bool hardware_ok, std::string & reason)
{
  if (state_ != ToolActuatorState::CUT_COMMAND_SENT) {
    reason = "cut_command_not_sent";
    return false;
  }
  if (!hardware_ok) {
    reason = "cut_feedback_unavailable";
    return false;
  }
  state_ = ToolActuatorState::CUT_FEEDBACK_CONFIRMED;
  reason = "CUT_CONFIRMED";
  return true;
}

void ToolActuator::resetSafe()
{
  state_ = ToolActuatorState::SAFE;
  cut_commanded_ = false;
  ctx_ = ToolCommandContext{};
}

void ToolActuator::markUnknown()
{
  state_ = ToolActuatorState::UNKNOWN;
}

bool ToolActuator::sameTransaction(const std::string & transaction_id) const
{
  return !transaction_id.empty() && transaction_id == ctx_.contact_transaction_id;
}

bool ToolActuator::cutAlreadyCommanded() const
{
  return cut_commanded_;
}

}  // namespace peach_arm
