// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "peach_manipulation_skills/tool_actuator.hpp"

namespace peach_manipulation_skills
{

ToolActuator::ToolActuator(ToolProfile profile)
: profile_(std::move(profile))
{
}

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
  if (!send_io_(profile_.io_fun, profile_.io_pin, profile_.close_state, reason)) {
    state_ = ToolActuatorState::SAFE;
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

bool ToolActuator::sameTransaction(const std::string & transaction_id) const
{
  return !transaction_id.empty() && transaction_id == ctx_.contact_transaction_id;
}

bool ToolActuator::cutAlreadyCommanded() const
{
  return cut_commanded_;
}

}  // namespace peach_manipulation_skills
