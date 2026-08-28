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
#ifndef PEACH_MANIPULATION_SKILLS__TOOL_ACTUATOR_HPP_
#define PEACH_MANIPULATION_SKILLS__TOOL_ACTUATOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "peach_manipulation_skills/tool_profile.hpp"

namespace peach_manipulation_skills
{

enum class ToolActuatorState : uint8_t
{
  SAFE = 0,
  ARMED = 1,
  CUT_COMMAND_SENT = 2,
  CUT_FEEDBACK_CONFIRMED = 3,
  SAFE_OR_HOLDING = 4
};

struct ToolCommandContext
{
  std::string run_id;
  std::string target_id;
  std::string model_revision;
  std::string contact_transaction_id;
};

class ToolActuator
{
public:
  using SendIo = std::function<bool(int fun, int pin, double state, std::string & reason)>;

  explicit ToolActuator(ToolProfile profile = defaultHollowCylinderV1());

  void setSendIo(SendIo send_io);
  bool arm(const ToolCommandContext & ctx, std::string & reason);
  // SetIO ACK 只产生 CUT_COMMAND_ACCEPTED，不得自称切断确认。
  bool sendCut(std::string & reason);
  bool confirmFeedback(bool hardware_ok, std::string & reason);
  void resetSafe();
  bool sameTransaction(const std::string & transaction_id) const;
  bool cutAlreadyCommanded() const;

  ToolActuatorState state() const {return state_;}
  const ToolCommandContext & context() const {return ctx_;}
  const ToolProfile & profile() const {return profile_;}

private:
  ToolProfile profile_;
  ToolActuatorState state_{ToolActuatorState::SAFE};
  ToolCommandContext ctx_;
  SendIo send_io_;
  bool cut_commanded_{false};
};

}  // namespace peach_manipulation_skills

#endif  // PEACH_MANIPULATION_SKILLS__TOOL_ACTUATOR_HPP_
