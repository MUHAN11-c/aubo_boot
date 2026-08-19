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
// 执行前安全门纯核实现：判定语义与原节点内联 safetyReady/cycleTargetReady 一致。
#include "peach_manipulation_skills/safety_gate.hpp"

#include <string>
#include <utility>

namespace peach_manipulation_skills
{
SafetyGate::SafetyGate(SafetyGateConfig config, std::function<double()> clock_s)
: config_(config), clock_s_(std::move(clock_s))
{
}

bool SafetyGate::robotReady(const RobotStatusSample & sample, std::string & reason) const
{
  if (!config_.require_robot_status) {
    return true;
  }
  if (!sample.received) {
    reason = "robot_status_missing";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.robot_status_max_age_s) {
    reason = "robot_status_stale";
    return false;
  }
  if (sample.e_stopped || sample.in_error || !sample.drives_powered ||
    !sample.motion_possible)
  {
    reason = "robot_status_not_motion_ready";
    return false;
  }
  return true;
}

bool SafetyGate::targetReady(
  const TargetGateSample & sample, const std::string & target_id,
  std::string & reason) const
{
  if (sample.id != target_id) {
    reason = "selected_target_changed";
    return false;
  }
  if (!sample.valid) {
    reason = "selected_target_not_observed";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.target_observation_max_age_s) {
    reason = "selected_target_stale";
    return false;
  }
  return true;
}

}  // namespace peach_manipulation_skills
