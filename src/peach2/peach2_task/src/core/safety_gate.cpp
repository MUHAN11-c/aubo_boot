// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/safety_gate.hpp"

#include "peach2_task/core/batch_policy.hpp"

namespace peach2_task::core
{

std::string SafetyVerdict::reason() const
{
  std::string out;
  for (const auto & blocker : blockers) {
    if (!out.empty()) {
      out += ',';
    }
    out += blocker;
  }
  return out;
}

SafetyVerdict evaluate_safety(const SafetyConfig & config, const SafetyInputs & inputs)
{
  SafetyVerdict verdict;
  uint32_t code = fc::NONE;
  auto block = [&](const std::string & token, uint32_t token_code) {
      verdict.blockers.push_back(token);
      if (code == fc::NONE) {
        code = token_code;
      }
    };

  if (config.require_robot_status) {
    const auto & robot = inputs.robot;
    if (!robot.received) {
      block("robot_status_missing", fc::ROBOT_NOT_READY);
    } else {
      if (!(robot.age_s < config.robot_status_max_age_s)) {
        block("robot_status_stale", fc::ROBOT_NOT_READY);
      }
      if (robot.e_stopped != 0) {
        block("e_stopped", fc::ROBOT_NOT_READY);
      }
      if (robot.drives_powered == 0) {
        block("drives_unpowered", fc::ROBOT_NOT_READY);
      }
      if (robot.in_error != 0) {
        block("robot_in_error", fc::ROBOT_NOT_READY);
      }
    }
  }

  if (!validate_chain(inputs.enables).empty()) {
    block("enables_chain_invalid", fc::SAFETY_GATE_CLOSED);
  }
  for (const auto & name : missing_for(inputs.enables, inputs.intent)) {
    block("enable_missing:" + name, fc::SAFETY_GATE_CLOSED);
  }

  if (inputs.intent == Intent::FULL && inputs.tool_fault) {
    block("tool_fault", fc::TOOL_FAULT);
  }

  verdict.ok = verdict.blockers.empty();
  verdict.failure_code = code;
  return verdict;
}

}  // namespace peach2_task::core
