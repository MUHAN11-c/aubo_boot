// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "peach2_task/core/enables_policy.hpp"

/// CheckSafety verdict. Application guard only: the hardware e-stop is on the cabinet /
/// teach pendant and never goes through ROS; this gate only stops *issuing* new work.
namespace peach2_task::core
{

/// aubo_msgs/RobotStatus has no header: age is measured at receipt time by the caller.
struct RobotStatusSample
{
  bool received = false;
  double age_s = std::numeric_limits<double>::infinity();
  int8_t drives_powered = 0;
  int8_t e_stopped = 0;
  int8_t in_error = 0;
};

struct SafetyConfig
{
  bool require_robot_status = true;     ///< false only for mock hardware (no io controller)
  double robot_status_max_age_s = 0.3;  ///< [s]
};

struct SafetyInputs
{
  RobotStatusSample robot;
  Enables enables;
  Intent intent = Intent::PREGRASP_ONLY;
  bool tool_fault = false;  ///< ToolState.FAULT on the latched tool state
};

struct SafetyVerdict
{
  bool ok = true;
  uint32_t failure_code = 0;          ///< FailureCode of the highest priority blocker
  std::vector<std::string> blockers;  ///< stable machine-readable tokens
  std::string reason() const;
};

/// motion_possible is deliberately not checked: it reads 0 while a trajectory streams
/// (continuous condition = drives_powered && !e_stopped && !in_error && fresh).
SafetyVerdict evaluate_safety(const SafetyConfig & config, const SafetyInputs & inputs);

}  // namespace peach2_task::core
