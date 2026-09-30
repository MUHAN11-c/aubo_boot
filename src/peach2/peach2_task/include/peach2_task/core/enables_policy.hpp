// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>
#include <vector>

/// Operator enables (execution / grasp / tool), their dependency chain and the per-intent
/// requirement. The task node is the only publisher of /peach/enables; manipulation enforces.
namespace peach2_task::core
{

/// Mirror of RunBatch.INTENT_*.
enum class Intent : uint8_t { SURVEY_ONLY = 0, PREGRASP_ONLY = 1, FULL = 2 };

bool intent_from_uint(uint32_t value, Intent * out);
const char * intent_name(Intent intent);
/// Accepts "SURVEY_ONLY" / "PREGRASP_ONLY" / "FULL".
bool intent_from_name(const std::string & name, Intent * out);

struct Enables
{
  bool execution = false;
  bool grasp = false;
  bool tool = false;

  bool operator==(const Enables & other) const
  {
    return execution == other.execution && grasp == other.grasp && tool == other.tool;
  }
};

/// Dependency chain tool => grasp => execution. Empty string when valid.
std::string validate_chain(const Enables & enables);

/// Every intent moves the arm (Survey = MoveTo photo pose), so execution is always needed;
/// only FULL touches the bag with the sleeve and the blade.
Enables required_for(Intent intent);

/// Names of enables that `required_for(intent)` needs but `enables` lacks.
std::vector<std::string> missing_for(const Enables & enables, Intent intent);

/// RunBatch admission: empty when a batch of `intent` may start with `enables`.
/// Stop-and-go perception has to move the arm to survey, so no intent runs without execution
/// (plan-only goes through CheckReachability / HarvestTarget directly, not a batch). FULL also
/// needs grasp and tool: manipulation refuses MODE_FULL without both before moving, so admitting
/// would only defer the same rejection past the survey.
std::string admission_error(const Enables & enables, Intent intent);

}  // namespace peach2_task::core
