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

}  // namespace peach2_task::core
