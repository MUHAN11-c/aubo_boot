// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include "peach2_task/core/enables_policy.hpp"

namespace peach2_task::core
{

bool intent_from_uint(uint32_t value, Intent * out)
{
  switch (value) {
    case 0: *out = Intent::SURVEY_ONLY; return true;
    case 1: *out = Intent::PREGRASP_ONLY; return true;
    case 2: *out = Intent::FULL; return true;
    default: return false;
  }
}

const char * intent_name(Intent intent)
{
  switch (intent) {
    case Intent::SURVEY_ONLY: return "SURVEY_ONLY";
    case Intent::PREGRASP_ONLY: return "PREGRASP_ONLY";
    case Intent::FULL: return "FULL";
  }
  return "UNKNOWN";
}

bool intent_from_name(const std::string & name, Intent * out)
{
  for (const Intent intent : {Intent::SURVEY_ONLY, Intent::PREGRASP_ONLY, Intent::FULL}) {
    if (name == intent_name(intent)) {
      *out = intent;
      return true;
    }
  }
  return false;
}

std::string validate_chain(const Enables & enables)
{
  if (enables.tool && !enables.grasp) {
    return "tool requires grasp (chain tool => grasp => execution)";
  }
  if (enables.grasp && !enables.execution) {
    return "grasp requires execution (chain tool => grasp => execution)";
  }
  return "";
}

Enables required_for(Intent intent)
{
  Enables need;
  need.execution = true;
  if (intent == Intent::FULL) {
    need.grasp = true;
    need.tool = true;
  }
  return need;
}

std::vector<std::string> missing_for(const Enables & enables, Intent intent)
{
  const Enables need = required_for(intent);
  std::vector<std::string> missing;
  if (need.execution && !enables.execution) {
    missing.emplace_back("execution");
  }
  if (need.grasp && !enables.grasp) {
    missing.emplace_back("grasp");
  }
  if (need.tool && !enables.tool) {
    missing.emplace_back("tool");
  }
  return missing;
}

}  // namespace peach2_task::core
