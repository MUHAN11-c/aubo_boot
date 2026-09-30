// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "peach2_task/core/batch_policy.hpp"
#include "peach2_task/core/enables_policy.hpp"
#include "peach2_task/core/safety_gate.hpp"

namespace core = peach2_task::core;
namespace fc = peach2_task::core::fc;
using core::Enables;
using core::Intent;

TEST(EnablesPolicy, DefaultIsAllFalse)
{
  const Enables e;
  EXPECT_FALSE(e.execution);
  EXPECT_FALSE(e.grasp);
  EXPECT_FALSE(e.tool);
  EXPECT_TRUE(core::validate_chain(e).empty());
}

TEST(EnablesPolicy, ChainToolGraspExecution)
{
  EXPECT_TRUE(core::validate_chain({true, false, false}).empty());
  EXPECT_TRUE(core::validate_chain({true, true, false}).empty());
  EXPECT_TRUE(core::validate_chain({true, true, true}).empty());
  EXPECT_FALSE(core::validate_chain({false, true, false}).empty());
  EXPECT_FALSE(core::validate_chain({true, false, true}).empty());
  EXPECT_FALSE(core::validate_chain({false, false, true}).empty());
}

TEST(EnablesPolicy, RequiredPerIntent)
{
  EXPECT_EQ(core::required_for(Intent::SURVEY_ONLY), (Enables{true, false, false}));
  EXPECT_EQ(core::required_for(Intent::PREGRASP_ONLY), (Enables{true, false, false}));
  EXPECT_EQ(core::required_for(Intent::FULL), (Enables{true, true, true}));
  EXPECT_EQ(
    core::missing_for({true, false, false}, Intent::FULL),
    (std::vector<std::string>{"grasp", "tool"}));
  EXPECT_TRUE(core::missing_for({true, false, false}, Intent::PREGRASP_ONLY).empty());
}

TEST(EnablesPolicy, IntentNames)
{
  Intent i = Intent::FULL;
  EXPECT_TRUE(core::intent_from_uint(0, &i));
  EXPECT_EQ(i, Intent::SURVEY_ONLY);
  EXPECT_FALSE(core::intent_from_uint(3, &i));
  EXPECT_TRUE(core::intent_from_name("PREGRASP_ONLY", &i));
  EXPECT_EQ(i, Intent::PREGRASP_ONLY);
  EXPECT_FALSE(core::intent_from_name("full", &i));
  EXPECT_STREQ(core::intent_name(Intent::FULL), "FULL");
}

namespace
{

core::SafetyInputs healthy(Intent intent)
{
  core::SafetyInputs in;
  in.robot.received = true;
  in.robot.age_s = 0.05;
  in.robot.drives_powered = 1;
  in.intent = intent;
  in.enables = intent == Intent::FULL ? Enables{true, true, true} : Enables{true, false, false};
  return in;
}

}  // namespace

TEST(SafetyGate, HealthyInputsPass)
{
  const core::SafetyConfig cfg;
  for (const Intent intent : {Intent::SURVEY_ONLY, Intent::PREGRASP_ONLY, Intent::FULL}) {
    const auto v = core::evaluate_safety(cfg, healthy(intent));
    EXPECT_TRUE(v.ok) << v.reason();
    EXPECT_EQ(v.failure_code, fc::NONE);
  }
}

TEST(SafetyGate, RobotStatusConditions)
{
  const core::SafetyConfig cfg;
  auto in = healthy(Intent::PREGRASP_ONLY);
  in.robot.received = false;
  auto v = core::evaluate_safety(cfg, in);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.blockers, (std::vector<std::string>{"robot_status_missing"}));
  EXPECT_EQ(v.failure_code, fc::ROBOT_NOT_READY);

  in = healthy(Intent::PREGRASP_ONLY);
  in.robot.age_s = 0.5;
  in.robot.e_stopped = 1;
  in.robot.drives_powered = 0;
  in.robot.in_error = 1;
  v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(
    v.blockers, (std::vector<std::string>{
    "robot_status_stale", "e_stopped", "drives_unpowered", "robot_in_error"}));
  EXPECT_EQ(v.reason(), "robot_status_stale,e_stopped,drives_unpowered,robot_in_error");
}

TEST(SafetyGate, MockSkipsRobotStatus)
{
  core::SafetyConfig cfg;
  cfg.require_robot_status = false;
  auto in = healthy(Intent::PREGRASP_ONLY);
  in.robot = core::RobotStatusSample{};
  EXPECT_TRUE(core::evaluate_safety(cfg, in).ok);
}

TEST(SafetyGate, EnablesMissingOrBrokenChain)
{
  const core::SafetyConfig cfg;
  auto in = healthy(Intent::FULL);
  in.enables = {true, true, false};
  auto v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(v.blockers, (std::vector<std::string>{"enable_missing:tool"}));
  EXPECT_EQ(v.failure_code, fc::SAFETY_GATE_CLOSED);

  in = healthy(Intent::SURVEY_ONLY);
  in.enables = {};
  v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(v.blockers, (std::vector<std::string>{"enable_missing:execution"}));

  in.enables = {false, true, false};
  v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(v.blockers.front(), "enables_chain_invalid");
}

TEST(SafetyGate, ToolFaultOnlyBlocksFull)
{
  const core::SafetyConfig cfg;
  auto in = healthy(Intent::PREGRASP_ONLY);
  in.tool_fault = true;
  EXPECT_TRUE(core::evaluate_safety(cfg, in).ok);
  in = healthy(Intent::FULL);
  in.tool_fault = true;
  const auto v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(v.blockers, (std::vector<std::string>{"tool_fault"}));
  EXPECT_EQ(v.failure_code, fc::TOOL_FAULT);
}

TEST(SafetyGate, FirstBlockerDecidesCode)
{
  const core::SafetyConfig cfg;
  auto in = healthy(Intent::FULL);
  in.robot.received = false;
  in.enables = {};
  in.tool_fault = true;
  const auto v = core::evaluate_safety(cfg, in);
  EXPECT_EQ(v.failure_code, fc::ROBOT_NOT_READY);
  EXPECT_EQ(v.blockers.size(), 5U);
}
