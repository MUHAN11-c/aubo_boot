// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
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
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <type_traits>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>

#include "peach_approach_grasp/cycle_state.hpp"
#include "peach_approach_grasp/stage_timing.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"

namespace peach_approach_grasp
{
TEST(ActionContract, SuccessTerminalStates)
{
  EXPECT_EQ(terminalOutcome(CycleState::SUCCEEDED), CycleOutcome::SUCCEEDED);
  EXPECT_EQ(terminalOutcome(CycleState::PREVIEW_READY), CycleOutcome::SUCCEEDED);
  // 回归用例：只规划与 grasp 关闭两档的圆满终态必须上报成功，不能再落入 FAILED。
  EXPECT_EQ(terminalOutcome(CycleState::PLAN_READY), CycleOutcome::SUCCEEDED);
  EXPECT_EQ(terminalOutcome(CycleState::READY_FOR_GRASP), CycleOutcome::SUCCEEDED);
}

TEST(ActionContract, NonSuccessTerminalStatesKeepTheirMeaning)
{
  EXPECT_EQ(terminalOutcome(CycleState::CANCELED), CycleOutcome::CANCELED);
  EXPECT_EQ(terminalOutcome(CycleState::FAILED), CycleOutcome::FAILED);
  EXPECT_EQ(terminalOutcome(CycleState::PREVIEW_FAILED), CycleOutcome::FAILED);
  EXPECT_EQ(terminalOutcome(CycleState::RECOVERY_REQUIRED), CycleOutcome::RECOVERY_REQUIRED);
}

TEST(ActionContract, ActionResultOutcomeConstantsMatchContract)
{
  // RunTargetCycle.Result.outcome 契约钉死：取消终局必须有独立的 CANCELED=4，
  // 不得复用 FAILED（executeAction 取消分支直接填充该常量上报编排器）。
  using Result = peach_harvest_msgs::action::RunTargetCycle::Result;
  EXPECT_EQ(Result::SUCCEEDED, 0);
  EXPECT_EQ(Result::SKIPPED_QUALITY, 1);
  EXPECT_EQ(Result::SKIPPED_UNREACHABLE, 2);
  EXPECT_EQ(Result::FAILED, 3);
  EXPECT_EQ(Result::CANCELED, 4);
  EXPECT_NE(Result::CANCELED, Result::FAILED);
  // goal mode 三档契约：OBSERVE_ONLY 已受理（不再是死模式）。
  using Goal = peach_harvest_msgs::action::RunTargetCycle::Goal;
  EXPECT_EQ(Goal::PREVIEW, 0);
  EXPECT_EQ(Goal::OBSERVE_ONLY, 1);
  EXPECT_EQ(Goal::FULL, 2);
}

TEST(ActionContract, RunningStatesAreNotReportedAsTerminal)
{
  EXPECT_EQ(terminalOutcome(CycleState::IDLE), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::PLAN_OBSERVATION), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::MOVE_TO_VIEW), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::WAIT_FRAME), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::FINALIZE), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::RECONFIRM), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::MTC_APPROACH_INSERT), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::ACTUATE_TOOL), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::MTC_RETREAT), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::PREVIEW_CONTACT_PLANNING), CycleOutcome::RUNNING);
}

TEST(ActionContract, StateStringsMatchLegacyJsonProjection)
{
  // 发布层投影字符串必须与历史状态 JSON 一致（dashboard/web 只读消费）。
  EXPECT_EQ(toString(CycleState::IDLE), "IDLE");
  EXPECT_EQ(toString(CycleState::PLAN_OBSERVATION), "PLAN_OBSERVATION");
  EXPECT_EQ(toString(CycleState::MOVE_TO_VIEW), "MOVE_TO_VIEW");
  EXPECT_EQ(toString(CycleState::WAIT_FRAME), "WAIT_FRAME");
  EXPECT_EQ(toString(CycleState::FINALIZE), "FINALIZE");
  EXPECT_EQ(toString(CycleState::RECONFIRM), "RECONFIRM");
  EXPECT_EQ(toString(CycleState::MTC_APPROACH_INSERT), "MTC_APPROACH_INSERT");
  EXPECT_EQ(toString(CycleState::ACTUATE_TOOL), "ACTUATE_TOOL");
  EXPECT_EQ(toString(CycleState::MTC_RETREAT), "MTC_RETREAT");
  EXPECT_EQ(toString(CycleState::PREVIEW_CONTACT_PLANNING), "PREVIEW_CONTACT_PLANNING");
  EXPECT_EQ(toString(CycleState::PREVIEW_READY), "PREVIEW_READY");
  EXPECT_EQ(toString(CycleState::PREVIEW_FAILED), "PREVIEW_FAILED");
  EXPECT_EQ(toString(CycleState::PLAN_READY), "PLAN_READY");
  EXPECT_EQ(toString(CycleState::READY_FOR_GRASP), "READY_FOR_GRASP");
  EXPECT_EQ(toString(CycleState::SUCCEEDED), "SUCCEEDED");
  EXPECT_EQ(toString(CycleState::CANCELED), "CANCELED");
  EXPECT_EQ(toString(CycleState::FAILED), "FAILED");
  EXPECT_EQ(toString(CycleState::RECOVERY_REQUIRED), "RECOVERY_REQUIRED");
}

TEST(ActionContract, ResultStageDurationFieldsExist)
{
  // 阶段耗时埋点字段契约（重构阶段 C）：stage_durations 与 stage_names 为
  // 等长并行数组，类型钉死（编排器按 name 对齐 duration 消费）。
  using Result = peach_harvest_msgs::action::RunTargetCycle::Result;
  Result result;
  static_assert(
    std::is_same_v<
      decltype(result.stage_durations),
      std::vector<builtin_interfaces::msg::Duration>>);
  static_assert(
    std::is_same_v<decltype(result.stage_names), std::vector<std::string>>);
  // 默认构造两数组为空（未启动周期/启动即失败的合法上报形态）。
  EXPECT_TRUE(result.stage_names.empty());
  EXPECT_TRUE(result.stage_durations.empty());
  result.stage_names.push_back("prepare");
  builtin_interfaces::msg::Duration duration;
  duration.sec = 1;
  duration.nanosec = 500000000u;
  result.stage_durations.push_back(duration);
  ASSERT_EQ(result.stage_names.size(), result.stage_durations.size());
  EXPECT_EQ(result.stage_names[0], "prepare");
  EXPECT_EQ(result.stage_durations[0].sec, 1);
  EXPECT_EQ(result.stage_durations[0].nanosec, 500000000u);
}

TEST(ActionContract, StageNameLegalSetMatchesActionContract)
{
  // 合法阶段名集合与 RunTargetCycle.action 注释中的契约集合钉死；
  // reconfirm 由 CycleState::RECONFIRM 投影（阶段 E1 已落地）。
  const std::vector<std::string> expected = {
    "prepare", "observe", "finalize", "reconfirm", "approach_insert", "tool",
    "retreat"};
  const std::vector<std::string> actual(kStageNames.begin(), kStageNames.end());
  EXPECT_EQ(actual, expected);
  EXPECT_STREQ(stageForState(CycleState::RECONFIRM), "reconfirm");
}

TEST(ActionContract, TargetPhaseProjectionMatchesHarvestStateContract)
{
  // A13：CycleState→HarvestState.target_phase 投影（RunTargetCycle 反馈携带，
  // 编排器批次过程线消费）；常量与 peach_harvest_msgs/HarvestState.msg 钉死。
  using HarvestStateMsg = peach_harvest_msgs::msg::HarvestState;
  EXPECT_EQ(targetPhase(CycleState::IDLE), HarvestStateMsg::TARGET_IDLE);
  EXPECT_EQ(targetPhase(CycleState::CANCELED), HarvestStateMsg::TARGET_IDLE);
  EXPECT_EQ(targetPhase(CycleState::PLAN_OBSERVATION), HarvestStateMsg::OBSERVING);
  EXPECT_EQ(targetPhase(CycleState::MOVE_TO_VIEW), HarvestStateMsg::OBSERVING);
  EXPECT_EQ(targetPhase(CycleState::WAIT_FRAME), HarvestStateMsg::OBSERVING);
  EXPECT_EQ(targetPhase(CycleState::FINALIZE), HarvestStateMsg::FINALIZING);
  // 抓取前再确认（2.7-RECONFIRM）映射 VALIDATING（接触段前最后一道验证关）。
  EXPECT_EQ(targetPhase(CycleState::RECONFIRM), HarvestStateMsg::VALIDATING);
  EXPECT_EQ(
    targetPhase(CycleState::MTC_APPROACH_INSERT), HarvestStateMsg::APPROACHING);
  EXPECT_EQ(
    targetPhase(CycleState::PREVIEW_CONTACT_PLANNING),
    HarvestStateMsg::APPROACHING);
  EXPECT_EQ(targetPhase(CycleState::ACTUATE_TOOL), HarvestStateMsg::TOOL_ACTION);
  EXPECT_EQ(targetPhase(CycleState::MTC_RETREAT), HarvestStateMsg::RETREATING);
  // plan-only 圆满终态映射 COMPLETING（周期收尾、结果即出）。
  EXPECT_EQ(targetPhase(CycleState::PLAN_READY), HarvestStateMsg::COMPLETING);
  EXPECT_EQ(targetPhase(CycleState::READY_FOR_GRASP), HarvestStateMsg::COMPLETING);
  EXPECT_EQ(targetPhase(CycleState::PREVIEW_READY), HarvestStateMsg::COMPLETING);
  EXPECT_EQ(targetPhase(CycleState::SUCCEEDED), HarvestStateMsg::TARGET_SUCCEEDED);
  EXPECT_EQ(targetPhase(CycleState::FAILED), HarvestStateMsg::TARGET_FAILED);
  EXPECT_EQ(targetPhase(CycleState::PREVIEW_FAILED), HarvestStateMsg::TARGET_FAILED);
  EXPECT_EQ(
    targetPhase(CycleState::RECOVERY_REQUIRED), HarvestStateMsg::TARGET_FAILED);
}
}  // namespace peach_approach_grasp
