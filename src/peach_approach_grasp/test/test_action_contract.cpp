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

#include <string>

#include "peach_approach_grasp/cycle_state.hpp"

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

TEST(ActionContract, RunningStatesAreNotReportedAsTerminal)
{
  EXPECT_EQ(terminalOutcome(CycleState::IDLE), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::PLAN_OBSERVATION), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::MOVE_TO_VIEW), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::WAIT_FRAME), CycleOutcome::RUNNING);
  EXPECT_EQ(terminalOutcome(CycleState::FINALIZE), CycleOutcome::RUNNING);
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
}  // namespace peach_approach_grasp
