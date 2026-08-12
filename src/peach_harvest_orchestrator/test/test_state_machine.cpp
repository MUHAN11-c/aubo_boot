// Copyright 2026 mu
//
// Use of this source code is governed by the BSD-3-Clause license in LICENSE.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
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

#include "peach_harvest_orchestrator/state_machine.hpp"

namespace peach_harvest_orchestrator
{

TEST(HarvestStateMachine, AutoStartWaitsUntilEveryReadinessGateIsHealthy)
{
  HarvestStateMachine machine;

  machine.update_readiness({true, true, false, true});
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::WAITING_READY);
  EXPECT_EQ(machine.snapshot().blockers, std::vector<std::string>({"motion"}));

  machine.update_readiness({true, true, true, true});
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
  EXPECT_TRUE(machine.snapshot().run_active);
}

TEST(HarvestStateMachine, PauseDuringTargetWaitsForSafeCheckpoint)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-pause-test", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-7"));

  const auto requested = machine.control(
    ControlCommand::PAUSE, "pause-1", machine.snapshot().revision);
  ASSERT_TRUE(requested.accepted);
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSE_PENDING);

  machine.reach_safe_checkpoint();
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
  EXPECT_EQ(machine.snapshot().mode, OperationMode::PAUSED);
}

TEST(HarvestStateMachine, MaintenanceCannotStealAnActiveTarget)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-maintenance-test", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-8"));

  const auto result = machine.control(
    ControlCommand::ENTER_MAINTENANCE, "maintenance-1", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(machine.snapshot().mode, OperationMode::AUTO);
}

TEST(HarvestStateMachine, RecoveryMustBeAcknowledgedBeforeResume)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  machine.require_recovery("tool result uncertain");

  auto result = machine.control(ControlCommand::RESUME, "resume-1", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);

  result = machine.control(
    ControlCommand::ACKNOWLEDGE_RECOVERY, "ack-1", machine.snapshot().revision);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(machine.snapshot().recovery_required);
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
}

TEST(HarvestStateMachine, RevisionRejectsStaleCommandsAndRequestIdsAreIdempotent)
{
  HarvestStateMachine machine;
  const auto first = machine.control(ControlCommand::PAUSE, "pause-1", 0);
  ASSERT_TRUE(first.accepted);

  const auto duplicate = machine.control(ControlCommand::PAUSE, "pause-1", 0);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_EQ(duplicate.revision, first.revision);

  const auto stale = machine.control(ControlCommand::RESUME, "resume-2", 0);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.message, "状态版本已过期");
}

TEST(HarvestStateMachine, PolicyRejectsUnsafeDependencyCombination)
{
  HarvestStateMachine machine;
  const auto result = machine.set_policy(
    OperationPolicy{true, false, true, true}, "policy-1", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(machine.snapshot().policy.grasp_enabled);
  EXPECT_FALSE(machine.snapshot().policy.tool_enabled);
}

TEST(HarvestStateMachine, UnchangedHealthPollDoesNotInvalidateWebRevision)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  const auto stable_revision = machine.snapshot().revision;

  machine.update_readiness({true, true, true, true});
  EXPECT_EQ(machine.snapshot().revision, stable_revision);
}

TEST(HarvestStateMachine, ExecutionPolicyMustBeEnabledBeforeTargetDispatch)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});

  EXPECT_FALSE(machine.begin_target("target-safe"));

  const auto policy_result = machine.set_policy(
    OperationPolicy{true, true, false, false}, "enable-execution",
    machine.snapshot().revision);
  ASSERT_TRUE(policy_result.accepted);
  EXPECT_TRUE(machine.begin_target("target-safe"));
}

}  // namespace peach_harvest_orchestrator
