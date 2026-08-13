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

TEST(HarvestStateMachine, RecordTargetOutcomeBooksCountersAndClearsActiveTarget)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-record-test", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-1"));
  ASSERT_TRUE(machine.set_target_phase(TargetPhase::APPROACHING));

  machine.record_target_outcome("target-1", TargetOutcome::SUCCEEDED, "采摘成功");

  const auto & snap = machine.snapshot();
  EXPECT_FALSE(snap.target_active);
  EXPECT_EQ(snap.target_phase, TargetPhase::IDLE);
  EXPECT_EQ(snap.batch_state, BatchState::RUNNING);
  EXPECT_EQ(snap.message, "采摘成功");
  EXPECT_EQ(snap.counters.attempted, 1u);
  EXPECT_EQ(snap.counters.succeeded, 1u);
  ASSERT_EQ(snap.outcomes.size(), 1u);
  EXPECT_EQ(snap.outcomes[0].target_id, "target-1");
  EXPECT_EQ(snap.outcomes[0].outcome, TargetOutcome::SUCCEEDED);
  EXPECT_EQ(snap.outcomes[0].reason, "采摘成功");
}

TEST(HarvestStateMachine, RecordTargetOutcomeLandsPausePendingIntoPaused)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-pause-landing", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-2"));
  ASSERT_TRUE(machine.control(
      ControlCommand::PAUSE, "pause-pending", machine.snapshot().revision).accepted);
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::PAUSE_PENDING);

  machine.record_target_outcome("target-2", TargetOutcome::SKIPPED_QUALITY, "质量门未通过");

  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
  EXPECT_EQ(machine.snapshot().mode, OperationMode::PAUSED);
  EXPECT_EQ(machine.snapshot().message, "已在安全检查点暂停");
  EXPECT_EQ(machine.snapshot().counters.skipped_quality, 1u);
}

TEST(HarvestStateMachine, RecordTargetOutcomeWithoutActiveTargetIsIgnored)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  const auto revision = machine.snapshot().revision;

  machine.record_target_outcome("ghost", TargetOutcome::FAILED, "迟到终态");

  EXPECT_EQ(machine.snapshot().revision, revision);
  EXPECT_EQ(machine.snapshot().counters.attempted, 0u);
  EXPECT_TRUE(machine.snapshot().outcomes.empty());
}

TEST(HarvestStateMachine, CountersAccumulateAcrossTargets)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-accumulate", 0).accepted);
  machine.update_readiness({true, true, true, true});

  ASSERT_TRUE(machine.begin_target("target-a"));
  machine.record_target_outcome("target-a", TargetOutcome::SKIPPED_UNREACHABLE, "视角不可达");
  ASSERT_TRUE(machine.begin_target("target-b"));
  machine.record_target_outcome("target-b", TargetOutcome::FAILED, "执行失败");
  ASSERT_TRUE(machine.begin_target("target-c"));
  machine.record_target_outcome("target-c", TargetOutcome::CANCELED, "操作员跳过");

  const auto & snap = machine.snapshot();
  EXPECT_EQ(snap.counters.attempted, 3u);
  EXPECT_EQ(snap.counters.skipped_unreachable, 1u);
  EXPECT_EQ(snap.counters.failed, 1u);
  EXPECT_EQ(snap.counters.canceled, 1u);
  ASSERT_EQ(snap.outcomes.size(), 3u);
  EXPECT_EQ(snap.outcomes[0].target_id, "target-a");
  EXPECT_EQ(snap.outcomes[1].outcome, TargetOutcome::FAILED);
  EXPECT_EQ(snap.outcomes[2].reason, "操作员跳过");
}

TEST(HarvestStateMachine, CompleteBatchGuardsStateAndActiveTarget)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-complete-test", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);

  ASSERT_TRUE(machine.begin_target("target-6"));
  EXPECT_FALSE(machine.complete_batch());
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::RUNNING);

  machine.record_target_outcome("target-6", TargetOutcome::SUCCEEDED, "");
  ASSERT_TRUE(machine.complete_batch());
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::COMPLETED);
  EXPECT_FALSE(machine.snapshot().run_active);
  EXPECT_EQ(machine.snapshot().message, "批次完成");
  // 终态不可重复完成
  EXPECT_FALSE(machine.complete_batch());
}

TEST(HarvestStateMachine, CompleteBatchRejectedWhilePaused)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.control(
      ControlCommand::PAUSE, "pause-guard", machine.snapshot().revision).accepted);
  EXPECT_FALSE(machine.complete_batch());
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
}

TEST(HarvestStateMachine, ResetBatchClearsAccountingAndRestarts)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-reset-test", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-9"));
  machine.record_target_outcome("target-9", TargetOutcome::SUCCEEDED, "完成");
  ASSERT_TRUE(machine.complete_batch());

  ASSERT_TRUE(machine.reset_batch());
  const auto & snap = machine.snapshot();
  EXPECT_EQ(snap.batch_state, BatchState::DISCOVERY);
  EXPECT_EQ(snap.mode, OperationMode::AUTO);
  EXPECT_TRUE(snap.run_active);
  EXPECT_TRUE(snap.target_id.empty());
  EXPECT_EQ(snap.counters.attempted, 0u);
  EXPECT_TRUE(snap.outcomes.empty());
  // 运行中的批次不允许复位
  EXPECT_FALSE(machine.reset_batch());
}

TEST(HarvestStateMachine, ResetBatchKeepsRecoveryRequired)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  machine.require_recovery("工具结果不确定");
  ASSERT_TRUE(machine.control(
      ControlCommand::CANCEL_NOW, "cancel-recovery", machine.snapshot().revision).accepted);
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::INTERRUPTED);

  ASSERT_TRUE(machine.reset_batch());
  EXPECT_TRUE(machine.snapshot().recovery_required);
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
}

TEST(HarvestStateMachine, CompleteBatchRecordsCustomMessage)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});

  ASSERT_TRUE(machine.complete_batch("达到最大复拍轮次，批次完成"));
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::COMPLETED);
  EXPECT_EQ(machine.snapshot().message, "达到最大复拍轮次，批次完成");
}

TEST(HarvestStateMachine, DecideRoundWaitsUntilLockedAndProcessed)
{
  // 未锁定：等待感知收齐窗口
  EXPECT_EQ(decide_round(false, 0, 0, 1, 3, true).decision, RoundDecision::WAIT);
  // 已锁定但本轮目标未全部处理：selected 暂空可能只是质量待恢复，不算轮次结束
  EXPECT_EQ(decide_round(true, 3, 2, 1, 3, true).decision, RoundDecision::WAIT);
  EXPECT_EQ(decide_round(true, 1, 0, 1, 3, false).decision, RoundDecision::WAIT);
}

TEST(HarvestStateMachine, DecideRoundRescansWhenProcessedAndRoundsLeft)
{
  const auto verdict = decide_round(true, 3, 3, 1, 3, true);
  EXPECT_EQ(verdict.decision, RoundDecision::RESCAN);
  EXPECT_TRUE(verdict.message.empty());
  // 第 2 轮仍未达上限（max_rounds=3 含首轮）：继续复扫
  EXPECT_EQ(decide_round(true, 1, 1, 2, 3, true).decision, RoundDecision::RESCAN);
}

TEST(HarvestStateMachine, DecideRoundCompletesOnEmptyLock)
{
  // 空集也锁定：首轮/复扫轮锁定 0 个目标都直接完成批次
  const auto first_round = decide_round(true, 0, 0, 1, 3, true);
  EXPECT_EQ(first_round.decision, RoundDecision::COMPLETE);
  EXPECT_EQ(first_round.message, "本轮未锁定到目标，批次完成");
  EXPECT_EQ(
    decide_round(true, 0, 0, 2, 3, true).decision, RoundDecision::COMPLETE);
}

TEST(HarvestStateMachine, DecideRoundCompletesAtMaxRounds)
{
  const auto verdict = decide_round(true, 2, 2, 3, 3, true);
  EXPECT_EQ(verdict.decision, RoundDecision::COMPLETE);
  EXPECT_EQ(verdict.message, "达到最大复拍轮次，批次完成");
}

TEST(HarvestStateMachine, DecideRoundCompletesWhenRescanDisabled)
{
  const auto verdict = decide_round(true, 2, 2, 1, 3, false);
  EXPECT_EQ(verdict.decision, RoundDecision::COMPLETE);
  EXPECT_EQ(verdict.message, "本轮目标已处理完，批次完成");
}

TEST(HarvestStateMachine, SkipTargetAcceptedOnlyWithActiveTarget)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-skip-test", 0).accepted);
  machine.update_readiness({true, true, true, true});

  auto result = machine.control(
    ControlCommand::SKIP_TARGET, "skip-0", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);

  ASSERT_TRUE(machine.begin_target("target-7"));
  result = machine.control(
    ControlCommand::SKIP_TARGET, "skip-1", machine.snapshot().revision);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.message, "已请求跳过当前目标");
  // 状态机只登记意图：目标仍活动，真正的取消由节点层向能力端传播
  EXPECT_TRUE(machine.snapshot().target_active);

  result = machine.control(
    ControlCommand::RETRY_TARGET, "retry-1", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);
}

TEST(HarvestStateMachine, PauseRejectedWhileRecoveryRequired)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  machine.require_recovery("工具结果不确定");

  const auto result = machine.control(
    ControlCommand::PAUSE, "pause-recovery", machine.snapshot().revision);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.message, "需先确认恢复");
  // 拒绝不得改写批次状态
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::RECOVERY_REQUIRED);
}

TEST(HarvestStateMachine, SetTargetPhaseOnlyWhileTargetActive)
{
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-phase-test", 0).accepted);
  machine.update_readiness({true, true, true, true});

  EXPECT_FALSE(machine.set_target_phase(TargetPhase::APPROACHING));
  EXPECT_EQ(machine.snapshot().target_phase, TargetPhase::IDLE);

  ASSERT_TRUE(machine.begin_target("target-10"));
  EXPECT_TRUE(machine.set_target_phase(TargetPhase::APPROACHING));
  EXPECT_EQ(machine.snapshot().target_phase, TargetPhase::APPROACHING);
  // 阶段无变化时不推 revision
  const auto revision = machine.snapshot().revision;
  EXPECT_FALSE(machine.set_target_phase(TargetPhase::APPROACHING));
  EXPECT_EQ(machine.snapshot().revision, revision);
}

TEST(HarvestStateMachine, RequestResultCacheEvictsOldestBeyondLimit)
{
  HarvestStateMachine machine;
  for (int i = 0; i < 260; ++i) {
    const auto result = machine.control(
      ControlCommand::PAUSE, "pause-" + std::to_string(i), machine.snapshot().revision);
    ASSERT_TRUE(result.accepted);
  }
  // 260 条插入超过 256 上限：最近的仍命中缓存（直接返回旧结果，无视过期 revision）
  const auto cached = machine.control(ControlCommand::PAUSE, "pause-259", 0);
  EXPECT_TRUE(cached.accepted);
  // 最旧的已被淘汰：按新命令处理，因过期 revision 拒绝
  const auto evicted = machine.control(ControlCommand::PAUSE, "pause-0", 0);
  EXPECT_FALSE(evicted.accepted);
}

TEST(ReadinessTracker, AllFreshAndRobotHealthyIsFullyReady)
{
  ReadinessSample sample;
  sample.now_s = 100.0;
  sample.targets_received_s = 99.5;
  sample.reconstruction_received_s = 99.0;
  sample.robot_received_s = 99.8;
  sample.robot_drives_powered = true;
  sample.robot_motion_possible = true;
  sample.action_server_ready = true;
  const auto readiness = ReadinessTracker(2.0, true).evaluate(sample);
  EXPECT_TRUE(readiness.perception);
  EXPECT_TRUE(readiness.reconstruction);
  EXPECT_TRUE(readiness.motion);
  EXPECT_TRUE(readiness.web);
  EXPECT_TRUE(ReadinessTracker::blockers(readiness).empty());
}

TEST(ReadinessTracker, FreshnessBoundaryAndNeverReceived)
{
  ReadinessSample sample;
  sample.now_s = 100.0;
  sample.targets_received_s = 98.0;  // age 恰等于 timeout：仍 fresh
  sample.reconstruction_received_s = 97.9;  // age 超过 timeout：不再 fresh
  sample.robot_received_s = 0.0;  // 从未收到：不 fresh
  sample.robot_drives_powered = true;
  sample.robot_motion_possible = true;
  sample.action_server_ready = true;
  const auto readiness = ReadinessTracker(2.0, true).evaluate(sample);
  EXPECT_TRUE(readiness.perception);
  EXPECT_FALSE(readiness.reconstruction);
  EXPECT_FALSE(readiness.motion);
  // 超龄/未收到各映射到固定路名，顺序与状态机 blockers 投影一致
  const auto blockers = ReadinessTracker::blockers(readiness);
  ASSERT_EQ(blockers.size(), 2U);
  EXPECT_EQ(blockers[0], "reconstruction");
  EXPECT_EQ(blockers[1], "motion");
}

TEST(ReadinessTracker, RobotFlagsGateMotionAndRequireFlagBypasses)
{
  ReadinessSample sample;
  sample.now_s = 50.0;
  sample.targets_received_s = 49.5;
  sample.reconstruction_received_s = 49.5;
  sample.robot_received_s = 49.5;
  sample.robot_drives_powered = true;
  sample.robot_motion_possible = true;
  sample.action_server_ready = true;

  // 急停/错误/未上电/不可运动任一置位都拉掉 motion
  sample.robot_e_stopped = true;
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).motion);
  sample.robot_e_stopped = false;
  sample.robot_in_error = true;
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).motion);
  sample.robot_in_error = false;
  sample.robot_drives_powered = false;
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).motion);
  sample.robot_drives_powered = true;
  sample.robot_motion_possible = false;
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).motion);

  // require_robot_status=false：robot 通路整体旁路，motion 只看 action 服务
  sample.robot_e_stopped = true;
  sample.robot_received_s = 0.0;
  EXPECT_TRUE(ReadinessTracker(2.0, false).evaluate(sample).motion);
  sample.action_server_ready = false;
  EXPECT_FALSE(ReadinessTracker(2.0, false).evaluate(sample).motion);
}

TEST(AllowDispatch, RequiresAllFivePreconditions)
{
  // 五项齐备才允许派发
  EXPECT_TRUE(allow_dispatch(true, "peach_1", "peach_0", false, true));
  // 拍照前置未完成（含等待收齐锁定）
  EXPECT_FALSE(allow_dispatch(false, "peach_1", "peach_0", false, true));
  // selected 为空
  EXPECT_FALSE(allow_dispatch(true, "", "peach_0", false, true));
  // 去重：与上次派发同 id
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_1", false, true));
  // 已有活动目标
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", true, true));
  // 能力端 action 服务未就绪
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", false, false));
}

}  // namespace peach_harvest_orchestrator
