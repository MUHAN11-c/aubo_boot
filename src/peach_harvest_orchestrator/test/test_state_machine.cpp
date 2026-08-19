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

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

// 协议 2.4 目标丢失（阶段 D2）：未派发目标被感知 anchor_drop 移出锁定集时
// 被动记账——只动 counters/outcomes，不触碰活动周期与批次状态。
TEST(HarvestStateMachine, RecordPassiveOutcomeBooksWithoutActiveTarget)
{
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
  const auto revision = machine.snapshot().revision;

  EXPECT_TRUE(machine.record_passive_outcome(
      "peach_9", TargetOutcome::SKIPPED_UNREACHABLE, "目标丢失超时(anchor_drop)"));

  const auto & snap = machine.snapshot();
  EXPECT_FALSE(snap.target_active);
  EXPECT_EQ(snap.target_phase, TargetPhase::IDLE);
  EXPECT_EQ(snap.batch_state, BatchState::DISCOVERY);
  EXPECT_GT(snap.revision, revision);
  EXPECT_EQ(snap.counters.attempted, 1u);
  EXPECT_EQ(snap.counters.skipped_unreachable, 1u);
  ASSERT_EQ(snap.outcomes.size(), 1u);
  EXPECT_EQ(snap.outcomes[0].target_id, "peach_9");
  EXPECT_EQ(snap.outcomes[0].outcome, TargetOutcome::SKIPPED_UNREACHABLE);
  EXPECT_EQ(snap.outcomes[0].reason, "目标丢失超时(anchor_drop)");
  EXPECT_DOUBLE_EQ(snap.outcomes[0].elapsed_s, 0.0);
}

// 被动记账的批次可记窗口：未开批次/批次终态/待恢复一律拒绝（无账可记）。
TEST(HarvestStateMachine, RecordPassiveOutcomeRejectedOutsideAccountingWindow)
{
  // 未就绪（WAITING_READY）：批次未开始，拒绝。
  HarvestStateMachine waiting;
  EXPECT_FALSE(waiting.record_passive_outcome(
      "peach_1", TargetOutcome::SKIPPED_UNREACHABLE, "目标丢失超时(anchor_drop)"));
  EXPECT_EQ(waiting.snapshot().counters.attempted, 0u);

  // 批次完成（COMPLETED）：拒绝。
  HarvestStateMachine completed;
  completed.update_readiness({true, true, true, true});
  ASSERT_TRUE(completed.complete_batch("本轮未锁定到目标"));
  EXPECT_FALSE(completed.record_passive_outcome(
      "peach_1", TargetOutcome::SKIPPED_UNREACHABLE, "目标丢失超时(anchor_drop)"));
  EXPECT_TRUE(completed.snapshot().outcomes.empty());

  // 待恢复（RECOVERY_REQUIRED）：拒绝。
  HarvestStateMachine recovery;
  recovery.update_readiness({true, true, true, true});
  recovery.require_recovery("测试故障");
  EXPECT_FALSE(recovery.record_passive_outcome(
      "peach_1", TargetOutcome::SKIPPED_UNREACHABLE, "目标丢失超时(anchor_drop)"));
  EXPECT_TRUE(recovery.snapshot().outcomes.empty());
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

TEST(AllowDispatch, RequiresAllSixPreconditions)
{
  // 六项齐备才允许派发（协议 2.5 G-DISP：G1–G6）
  EXPECT_TRUE(allow_dispatch(true, "peach_1", "peach_0", false, true, true));
  // G1 拍照前置未完成（含等待收齐锁定）
  EXPECT_FALSE(allow_dispatch(false, "peach_1", "peach_0", false, true, true));
  // G2 selected 为空
  EXPECT_FALSE(allow_dispatch(true, "", "peach_0", false, true, true));
  // G3 去重：与上次派发同 id
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_1", false, true, true));
  // G4 已有活动目标
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", true, true, true));
  // G5 能力端 action 服务未就绪
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", false, false, true));
}

TEST(AllowDispatch, MotionGateBlocksDispatchWhenRobotNotReady)
{
  // G6 失能门（协议 2.5，I5）：robot_status 失能（急停/故障/未上电/不新鲜）
  // 或能力端未 Active 时，即使其余五项齐备也不得派发——否则 goal 必被能力端
  // 拒绝并误入拒绝熔断链（R3 根源）。
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", false, true, false));
  // G5 与 G6 独立成项：任一不满足都阻断
  EXPECT_FALSE(allow_dispatch(true, "peach_1", "peach_0", false, false, false));
}

TEST(AllowedCommands, TracksStateMachineAcceptanceTable)
{
  // 允许表与 control() 接受规则一致（HarvestState.permissions 填充源）。
  HarvestStateMachine machine;
  // 初始 WAITING_READY：可暂停/进维护/立即取消
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::PAUSE, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW}));

  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-commands", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::PAUSE, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW}));

  // 活动目标：PAUSE/SKIP_TARGET/CANCEL_NOW
  ASSERT_TRUE(machine.begin_target("target-cmd"));
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::PAUSE, ControlCommand::SKIP_TARGET,
        ControlCommand::CANCEL_NOW}));

  // PAUSE_PENDING（目标仍活动）：命令集不变
  ASSERT_TRUE(machine.control(
      ControlCommand::PAUSE, "pause-cmd", machine.snapshot().revision).accepted);
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::PAUSE_PENDING);
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::PAUSE, ControlCommand::SKIP_TARGET,
        ControlCommand::CANCEL_NOW}));

  // 落到 PAUSED：RESUME/ENTER_MAINTENANCE/CANCEL_NOW
  machine.record_target_outcome("target-cmd", TargetOutcome::SUCCEEDED, "");
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::RESUME, ControlCommand::ENTER_MAINTENANCE,
        ControlCommand::CANCEL_NOW}));

  // 维护模式：仅退出维护/立即取消
  ASSERT_TRUE(machine.control(
      ControlCommand::ENTER_MAINTENANCE, "maint-cmd",
      machine.snapshot().revision).accepted);
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>(
      {ControlCommand::EXIT_MAINTENANCE, ControlCommand::CANCEL_NOW}));
}

TEST(AllowedCommands, TerminalAndRecoveryStates)
{
  // RECOVERY_REQUIRED：仅 ACKNOWLEDGE_RECOVERY
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  machine.require_recovery("工具结果不确定");
  EXPECT_EQ(
    allowed_commands(machine.snapshot()),
    std::vector<ControlCommand>({ControlCommand::ACKNOWLEDGE_RECOVERY}));

  // COMPLETED/INTERRUPTED：无可控命令（重开走 RunHarvest goal）
  HarvestStateMachine completed;
  completed.update_readiness({true, true, true, true});
  ASSERT_TRUE(completed.complete_batch());
  EXPECT_TRUE(allowed_commands(completed.snapshot()).empty());

  HarvestStateMachine interrupted;
  interrupted.update_readiness({true, true, true, true});
  ASSERT_TRUE(interrupted.control(
      ControlCommand::CANCEL_NOW, "cancel-cmd",
      interrupted.snapshot().revision).accepted);
  ASSERT_EQ(interrupted.snapshot().batch_state, BatchState::INTERRUPTED);
  EXPECT_TRUE(allowed_commands(interrupted.snapshot()).empty());
}

TEST(HarvestStateMachine, BlockersProjectionDoesNotBumpRevision)
{
  // I7（协议 2.1）：blockers 投影变化与 readiness 翻转不推 revision——
  // 否则 0.78FPS 感知的就绪 flap 会持续 invalidate 乐观锁，控制命令被
  // 批量误拒"状态版本已过期"。
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
  const auto stable_revision = machine.snapshot().revision;

  // 感知路掉线→恢复：blockers 投影来回变化，revision 必须不动
  machine.update_readiness({false, true, true, true});
  EXPECT_EQ(machine.snapshot().blockers, std::vector<std::string>({"perception"}));
  EXPECT_EQ(machine.snapshot().revision, stable_revision);
  // 批次仍在 DISCOVERY（运行中就绪丢失只刷投影，不迁移批次状态，2.2）
  EXPECT_EQ(machine.snapshot().batch_state, BatchState::DISCOVERY);
  machine.update_readiness({true, true, true, true});
  EXPECT_TRUE(machine.snapshot().blockers.empty());
  EXPECT_EQ(machine.snapshot().revision, stable_revision);

  // 对照：真状态迁移（PAUSE 命令）仍推 revision
  ASSERT_TRUE(machine.control(
      ControlCommand::PAUSE, "pause-i7", machine.snapshot().revision).accepted);
  EXPECT_GT(machine.snapshot().revision, stable_revision);
}

TEST(HarvestStateMachine, TargetPhaseProjectionDoesNotBumpRevision)
{
  // I7：TargetPhase 透写是能力端反馈投影，不推 revision（返回 true 仅供
  // 节点侧节流发布状态）。
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-phase-i7", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-phase-i7"));
  const auto stable_revision = machine.snapshot().revision;

  EXPECT_TRUE(machine.set_target_phase(TargetPhase::OBSERVING));
  EXPECT_EQ(machine.snapshot().target_phase, TargetPhase::OBSERVING);
  EXPECT_EQ(machine.snapshot().revision, stable_revision);
  EXPECT_TRUE(machine.set_target_phase(TargetPhase::APPROACHING));
  EXPECT_EQ(machine.snapshot().revision, stable_revision);
}

TEST(HarvestStateMachine, RecordTargetOutcomeStoresElapsedAndQuality)
{
  // 2.13-E5 埋点：记账条目携带派发→终局墙钟与质量占位分，
  // HarvestSummary.outcomes 逐项透出的数据源。
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-elapsed", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("target-elapsed"));

  machine.record_target_outcome(
    "target-elapsed", TargetOutcome::SUCCEEDED, "采摘成功", 12.5, 1.0f);

  ASSERT_EQ(machine.snapshot().outcomes.size(), 1u);
  EXPECT_DOUBLE_EQ(machine.snapshot().outcomes[0].elapsed_s, 12.5);
  EXPECT_FLOAT_EQ(machine.snapshot().outcomes[0].quality_score, 1.0f);
}

TEST(PeriodEstimator, EmasAcrossArrivalIntervals)
{
  // 2.11 自适应新鲜度的实测通道：不足两次到达无估计（0=回退配置下限）；
  // 首个区间直接播种，其后按 alpha 平滑。
  PeriodEstimator estimator(0.3);
  EXPECT_DOUBLE_EQ(estimator.period_ema(), 0.0);
  estimator.observe(10.0);
  EXPECT_DOUBLE_EQ(estimator.period_ema(), 0.0);
  estimator.observe(12.0);
  EXPECT_DOUBLE_EQ(estimator.period_ema(), 2.0);
  estimator.observe(13.0);  // 区间 1.0：0.3*1.0 + 0.7*2.0 = 1.7
  EXPECT_DOUBLE_EQ(estimator.period_ema(), 1.7);
  // 非正间隔（时钟回拨/同刻重复）跳过不计，EMA 不被污染
  estimator.observe(13.0);
  estimator.observe(12.5);
  EXPECT_DOUBLE_EQ(estimator.period_ema(), 1.7);
}

TEST(AdaptiveFreshThreshold, MaxOfConfiguredFloorAndMeasuredPeriod)
{
  // fresh ⟺ age ≤ max(配置下限, 2.5×实测发布周期EMA)（协议 2.11）
  EXPECT_DOUBLE_EQ(adaptive_fresh_threshold(2.0, 0.0), 2.0);   // 无实测回退下限
  EXPECT_DOUBLE_EQ(adaptive_fresh_threshold(2.0, 0.1), 2.0);   // 快话题仍保下限
  EXPECT_DOUBLE_EQ(adaptive_fresh_threshold(2.0, 1.3), 3.25);  // 0.78FPS 感知放宽
  EXPECT_DOUBLE_EQ(adaptive_fresh_threshold(4.0, 1.3), 4.0);   // 下限更高时不缩
}

TEST(ReadinessTracker, FreshnessAdaptsToMeasuredPublishPeriod)
{
  // 注入间隔序列：感知帧间隔实测 1.3s（0.78FPS），固定 2s 下限会 flap
  // （age 2.5s 即判失连），自适应阈值 3.25s 下仍 fresh。
  ReadinessSample sample;
  sample.now_s = 100.0;
  sample.targets_received_s = 97.5;  // age=2.5s
  sample.targets_period_s = 1.3;
  sample.reconstruction_received_s = 97.5;
  sample.reconstruction_period_s = 1.0;  // 1Hz 心跳：阈值 max(2, 2.5)=2.5
  sample.robot_received_s = 99.0;
  sample.robot_period_s = 0.005;  // 200Hz：阈值 max(2, 0.0125)=2
  sample.robot_drives_powered = true;
  sample.robot_motion_possible = true;
  sample.action_server_ready = true;
  const auto readiness = ReadinessTracker(2.0, true).evaluate(sample);
  EXPECT_TRUE(readiness.perception);
  EXPECT_TRUE(readiness.reconstruction);
  EXPECT_TRUE(readiness.motion);

  // 节点真死：EMA 停留在最后实测周期，断流超 2.5×EMA 后判失连
  sample.targets_received_s = 96.0;  // age=4.0s > 3.25s
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).perception);
  // 无实测通道（period=0）时行为与固定下限完全一致
  sample.targets_period_s = 0.0;
  sample.targets_received_s = 98.5;  // age=1.5s ≤ 2s
  EXPECT_TRUE(ReadinessTracker(2.0, true).evaluate(sample).perception);
  sample.targets_received_s = 97.5;  // age=2.5s > 2s
  EXPECT_FALSE(ReadinessTracker(2.0, true).evaluate(sample).perception);
}

// ============================================================================
// 残局抬质量（协议 2.8 残局 OBSERVE_ONLY，阶段 E3）纯核用例：资格判据、
// 优先级序、次数上限截断、显式清单约束（清单外目标不入优先级序即不可选）、
// 审计记账（不进 counters、不动轮次判定口径）。
// ============================================================================
namespace observe_retry_tests
{
// 造一条终局账条目（elapsed/quality 与资格判定无关，留默认）。
TargetOutcomeRecord record(const std::string & id, TargetOutcome outcome)
{
  return TargetOutcomeRecord{id, outcome, "", 0.0, 0.0f};
}
}  // namespace observe_retry_tests

TEST(HarvestStateMachine, RecordObserveRetryOutcomeBooksAuditWithoutCounters)
{
  // E3 核心记账语义：observe 终局只进 outcomes 审计账，counters/attempted
  // 不动——observe 的 SUCCEEDED 是"观察+精化验证完成"而非采摘成功，计入
  // succeeded 会虚报批次成果；decide_round 的 attempted 口径也不受扰动。
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-observe-booking", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("peach_7"));
  const auto revision = machine.snapshot().revision;

  machine.record_observe_retry_outcome(
    "peach_7", TargetOutcome::SUCCEEDED, "observe_retry：精化验证通过", 4.5);

  const auto & snap = machine.snapshot();
  EXPECT_FALSE(snap.target_active);
  EXPECT_EQ(snap.target_phase, TargetPhase::IDLE);
  EXPECT_GT(snap.revision, revision);
  EXPECT_EQ(snap.counters.attempted, 0u);
  EXPECT_EQ(snap.counters.succeeded, 0u);
  ASSERT_EQ(snap.outcomes.size(), 1u);
  EXPECT_EQ(snap.outcomes[0].target_id, "peach_7");
  EXPECT_EQ(snap.outcomes[0].outcome, TargetOutcome::SUCCEEDED);
  EXPECT_EQ(snap.outcomes[0].reason, "observe_retry：精化验证通过");
  EXPECT_DOUBLE_EQ(snap.outcomes[0].elapsed_s, 4.5);
  EXPECT_FLOAT_EQ(snap.outcomes[0].quality_score, 0.0f);
  EXPECT_EQ(snap.message, "observe_retry：精化验证通过");
}

TEST(HarvestStateMachine, RecordObserveRetryOutcomeLandsPausePendingIntoPaused)
{
  // 与 record_target_outcome 同语义：暂停挂起期间 observe 周期终局落 PAUSED。
  HarvestStateMachine machine;
  ASSERT_TRUE(machine.set_policy(
      OperationPolicy{true, true, false, false}, "enable-observe-pause", 0).accepted);
  machine.update_readiness({true, true, true, true});
  ASSERT_TRUE(machine.begin_target("peach_8"));
  ASSERT_TRUE(machine.control(
      ControlCommand::PAUSE, "pause-observe", machine.snapshot().revision).accepted);
  ASSERT_EQ(machine.snapshot().batch_state, BatchState::PAUSE_PENDING);

  machine.record_observe_retry_outcome(
    "peach_8", TargetOutcome::SKIPPED_QUALITY, "observe_retry：精化仍未达标");

  EXPECT_EQ(machine.snapshot().batch_state, BatchState::PAUSED);
  EXPECT_EQ(machine.snapshot().mode, OperationMode::PAUSED);
  EXPECT_EQ(machine.snapshot().counters.skipped_quality, 0u);
  ASSERT_EQ(machine.snapshot().outcomes.size(), 1u);
  EXPECT_EQ(
    machine.snapshot().outcomes[0].outcome, TargetOutcome::SKIPPED_QUALITY);
}

TEST(HarvestStateMachine, RecordObserveRetryOutcomeWithoutActiveTargetIsIgnored)
{
  // I9：迟到终局（无活动目标）直接忽略，不记账不推 revision。
  HarvestStateMachine machine;
  machine.update_readiness({true, true, true, true});
  const auto revision = machine.snapshot().revision;

  machine.record_observe_retry_outcome(
    "ghost", TargetOutcome::SUCCEEDED, "observe_retry：迟到终局");

  EXPECT_EQ(machine.snapshot().revision, revision);
  EXPECT_TRUE(machine.snapshot().outcomes.empty());
}

TEST(ObserveRetryCandidate, SelectsFirstEligibleTargetInPriorityOrder)
{
  // 资格齐备时按优先级序取第一个（自动批次=感知锁定帧观测序；
  // 显式批次=清单顺序）。
  const std::vector<TargetOutcomeRecord> outcomes{
    observe_retry_tests::record("peach_1", TargetOutcome::SKIPPED_QUALITY),
    observe_retry_tests::record("peach_2", TargetOutcome::SKIPPED_QUALITY)};
  const std::vector<std::string> order{"peach_2", "peach_1"};
  const std::unordered_set<std::string> known{"peach_1", "peach_2"};

  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, order, known, {}, {}, 2),
    "peach_2");
  // 优先级序翻转则候选随之翻转
  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, {"peach_1", "peach_2"}, known, {}, {}, 2),
    "peach_1");
}

TEST(ObserveRetryCandidate, RequiresLatestOutcomeSkippedQuality)
{
  const std::vector<std::string> order{"peach_1"};
  const std::unordered_set<std::string> known{"peach_1"};

  // 无终局账（未派发过的 WAITING_QUALITY 目标不属于 E3 残局定义）
  EXPECT_TRUE(pick_observe_retry_candidate({}, order, known, {}, {}, 2).empty());
  // 最新账非 SKIPPED_QUALITY（已成功/不可达/失败/取消）不抬
  for (const auto outcome : {TargetOutcome::SUCCEEDED,
      TargetOutcome::SKIPPED_UNREACHABLE, TargetOutcome::FAILED,
      TargetOutcome::CANCELED})
  {
    EXPECT_TRUE(
      pick_observe_retry_candidate(
        {observe_retry_tests::record("peach_1", outcome)}, order, known,
        {}, {}, 2).empty());
  }
  // 历史账为 SKIPPED_QUALITY 但其后已有更新账（如抬质量成功后 FULL 重试
  // 成功）：以最新账为准，不再抬
  const std::vector<TargetOutcomeRecord> outcomes{
    observe_retry_tests::record("peach_1", TargetOutcome::SKIPPED_QUALITY),
    observe_retry_tests::record("peach_1", TargetOutcome::SUCCEEDED)};
  EXPECT_TRUE(pick_observe_retry_candidate(outcomes, order, known, {}, {}, 2).empty());
}

TEST(ObserveRetryCandidate, RespectsKnownSetCountBudgetAndRoundRetirement)
{
  const std::vector<TargetOutcomeRecord> outcomes{
    observe_retry_tests::record("peach_1", TargetOutcome::SKIPPED_QUALITY),
    observe_retry_tests::record("peach_2", TargetOutcome::SKIPPED_QUALITY)};
  const std::vector<std::string> order{"peach_1", "peach_2"};

  // 掉出当前锁定集（感知锚点已移除，派发必被能力端拒绝）：跳过该目标，
  // 顺位取下一个；掉出的是唯一候选时无候选
  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, order, {"peach_2"}, {}, {}, 2),
    "peach_2");
  EXPECT_TRUE(
    pick_observe_retry_candidate(
      outcomes, order, {"peach_2"}, {}, {"peach_2"}, 2).empty());
  // 次数预算耗尽（每目标每批次上限）：跳过该目标，顺位取下一个
  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, order, {"peach_1", "peach_2"}, {{"peach_1", 2}}, {}, 2),
    "peach_2");
  // 全部耗尽：无候选（随后才允许停滞计时/RESCAN 判定）
  EXPECT_TRUE(
    pick_observe_retry_candidate(
      outcomes, order, {"peach_1", "peach_2"},
      {{"peach_1", 2}, {"peach_2", 1}}, {}, 1)
    .empty());
  // 本轮已抬过（每轮每目标至多一次）：让位下一个；复扫清册后恢复资格
  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, order, {"peach_1", "peach_2"}, {{"peach_1", 1}}, {"peach_1"}, 2),
    "peach_2");
}

TEST(ObserveRetryCandidate, ExplicitBatchNeverIntroducesOffListTargets)
{
  // 显式清单约束（E3 协同）：清单外目标即使有 SKIPPED_QUALITY 账也不可选
  // ——节点侧显式批次以清单自身为优先级序，清单外目标天然不在序中。
  const std::vector<TargetOutcomeRecord> outcomes{
    observe_retry_tests::record("peach_list", TargetOutcome::SKIPPED_QUALITY),
    observe_retry_tests::record("peach_offlist", TargetOutcome::SKIPPED_QUALITY)};
  const std::vector<std::string> explicit_queue{"peach_list"};
  const std::unordered_set<std::string> known{"peach_list", "peach_offlist"};

  EXPECT_EQ(
    pick_observe_retry_candidate(
      outcomes, explicit_queue, known, {}, {}, 2),
    "peach_list");
}

// ---- 批次启动就位自校（阶段 F3，harvest.preflight_check）纯核判定 ----
// evaluate_preflight 是无状态纯函数：下列用例全部用 fake 事实驱动，
// 不依赖 DDS/TF；节点层职责只剩事实采集（见 harvest_orchestrator_node.cpp
// preflight_gate_passed_locked）。

TEST(PreflightCheck, AllFactsReadyPasses)
{
  // 全过放行：TF 可查 + robot_status 已收且无故障 → 空失败清单。
  const PreflightFacts facts{
    true,   // check_enabled
    true,   // tf_extrinsics_ready
    true,   // robot_check_required
    true,   // robot_status_received
    false   // robot_fault
  };
  EXPECT_TRUE(evaluate_preflight(facts).empty());
}

TEST(PreflightCheck, MissingTfExtrinsicsBlocks)
{
  // TF 外参链缺失阻断：robot 正常也救不回（合取语义）。
  PreflightFacts facts;
  facts.tf_extrinsics_ready = false;
  facts.robot_status_received = true;
  facts.robot_fault = false;
  EXPECT_EQ(
    evaluate_preflight(facts), std::vector<std::string>({"tf_extrinsics"}));
}

TEST(PreflightCheck, RobotStatusMissingBlocks)
{
  PreflightFacts facts;
  facts.tf_extrinsics_ready = true;
  facts.robot_status_received = false;
  EXPECT_EQ(
    evaluate_preflight(facts),
    std::vector<std::string>({"robot_status_missing"}));
}

TEST(PreflightCheck, RobotFaultBlocks)
{
  // 故障/急停标志阻断；多项失败按固定序同时列出（TF + robot 同时坏）。
  PreflightFacts facts;
  facts.tf_extrinsics_ready = false;
  facts.robot_status_received = true;
  facts.robot_fault = true;
  EXPECT_EQ(
    evaluate_preflight(facts),
    std::vector<std::string>({"tf_extrinsics", "robot_fault"}));
}

TEST(PreflightCheck, RobotCheckFollowsRequireRobotStatus)
{
  // readiness.require_robot_status=false 时 robot 两项不参与判定（对齐
  // 就绪门可选语义：该路可能根本无发布端，不应把批次卡在门外）。
  PreflightFacts facts;
  facts.tf_extrinsics_ready = true;
  facts.robot_check_required = false;
  facts.robot_status_received = false;
  facts.robot_fault = true;  // 即使缓存里有故障标志也不查
  EXPECT_TRUE(evaluate_preflight(facts).empty());
}

TEST(PreflightCheck, DisabledPassesThrough)
{
  // 开关关闭直通（旧行为）：全部事实恶劣也放行。
  PreflightFacts facts;
  facts.check_enabled = false;
  facts.tf_extrinsics_ready = false;
  facts.robot_status_received = false;
  facts.robot_fault = true;
  EXPECT_TRUE(evaluate_preflight(facts).empty());
}

TEST(PreflightCheck, EvaluationIsIdempotentForRetry)
{
  // 重试幂等：纯函数无状态，同一事实反复判定恒得同一输出——节点层
  // "下拍 refresh 重试、连续失败不熔断、就位后自动放行"由本性质保证。
  PreflightFacts facts;
  facts.tf_extrinsics_ready = false;
  facts.robot_status_received = false;
  const auto first = evaluate_preflight(facts);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(evaluate_preflight(facts), first);
  }
  // 环境就位后同一函数立即放行（无粘滞状态）。
  facts.tf_extrinsics_ready = true;
  facts.robot_status_received = true;
  EXPECT_TRUE(evaluate_preflight(facts).empty());
}

}  // namespace peach_harvest_orchestrator
