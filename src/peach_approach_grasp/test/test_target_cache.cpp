// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
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

#include <atomic>
#include <chrono>
#include <string>

#include "peach_approach_grasp/target_cache.hpp"

namespace peach_approach_grasp
{
namespace
{
// 带有效几何的选中目标更新（id 可变）。
SelectedTargetUpdate makeObservedTarget(const std::string & id, const std::string & run)
{
  SelectedTargetUpdate update;
  update.selected_id = id;
  update.harvest_run_id = run;
  update.observed = true;
  update.bottom = Eigen::Vector3d(0.30, 0.0, 0.10);
  update.neck = Eigen::Vector3d(0.30, 0.0, 0.18);
  update.axis = Eigen::Vector3d::UnitZ();
  update.suggested_travel_m = 0.05;
  return update;
}

RefinedFittingUpdate makeFitting(const std::string & id)
{
  RefinedFittingUpdate update;
  update.target_id = id;
  update.is_fruit = true;
  update.sphere_rms_m = 0.003;
  update.sphere_inlier_ratio = 0.6;
  update.accepted = true;
  return update;
}

// 锁定集单目标更新（observed 可控；x 偏移区分不同目标几何）。
LockedTargetUpdate makeLockedTarget(
  const std::string & id, bool observed = true, double x = 0.30)
{
  LockedTargetUpdate update;
  update.target_id = id;
  update.observed = observed;
  update.bottom = Eigen::Vector3d(x, 0.0, 0.10);
  update.neck = Eigen::Vector3d(x, 0.0, 0.18);
  update.axis = Eigen::Vector3d::UnitZ();
  update.suggested_travel_m = 0.05;
  return update;
}
}  // namespace

// transient_local 的精化结果可能早于 volatile 目标观测到达：同 ID 首次观测必须
// 保留已锁存的拟合指标；确认 ID 冲突时才清理。
TEST(TargetCache, LatchedFittingSurvivesSameIdAndClearsOnConflict)
{
  double now_s = 10.0;
  TargetCache cache([&now_s]() {return now_s;});

  ASSERT_TRUE(cache.updateRefinedFitting(makeFitting("peach_1")));
  cache.updateSelectedTarget(makeObservedTarget("peach_1", "run-1"));
  auto quality = cache.qualitySnapshot();
  EXPECT_EQ(quality.refined_target_id, "peach_1");
  EXPECT_DOUBLE_EQ(quality.refined_rmse_m, 0.003);
  EXPECT_TRUE(quality.refined_accept);
  ASSERT_TRUE(cache.targetSnapshot().has_value());
  EXPECT_EQ(cache.targetGateSample().id, "peach_1");

  // 切到不同目标：精化/决策缓存按冲突清理；新目标本帧带有效几何故自身有效。
  cache.updateSelectedTarget(makeObservedTarget("peach_2", "run-1"));
  quality = cache.qualitySnapshot();
  EXPECT_EQ(quality.selected_target_id, "peach_2");
  EXPECT_TRUE(quality.refined_target_id.empty());
  EXPECT_DOUBLE_EQ(quality.refined_rmse_m, -1.0);
  EXPECT_FALSE(quality.refined_accept);
  EXPECT_FALSE(quality.grasp_allowed);
  ASSERT_TRUE(cache.targetSnapshot().has_value());
  EXPECT_EQ(cache.targetSnapshot()->id, "peach_2");
  EXPECT_FALSE(cache.refinedSnapshot().has_value());
}

TEST(TargetCache, GraspDecisionIgnoredForOtherTarget)
{
  double now_s = 20.0;
  TargetCache cache([&now_s]() {return now_s;});
  cache.updateSelectedTarget(makeObservedTarget("peach_1", "run-2"));

  // 非当前目标的决策被忽略，不覆盖 grasp_allowed
  EXPECT_FALSE(cache.updateGraspDecision("peach_9", true));
  EXPECT_TRUE(cache.graspDecisionTarget().empty());
  EXPECT_FALSE(cache.qualitySnapshot().grasp_allowed);

  // 当前目标的决策被接受
  EXPECT_TRUE(cache.updateGraspDecision("peach_1", true));
  EXPECT_EQ(cache.graspDecisionTarget(), "peach_1");
  EXPECT_TRUE(cache.qualitySnapshot().grasp_allowed);

  // 清空候选的精化位姿调用不视为冲突
  RefinedPoseUpdate clear_update;
  clear_update.clear = true;
  EXPECT_TRUE(cache.updateRefinedPose(clear_update));
  EXPECT_FALSE(cache.refinedSnapshot().has_value());
}

// 检测闪烁（短暂未观测帧）不得清除最后有效样本：安全门按 max_age 陈旧兜底，
// 单帧闪烁不判 not_observed（0.78FPS 相机下两帧间隔已达 1.28s）。
TEST(TargetCache, TransientLossKeepsLastValidSample)
{
  double now_s = 50.0;
  TargetCache cache([&now_s]() {return now_s;});
  cache.updateSelectedTarget(makeObservedTarget("peach_1", "run-3"));
  ASSERT_TRUE(cache.targetSnapshot().has_value());

  // 一帧未观测（observed=false）：有效样本保留，received_s 不刷新
  now_s = 51.0;
  SelectedTargetUpdate lost;
  lost.selected_id = "peach_1";
  lost.harvest_run_id = "run-3";
  lost.observed = false;
  cache.updateSelectedTarget(lost);
  const auto sample = cache.targetGateSample();
  EXPECT_EQ(sample.id, "peach_1");
  EXPECT_TRUE(sample.valid);
  EXPECT_DOUBLE_EQ(sample.received_s, 50.0);
  EXPECT_TRUE(cache.targetSnapshot().has_value());

  // 有效观测恢复：received_s 刷新到新帧时刻
  now_s = 52.0;
  cache.updateSelectedTarget(makeObservedTarget("peach_1", "run-3"));
  EXPECT_DOUBLE_EQ(cache.targetGateSample().received_s, 52.0);
}

TEST(TargetCache, WaitForFreshTargetSemantics)
{
  double now_s = 60.0;
  TargetCache cache([&now_s]() {return now_s;});
  std::atomic_bool cancel{false};
  cache.updateSelectedTarget(makeObservedTarget("peach_1", "run-4"));

  // 已有有效样本晚于阈值：立即返回 true
  EXPECT_TRUE(cache.waitForFreshTarget(59.0, 0.01, cancel));
  // 有效样本不晚于阈值：无新帧时超时返回 false
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_FALSE(cache.waitForFreshTarget(61.0, 0.05, cancel));
  EXPECT_GE(
    std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
    .count(), 0.04);
  // cancel 置位：立即返回 false
  cancel.store(true);
  EXPECT_FALSE(cache.waitForFreshTarget(59.0, 5.0, cancel));
}

TEST(TargetCache, QualitySnapshotDataAgeFollowsInjectedClock)
{
  double now_s = 100.0;
  TargetCache cache([&now_s]() {return now_s;});

  // 未收到 diagnostics：保持缺省大 age
  EXPECT_GT(cache.qualitySnapshot().data_age_s, 1.0e8);

  ReconstructionDiagnosticsUpdate update;
  update.target_id = "peach_1";
  update.state = "READY";
  update.captured_views = 3;
  update.max_baseline_deg = 25.0;
  update.view_directions = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};
  cache.updateReconstructionDiagnostics(update);
  auto quality = cache.qualitySnapshot();
  EXPECT_EQ(quality.reconstruction_state, "READY");
  EXPECT_EQ(quality.captured_views, 3U);
  EXPECT_EQ(cache.observedDirections().size(), 2U);
  EXPECT_DOUBLE_EQ(quality.data_age_s, 0.0);

  // 时钟前进 1.5s：快照 age 跟随注入时钟
  now_s = 101.5;
  EXPECT_DOUBLE_EQ(cache.qualitySnapshot().data_age_s, 1.5);
}

TEST(TargetCache, SwingingAndTrackingStatusPassThrough)
{
  // 再确认段诊断透传（2.7-RECONFIRM）：swinging/tracking_status 每帧刷新
  // （含非观测帧——非观测帧不刷新 received_s 但必须更新摆动/跟踪投影）。
  double now_s = 80.0;
  TargetCache cache([&now_s]() {return now_s;});
  auto update = makeObservedTarget("peach_1", "run-5");
  update.swinging = false;
  update.tracking_status = 0;  // OBSERVED
  cache.updateSelectedTarget(update);
  auto target = cache.targetSnapshot();
  ASSERT_TRUE(target.has_value());
  EXPECT_FALSE(target->swinging);
  EXPECT_EQ(target->tracking_status, 0);

  // 摆动中的观测帧：旗标随帧透传。
  now_s = 81.0;
  update.swinging = true;
  cache.updateSelectedTarget(update);
  EXPECT_TRUE(cache.targetSnapshot()->swinging);

  // 出视野帧（非观测）：received_s 不刷新，但跟踪状态投影要更新。
  now_s = 82.0;
  update.observed = false;
  update.swinging = false;
  update.tracking_status = 4;  // OUT_OF_VIEW
  cache.updateSelectedTarget(update);
  target = cache.targetSnapshot();
  ASSERT_TRUE(target.has_value());
  EXPECT_EQ(target->tracking_status, 4);
  EXPECT_DOUBLE_EQ(target->received_s, 81.0);
  EXPECT_FALSE(target->swinging);
}

// 锁定集锚点缓存（阶段 E 残局抬质量能力端）：多目标共存、按 ID 取快照，
// 与 selected 缓存互不干扰（残期 selected 为空时锁定集仍是可用数据源）。
TEST(TargetCache, LockedSetCachesMultipleTargetsById)
{
  double now_s = 200.0;
  TargetCache cache([&now_s]() {return now_s;});

  cache.updateLockedTargets(
    true, "run-100", {makeLockedTarget("peach_1", true, 0.30),
      makeLockedTarget("peach_2", true, 0.45)});
  const auto first = cache.lockedTargetSnapshot("peach_1");
  const auto second = cache.lockedTargetSnapshot("peach_2");
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->id, "peach_1");
  EXPECT_DOUBLE_EQ(first->center.x(), 0.30);
  EXPECT_DOUBLE_EQ(second->center.x(), 0.45);
  EXPECT_DOUBLE_EQ(first->received_s, 200.0);
  EXPECT_DOUBLE_EQ(second->received_s, 200.0);
  // 未收录目标：快照为空，门样本 id 为空（受理门判"不在锁定集"）。
  EXPECT_FALSE(cache.lockedTargetSnapshot("peach_9").has_value());
  EXPECT_TRUE(cache.lockedTargetGateSample("peach_9").id.empty());
  // 锁定集缓存不影响 selected 缓存（两者数据源独立）。
  EXPECT_FALSE(cache.targetSnapshot().has_value());
}

// 受理门拒绝原因区分的数据源：门样本 id 命中但 valid=false = 锚点缺失
// （条目在但从未携带有效几何）；id 空 = 不在锁定集。
TEST(TargetCache, LockedSetDistinguishesMissingAnchorFromAbsent)
{
  double now_s = 210.0;
  TargetCache cache([&now_s]() {return now_s;});
  LockedTargetUpdate no_anchor = makeLockedTarget("peach_1");
  no_anchor.bottom = Eigen::Vector3d::Zero();
  no_anchor.neck = Eigen::Vector3d::Zero();
  no_anchor.axis = Eigen::Vector3d::Zero();
  cache.updateLockedTargets(true, "run-101", {no_anchor});

  const auto sample = cache.lockedTargetGateSample("peach_1");
  EXPECT_EQ(sample.id, "peach_1");
  EXPECT_FALSE(sample.valid);
  EXPECT_FALSE(cache.lockedTargetSnapshot("peach_1").has_value());
  // 后续帧带来有效锚点：条目转正，快照可用。
  now_s = 211.0;
  cache.updateLockedTargets(true, "run-101", {makeLockedTarget("peach_1")});
  ASSERT_TRUE(cache.lockedTargetSnapshot("peach_1").has_value());
  EXPECT_TRUE(cache.lockedTargetGateSample("peach_1").valid);
}

// target_set_locked=false 清空锁定集缓存（锁定前 observations 恒空，
// 缓存必须随之失效，防止拿上一锁定期的锚点误受理）。
TEST(TargetCache, LockedSetClearsOnUnlock)
{
  double now_s = 220.0;
  TargetCache cache([&now_s]() {return now_s;});
  cache.updateLockedTargets(true, "run-102", {makeLockedTarget("peach_1")});
  ASSERT_TRUE(cache.lockedTargetSnapshot("peach_1").has_value());

  cache.updateLockedTargets(false, "run-102", {});
  EXPECT_FALSE(cache.lockedTargetSnapshot("peach_1").has_value());
  EXPECT_TRUE(cache.lockedTargetGateSample("peach_1").id.empty());
}

// harvest_run_id 变化（批次切换）清空锁定集缓存：跨批次身份不复用。
TEST(TargetCache, LockedSetClearsOnRunIdChange)
{
  double now_s = 230.0;
  TargetCache cache([&now_s]() {return now_s;});
  cache.updateLockedTargets(true, "run-103", {makeLockedTarget("peach_1")});
  ASSERT_TRUE(cache.lockedTargetSnapshot("peach_1").has_value());

  // 新批次首帧尚未携带该目标：run_id 切换即清旧锚点。
  cache.updateLockedTargets(
    true, "run-104", {makeLockedTarget("peach_7", true, 0.60)});
  EXPECT_FALSE(cache.lockedTargetSnapshot("peach_1").has_value());
  ASSERT_TRUE(cache.lockedTargetSnapshot("peach_7").has_value());
  EXPECT_EQ(
    cache.lockedTargetSnapshot("peach_7")->harvest_run_id, "run-104");
}

// 非观测帧只刷新诊断透传（swinging/tracking_status），不刷新 received_s 与
// entry_pose——与 selected 缓存语义一致（安全门按 max_age 判陈旧兜底）。
TEST(TargetCache, LockedSetNonObservedFrameRefreshesDiagnosticsOnly)
{
  double now_s = 240.0;
  TargetCache cache([&now_s]() {return now_s;});
  auto observed = makeLockedTarget("peach_1");
  observed.swinging = false;
  observed.tracking_status = 0;  // OBSERVED
  cache.updateLockedTargets(true, "run-105", {observed});
  ASSERT_TRUE(cache.lockedTargetSnapshot("peach_1").has_value());
  EXPECT_DOUBLE_EQ(cache.lockedTargetSnapshot("peach_1")->received_s, 240.0);

  // 摆动中的出视野帧（非观测）：诊断投影更新，received_s 保持。
  now_s = 241.0;
  auto lost = makeLockedTarget("peach_1", false);
  lost.swinging = true;
  lost.tracking_status = 4;  // OUT_OF_VIEW
  cache.updateLockedTargets(true, "run-105", {lost});
  const auto target = cache.lockedTargetSnapshot("peach_1");
  ASSERT_TRUE(target.has_value());
  EXPECT_TRUE(target->swinging);
  EXPECT_EQ(target->tracking_status, 4);
  EXPECT_DOUBLE_EQ(target->received_s, 240.0);

  // 有效观测恢复：received_s 刷新，诊断投影随帧更新。
  now_s = 242.0;
  cache.updateLockedTargets(true, "run-105", {makeLockedTarget("peach_1")});
  EXPECT_DOUBLE_EQ(cache.lockedTargetSnapshot("peach_1")->received_s, 242.0);
  EXPECT_FALSE(cache.lockedTargetSnapshot("peach_1")->swinging);
}

// 锁定集新鲜帧等待谓词：与 waitForFreshTarget 同语义（新鲜即返回/超时/
// cancel），只是按指定 ID 查锁定集条目；条目被清空后不再满足。
TEST(TargetCache, WaitForFreshLockedTargetSemantics)
{
  double now_s = 250.0;
  TargetCache cache([&now_s]() {return now_s;});
  std::atomic_bool cancel{false};
  cache.updateLockedTargets(true, "run-106", {makeLockedTarget("peach_1")});

  // 已有有效样本晚于阈值：立即返回 true。
  EXPECT_TRUE(cache.waitForFreshLockedTarget("peach_1", 249.0, 0.01, cancel));
  // 有效样本不晚于阈值：无新帧时超时返回 false。
  EXPECT_FALSE(cache.waitForFreshLockedTarget("peach_1", 251.0, 0.05, cancel));
  // 未收录目标：恒不满足（不会在别目标的新帧上误唤醒）。
  cache.updateLockedTargets(true, "run-106", {makeLockedTarget("peach_2")});
  EXPECT_FALSE(cache.waitForFreshLockedTarget("peach_9", 0.0, 0.05, cancel));
  // 解锁清空后原目标立即不再满足。
  cache.updateLockedTargets(false, "run-106", {});
  EXPECT_FALSE(cache.waitForFreshLockedTarget("peach_1", 249.0, 0.01, cancel));
  // cancel 置位：立即返回 false。
  cache.updateLockedTargets(true, "run-106", {makeLockedTarget("peach_1")});
  cancel.store(true);
  EXPECT_FALSE(cache.waitForFreshLockedTarget("peach_1", 0.0, 5.0, cancel));
}

}  // namespace peach_approach_grasp
