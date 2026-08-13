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

}  // namespace peach_approach_grasp
