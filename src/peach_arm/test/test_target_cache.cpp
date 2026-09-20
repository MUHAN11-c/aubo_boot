// 功能：目标缓存 ID 调和、快照投影与短超时等待测试（注入时钟）。
#include "peach_arm/target_cache.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <atomic>
#include <string>
#include <vector>

namespace
{

// 有效观测帧更新（t1，竖直袋：底 (0,0,0.4)、颈 (0,0,0.6)）。
peach_arm::SelectedTargetUpdate observedUpdate(
  const std::string & id = "t1")
{
  peach_arm::SelectedTargetUpdate update;
  update.selected_id = id;
  update.harvest_run_id = "run1";
  update.observed = true;
  update.bottom = Eigen::Vector3d(0.0, 0.0, 0.4);
  update.neck = Eigen::Vector3d(0.0, 0.0, 0.6);
  update.axis = Eigen::Vector3d::UnitZ();
  update.suggested_travel_m = 0.05;
  update.tracking_status = 0;  // OBSERVED
  return update;
}

// 有效精化位姿更新（与 selected 同 ID）。
peach_arm::RefinedPoseUpdate refinedPoseUpdate(const std::string & id = "t1")
{
  peach_arm::RefinedPoseUpdate update;
  update.target_id = id;
  update.entry = Eigen::Vector3d(0.1, 0.2, 0.7);
  update.bottom = Eigen::Vector3d(0.0, 0.0, 0.4);
  update.neck = Eigen::Vector3d(0.0, 0.0, 0.6);
  update.axis = Eigen::Vector3d::UnitX();
  update.suggested_travel_m = 0.18;
  update.bag_diameter_upper_m = 0.06;
  update.accepted = true;
  return update;
}

}  // namespace

// ---------- updateSelectedTarget：锚点采用与观测帧时间戳 ----------

TEST(TargetCache, SelectedTargetStoresAnchorAndStamp)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  const auto snapshot = cache.targetSnapshot();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->id, "t1");
  EXPECT_EQ(snapshot->harvest_run_id, "run1");
  EXPECT_TRUE(
    snapshot->center.isApprox(Eigen::Vector3d(0.0, 0.0, 0.5), 1e-12));
  EXPECT_TRUE(snapshot->initial_axis.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(snapshot->valid);
  EXPECT_NEAR(snapshot->received_s, 100.0, 1e-12);
  EXPECT_NEAR(snapshot->suggested_travel_m, 0.05, 1e-12);
  const peach_arm::TargetGateSample gate = cache.targetGateSample();
  EXPECT_EQ(gate.id, "t1");
  EXPECT_TRUE(gate.valid);
  EXPECT_NEAR(gate.received_s, 100.0, 1e-12);
}

TEST(TargetCache, InvalidAnchorKeepsTargetInvalid)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  peach_arm::SelectedTargetUpdate update = observedUpdate();
  update.bottom = Eigen::Vector3d::Zero();
  update.neck = Eigen::Vector3d::Zero();
  cache.updateSelectedTarget(update);
  EXPECT_FALSE(cache.targetSnapshot().has_value());
  EXPECT_FALSE(cache.targetGateSample().valid);
}

// ---------- ID 调和：同 ID 保留，冲突清空精化/决策缓存 ----------

TEST(TargetCache, SameIdRefreshKeepsRefinedAndDecision)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  ASSERT_TRUE(cache.updateRefinedPose(refinedPoseUpdate()));
  ASSERT_TRUE(cache.updateGraspDecision("t1", true));
  EXPECT_EQ(cache.graspDecisionTarget(), "t1");
  EXPECT_TRUE(cache.qualitySnapshot().grasp_allowed);
  // 同 ID 刷新（非观测帧，携带有效锚点）：精化与决策保留。
  now = 101.0;
  peach_arm::SelectedTargetUpdate refresh = observedUpdate();
  refresh.observed = false;
  cache.updateSelectedTarget(refresh);
  const auto refined = cache.refinedSnapshot();
  ASSERT_TRUE(refined.has_value());
  EXPECT_EQ(refined->id, "t1");
  EXPECT_EQ(cache.graspDecisionTarget(), "t1");
  EXPECT_TRUE(cache.qualitySnapshot().grasp_allowed);
  // received_s 只在有效观测帧刷新：非观测帧不抬新鲜度。
  const auto snapshot = cache.targetSnapshot();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_NEAR(snapshot->received_s, 100.0, 1e-12);
}

TEST(TargetCache, IdConflictClearsRefinedAndDecision)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  cache.updateRefinedPose(refinedPoseUpdate());
  cache.updateGraspDecision("t1", true);
  // 换目标：旧精化/决策全部作废。
  now = 102.0;
  cache.updateSelectedTarget(observedUpdate("t2"));
  EXPECT_FALSE(cache.refinedSnapshot().has_value());
  EXPECT_EQ(cache.graspDecisionTarget(), "");
  const peach_arm::QualitySnapshot quality = cache.qualitySnapshot();
  EXPECT_EQ(quality.selected_target_id, "t2");
  EXPECT_TRUE(quality.refined_target_id.empty());
  EXPECT_FALSE(quality.refined_accept);
  EXPECT_FALSE(quality.grasp_allowed);
  EXPECT_NEAR(quality.refined_rmse_m, -1.0, 1e-12);
  const auto snapshot = cache.targetSnapshot();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->id, "t2");
}

TEST(TargetCache, RefinedBeforeSelectedIsRetainedOnSameId)
{
  // transient_local 精化先于 volatile 观测到达：同 ID 必须保留。
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  ASSERT_TRUE(cache.updateRefinedPose(refinedPoseUpdate("t1")));
  cache.updateSelectedTarget(observedUpdate("t1"));
  const auto refined = cache.refinedSnapshot();
  ASSERT_TRUE(refined.has_value());
  EXPECT_EQ(refined->id, "t1");
  EXPECT_NEAR(refined->suggested_travel_m, 0.18, 1e-12);
  EXPECT_TRUE(cache.qualitySnapshot().refined_accept);
}

TEST(TargetCache, GraspDecisionForOtherTargetIgnored)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  EXPECT_FALSE(cache.updateGraspDecision("t9", true));
  EXPECT_TRUE(cache.targetSnapshot().has_value());
  EXPECT_EQ(cache.graspDecisionTarget(), "");
  EXPECT_FALSE(cache.qualitySnapshot().grasp_allowed);
}

// ---------- 精化位姿投影与有效性 ----------

TEST(TargetCache, RefinedSnapshotProjection)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  ASSERT_TRUE(cache.updateRefinedPose(refinedPoseUpdate()));
  const auto refined = cache.refinedSnapshot();
  ASSERT_TRUE(refined.has_value());
  EXPECT_TRUE(refined->entry.isApprox(Eigen::Vector3d(0.1, 0.2, 0.7), 1e-12));
  EXPECT_TRUE(
    refined->bottom.isApprox(Eigen::Vector3d(0.0, 0.0, 0.4), 1e-12));
  EXPECT_TRUE(refined->neck.isApprox(Eigen::Vector3d(0.0, 0.0, 0.6), 1e-12));
  EXPECT_TRUE(refined->axis.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(refined->bag_diameter_upper_m, 0.06, 1e-12);
  EXPECT_TRUE(refined->valid);
  // 轴无效（零向量）→ 精化不可用；clear 显式清空。
  peach_arm::RefinedPoseUpdate bad_axis = refinedPoseUpdate();
  bad_axis.axis = Eigen::Vector3d::Zero();
  ASSERT_TRUE(cache.updateRefinedPose(bad_axis));
  EXPECT_FALSE(cache.refinedSnapshot().has_value());
  peach_arm::RefinedPoseUpdate clear;
  clear.clear = true;
  ASSERT_TRUE(cache.updateRefinedPose(clear));
  EXPECT_FALSE(cache.refinedSnapshot().has_value());
}

// ---------- 拟合指标调和：球/柱分档与期望 ID ----------

TEST(TargetCache, RefinedFittingSelectsSphereOrCylinder)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  peach_arm::RefinedFittingUpdate update;
  update.target_id = "t1";
  update.is_fruit = true;
  update.sphere_rms_m = 0.006;
  update.sphere_inlier_ratio = 0.9;
  update.cylinder_rms_m = 0.02;
  update.cylinder_inlier_ratio = 0.5;
  update.accepted = true;
  ASSERT_TRUE(cache.updateRefinedFitting(update));
  peach_arm::QualitySnapshot quality = cache.qualitySnapshot();
  EXPECT_NEAR(quality.refined_rmse_m, 0.006, 1e-12);
  EXPECT_NEAR(quality.refined_inlier_ratio, 0.9, 1e-12);
  EXPECT_TRUE(quality.refined_accept);
  update.is_fruit = false;
  ASSERT_TRUE(cache.updateRefinedFitting(update));
  quality = cache.qualitySnapshot();
  EXPECT_NEAR(quality.refined_rmse_m, 0.02, 1e-12);
  EXPECT_NEAR(quality.refined_inlier_ratio, 0.5, 1e-12);
  // 期望 ID：精化优先、selected 兜底；非期望目标的指标被忽略。
  EXPECT_EQ(cache.expectedFittingTargetId(), "t1");
  update.target_id = "t9";
  EXPECT_FALSE(cache.updateRefinedFitting(update));
  // clear 复位指标（W5-13：置 -1=无效，不再误投影成 0=完美拟合）。
  peach_arm::RefinedFittingUpdate clear;
  clear.clear = true;
  ASSERT_TRUE(cache.updateRefinedFitting(clear));
  quality = cache.qualitySnapshot();
  EXPECT_NEAR(quality.refined_rmse_m, -1.0, 1e-12);
  EXPECT_NEAR(quality.refined_inlier_ratio, -1.0, 1e-12);
  EXPECT_FALSE(quality.refined_accept);
}

// ---------- qualitySnapshot 投影：诊断字段、时效与轴夹角 ----------

TEST(TargetCache, QualitySnapshotProjectsDiagnosticsAndAge)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  cache.updateSelectedTarget(observedUpdate());
  cache.updateRefinedPose(refinedPoseUpdate());
  now = 200.0;
  peach_arm::ReconstructionDiagnosticsUpdate diagnostics;
  diagnostics.target_id = "t1";
  diagnostics.state = "READY";
  diagnostics.captured_views = 5;
  diagnostics.max_baseline_deg = 10.0;
  diagnostics.mean_nearest_baseline_deg = 7.0;
  diagnostics.mean_depth_ratio = 0.5;
  diagnostics.view_directions = {
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ()};
  cache.updateReconstructionDiagnostics(diagnostics);
  now = 202.0;
  const peach_arm::QualitySnapshot quality = cache.qualitySnapshot();
  EXPECT_EQ(quality.reconstruction_target_id, "t1");
  EXPECT_EQ(quality.reconstruction_state, "READY");
  EXPECT_EQ(quality.captured_views, 5U);
  EXPECT_EQ(quality.station_count, 3U);
  EXPECT_NEAR(quality.max_baseline_deg, 10.0, 1e-12);
  EXPECT_NEAR(quality.mean_nearest_baseline_deg, 7.0, 1e-12);
  EXPECT_NEAR(quality.mean_depth_ratio, 0.5, 1e-12);
  EXPECT_NEAR(quality.data_age_s, 2.0, 1e-9);
  // 观测轴 +Z vs 精化轴 +X → 轴夹角 90°。
  EXPECT_NEAR(quality.axis_angle_deg, 90.0, 1e-6);
  EXPECT_EQ(cache.observedDirections().size(), 3U);
  // 精化失效后轴夹角回退不可算（-1）。
  peach_arm::RefinedPoseUpdate clear;
  clear.clear = true;
  cache.updateRefinedPose(clear);
  EXPECT_NEAR(cache.qualitySnapshot().axis_angle_deg, -1.0, 1e-12);
}

// ---------- 锁定集锚点缓存 ----------

TEST(TargetCache, LockedTargetsRefreshClearAndRunBoundary)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  std::vector<peach_arm::LockedTargetUpdate> updates;
  peach_arm::LockedTargetUpdate first;
  first.target_id = "a";
  first.observed = true;
  first.bottom = Eigen::Vector3d(0.1, 0.0, 0.4);
  first.neck = Eigen::Vector3d(0.1, 0.0, 0.6);
  first.axis = Eigen::Vector3d::UnitZ();
  updates.push_back(first);
  peach_arm::LockedTargetUpdate invalid;
  invalid.target_id = "b";
  updates.push_back(invalid);
  cache.updateLockedTargets(true, "run1", updates);

  const auto a = cache.lockedTargetSnapshot("a");
  ASSERT_TRUE(a.has_value());
  EXPECT_EQ(a->id, "a");
  EXPECT_TRUE(a->center.isApprox(Eigen::Vector3d(0.1, 0.0, 0.5), 1e-12));
  EXPECT_TRUE(a->valid);
  EXPECT_NEAR(a->received_s, 100.0, 1e-12);
  EXPECT_FALSE(cache.lockedTargetSnapshot("b").has_value());
  // 安全门样本：命中给 id/valid；未命中给空 ID（身份不匹配拒绝）。
  const peach_arm::TargetGateSample gate = cache.lockedTargetGateSample("a");
  EXPECT_EQ(gate.id, "a");
  EXPECT_TRUE(gate.valid);
  EXPECT_TRUE(cache.lockedTargetGateSample("zzz").id.empty());
  // 邻果中心只含有效条目、排除 exclude_id。
  EXPECT_TRUE(cache.lockedNeighborCenters("a").empty());

  // 非观测帧携带锚点：中心刷新、received_s 不抬。
  now = 101.0;
  peach_arm::LockedTargetUpdate moved = first;
  moved.observed = false;
  moved.bottom = Eigen::Vector3d(0.2, 0.0, 0.4);
  moved.neck = Eigen::Vector3d(0.2, 0.0, 0.6);
  cache.updateLockedTargets(true, "run1", {moved});
  const auto a2 = cache.lockedTargetSnapshot("a");
  ASSERT_TRUE(a2.has_value());
  EXPECT_TRUE(a2->center.isApprox(Eigen::Vector3d(0.2, 0.0, 0.5), 1e-12));
  EXPECT_NEAR(a2->received_s, 100.0, 1e-12);

  // 跨批次身份不复用：run 变更后旧条目作废，仅剩新帧携带的目标。
  peach_arm::LockedTargetUpdate other;
  other.target_id = "d";
  other.bottom = Eigen::Vector3d(0.3, 0.0, 0.4);
  other.neck = Eigen::Vector3d(0.3, 0.0, 0.6);
  other.axis = Eigen::Vector3d::UnitZ();
  cache.updateLockedTargets(true, "run2", {other});
  EXPECT_FALSE(cache.lockedTargetSnapshot("a").has_value());
  EXPECT_TRUE(cache.lockedTargetSnapshot("d").has_value());
  EXPECT_EQ(cache.lockedNeighborCenters("d").size(), 0U);
  // 未锁定（target_set_locked=false）：整表清空。
  cache.updateLockedTargets(false, "", {});
  EXPECT_FALSE(cache.lockedTargetSnapshot("d").has_value());
}

// ---------- waitForNewView：短超时路径（不测长等待） ----------

TEST(TargetCache, WaitForNewViewTimesOutAndSatisfiesFast)
{
  double now = 100.0;
  peach_arm::TargetCache cache([&now] {return now;});
  std::atomic_bool cancel{false};
  // 无新帧：50 ms 超时返回 false。
  EXPECT_FALSE(cache.waitForNewView(0, 0.05, cancel));
  // 谓词已满足：立即返回 true，不占等待预算。
  peach_arm::ReconstructionDiagnosticsUpdate diagnostics;
  diagnostics.target_id = "t1";
  diagnostics.captured_views = 1;
  cache.updateReconstructionDiagnostics(diagnostics);
  EXPECT_TRUE(cache.waitForNewView(0, 0.05, cancel));
  // cancel 预置：立即 false。
  std::atomic_bool cancelled{true};
  EXPECT_FALSE(cache.waitForNewView(99, 0.05, cancelled));
}
