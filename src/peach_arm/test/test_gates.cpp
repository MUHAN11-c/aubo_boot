// 功能：质量门四档 + 安全门（注入时钟）边界值测试。
#include "peach_arm/quality_gate.hpp"
#include "peach_arm/safety_gate.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

// 默认配置阈值：views>=2、基线>=8°、平均最近基线>=6°、深度比>=0.40、
// 时效<=3s、轴夹角上限 35°。goodSnapshot 全部踩在边界值上（等于阈值应过）。
peach_arm::QualitySnapshot goodSnapshot()
{
  peach_arm::QualitySnapshot snapshot;
  snapshot.selected_target_id = "t1";
  snapshot.reconstruction_target_id = "t1";
  snapshot.refined_target_id = "t1";
  snapshot.reconstruction_state = "READY";
  snapshot.captured_views = 2;
  snapshot.station_count = 0;
  snapshot.max_baseline_deg = 8.0;
  snapshot.mean_nearest_baseline_deg = 6.0;
  snapshot.mean_depth_ratio = 0.40;
  snapshot.refined_rmse_m = 0.008;
  snapshot.refined_inlier_ratio = 0.85;
  snapshot.data_age_s = 3.0;
  snapshot.axis_angle_deg = 10.0;
  snapshot.refined_accept = true;
  snapshot.grasp_allowed = true;
  return snapshot;
}

}  // namespace

// ---------- readyToFinalize：覆盖证据与身份/时效门 ----------

TEST(QualityGate, ReadyToFinalizeBoundaryValues)
{
  const peach_arm::QualityGate gate;
  const peach_arm::QualitySnapshot good = goodSnapshot();
  // 全部踩边界（等于阈值）→ 过。
  const peach_arm::GateResult pass = gate.readyToFinalize(good);
  EXPECT_TRUE(pass.allowed);
  EXPECT_EQ(pass.reason, "view_coverage_ready");
  // 逐项降到阈值以下 → 各自的稳定原因串。
  peach_arm::QualitySnapshot views = good;
  views.captured_views = 1;
  EXPECT_FALSE(gate.readyToFinalize(views).allowed);
  EXPECT_EQ(gate.readyToFinalize(views).reason, "insufficient_views");
  peach_arm::QualitySnapshot baseline = good;
  baseline.max_baseline_deg = 7.99;
  EXPECT_EQ(
    gate.readyToFinalize(baseline).reason, "insufficient_angular_baseline");
  peach_arm::QualitySnapshot nearest = good;
  nearest.mean_nearest_baseline_deg = 5.99;
  EXPECT_EQ(
    gate.readyToFinalize(nearest).reason, "insufficient_view_distribution");
  peach_arm::QualitySnapshot depth = good;
  depth.mean_depth_ratio = 0.399;
  EXPECT_EQ(gate.readyToFinalize(depth).reason, "insufficient_depth_quality");
  // 机位数优先于积分帧数：station_count=1 覆盖 captured_views=2。
  peach_arm::QualitySnapshot stations = good;
  stations.station_count = 1;
  EXPECT_EQ(gate.readyToFinalize(stations).reason, "insufficient_views");
}

TEST(QualityGate, ReadyToFinalizeIdentityAndFreshness)
{
  const peach_arm::QualityGate gate;
  const peach_arm::QualitySnapshot good = goodSnapshot();
  peach_arm::QualitySnapshot no_selected = good;
  no_selected.selected_target_id.clear();
  EXPECT_EQ(
    gate.readyToFinalize(no_selected).reason, "selected_target_missing");
  peach_arm::QualitySnapshot unbound = good;
  unbound.reconstruction_target_id.clear();
  EXPECT_EQ(gate.readyToFinalize(unbound).reason, "reconstruction_unbound");
  peach_arm::QualitySnapshot mismatch = good;
  mismatch.reconstruction_target_id = "t2";
  EXPECT_EQ(
    gate.readyToFinalize(mismatch).reason,
    "perception_reconstruction_id_mismatch");
  peach_arm::QualitySnapshot stale = good;
  stale.data_age_s = 3.5;
  EXPECT_EQ(gate.readyToFinalize(stale).reason, "reconstruction_data_stale");
}

// ---------- readyToPreviewContact：只读锁存几何，不查时效 ----------

TEST(QualityGate, PreviewContactIgnoresAgeButRequiresDecision)
{
  const peach_arm::QualityGate gate;
  peach_arm::QualitySnapshot preview = goodSnapshot();
  // 预览读取 finalize 后的锁存几何：数据陈旧不拦（真实执行仍走 grasp 门）。
  preview.data_age_s = 100.0;
  const peach_arm::GateResult pass = gate.readyToPreviewContact(preview);
  EXPECT_TRUE(pass.allowed);
  EXPECT_EQ(pass.reason, "contact_preview_ready");
  // 重建未 READY / 精化身份不符 / 精化未接受 / 决策未许可 逐一拒。
  peach_arm::QualitySnapshot busy = preview;
  busy.reconstruction_state = "REFINING";
  EXPECT_EQ(
    gate.readyToPreviewContact(busy).reason, "reconstruction_not_ready");
  peach_arm::QualitySnapshot other = preview;
  other.refined_target_id = "t2";
  EXPECT_EQ(
    gate.readyToPreviewContact(other).reason, "refined_target_id_mismatch");
  peach_arm::QualitySnapshot unrefined = preview;
  unrefined.refined_accept = false;
  EXPECT_EQ(
    gate.readyToPreviewContact(unrefined).reason,
    "refined_geometry_unavailable");
  peach_arm::QualitySnapshot denied = preview;
  denied.grasp_allowed = false;
  EXPECT_EQ(
    gate.readyToPreviewContact(denied).reason,
    "refined_quality_not_allowed");
  // 轴夹角超 35° 只是诊断不拒（allowed 恒 true）；误差进诊断字段透传，
  // reason 令牌不受影响（W13-B）。
  peach_arm::QualitySnapshot bad_axis = preview;
  bad_axis.axis_angle_deg = 40.0;
  const peach_arm::GateResult diagnostic = gate.readyToPreviewContact(bad_axis);
  EXPECT_TRUE(diagnostic.allowed);
  EXPECT_EQ(diagnostic.reason, "contact_preview_ready");
  EXPECT_DOUBLE_EQ(diagnostic.axis_angle_deg, 40.0);
  EXPECT_TRUE(diagnostic.axis_mismatch);
}

// ---------- readyToApproach / readyToGrasp：几何门与接触许可分离 ----------

TEST(QualityGate, ApproachRequiresGeometryNotDecision)
{
  const peach_arm::QualityGate gate;
  const peach_arm::QualitySnapshot good = goodSnapshot();
  EXPECT_TRUE(gate.readyToApproach(good).allowed);
  EXPECT_EQ(gate.readyToApproach(good).reason, "pregrasp_geometry_ready");
  // 预抓取不要求 GraspDecision.allowed：未许可仍可接近。
  peach_arm::QualitySnapshot denied = good;
  denied.grasp_allowed = false;
  const peach_arm::GateResult approach = gate.readyToApproach(denied);
  EXPECT_TRUE(approach.allowed);
  EXPECT_TRUE(gate.readyToGrasp(good).allowed);
  EXPECT_EQ(gate.readyToGrasp(good).reason, "grasp_quality_ready");
  EXPECT_FALSE(gate.readyToGrasp(denied).allowed);
  EXPECT_EQ(
    gate.readyToGrasp(denied).reason, "refined_quality_not_allowed");
  // 接近门仍要求数据时效与重建 READY（共享身份门）。
  peach_arm::QualitySnapshot stale = denied;
  stale.data_age_s = 3.5;
  EXPECT_EQ(
    gate.readyToApproach(stale).reason, "reconstruction_data_stale");
  peach_arm::QualitySnapshot unrefined = good;
  unrefined.refined_accept = false;
  EXPECT_EQ(
    gate.readyToApproach(unrefined).reason, "refined_geometry_unavailable");
}

// ---------- SafetyGate：注入时钟下的陈旧/新鲜判定 ----------

TEST(SafetyGate, RobotReadyFreshStaleAndNotReady)
{
  double now = 100.0;
  peach_arm::SafetyGateConfig config;
  config.robot_status_max_age_s = 1.0;
  const peach_arm::SafetyGate gate(
    config, [&now] {return now;});
  std::string reason;
  // 新鲜（恰好压线 1.0s）且运动就绪 → 过。
  peach_arm::RobotStatusSample fresh;
  fresh.received = true;
  fresh.received_s = 99.0;
  fresh.drives_powered = true;
  fresh.motion_possible = true;
  EXPECT_TRUE(gate.robotReady(fresh, reason));
  // 陈旧 1.5s → 拒。
  peach_arm::RobotStatusSample stale = fresh;
  stale.received_s = 98.5;
  EXPECT_FALSE(gate.robotReady(stale, reason));
  EXPECT_EQ(reason, "robot_status_stale");
  // 从未收到 → 拒。
  peach_arm::RobotStatusSample missing;
  EXPECT_FALSE(gate.robotReady(missing, reason));
  EXPECT_EQ(reason, "robot_status_missing");
  // 急停/错误/未上电/不可运动 → 同一原因串拒。
  peach_arm::RobotStatusSample stopped = fresh;
  stopped.e_stopped = true;
  EXPECT_FALSE(gate.robotReady(stopped, reason));
  EXPECT_EQ(reason, "robot_status_not_motion_ready");
  peach_arm::RobotStatusSample unpowered = fresh;
  unpowered.drives_powered = false;
  EXPECT_FALSE(gate.robotReady(unpowered, reason));
  EXPECT_EQ(reason, "robot_status_not_motion_ready");
  // require_robot_status=false：门关闭直接放行。
  peach_arm::SafetyGateConfig off = config;
  off.require_robot_status = false;
  const peach_arm::SafetyGate gate_off(off, [&now] {return now;});
  EXPECT_TRUE(gate_off.robotReady(missing, reason));
}

TEST(SafetyGate, TargetReadyIdentityValidityAndAge)
{
  double now = 100.0;
  peach_arm::SafetyGateConfig config;
  config.target_observation_max_age_s = 3.0;
  // 运行期会调 set_target_observation_max_age_s（订阅回调语义），不能 const。
  peach_arm::SafetyGate gate(
    config, [&now] {return now;});
  std::string reason;
  peach_arm::TargetGateSample sample;
  sample.id = "t1";
  sample.valid = true;
  sample.received_s = 97.0;
  EXPECT_TRUE(gate.targetReady(sample, "t1", reason));
  // 身份不一致（含空 ID）→ 拒。
  EXPECT_FALSE(gate.targetReady(sample, "t2", reason));
  EXPECT_EQ(reason, "selected_target_changed");
  // 观测无效 → 拒。
  peach_arm::TargetGateSample invalid = sample;
  invalid.valid = false;
  EXPECT_FALSE(gate.targetReady(invalid, "t1", reason));
  EXPECT_EQ(reason, "selected_target_not_observed");
  // 陈旧（3.5s > 3.0s）→ 拒；运行期收紧到 0.5s 后 1.0s 前的样本也陈旧。
  peach_arm::TargetGateSample stale = sample;
  stale.received_s = 96.5;
  EXPECT_FALSE(gate.targetReady(stale, "t1", reason));
  EXPECT_EQ(reason, "selected_target_stale");
  gate.set_target_observation_max_age_s(0.5);
  EXPECT_FALSE(gate.targetReady(sample, "t1", reason));
  EXPECT_EQ(reason, "selected_target_stale");
}

TEST(SafetyGate, AdaptiveTimeoutClampsToFloorAndCap)
{
  // 帧率自适应超时：mult*ema+margin 夹在 [floor, cap]。
  EXPECT_NEAR(
    peach_arm::adaptive_timeout_s(0.1, 5.0, 0.5, 1.0, 10.0), 1.0, 1e-12);
  EXPECT_NEAR(
    peach_arm::adaptive_timeout_s(1.0, 5.0, 0.5, 1.0, 10.0), 5.5, 1e-12);
  EXPECT_NEAR(
    peach_arm::adaptive_timeout_s(3.0, 5.0, 0.5, 1.0, 10.0), 10.0, 1e-12);
}
