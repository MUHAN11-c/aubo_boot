// 功能：观察/接触质量门默认实现。纯核，零 ROS。
#include "peach_manipulation/quality_gate.hpp"

#include <utility>

namespace peach_manipulation
{

QualityGate::QualityGate(QualityGateConfig config)
: config_(std::move(config))
{
}

GateResult QualityGate::commonIdentityGate(const QualitySnapshot & snapshot) const
{
  if (snapshot.selected_target_id.empty()) {
    return {false, "selected_target_missing"};
  }
  if (snapshot.reconstruction_target_id.empty()) {
    return {false, "reconstruction_unbound"};
  }
  if (snapshot.reconstruction_target_id != snapshot.selected_target_id) {
    return {false, "perception_reconstruction_id_mismatch"};
  }
  if (snapshot.data_age_s > config_.maximum_data_age_s) {
    return {false, "reconstruction_data_stale"};
  }
  return {true, "identity_and_freshness_ok"};
}

GateResult QualityGate::axisConsistencyGate(const QualitySnapshot & snapshot) const
{
  // 固定 35° 只诊断完全错轴；接触许可只信 GraspDecision.allowed。
  if (snapshot.axis_angle_deg > config_.maximum_axis_angle_deg) {
    return {true, "axis_mismatch_diagnostic_only"};
  }
  return {true, "axis_consistency_ok"};
}

GateResult QualityGate::readyToFinalize(const QualitySnapshot & snapshot) const
{
  const GateResult common = commonIdentityGate(snapshot);
  if (!common.allowed) {
    return common;
  }
  const std::size_t views = snapshot.station_count > 0 ?
    snapshot.station_count : snapshot.captured_views;
  if (views < config_.minimum_views) {
    return {false, "insufficient_views"};
  }
  if (snapshot.max_baseline_deg < config_.minimum_baseline_deg) {
    return {false, "insufficient_angular_baseline"};
  }
  if (snapshot.mean_nearest_baseline_deg <
    config_.minimum_mean_nearest_baseline_deg)
  {
    return {false, "insufficient_view_distribution"};
  }
  if (snapshot.mean_depth_ratio < config_.minimum_mean_depth_ratio) {
    return {false, "insufficient_depth_quality"};
  }
  return {true, "view_coverage_ready"};
}

GateResult QualityGate::readyToPreviewContact(const QualitySnapshot & snapshot) const
{
  // 预览只读取 finalize 后锁存的几何，不执行运动，因此不要求诊断消息持续刷新；
  // 真实执行仍走 readyToGrasp()，保留 maximum_data_age_s 时效门。
  if (snapshot.selected_target_id.empty()) {
    return {false, "selected_target_missing"};
  }
  if (snapshot.reconstruction_target_id.empty()) {
    return {false, "reconstruction_unbound"};
  }
  if (snapshot.reconstruction_target_id != snapshot.selected_target_id) {
    return {false, "perception_reconstruction_id_mismatch"};
  }
  if (snapshot.reconstruction_state != "READY") {
    return {false, "reconstruction_not_ready"};
  }
  if (snapshot.refined_target_id != snapshot.selected_target_id) {
    return {false, "refined_target_id_mismatch"};
  }
  const GateResult axis_gate = axisConsistencyGate(snapshot);
  if (!axis_gate.allowed) {
    return axis_gate;
  }
  if (!snapshot.refined_accept) {
    return {false, "refined_geometry_unavailable"};
  }
  if (!snapshot.grasp_allowed) {
    return {false, "refined_quality_not_allowed"};
  }
  return {true, "contact_preview_ready"};
}

GateResult QualityGate::readyToApproach(const QualitySnapshot & snapshot) const
{
  const GateResult common = commonIdentityGate(snapshot);
  if (!common.allowed) {
    return common;
  }
  if (snapshot.reconstruction_state != "READY") {
    return {false, "reconstruction_not_ready"};
  }
  if (snapshot.refined_target_id != snapshot.selected_target_id) {
    return {false, "refined_target_id_mismatch"};
  }
  const GateResult axis_gate = axisConsistencyGate(snapshot);
  if (!axis_gate.allowed) {
    return axis_gate;
  }
  if (!snapshot.refined_accept) {
    return {false, "refined_geometry_unavailable"};
  }
  return {true, "pregrasp_geometry_ready"};
}

GateResult QualityGate::readyToGrasp(const QualitySnapshot & snapshot) const
{
  const GateResult approach = readyToApproach(snapshot);
  if (!approach.allowed) {
    return approach;
  }
  if (!snapshot.grasp_allowed) {
    return {false, "refined_quality_not_allowed"};
  }
  return {true, "grasp_quality_ready"};
}

}  // namespace peach_manipulation
