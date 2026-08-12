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
#include "peach_approach_grasp/quality_gate.hpp"

#include <utility>

namespace peach_approach_grasp
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
  if (snapshot.reconstruction_target_id != snapshot.selected_target_id) {
    return {false, "perception_reconstruction_id_mismatch"};
  }
  if (snapshot.data_age_s > config_.maximum_data_age_s) {
    return {false, "reconstruction_data_stale"};
  }
  return {true, "identity_and_freshness_ok"};
}

GateResult QualityGate::readyToFinalize(const QualitySnapshot & snapshot) const
{
  const GateResult common = commonIdentityGate(snapshot);
  if (!common.allowed) {
    return common;
  }
  if (snapshot.captured_views < config_.minimum_views) {
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
  if (snapshot.reconstruction_target_id != snapshot.selected_target_id) {
    return {false, "perception_reconstruction_id_mismatch"};
  }
  if (snapshot.reconstruction_state != "READY") {
    return {false, "reconstruction_not_ready"};
  }
  if (snapshot.refined_target_id != snapshot.selected_target_id) {
    return {false, "refined_target_id_mismatch"};
  }
  if (!snapshot.refined_accept || !snapshot.grasp_allowed) {
    return {false, "refined_quality_not_allowed"};
  }
  if (snapshot.refined_rmse_m < 0.0 ||
    snapshot.refined_rmse_m > config_.maximum_refined_rmse_m)
  {
    return {false, "refined_rmse_out_of_bounds"};
  }
  if (snapshot.refined_inlier_ratio < config_.minimum_refined_inlier_ratio) {
    return {false, "refined_inlier_ratio_too_low"};
  }
  return {true, "contact_preview_ready"};
}

GateResult QualityGate::readyToGrasp(const QualitySnapshot & snapshot) const
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
  if (!snapshot.refined_accept || !snapshot.grasp_allowed) {
    return {false, "refined_quality_not_allowed"};
  }
  if (snapshot.refined_rmse_m < 0.0 ||
    snapshot.refined_rmse_m > config_.maximum_refined_rmse_m)
  {
    return {false, "refined_rmse_out_of_bounds"};
  }
  if (snapshot.refined_inlier_ratio < config_.minimum_refined_inlier_ratio) {
    return {false, "refined_inlier_ratio_too_low"};
  }
  return {true, "grasp_quality_ready"};
}

}  // namespace peach_approach_grasp
