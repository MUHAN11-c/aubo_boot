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
// 目标数据缓存纯核实现：四源 ID 一致性调和，语义与原节点订阅回调内联实现一致。
#include "peach_approach_grasp/target_cache.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace peach_approach_grasp
{
TargetCache::TargetCache(std::function<double()> clock_s)
: clock_s_(std::move(clock_s))
{
}

void TargetCache::updateSelectedTarget(const SelectedTargetUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // transient_local 的精化结果可能早于 volatile 的目标观测到达。首次获知目标时，
  // 若已缓存结果属于同一 target_id，必须保留；仅在确认 ID 冲突时清理旧目标数据。
  const bool target_changed = !target_.id.empty() && target_.id != update.selected_id;
  const bool refined_changed =
    (!refined_.id.empty() && refined_.id != update.selected_id) ||
    (!quality_.refined_target_id.empty() &&
    quality_.refined_target_id != update.selected_id);
  const bool decision_changed =
    !grasp_decision_target_id_.empty() &&
    grasp_decision_target_id_ != update.selected_id;
  if (target_changed || refined_changed || decision_changed) {
    target_.valid = false;
    refined_ = CachedRefined();
    quality_.refined_target_id.clear();
    quality_.refined_rmse_m = -1.0;
    quality_.refined_inlier_ratio = -1.0;
    quality_.refined_accept = false;
    quality_.grasp_allowed = false;
    grasp_decision_target_id_.clear();
  }
  target_.id = update.selected_id;
  target_.harvest_run_id = update.harvest_run_id;
  const bool has_anchor = nonzeroFinite(update.bottom) && nonzeroFinite(update.neck) &&
    nonzeroFinite(update.axis);
  if (has_anchor) {
    // 锚点几何即采用：LOST 帧携带的注册表记忆锚点同样可用（世界系身份记忆
    // 的意义所在），短暂不可见的目标保持可派发/可规划；观测新鲜度仍由
    // received_s 只在有效观测帧刷新来把关（安全门按 max_age 判陈旧）。
    target_.center = 0.5 * (update.bottom + update.neck);
    target_.initial_axis = update.axis.normalized();
    target_.suggested_travel_m = update.suggested_travel_m;
    target_.valid = true;
  }
  if (update.observed && has_anchor) {
    target_.initial_pose = update.entry_pose;
    // 仅在有效观测帧刷新时间戳：短暂检测闪烁保留最后有效样本（安全门按
    // max_age 判陈旧），真消失的目标会在 max_age 后按 stale 拒绝。
    target_.received_s = clock_s_();
  }
  quality_.selected_target_id = target_.id;
  cv_.notify_all();
}

void TargetCache::updateReconstructionDiagnostics(
  const ReconstructionDiagnosticsUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  quality_.reconstruction_target_id = update.target_id;
  quality_.reconstruction_state = update.state;
  quality_.captured_views = update.captured_views;
  observed_directions_ = update.view_directions;
  quality_.max_baseline_deg = update.max_baseline_deg;
  quality_.mean_nearest_baseline_deg = update.mean_nearest_baseline_deg;
  quality_.mean_depth_ratio = update.mean_depth_ratio;
  diagnostics_received_s_ = clock_s_();
  diagnostics_seen_ = true;
  quality_.data_age_s = 0.0;
  cv_.notify_all();
}

bool TargetCache::updateGraspDecision(const std::string & target_id, bool allowed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_.id.empty() && target_id != target_.id) {
    return false;
  }
  grasp_decision_target_id_ = target_id;
  quality_.grasp_allowed = allowed;
  cv_.notify_all();
  return true;
}

bool TargetCache::updateRefinedPose(const RefinedPoseUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (update.clear) {
    refined_ = CachedRefined();
    quality_.refined_target_id.clear();
    return true;
  }
  if (!target_.id.empty() && update.target_id != target_.id) {
    return false;
  }
  refined_ = CachedRefined();
  refined_.id = update.target_id;
  refined_.entry = update.entry;
  refined_.bottom = update.bottom;
  refined_.neck = update.neck;
  refined_.axis = update.axis;
  refined_.suggested_travel_m = update.suggested_travel_m;
  refined_.valid = nonzeroFinite(refined_.axis) && refined_.entry.allFinite();
  quality_.refined_target_id = update.target_id;
  quality_.refined_accept = update.accepted;
  cv_.notify_all();
  return true;
}

bool TargetCache::updateRefinedFitting(const RefinedFittingUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string expected_id = refined_.id.empty() ? target_.id : refined_.id;
  if (!expected_id.empty() && update.target_id != expected_id) {
    return false;
  }
  // 节点刚启动时可能尚未收到 volatile 目标观测。先按 target_id 缓存锁存的
  // 拟合指标，后续 updateSelectedTarget() 会保留同 ID 数据或清除冲突数据。
  quality_.refined_target_id = update.target_id;
  if (update.is_fruit) {
    quality_.refined_rmse_m = update.sphere_rms_m;
    quality_.refined_inlier_ratio = update.sphere_inlier_ratio;
  } else {
    quality_.refined_rmse_m = update.cylinder_rms_m;
    quality_.refined_inlier_ratio = update.cylinder_inlier_ratio;
  }
  quality_.refined_accept = update.accepted;
  cv_.notify_all();
  return true;
}

std::optional<CachedTarget> TargetCache::targetSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_.valid) {
    return std::nullopt;
  }
  return target_;
}

std::optional<CachedRefined> TargetCache::refinedSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!refined_.valid) {
    return std::nullopt;
  }
  return refined_;
}

QualitySnapshot TargetCache::qualitySnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  QualitySnapshot snapshot = quality_;
  if (diagnostics_seen_) {
    snapshot.data_age_s = std::max(0.0, clock_s_() - diagnostics_received_s_);
  }
  return snapshot;
}

std::string TargetCache::graspDecisionTarget() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return grasp_decision_target_id_;
}

std::vector<Eigen::Vector3d> TargetCache::observedDirections() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return observed_directions_;
}

TargetGateSample TargetCache::targetGateSample() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return TargetGateSample{target_.id, target_.valid, target_.received_s};
}

std::string TargetCache::expectedFittingTargetId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return refined_.id.empty() ? target_.id : refined_.id;
}

bool TargetCache::waitForNewView(
  std::size_t previous_views, double timeout_s, const std::atomic_bool & cancel) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, previous_views, &cancel]() {
      return cancel.load() || quality_.captured_views > previous_views;
    }) && !cancel.load();
}

bool TargetCache::waitForRefined(
  const std::string & target_id, double timeout_s,
  const std::atomic_bool & cancel) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, &target_id, &cancel]() {
      // 谓词以锁存精化位姿有效且 ID 匹配为准：refined_.id/valid 仅由
      // updateRefinedPose 写入；updateRefinedFitting 只写
      // quality_.refined_target_id，单独到达不再满足谓词（旧路径会让随后的
      // refinedSnapshot 为空 → 硬 FAILED 绕过降级链）。
      return cancel.load() ||
             (quality_.reconstruction_state == "READY" &&
             refined_.valid && refined_.id == target_id);
    }) && !cancel.load();
}

bool TargetCache::waitForFreshTarget(
  double after_s, double timeout_s, const std::atomic_bool & cancel) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, after_s, &cancel]() {
      return cancel.load() ||
             (target_.valid && target_.received_s > after_s);
    }) && !cancel.load();
}

void TargetCache::notifyAll()
{
  cv_.notify_all();
}

}  // namespace peach_approach_grasp
