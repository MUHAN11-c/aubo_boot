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
#include "core.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// === quality_gate.cpp ===
namespace peach_manipulation_skills
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

}  // namespace peach_manipulation_skills

// === safety_gate.cpp ===
namespace peach_manipulation_skills
{
SafetyGate::SafetyGate(SafetyGateConfig config, std::function<double()> clock_s)
: config_(config), clock_s_(std::move(clock_s))
{
}

bool SafetyGate::robotReady(const RobotStatusSample & sample, std::string & reason) const
{
  if (!config_.require_robot_status) {
    return true;
  }
  if (!sample.received) {
    reason = "robot_status_missing";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.robot_status_max_age_s) {
    reason = "robot_status_stale";
    return false;
  }
  if (sample.e_stopped || sample.in_error || !sample.drives_powered ||
    !sample.motion_possible)
  {
    reason = "robot_status_not_motion_ready";
    return false;
  }
  return true;
}

bool SafetyGate::targetReady(
  const TargetGateSample & sample, const std::string & target_id,
  std::string & reason) const
{
  if (sample.id != target_id) {
    reason = "selected_target_changed";
    return false;
  }
  if (!sample.valid) {
    reason = "selected_target_not_observed";
    return false;
  }
  if (clock_s_() - sample.received_s > config_.target_observation_max_age_s) {
    reason = "selected_target_stale";
    return false;
  }
  return true;
}

}  // namespace peach_manipulation_skills

// === view_planner.cpp ===
namespace peach_manipulation_skills
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

Eigen::Vector3d safeUnit(const Eigen::Vector3d & value, const Eigen::Vector3d & fallback)
{
  if (!value.allFinite() || value.norm() < 1.0e-9) {
    return fallback;
  }
  return value.normalized();
}
Eigen::Vector2d visibilityDesired(
  const ViewContext & context,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & side,
  const Eigen::Vector3d & up)
{
  Eigen::Vector2d desired = Eigen::Vector2d::Zero();
  const int width = std::max(1, context.image_width);
  const int height = std::max(1, context.image_height);
  if (context.bbox_valid && context.bbox_w > 0 && context.bbox_h > 0) {
    const double cx = static_cast<double>(context.bbox_x) +
      0.5 * static_cast<double>(context.bbox_w);
    const double cy = static_cast<double>(context.bbox_y) +
      0.5 * static_cast<double>(context.bbox_h);
    desired.x() += (cx - 0.5 * width) / (0.5 * width);
    desired.y() += (cy - 0.5 * height) / (0.5 * height);
    constexpr double kMarginPx = 8.0;
    const double clip_left = std::max(
      0.0, kMarginPx - static_cast<double>(context.bbox_x));
    const double clip_right = std::max(
      0.0, static_cast<double>(context.bbox_x + context.bbox_w) -
      (width - kMarginPx));
    const double clip_top = std::max(
      0.0, kMarginPx - static_cast<double>(context.bbox_y));
    const double clip_bottom = std::max(
      0.0, static_cast<double>(context.bbox_y + context.bbox_h) -
      (height - kMarginPx));
    // 框贴边说明袋/果被裁切：相机沿光学 +X/+Y 移动才能把裁掉的一侧纳入画面。
    desired.x() += 1.5 * (clip_right - clip_left) / width;
    desired.y() += 1.5 * (clip_bottom - clip_top) / height;
  }
  for (const auto & neighbor : context.neighbor_centers) {
    const Eigen::Vector3d rel = neighbor - target;
    desired.x() += 0.35 * rel.dot(side);
    desired.y() += 0.35 * rel.dot(up);
  }
  if (desired.norm() < 1.0e-6) {
    desired.x() = 1.0;
  }
  return desired.normalized();
}

bool bboxTooSmall(const ViewContext & context)
{
  if (!context.bbox_valid || context.bbox_w <= 0 || context.bbox_h <= 0) {
    return false;
  }
  const double area = static_cast<double>(context.bbox_w) *
    static_cast<double>(context.bbox_h);
  const double image = static_cast<double>(
    std::max(1, context.image_width) * std::max(1, context.image_height));
  return area / image < 0.04;
}
}  // namespace

double angleDegrees(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const Eigen::Vector3d a = safeUnit(first, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d b = safeUnit(second, Eigen::Vector3d::UnitX());
  const double dot = std::clamp(a.dot(b), -1.0, 1.0);
  return std::acos(dot) * 180.0 / kPi;
}

ViewPlanner::ViewPlanner(ViewPlannerConfig config)
: config_(std::move(config))
{
}

Eigen::Matrix3d ViewPlanner::lookAtOptical(
  const Eigen::Vector3d & camera_position,
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & world_up)
{
  const Eigen::Vector3d optical_z = safeUnit(
    target - camera_position, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d down = -safeUnit(world_up, Eigen::Vector3d::UnitZ());
  if (std::abs(down.dot(optical_z)) > 0.97) {
    down = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d optical_x = safeUnit(
    down.cross(optical_z), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d optical_y = safeUnit(
    optical_z.cross(optical_x), Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rotation;
  rotation.col(0) = optical_x;
  rotation.col(1) = optical_y;
  rotation.col(2) = optical_z;
  return rotation;
}

Eigen::Matrix3d ViewPlanner::toolOrientation(
  const Eigen::Vector3d & approach_axis,
  const Eigen::Vector3d & preferred_x)
{
  const Eigen::Vector3d z_axis = safeUnit(approach_axis, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d x_axis = preferred_x - preferred_x.dot(z_axis) * z_axis;
  if (x_axis.norm() < 1.0e-6) {
    const Eigen::Vector3d fallback =
      std::abs(z_axis.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    x_axis = fallback - fallback.dot(z_axis) * z_axis;
  }
  x_axis.normalize();
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return rotation;
}

std::vector<ViewCandidate> ViewPlanner::generate(
  const Eigen::Vector3d & target,
  const Eigen::Vector3d & current_camera_position,
  const std::vector<Eigen::Vector3d> & observed_directions) const
{
  ViewContext context;
  context.target = target;
  context.current_camera_position = current_camera_position;
  context.observed_directions = observed_directions;
  return generate(context);
}

std::vector<ViewCandidate> ViewPlanner::generate(const ViewContext & context) const
{
  const Eigen::Vector3d & target = context.target;
  const Eigen::Vector3d & current_camera_position = context.current_camera_position;
  const std::vector<Eigen::Vector3d> & observed_directions =
    context.observed_directions;
  const Eigen::Vector3d front = safeUnit(
    current_camera_position - target, Eigen::Vector3d::UnitX());
  Eigen::Vector3d side = Eigen::Vector3d::UnitZ().cross(front);
  if (side.norm() < 1.0e-6) {
    side = Eigen::Vector3d::UnitY();
  }
  side.normalize();
  const Eigen::Vector3d up = front.cross(side).normalized();
  std::vector<Eigen::Vector3d> observed = observed_directions;
  if (observed.empty()) {
    observed.push_back(front);
  }

  const double current_radius = (current_camera_position - target).norm();
  const double radius0 = std::clamp(
    current_radius, config_.minimum_radius_m, 2.0);
  const bool want_closer = bboxTooSmall(context) &&
    radius0 > config_.minimum_radius_m + 0.5 * config_.radial_step_m;
  const Eigen::Vector2d desired = visibilityDesired(context, target, side, up);

  std::vector<ViewCandidate> result;
  const int azimuth_steps = 1;
  const int elevation_steps = config_.elevation_limit_deg > 1.0e-6 ? 1 : 0;
  const int layer_count = want_closer ? 2 : 1;
  for (int layer = 0; layer < layer_count; ++layer) {
    const double radius = std::max(
      config_.minimum_radius_m, radius0 - layer * config_.radial_step_m);
    for (int azimuth_index = -azimuth_steps;
      azimuth_index <= azimuth_steps; ++azimuth_index)
    {
      for (int elevation_index = -elevation_steps;
        elevation_index <= elevation_steps; ++elevation_index)
      {
        if (azimuth_index == 0 && elevation_index == 0) {
          continue;
        }
        const double azimuth_deg = azimuth_index * config_.azimuth_step_deg;
        const double elevation_deg = elevation_index * config_.elevation_step_deg;
        const double azimuth = radians(azimuth_deg);
        const double elevation = radians(elevation_deg);
        Eigen::Vector3d direction =
          std::cos(elevation) * std::cos(azimuth) * front +
          std::cos(elevation) * std::sin(azimuth) * side +
          std::sin(elevation) * up;
        direction.normalize();
        const Eigen::Vector3d camera_position = target + radius * direction;
        if (camera_position.z() < config_.min_camera_height_m) {
          continue;
        }
        if (protectedZoneHit(camera_position, config_.protected_zones)) {
          continue;
        }

        const double motion = angleDegrees(direction, front);
        if (motion < 3.0) {
          continue;
        }
        double nearest = std::numeric_limits<double>::max();
        for (const auto & previous : observed) {
          nearest = std::min(nearest, angleDegrees(direction, previous));
        }
        const double move_side = direction.dot(side);
        const double move_up = direction.dot(up);
        Eigen::Vector2d move(move_side, move_up);
        if (move.norm() > 1.0e-9) {
          move.normalize();
        }
        const double align = 0.5 * (move.dot(desired) + 1.0);
        const double baseline_error =
          (nearest - config_.preferred_baseline_deg) /
          std::max(1.0, config_.preferred_baseline_deg * 0.7);
        const double overlap_score = std::exp(-0.5 * baseline_error * baseline_error);
        const double motion_score = 1.0 - std::clamp(
          motion / std::max(1.0, config_.azimuth_step_deg +
          std::max(1.0, config_.elevation_step_deg)), 0.0, 1.0);

        ViewCandidate candidate;
        candidate.direction_target_to_camera = direction;
        candidate.radius_m = radius;
        candidate.azimuth_deg = azimuth_deg;
        candidate.elevation_deg = elevation_deg;
        candidate.nearest_baseline_deg = nearest;
        candidate.motion_angle_deg = motion;
        candidate.score = 0.55 * align + 0.25 * overlap_score + 0.20 * motion_score;
        candidate.camera_pose.translation() = camera_position;
        candidate.camera_pose.linear() = lookAtOptical(
          candidate.camera_pose.translation(), target);
        std::ostringstream label;
        label << "see_a" << azimuth_index << "_e" << elevation_index <<
          "_r" << layer;
        candidate.label = label.str();
        result.push_back(candidate);
      }
    }
  }
  std::stable_sort(
    result.begin(), result.end(),
    [](const ViewCandidate & first, const ViewCandidate & second) {
      return first.score > second.score;
    });
  return result;
}

}  // namespace peach_manipulation_skills

// === target_cache.cpp ===
namespace peach_manipulation_skills
{
namespace
{
constexpr uint8_t kTrackingObserved = 0;  // PeachTargetObservation.OBSERVED

template<typename Src>
void copyBbox(CachedTarget & dest, const Src & src)
{
  dest.bbox_x = src.bbox_x;
  dest.bbox_y = src.bbox_y;
  dest.bbox_w = src.bbox_w;
  dest.bbox_h = src.bbox_h;
  dest.image_width = src.image_width;
  dest.image_height = src.image_height;
  dest.bbox_valid = src.bbox_valid;
}

double axisAngleDeg(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  if (!nonzeroFinite(first) || !nonzeroFinite(second)) {
    return -1.0;
  }
  const double cosine = std::clamp(
    first.normalized().dot(second.normalized()), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / kPi;
}

bool freshEnough(const CachedTarget & target, double after_s, bool live_required)
{
  if (!target.valid) {
    return false;
  }
  if (live_required) {
    return target.received_s > after_s;
  }
  return target.tracking_status == kTrackingObserved && target.updated_s > after_s;
}

double freshnessStamp(const CachedTarget & target)
{
  if (target.tracking_status == kTrackingObserved &&
    target.updated_s > target.received_s)
  {
    return target.updated_s;
  }
  return target.received_s;
}
}  // namespace

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
  // 诊断透传每帧都刷新（含非观测帧）：再确认段摆动判定与失败原因文案以
  // 最近一帧为准；received_s 仍只在有效观测帧刷新（见下）。
  target_.swinging = update.swinging;
  target_.tracking_status = update.tracking_status;
  copyBbox(target_, update);
  target_.updated_s = clock_s_();
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

void TargetCache::updateLockedTargets(
  bool target_set_locked, const std::string & harvest_run_id,
  const std::vector<LockedTargetUpdate> & updates)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_set_locked) {
    // 未锁定帧 observations 恒空（感知锁定前只发收齐摘要），锁定集缓存无
    // 意义：清空并复位批次记钥，下次锁定按新集合整体重建。
    if (!locked_targets_.empty() || !locked_run_id_.empty()) {
      locked_targets_.clear();
      locked_run_id_.clear();
      cv_.notify_all();
    }
    return;
  }
  if (locked_run_id_ != harvest_run_id) {
    // 批次切换：跨批次身份不复用（run_id 是感知身份记忆的生命期边界），
    // 旧批次锚点全部作废后按新批次重建。
    locked_targets_.clear();
    locked_run_id_ = harvest_run_id;
  }
  for (const auto & update : updates) {
    if (update.target_id.empty()) {
      continue;
    }
    CachedTarget & entry = locked_targets_[update.target_id];
    entry.id = update.target_id;
    entry.harvest_run_id = harvest_run_id;
    // 诊断透传每帧刷新（含非观测帧，与 selected 缓存同语义）：残局目标的
    // 摆动旗标/跟踪状态是再确认段与失败原因文案的数据源。
    entry.swinging = update.swinging;
    entry.tracking_status = update.tracking_status;
    copyBbox(entry, update);
    entry.updated_s = clock_s_();
    const bool has_anchor = nonzeroFinite(update.bottom) &&
      nonzeroFinite(update.neck) && nonzeroFinite(update.axis);
    if (has_anchor) {
      // 锚点几何即采用（含 LOST 帧的记忆锚点，理由同 updateSelectedTarget）；
      // 观测新鲜度仍由 received_s 只在有效观测帧刷新来把关。
      entry.center = 0.5 * (update.bottom + update.neck);
      entry.initial_axis = update.axis.normalized();
      entry.suggested_travel_m = update.suggested_travel_m;
      entry.valid = true;
    }
    if (update.observed && has_anchor) {
      entry.initial_pose = update.entry_pose;
      entry.received_s = clock_s_();
    }
  }
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
  quality_.station_count = update.view_directions.size();
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
  if (target_id.empty()) {
    grasp_decision_target_id_.clear();
    quality_.grasp_allowed = false;
    cv_.notify_all();
    return true;
  }
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
  if (update.clear) {
    quality_.refined_rmse_m = 0.0;
    quality_.refined_inlier_ratio = 0.0;
    quality_.refined_accept = false;
    if (refined_.id.empty()) {
      quality_.refined_target_id.clear();
    }
    cv_.notify_all();
    return true;
  }
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

std::optional<CachedTarget> TargetCache::lockedTargetSnapshot(
  const std::string & target_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = locked_targets_.find(target_id);
  if (it == locked_targets_.end() || !it->second.valid) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<Eigen::Vector3d> TargetCache::lockedNeighborCenters(
  const std::string & exclude_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(locked_targets_.size());
  for (const auto & item : locked_targets_) {
    if (item.first == exclude_id || !item.second.valid) {
      continue;
    }
    centers.push_back(item.second.center);
  }
  return centers;
}

TargetGateSample TargetCache::lockedTargetGateSample(
  const std::string & target_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = locked_targets_.find(target_id);
  if (it == locked_targets_.end()) {
    // 不在锁定集：空 ID 样本，SafetyGate::targetReady 按身份不匹配拒绝。
    return TargetGateSample{};
  }
  return TargetGateSample{it->second.id, it->second.valid, freshnessStamp(it->second)};
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
  if (target_.valid && refined_.valid) {
    snapshot.axis_angle_deg = axisAngleDeg(target_.initial_axis, refined_.axis);
  } else {
    snapshot.axis_angle_deg = -1.0;
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
  return TargetGateSample{target_.id, target_.valid, freshnessStamp(target_)};
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

bool TargetCache::waitForNewStation(
  std::size_t previous_stations, double timeout_s,
  const std::atomic_bool & cancel) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, previous_stations, &cancel]() {
      return cancel.load() || observed_directions_.size() > previous_stations;
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
  double after_s, double timeout_s, const std::atomic_bool & cancel,
  bool live_observation_required) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, after_s, &cancel, live_observation_required]() {
      return cancel.load() ||
             freshEnough(target_, after_s, live_observation_required);
    }) && !cancel.load();
}

bool TargetCache::waitForFreshLockedTarget(
  const std::string & target_id, double after_s, double timeout_s,
  const std::atomic_bool & cancel, bool live_observation_required) const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(
    lock, std::chrono::duration<double>(timeout_s),
    [this, &target_id, after_s, &cancel, live_observation_required]() {
      const auto it = locked_targets_.find(target_id);
      return cancel.load() ||
             (it != locked_targets_.end() &&
             freshEnough(it->second, after_s, live_observation_required));
    }) && !cancel.load();
}

void TargetCache::notifyAll()
{
  cv_.notify_all();
}

}  // namespace peach_manipulation_skills
