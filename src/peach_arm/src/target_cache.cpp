// 功能：当前目标观测/精化/GraspDecision 缓存与 ID 调和。纯核，零 ROS；不写账本。
#include "peach_arm/target_cache.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "peach_arm/angles.hpp"
#include "peach_arm/math_utils.hpp"
#include "peach_arm/model_contract.hpp"

namespace peach_arm
{
namespace
{
constexpr uint8_t kTrackingObserved = 0;  // PeachTargetObservation.OBSERVED

// 观测 / 精化消息的检测框字段同名同义，无公共基类可借，逐字段拷入缓存。
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
  dest.foreground_ratio = src.foreground_ratio;
}

// 新鲜判据（io.md 门口径）：live_observation_required=true 只认末次 live
// 观测（received_s）；否则要求 OBSERVED 且 updated_s > after_s。
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

// 新鲜度比较基准：OBSERVED 且 updated_s 更新时取 updated_s，
// 否则退回末次 live 观测 received_s（非 OBSERVED 不被 updated_s 抬高）。
double freshnessStamp(const CachedTarget & target)
{
  if (target.tracking_status == kTrackingObserved &&
    target.updated_s > target.received_s)
  {
    return target.updated_s;
  }
  return target.received_s;
}

// 单目标观测调和（W5-13，自 updateSelectedTarget/updateLockedTargets 抽出，
// 两路 ~90% 重复的逐字段拷贝收敛单实现）：诊断透传每帧刷新（含非观测帧），
// 锚点几何（center/axis/travel）凡携带即采用（含 LOST 帧记忆锚点——世界系
// 身份记忆的意义所在，短暂不可见仍可派发/规划），entry_pose/received_s 仅
// OBSERVED 有效观测帧刷新（安全门按 max_age 判陈旧）。Update 取
// SelectedTargetUpdate / LockedTargetUpdate（字段同名同义）。
template<typename Update>
void applyObservation(CachedTarget & entry, const Update & update, double now_s)
{
  entry.swinging = update.swinging;
  entry.tracking_status = update.tracking_status;
  copyBbox(entry, update);
  entry.updated_s = now_s;
  const bool has_anchor = nonzeroFinite(update.bottom) &&
    nonzeroFinite(update.neck) && nonzeroFinite(update.axis);
  if (has_anchor) {
    entry.bottom = update.bottom;
    entry.neck = update.neck;
    entry.center = 0.5 * (update.bottom + update.neck);
    entry.initial_axis = update.axis.normalized();
    entry.suggested_travel_m = update.suggested_travel_m;
    entry.valid = true;
  }
  if (update.observed && has_anchor) {
    entry.initial_pose = update.entry_pose;
    entry.received_s = now_s;
  }
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
    unrefined_hold_ = false;
  }
  target_.id = update.selected_id;
  target_.harvest_run_id = update.harvest_run_id;
  applyObservation(target_, update, clock_s_());
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
    applyObservation(entry, update, clock_s_());
  }
  cv_.notify_all();
}

void TargetCache::updateReconstructionDiagnostics(
  const ReconstructionDiagnosticsUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (unrefined_hold_) {
    return;
  }
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
  // 诊断心跳只证明进程还在发，不得把 data_age 清零，也不得续签 valid_until。
  if (model_generated_s_ > 0.0) {
    quality_.data_age_s = std::max(0.0, clock_s_() - model_generated_s_);
  }
  cv_.notify_all();
}

bool TargetCache::updateGraspDecision(const std::string & target_id, bool allowed)
{
  ModelIdentity identity;
  identity.target_id = target_id;
  return updateGraspDecision(identity, allowed, 0.0);
}

bool TargetCache::updateGraspDecision(
  const ModelIdentity & identity, bool allowed, double valid_until_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  (void)valid_until_s;  // 心跳/决策不得续签；有效期只经 replaceModelSnapshot。
  if (identity.target_id.empty()) {
    grasp_decision_target_id_.clear();
    quality_.grasp_allowed = false;
    cv_.notify_all();
    return true;
  }
  if (!target_.id.empty() && identity.target_id != target_.id) {
    return false;
  }
  if (!model_.identity.run_id.empty() && !identity.run_id.empty() &&
    !identitiesMatch(model_.identity, identity))
  {
    quality_.grasp_allowed = false;
    return false;
  }
  grasp_decision_target_id_ = identity.target_id;
  quality_.grasp_allowed = allowed;
  cv_.notify_all();
  return true;
}

void TargetCache::replaceModelSnapshot(const ModelSnapshot & snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  model_ = snapshot;
  model_generated_s_ = snapshot.generated_s;
  quality_.data_age_s = 0.0;
  cv_.notify_all();
}

ModelSnapshot TargetCache::modelSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return model_;
}

bool TargetCache::updateRefinedPose(const RefinedPoseUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (unrefined_hold_ && !update.clear) {
    return false;
  }
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
  refined_.bag_diameter_upper_m = update.bag_diameter_upper_m;
  refined_.valid = nonzeroFinite(refined_.axis) && refined_.entry.allFinite();
  quality_.refined_target_id = update.target_id;
  quality_.refined_accept = update.accepted;
  cv_.notify_all();
  return true;
}

bool TargetCache::updateRefinedFitting(const RefinedFittingUpdate & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (unrefined_hold_ && !update.clear) {
    return false;
  }
  if (update.clear) {
    // 无效标量约定 -1（同 QualitySnapshot 默认）：原 0.0 会把「无精化」
    // 误投影成「完美拟合」（W5-13 修复）。
    quality_.refined_rmse_m = -1.0;
    quality_.refined_inlier_ratio = -1.0;
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
  if (unrefined_hold_) {
    snapshot.data_age_s = 0.0;
  } else if (diagnostics_seen_) {
    snapshot.data_age_s = std::max(0.0, clock_s_() - diagnostics_received_s_);
  }
  if (target_.valid && refined_.valid) {
    // 退化向量按 -1（不可判）——原 axisAngleDeg 包装语义，统一进 angles.hpp（W5-7）。
    snapshot.axis_angle_deg =
      angleDeg(target_.initial_axis, refined_.axis, AngleDegenerate::Invalid, 1.0e-6);
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

bool TargetCache::promoteUnrefinedGeometry(const std::string & target_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (target_id.empty()) {
    return false;
  }
  const CachedTarget * src = nullptr;
  const auto locked = locked_targets_.find(target_id);
  if (locked != locked_targets_.end() && locked->second.valid) {
    src = &locked->second;
  } else if (target_.id == target_id && target_.valid) {
    src = &target_;
  }
  if (src == nullptr || !nonzeroFinite(src->initial_axis)) {
    return false;
  }
  Eigen::Vector3d entry = src->initial_pose.translation();
  if (!nonzeroFinite(entry) && nonzeroFinite(src->bottom)) {
    entry = src->bottom - src->initial_axis.normalized() * 0.03;
  }
  if (!entry.allFinite()) {
    return false;
  }
  refined_ = CachedRefined();
  refined_.id = target_id;
  refined_.entry = entry;
  refined_.bottom = src->bottom;
  refined_.neck = src->neck;
  refined_.axis = src->initial_axis.normalized();
  refined_.suggested_travel_m = src->suggested_travel_m;
  refined_.valid = nonzeroFinite(refined_.axis) && refined_.entry.allFinite();
  if (!refined_.valid) {
    refined_ = CachedRefined();
    return false;
  }
  quality_.selected_target_id = target_id;
  quality_.refined_target_id = target_id;
  quality_.refined_accept = true;
  quality_.reconstruction_target_id = target_id;
  quality_.reconstruction_state = "READY";
  quality_.data_age_s = 0.0;
  diagnostics_seen_ = false;
  unrefined_hold_ = true;
  model_generated_s_ = clock_s_();
  cv_.notify_all();
  return true;
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

}  // namespace peach_arm
