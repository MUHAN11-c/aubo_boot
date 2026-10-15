// 功能：观测、重建和模型消息解码。几何/身份调和由零 ROS 的 TargetCache 完成。
#include "peach_arm/manipulation_skills_node.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "peach_arm/eigen_conversions.hpp"
#include "peach_arm/model_contract.hpp"

namespace peach_arm
{

void ManipulationSkillsNode::onTargets(
  const peach_interfaces::msg::PeachTargetObservationArray::SharedPtr message)
{
  // 每帧测量观测到达间隔并刷新帧率自适应超时（帧率以运行状态为准）。
  trackFrameInterval();
  safety_gate_->set_target_observation_max_age_s(effectiveTargetMaxAgeS());
  // 单条观测的公共字段提取（消息 ROS 类型留在节点薄壳）：observed 判定与
  // 摆动/跟踪诊断透传语义对 selected 与锁定集两路完全一致——记忆锚点帧
  // （anchor_from_memory）不算新鲜观测，锚点可用于派发/规划但不刷新观测
  // 新鲜度，让安全门的 stale 判定继续以真实观测为准。
  const auto extract = [](const auto & item, auto & out) {
      const bool anchor_from_memory = std::find(
        item.diagnostic_flags.begin(), item.diagnostic_flags.end(),
        "anchor_from_memory") != item.diagnostic_flags.end();
      out.observed =
        item.tracking_status == peach_interfaces::msg::PeachTargetObservation::OBSERVED &&
        item.candidate.status != peach_interfaces::msg::BagGraspCandidate::REJECT &&
        !anchor_from_memory;
      // 再确认段诊断透传（2.7-RECONFIRM）：摆动旗标与跟踪状态原始枚举随帧进
      // 缓存，摆动等平息与失败原因文案（出视野/深度空洞/跟踪丢失）在
      // stageReconfirmTarget 消费。
      out.swinging = std::find(
        item.diagnostic_flags.begin(), item.diagnostic_flags.end(),
        "target_swinging") != item.diagnostic_flags.end();
      out.tracking_status = item.tracking_status;
      out.bottom = pointToEigen(item.candidate.bag_bottom);
      out.neck = pointToEigen(item.candidate.bag_neck);
      out.axis = vectorToEigen(item.candidate.translation_direction);
      out.entry_pose = poseToEigen(item.candidate.entry_pose);
      out.suggested_travel_m = item.candidate.suggested_travel_m;
      out.bag_diameter_upper_m = item.candidate.bag_diameter_upper_m;
      out.bbox_x = item.candidate_2d.bbox_x;
      out.bbox_y = item.candidate_2d.bbox_y;
      out.bbox_w = item.candidate_2d.bbox_w;
      out.bbox_h = item.candidate_2d.bbox_h;
      out.bbox_valid =
        item.candidate_2d.bbox_w > 0 && item.candidate_2d.bbox_h > 0;
      out.foreground_ratio = item.fitting.foreground_ratio;
      out.image_width = item.mask.width > 0 ?
        static_cast<int>(item.mask.width) : 640;
      out.image_height = item.mask.height > 0 ?
        static_cast<int>(item.mask.height) : 480;
    };
  // 锁定集锚点缓存刷新（阶段 E 残局抬质量能力端）：observations 携带锁定集
  // 全部 confirmed 目标（含非 selected），逐条提取批量委托 cache_。必须置于
  // 下方 selected 早退之前——残局期感知 selected 恒空（FULL 终局后目标已被
  // 计划 complete），若先早退，残局目标锚点永远进不了缓存，OBSERVE_ONLY
  // 受理门与执行体都无数据源。
  std::vector<LockedTargetUpdate> locked_updates;
  if (message->target_set_locked) {
    locked_updates.reserve(message->observations.size());
    for (const auto & item : message->observations) {
      // confirmed 过滤在薄壳完成（缓存只存 confirmed 目标）；未确认/空 ID
      // 记录不进锁定集缓存。
      if (!item.confirmed || item.target_id.empty()) {
        continue;
      }
      LockedTargetUpdate locked;
      locked.target_id = item.target_id;
      extract(item, locked);
      locked_updates.push_back(std::move(locked));
    }
  }
  cache_.updateLockedTargets(
    message->target_set_locked, message->harvest_run_id, locked_updates);
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    last_snapshot_id_ = std::to_string(message->snapshot_id);
    last_observation_count_ = static_cast<uint32_t>(message->observations.size());
    last_target_set_locked_ = message->target_set_locked;
  }
  // selected 绑定以感知消息为准（不读周期上下文：周期身份钉在 ctx，由
  // stagePrepareCycle 的 goal 钉死校验单点把关；锁定集缓存是受理/执行的
  // 独立数据源）。感知切换 selected 时缓存跟随，周期侧身份不一致即失败，
  // 由编排按新 selected 重新派发。
  auto selected = std::find_if(
    message->observations.begin(), message->observations.end(),
    [&message](const auto & item) {
      return item.target_id == message->selected_target_id;
    });
  if (selected == message->observations.end()) {
    return;
  }
  // 消息字段提取（含 ROS 类型）留在节点薄壳，ID 一致性调和委托 cache_ 纯核。
  SelectedTargetUpdate update;
  update.selected_id = message->selected_target_id;
  update.harvest_run_id = message->harvest_run_id;
  extract(*selected, update);
  cache_.updateSelectedTarget(update);
}

// 帧率自适应超时族转发（W5-3）：公式在 FrameRateTimeouts 纯核，此处只供
// 数据（EMA/缓存状态）；调用点与语义不变。
void ManipulationSkillsNode::trackFrameInterval()
{
  frame_timeouts_.onTargetFrame(now().seconds());
}

double ManipulationSkillsNode::effectiveFrameWaitS() const
{
  return frame_timeouts_.frameWaitS();
}

double ManipulationSkillsNode::effectiveTargetMaxAgeS() const
{
  return frame_timeouts_.targetMaxAgeS();
}

double ManipulationSkillsNode::effectiveReconfirmWaitS() const
{
  return frame_timeouts_.reconfirmWaitS();
}

double ManipulationSkillsNode::effectiveRefinedWaitS() const
{
  // 窗口态感知（2026-09-09 真机）：COLLECTING 用配置上限覆盖采集→finalize
  // →refit 全程（公式与分支见 frame_timeouts.hpp 注释）。
  return frame_timeouts_.refinedWaitS(
    cache_.qualitySnapshot().reconstruction_state == "COLLECTING");
}

void ManipulationSkillsNode::onDiagnostics(
  const peach_interfaces::msg::ReconstructionStatus::SharedPtr message)
{
  // 类型化诊断（2026-08 起替换裸 JSON）：字段直读，不再解析字符串。
  // 无效标量约定 -1：质量证据按 0 汇入缓存（无覆盖=基线/深度证据为零，
  // 质量门自然不通过），不把 -1 当真实测量值参与比较。
  ReconstructionDiagnosticsUpdate update;
  update.target_id = message->target_id;
  update.state = message->state.empty() ? "IDLE" : message->state;
  update.captured_views = static_cast<std::size_t>(
    std::max(message->captured_views, 0));
  update.max_baseline_deg = std::max(message->max_baseline_deg, 0.0);
  update.mean_nearest_baseline_deg = std::max(
    message->mean_nearest_baseline_deg, 0.0);
  update.mean_depth_ratio = std::max(message->valid_depth_ratio, 0.0);
  update.view_directions.reserve(message->view_directions.size());
  for (const auto & direction : message->view_directions) {
    update.view_directions.emplace_back(direction.x, direction.y, direction.z);
  }
  cache_.updateReconstructionDiagnostics(update);
}

void ManipulationSkillsNode::onDecision(
  const peach_interfaces::msg::GraspDecision::SharedPtr message)
{
  ModelIdentity identity;
  identity.run_id = message->harvest_run_id;
  identity.scene_epoch = message->scene_epoch;
  identity.target_id = message->target_id;
  identity.model_revision = message->model_revision;
  identity.tool_profile_id = message->tool_profile_id;
  identity.calibration_revision = message->calibration_revision;
  identity.config_revision = message->config_revision;
  ModelSnapshot snap;
  snap.identity = identity;
  snap.generated_s = static_cast<double>(message->header.stamp.sec) +
    1e-9 * static_cast<double>(message->header.stamp.nanosec);
  snap.valid_until_s = static_cast<double>(message->valid_until.sec) +
    1e-9 * static_cast<double>(message->valid_until.nanosec);
  snap.geometry = static_cast<Capability>(message->geometry_capability);
  snap.pregrasp = static_cast<Capability>(message->pregrasp_capability);
  snap.sleeve = static_cast<Capability>(message->sleeve_capability);
  snap.cut = static_cast<Capability>(message->cut_capability);
  RefinedPoseUpdate geometry;
  geometry.target_id = message->target_id;
  geometry.entry = pointToEigen(message->entry);
  // 决策的 entry/cut 是执行参考点；物理袋端仍由感知观测持有。
  geometry.bottom = geometry.entry;
  geometry.neck = pointToEigen(message->cut_pose);
  geometry.axis = vectorToEigen(message->axis);
  geometry.suggested_travel_m = message->travel_m;
  geometry.bag_diameter_upper_m = message->diameter_m;
  geometry.accepted = snap.geometry == Capability::Valid;
  if (!cache_.updateDecisionSnapshot(snap, geometry, message->rmse_m, message->inlier_ratio)) {
    RCLCPP_DEBUG(
      get_logger(), "忽略不完整或非当前身份的 grasp_decision: target=%s revision=%s",
      identity.target_id.c_str(), identity.model_revision.c_str());
  }
}

void ManipulationSkillsNode::onRefinedPose(
  const peach_interfaces::msg::BagGraspCandidateArray::SharedPtr message)
{
  RefinedPoseUpdate update;
  if (message->candidates.empty()) {
    update.clear = true;
    cache_.updateRefinedPose(update);
    return;
  }
  const auto & candidate = message->candidates.front();
  update.target_id = candidate.target_id;
  update.entry = pointToEigen(candidate.entry_pose.position);
  update.bottom = pointToEigen(candidate.bag_bottom);
  update.neck = pointToEigen(candidate.bag_neck);
  update.axis = vectorToEigen(candidate.translation_direction);
  update.suggested_travel_m = candidate.suggested_travel_m;
  update.bag_diameter_upper_m = candidate.bag_diameter_upper_m;
  update.accepted = candidate.status == peach_interfaces::msg::BagGraspCandidate::ACCEPT;
  std::string reject_reason;
  if (!cache_.updateRefinedPose(update, &reject_reason)) {
    // unrefined_hold 是 skip_reconstruction 批的常态（latched 发布 0.5s
    // 心跳），不节流会把日志刷爆并淹没真信号（2026-09-23 单轮 8 万行）。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "忽略 refined pose（%s）: gate=%s actual=%s", reject_reason.c_str(),
      cache_.targetGateSample().id.c_str(), candidate.target_id.c_str());
  }
}

void ManipulationSkillsNode::onRefinedDiagnostics(
  const peach_interfaces::msg::BagFittingArray::SharedPtr message)
{
  if (message->fittings.empty()) {
    RefinedFittingUpdate clear;
    clear.clear = true;
    cache_.updateRefinedFitting(clear);
    return;
  }
  const auto & fitting = message->fittings.front();
  RefinedFittingUpdate update;
  update.target_id = fitting.target_id;
  update.is_fruit = fitting.target_kind == "fruit";
  update.sphere_rms_m = fitting.sphere_rms_m;
  update.sphere_inlier_ratio = fitting.sphere_inlier_ratio;
  update.cylinder_rms_m = fitting.cylinder_rms_m;
  update.cylinder_inlier_ratio = fitting.cylinder_inlier_ratio;
  update.accepted = fitting.status == peach_interfaces::msg::BagFitting::ACCEPT;
  std::string reject_reason;
  if (!cache_.updateRefinedFitting(update, &reject_reason)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "忽略 refined diagnostics（%s）: expected=%s actual=%s",
      reject_reason.c_str(), cache_.expectedFittingTargetId().c_str(),
      fitting.target_id.c_str());
  }
}

}  // namespace peach_arm
