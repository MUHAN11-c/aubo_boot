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
// 行为树节点体与注册：11 个 BT 节点、候选视点 marker 发布、重建帧/精化等待薄壳。
#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "approach_grasp_node_impl.hpp"

namespace peach_approach_grasp
{
// 数据快照薄壳：统一从 cache_ 取一致性快照，供 BT 节点体与运动接口使用。
QualitySnapshot ApproachGraspNode::qualitySnapshot()
{
  return cache_.qualitySnapshot();
}

std::optional<CachedTarget> ApproachGraspNode::targetSnapshot()
{
  return cache_.targetSnapshot();
}

std::vector<Eigen::Vector3d> ApproachGraspNode::observedDirectionsSnapshot()
{
  return cache_.observedDirections();
}

std::optional<CachedRefined> ApproachGraspNode::refinedSnapshot()
{
  return cache_.refinedSnapshot();
}

std::string ApproachGraspNode::graspDecisionTargetSnapshot()
{
  return cache_.graspDecisionTarget();
}

bool ApproachGraspNode::waitForNewView(std::size_t previous_views)
{
  return cache_.waitForNewView(previous_views, frame_wait_s_, cancel_requested_);
}

bool ApproachGraspNode::waitForFreshTarget(double after_s)
{
  return cache_.waitForFreshTarget(after_s, frame_wait_s_, cancel_requested_);
}

bool ApproachGraspNode::waitForRefined(const std::string & target_id)
{
  return cache_.waitForRefined(target_id, refined_timeout_s_, cancel_requested_);
}

void ApproachGraspNode::registerBehaviorTreeNodes()
{
  bt_factory_.registerSimpleAction(
    "PrepareCycle", [this](BT::TreeNode &) {return btPrepareCycle();});
  bt_factory_.registerSimpleCondition(
    "IsPlanOnly", [this](BT::TreeNode &) {
      return execution_enabled_.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  bt_factory_.registerSimpleAction(
    "PlanObservationPreview", [this](BT::TreeNode &) {return btPlanPreview();});
  bt_factory_.registerSimpleAction(
    "AcquireReconstructionViews", [this](BT::TreeNode &) {return btAcquireViews();});
  bt_factory_.registerSimpleAction(
    "FinalizeAndValidate", [this](BT::TreeNode &) {return btFinalizeAndValidate();});
  bt_factory_.registerSimpleCondition(
    "IsGraspDisabled", [this](BT::TreeNode &) {
      return grasp_enabled_.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  bt_factory_.registerSimpleAction(
    "ReportReadyForGrasp", [this](BT::TreeNode &) {return btReportReady();});
  bt_factory_.registerSimpleAction(
    "MTCApproachAndInsert", [this](BT::TreeNode &) {return btMtcApproachAndInsert();});
  bt_factory_.registerSimpleAction(
    "ActuateTool", [this](BT::TreeNode &) {return btActuateTool();});
  bt_factory_.registerSimpleAction(
    "MTCRetreat", [this](BT::TreeNode &) {return btMtcRetreat();});
  bt_factory_.registerSimpleAction(
    "CompleteTarget", [this](BT::TreeNode &) {return btCompleteTarget();});
}

BT::NodeStatus ApproachGraspNode::btFailure(const std::string & reason)
{
  bt_failure_reason_ = reason;
  setState(CycleState::FAILED, reason);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ApproachGraspNode::btPrepareCycle()
{
  cycle_target_ = targetSnapshot();
  cycle_refined_.reset();
  cycle_candidates_.clear();
  if (!cycle_target_) {
    return btFailure("selected_target 在启动后失效");
  }
  cycle_target_id_ = cycle_target_->id;
  setState(CycleState::PLAN_OBSERVATION, "行为树生成目标导向主动视点");
  const auto base_from_camera = lookupTransform(base_frame_, camera_frame_);
  if (!base_from_camera) {
    return btFailure("无法取得当前相机位姿");
  }
  cycle_candidates_ = view_planner_->generate(
    cycle_target_->center, base_from_camera->translation(), observedDirectionsSnapshot());
  publishViewMarkers(cycle_target_->center, cycle_candidates_);
  if (cycle_candidates_.empty()) {
    return btFailure("没有生成可用观察视点");
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btPlanPreview()
{
  for (const auto & candidate : cycle_candidates_) {
    if (planOrMoveCamera(candidate.camera_pose, "PTP", false, candidate.label)) {
      cycle_terminal_state_ = CycleState::PLAN_READY;
      cycle_terminal_message_ = "行为树只规划预览成功；未发送任何运动";
      return BT::NodeStatus::SUCCESS;
    }
  }
  return btFailure("所有候选观察位姿均不可规划");
}

BT::NodeStatus ApproachGraspNode::btAcquireViews()
{
  if (reset_reconstruction_on_start_ &&
    !callTrigger(reset_client_, "reset_reconstruction"))
  {
    return btFailure("重建重置失败");
  }
  int moves = 0;
  std::vector<std::string> attempted;
  while (!cancel_requested_.load() && moves < maximum_scan_moves_) {
    const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
    if (finalize_gate.allowed) {
      break;
    }
    const auto current_camera = lookupTransform(base_frame_, camera_frame_);
    if (!current_camera) {
      return btFailure("扫描中无法取得相机位姿");
    }
    cycle_candidates_ = view_planner_->generate(
      cycle_target_->center, current_camera->translation(), observedDirectionsSnapshot());
    publishViewMarkers(cycle_target_->center, cycle_candidates_);
    bool moved = false;
    for (const auto & candidate : cycle_candidates_) {
      if (std::find(attempted.begin(), attempted.end(), candidate.label) != attempted.end()) {
        continue;
      }
      attempted.push_back(candidate.label);
      std::string target_reason;
      if (!cycleTargetReady(cycle_target_id_, target_reason)) {
        return btFailure("目标身份/可见性安全门失败: " + target_reason);
      }
      const std::size_t before = qualitySnapshot().captured_views;
      setState(
        CycleState::MOVE_TO_VIEW,
        candidate.label + " score=" + std::to_string(candidate.score));
      const std::string planner = moves == 0 ? "PTP" : "LIN";
      if (!planOrMoveCamera(candidate.camera_pose, planner, true, candidate.label)) {
        continue;
      }
      ++moves;
      moved = true;
      // 先等到位后的新鲜目标观测：移动中途被重建接受的帧会让 waitForNewView
      // 立即返回，而感知有效样本仍是移动前的旧帧，直接复核安全门必然 stale
      // （0.78FPS 下到位后首帧约 1.3s 才到，6s 窗口约等 4-5 帧）。
      const double move_done_s = now().seconds();
      setState(CycleState::WAIT_FRAME, "等待到位后新鲜目标观测与重建帧");
      if (!waitForFreshTarget(move_done_s)) {
        RCLCPP_WARN(get_logger(), "视点到达但等待新鲜目标观测超时");
      }
      if (!waitForNewView(before)) {
        RCLCPP_WARN(get_logger(), "视点到达但等待新重建帧超时");
      }
      break;
    }
    if (!moved) {
      // 视角规划不可达：批次侧可按 SKIPPED_UNREACHABLE 直接跳过该目标。
      pending_outcome_.store(RunTargetCycle::Result::SKIPPED_UNREACHABLE);
      return btFailure("剩余候选视点均不可达或规划失败");
    }
  }
  if (cancel_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  const GateResult gate = quality_gate_->readyToFinalize(qualitySnapshot());
  if (!gate.allowed) {
    // 扫描上限内采集帧不足/不收敛：同样按目标不可达跳过。
    pending_outcome_.store(RunTargetCycle::Result::SKIPPED_UNREACHABLE);
    return btFailure("达到扫描上限仍未收敛: " + gate.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btFinalizeAndValidate()
{
  setState(CycleState::FINALIZE, "视角覆盖达标，提取 TSDF 与精化几何");
  if (!callTrigger(finalize_client_, "finalize_reconstruction") ||
    !waitForRefined(cycle_target_id_))
  {
    return btFailure("finalize 后未收到同 ID 精化结果");
  }
  const GateResult gate = quality_gate_->readyToGrasp(qualitySnapshot());
  if (!gate.allowed || graspDecisionTargetSnapshot() != cycle_target_id_) {
    // 质量门失败：批次侧可按 SKIPPED_QUALITY 记录质量原因并跳过。
    pending_outcome_.store(RunTargetCycle::Result::SKIPPED_QUALITY);
    return btFailure("最终抓取质量门失败: " + gate.reason);
  }
  callTrigger(save_client_, "save_session", false);
  cycle_refined_ = refinedSnapshot();
  if (!cycle_refined_) {
    return btFailure("精化位姿数据不存在");
  }
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = cycle_refined_->entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
  const auto tip_from_tool = lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    return btFailure("无法取得 tip 到 tool 的变换");
  }
  cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
  cycle_travel_m_ = insertionTravel(*cycle_refined_);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btReportReady()
{
  cycle_terminal_state_ = CycleState::READY_FOR_GRASP;
  cycle_terminal_message_ = "精化质量通过；grasp.enabled=false，未执行接触动作";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btMtcApproachAndInsert()
{
  std::string reason;
  if (!safetyReady(reason) || !cycleTargetReady(cycle_target_id_, reason)) {
    return btFailure("MTC 执行前安全门失败: " + reason);
  }
  setState(CycleState::MTC_APPROACH_INSERT, "MTC: OMPL 避障到入口，再沿精化轴直线插入");
  const auto result = grasp_task_->approachAndInsert(
    cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_, true);
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  if (!result.success) {
    // 规划阶段失败说明目标不可达；执行已启动后的失败维持 FAILED 分级。
    pending_outcome_.store(
      result.execution_started ?
      RunTargetCycle::Result::FAILED : RunTargetCycle::Result::SKIPPED_UNREACHABLE);
    return btFailure("MTC 接近/插入失败: " + result.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btActuateTool()
{
  setState(CycleState::ACTUATE_TOOL, "触发末端工具抓取/切割");
  if (!commandToolClose()) {
    const auto retreat = grasp_task_->retreat(
      cycle_refined_->axis, cycle_travel_m_, true);
    if (retreat.success) {
      contact_recovery_required_.store(false);
    }
    return btFailure(
      "末端工具失败；MTC 紧急撤离结果: " + retreat.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btMtcRetreat()
{
  setState(CycleState::MTC_RETREAT, "MTC 沿插入反方向保持直线撤离");
  const auto result = grasp_task_->retreat(
    cycle_refined_->axis, cycle_travel_m_, true);
  if (!result.success) {
    return btFailure("MTC 抓取后撤离失败，需要人工处理: " + result.reason);
  }
  contact_recovery_required_.store(false);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btCompleteTarget()
{
  // 完成推进权归属：action（RunTargetCycle）驱动的周期由批次编排器按终态统一
  // 推进，此处不再调用，避免双写；手动 Trigger 周期没有编排器，仍由本节点兜底。
  if (!cycle_action_driven_ && complete_target_after_retreat_ &&
    !callTrigger(complete_client_, "complete_selected_target"))
  {
    return btFailure("抓取已完成但目标计划推进失败");
  }
  cycle_terminal_state_ = CycleState::SUCCEEDED;
  cycle_terminal_message_ = "行为树完成主动视觉、MTC 抓取和撤离";
  return BT::NodeStatus::SUCCESS;
}

void ApproachGraspNode::publishViewMarkers(
  const Eigen::Vector3d & target,
  const std::vector<ViewCandidate> & candidates)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  const std::size_t limit = std::min<std::size_t>(candidates.size(), 24U);
  for (std::size_t index = 0; index < limit; ++index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "candidate_views";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start;
    start.x = candidates[index].camera_pose.translation().x();
    start.y = candidates[index].camera_pose.translation().y();
    start.z = candidates[index].camera_pose.translation().z();
    geometry_msgs::msg::Point end;
    end.x = target.x();
    end.y = target.y();
    end.z = target.z();
    marker.points = {start, end};
    marker.scale.x = 0.003;
    marker.scale.y = 0.007;
    marker.scale.z = 0.010;
    marker.color.r = static_cast<float>(1.0 - candidates[index].score);
    marker.color.g = static_cast<float>(candidates[index].score);
    marker.color.b = 0.25F;
    marker.color.a = 0.75F;
    array.markers.push_back(marker);
  }
  marker_pub_->publish(array);
}

}  // namespace peach_approach_grasp
