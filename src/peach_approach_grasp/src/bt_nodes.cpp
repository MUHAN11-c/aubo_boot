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
  return cache_.waitForNewView(previous_views, effectiveFrameWaitS(), cancel_requested_);
}

bool ApproachGraspNode::waitForFreshTarget(double after_s)
{
  return cache_.waitForFreshTarget(after_s, effectiveFrameWaitS(), cancel_requested_);
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
  cycle_degraded_grasp_ = false;
  if (!cycle_target_) {
    return btFailure("selected_target 在启动后失效");
  }
  // goal 钉死校验（设计文档第 7 节）：action 受理到本快照之间感知若已切换
  // selected，身份不一致即周期失败，由编排按新 selected 重新派发；
  // 手动周期钉入值为空，直接采纳当下快照身份。
  if (!cycle_target_id_.empty() && cycle_target_->id != cycle_target_id_) {
    return btFailure(
      "目标身份变更: goal=" + cycle_target_id_ +
      " 当前 selected=" + cycle_target_->id);
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
  const double scan_start_s = now().seconds();
  while (!cancel_requested_.load() && moves < maximum_scan_moves_ &&
    (now().seconds() - scan_start_s) < scan_time_budget_s_)
  {
    const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
    if (finalize_gate.allowed) {
      break;
    }
    const auto current_camera = lookupTransform(base_frame_, camera_frame_);
    if (!current_camera) {
      return btFailure("扫描中无法取得相机位姿");
    }
    // 每次规划前用缓存中的最新观测锚点：跨视角锚点偏差在近距获得观测后
    // 自动纠偏，避免按拍照位姿的旧锚点把目标指到画面外（stale 主因）。
    const auto latest_target = targetSnapshot();
    const Eigen::Vector3d scan_center =
      (latest_target && latest_target->id == cycle_target_id_) ?
      latest_target->center : cycle_target_->center;
    cycle_candidates_ = view_planner_->generate(
      scan_center, current_camera->translation(), observedDirectionsSnapshot());
    publishViewMarkers(scan_center, cycle_candidates_);
    bool moved = false;
    for (const auto & candidate : cycle_candidates_) {
      if (std::find(attempted.begin(), attempted.end(), candidate.label) != attempted.end()) {
        continue;
      }
      attempted.push_back(candidate.label);
      std::string target_reason;
      if (!cycleTargetReady(cycle_target_id_, target_reason)) {
        const bool stale = target_reason == "selected_target_stale";
        // 短暂闪烁/遮挡：等一个新鲜帧窗口后复核，恢复则继续扫描
        const bool recovered = stale && moves > 0 &&
          waitForFreshTarget(now().seconds()) &&
          cycleTargetReady(cycle_target_id_, target_reason);
        if (recovered) {
          RCLCPP_INFO(get_logger(), "目标观测短暂丢失后已恢复，继续扫描");
        }
        if (stale && moves == 0) {
          // 周期起步目标即 stale（残局视角暂不可见）：凭记忆锚点做获取性
          // 移动，到位后由 waitForFreshTarget 判定是否重新可见
          RCLCPP_INFO(get_logger(), "目标观测暂陈旧，凭记忆锚点执行获取性移动");
        } else if (!recovered) {
          return btFailure("目标身份/可见性安全门失败: " + target_reason);
        }
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
        // 无新鲜目标观测则本视点必无有效掩膜帧，不再等重建成帧，直接换视点
        RCLCPP_WARN(get_logger(), "视点到达但等待新鲜目标观测超时，换下一视点");
        break;
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
    if (moves < maximum_scan_moves_) {
      // 观察时间盒到期（移动次数未耗尽）：带现有覆盖强制 finalize，精化
      // 不达标由候选锚点降级抓取兜底（非极端必抓），控制单目标观察耗时。
      RCLCPP_WARN(
        get_logger(),
        "观察时间盒 %.1fs 到期且覆盖未达标（%s），强制 finalize 走降级抓取链",
        scan_time_budget_s_, gate.reason.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    // 扫描上限内采集帧不足/不收敛：同样按目标不可达跳过。
    pending_outcome_.store(RunTargetCycle::Result::SKIPPED_UNREACHABLE);
    return btFailure("达到扫描上限仍未收敛: " + gate.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachGraspNode::btFinalizeAndValidate()
{
  setState(CycleState::FINALIZE, "视角覆盖达标，提取 TSDF 与精化几何");
  const bool refined_arrived =
    callTrigger(finalize_client_, "finalize_reconstruction") &&
    waitForRefined(cycle_target_id_);
  // 每周期落盘重建 session（无论精化是否达标，重建过程持续记录）
  callTrigger(save_client_, "save_session", false);
  const GateResult gate = quality_gate_->readyToGrasp(qualitySnapshot());
  const bool grasp_ready = refined_arrived && gate.allowed &&
    graspDecisionTargetSnapshot() == cycle_target_id_;
  const auto tip_from_tool = lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    return btFailure("无法取得 tip 到 tool 的变换");
  }
  if (grasp_ready) {
    cycle_refined_ = refinedSnapshot();
    if (!cycle_refined_) {
      return btFailure("精化位姿数据不存在");
    }
    Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
    entry_tool_pose.translation() = cycle_refined_->entry;
    entry_tool_pose.linear() = ViewPlanner::toolOrientation(
      cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
    cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
    cycle_travel_m_ = insertionTravel(*cycle_refined_);
    return BT::NodeStatus::SUCCESS;
  }
  // 回退（非极端必抓）：精化未产出或质量门未过时，身份一致的锚点还在就用
  // 感知候选几何降级抓取；连锚点都没有才算极端情况，按 SKIPPED_QUALITY 跳过。
  const auto fallback = targetSnapshot();
  if (!fallback || fallback->id != cycle_target_id_) {
    pending_outcome_.store(RunTargetCycle::Result::SKIPPED_QUALITY);
    return btFailure(
      "最终抓取质量门失败且无候选锚点可回退: " + gate.reason);
  }
  CachedRefined degraded;
  degraded.id = fallback->id;
  degraded.axis = fallback->initial_axis.normalized();
  degraded.suggested_travel_m = fallback->suggested_travel_m;
  degraded.valid = true;
  // 降级路径无精化 neck/entry 几何，insertionTravel 的几何回退恒钳下限
  // （插入过浅）：suggested_travel_m 无效时直接取量程中点。
  cycle_travel_m_ = degraded.suggested_travel_m > 0.0 ?
    insertionTravel(degraded) :
    std::clamp(
      0.5 * (minimum_travel_m_ + maximum_travel_m_), minimum_travel_m_,
      maximum_travel_m_);
  // 入口点取锚点后方（沿轴后退 行程+5cm），不依赖可能缺省的 initial_pose。
  degraded.entry = fallback->center - degraded.axis * (cycle_travel_m_ + 0.05);
  cycle_refined_ = degraded;
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = degraded.entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    degraded.axis, fallback->initial_pose.linear().col(0));
  cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
  cycle_degraded_grasp_ = true;
  RCLCPP_WARN(
    get_logger(), "精化不可用/未过门（%s），回退候选锚点降级抓取 %s",
    gate.reason.c_str(), cycle_target_id_.c_str());
  setState(CycleState::FINALIZE, "精化未达标，按感知候选锚点降级抓取");
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
  if (!safetyReady(reason)) {
    return btFailure("MTC 执行前机器人安全门失败: " + reason);
  }
  if (!cycleTargetReady(cycle_target_id_, reason)) {
    if (reason != "selected_target_stale") {
      return btFailure("MTC 执行前安全门失败: " + reason);
    }
    // finalize 耗时必然超过观测新鲜度窗口：先等一窗新鲜观测再复核；
    // 仍不新鲜则按静态果实处置——锚点来自多视融合/身份记忆，MTC 碰撞
    // 检查兜底，继续执行（验证期策略：非极端必抓，准确性现场评估）。
    if (waitForFreshTarget(now().seconds()) &&
      cycleTargetReady(cycle_target_id_, reason))
    {
      RCLCPP_INFO(get_logger(), "MTC 前目标观测已刷新，继续执行");
    } else {
      RCLCPP_WARN(
        get_logger(), "MTC 前观测仍陈旧，按静态目标锚点继续（已等待复核）");
    }
  }
  setState(
    CycleState::MTC_APPROACH_INSERT,
    cycle_degraded_grasp_ ?
      "MTC（降级：候选锚点）: OMPL 避障到入口，再沿候选轴直线插入" :
      "MTC: OMPL 避障到入口，再沿精化轴直线插入");
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
