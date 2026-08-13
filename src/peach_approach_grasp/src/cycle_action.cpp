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
// 周期控制面：RunTargetCycle action 服务端（goal/cancel/accepted/执行监督）
// 与周期控制服务回调（start/cancel/acknowledge_recovery/query_state/arm）。
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "approach_grasp_node_impl.hpp"

using namespace std::chrono_literals;

namespace peach_approach_grasp
{
rclcpp_action::GoalResponse ApproachGraspNode::onActionGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const RunTargetCycle::Goal> goal)
{
  if (goal->target_id.empty() || goal->mode == RunTargetCycle::Goal::OBSERVE_ONLY ||
    running_.load() || contact_recovery_required_.load())
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto target = targetSnapshot();
  if (!target || target->id != goal->target_id) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ApproachGraspNode::onActionCancel(
  const std::shared_ptr<RunTargetGoalHandle>)
{
  cancel_requested_.store(true);
  if (move_group_) {move_group_->stop();}
  if (grasp_task_) {grasp_task_->cancel();}
  cache_.notifyAll();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ApproachGraspNode::onActionAccepted(
  const std::shared_ptr<RunTargetGoalHandle> goal_handle)
{
  // action 执行线程保持可 join：析构时先置取消标志再回收，避免 detach 后
  // 线程在 shutdown 之后访问已销毁成员。同一时刻至多一个周期在运行。
  if (action_thread_.joinable()) {
    action_thread_.join();
  }
  action_thread_ = std::thread([this, goal_handle]() {executeAction(goal_handle);});
}

void ApproachGraspNode::executeAction(
  const std::shared_ptr<RunTargetGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto trigger_response = std::make_shared<Trigger::Response>();
  if (goal->mode == RunTargetCycle::Goal::PREVIEW) {
    previewContact(false, trigger_response);
  } else {
    // Action 是自动编排专用入口；手动 Trigger 仍要求每周期单独 arm。
    if (execution_enabled_.load()) {execution_armed_.store(true);}
    onStart(std::make_shared<Trigger::Request>(), trigger_response, true);
  }
  if (!trigger_response->success) {
    auto result = std::make_shared<RunTargetCycle::Result>();
    result->outcome = RunTargetCycle::Result::FAILED;
    result->reason = trigger_response->message;
    result->recovery_required = contact_recovery_required_.load();
    goal_handle->abort(result);
    return;
  }

  while (rclcpp::ok() && running_.load()) {
    if (goal_handle->is_canceling()) {
      cancel_requested_.store(true);
      if (move_group_) {move_group_->stop();}
      if (grasp_task_) {grasp_task_->cancel();}
      cache_.notifyAll();
    }
    auto feedback = std::make_shared<RunTargetCycle::Feedback>();
    feedback->state.target_id = goal->target_id;
    feedback->state.action_active = running_.load();
    feedback->state.execution_enabled = execution_enabled_.load();
    feedback->state.grasp_enabled = grasp_enabled_.load();
    feedback->state.tool_enabled = tool_enabled_.load();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      feedback->state.message = state_json_.value("message", std::string());
    }
    goal_handle->publish_feedback(feedback);
    std::this_thread::sleep_for(200ms);
  }

  // 终局判定只读结构化的 CycleState 枚举；state_json_ 的字符串仅是发布层投影。
  CycleResult cycle_result;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cycle_result.outcome = terminalOutcome(current_state_);
    cycle_result.reason = state_json_.value("message", std::string());
  }
  cycle_result.recovery_required =
    cycle_result.outcome == CycleOutcome::RECOVERY_REQUIRED;
  const CycleOutcome outcome = cycle_result.outcome;
  auto result = std::make_shared<RunTargetCycle::Result>();
  result->reason = cycle_result.reason;
  result->recovery_required = cycle_result.recovery_required;
  // 线程可 join 后必须兜住 shutdown 竞态下的上报异常，避免 std::terminate。
  try {
    if (outcome == CycleOutcome::SUCCEEDED) {
      result->outcome = RunTargetCycle::Result::SUCCEEDED;
      goal_handle->succeed(result);
    } else if (outcome == CycleOutcome::CANCELED || goal_handle->is_canceling()) {
      result->outcome = RunTargetCycle::Result::FAILED;
      goal_handle->canceled(result);
    } else {
      // abort 路径按 BT 失败点记录的 pending_outcome_ 分级（质量/不可达/失败）。
      result->outcome = pending_outcome_.load();
      goal_handle->abort(result);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "action 终局上报失败（可能正在 shutdown）: %s", error.what());
  }
}

void ApproachGraspNode::onStart(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response,
  bool action_driven)
{
  if (!move_group_) {
    response->success = false;
    response->message = "MoveIt 尚未初始化";
    return;
  }
  if (contact_recovery_required_.load()) {
    response->success = false;
    response->message =
      "上一周期可能停在接触区；现场人工撤离并确认后调用 acknowledge_recovery";
    return;
  }
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    response->success = false;
    response->message = "已有靠近/抓取周期正在运行";
    return;
  }
  if (execution_enabled_.load() && !execution_armed_.load()) {
    running_.store(false);
    response->success = false;
    response->message = "execution.enabled=true 但尚未人工 arm";
    return;
  }
  {
    const auto target = cache_.targetSnapshot();
    if (!target || target->id.empty()) {
      running_.store(false);
      response->success = false;
      response->message = "没有可用的 selected_target 初始几何";
      return;
    }
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  cancel_requested_.store(false);
  // 每周期开始重置终局分级（BT 失败点按需覆盖），并记录本周期驱动来源。
  pending_outcome_.store(RunTargetCycle::Result::FAILED);
  cycle_action_driven_ = action_driven;
  worker_ = std::thread(&ApproachGraspNode::runCycle, this);
  response->success = true;
  response->message = execution_enabled_.load() ? "已启动主动视觉靠近周期" :
    "已启动只规划预览（不会发送运动）";
}

void ApproachGraspNode::onCancel(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  cancel_requested_.store(true);
  if (move_group_) {
    move_group_->stop();
  }
  if (grasp_task_) {
    grasp_task_->cancel();
  }
  cache_.notifyAll();
  response->success = true;
  response->message = "已请求取消；当前 MoveIt 执行将停止";
}

void ApproachGraspNode::onAcknowledgeRecovery(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  if (running_.load()) {
    response->success = false;
    response->message = "周期运行中不能确认恢复";
    return;
  }
  contact_recovery_required_.store(false);
  response->success = true;
  response->message = "已记录现场人工撤离确认；本服务不发送任何运动命令";
  setState(CycleState::IDLE, response->message);
}

void ApproachGraspNode::onQuery(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  response->success = true;
  response->message = state_json_.dump();
}

void ApproachGraspNode::onArm(
  const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response)
{
  if (running_.load()) {
    response->success = false;
    response->message = "周期运行中不能改变 arm 状态";
    return;
  }
  execution_armed_.store(request->data);
  response->success = true;
  response->message = request->data ?
    "已为下一次周期一次性 arm；周期结束自动解除" : "已解除执行 arm";
  publishState();
}

}  // namespace peach_approach_grasp
