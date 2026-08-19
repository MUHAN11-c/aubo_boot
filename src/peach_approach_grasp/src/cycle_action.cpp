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
// A13：targetPhase 投影字面量与 HarvestState.msg 的 TARGET_* 常量双向钉死
// （防消息常量重排后投影静默漂移）。
using HarvestStateMsg = peach_harvest_msgs::msg::HarvestState;
static_assert(targetPhase(CycleState::IDLE) == HarvestStateMsg::TARGET_IDLE);
static_assert(
  targetPhase(CycleState::PLAN_OBSERVATION) == HarvestStateMsg::OBSERVING);
static_assert(targetPhase(CycleState::FINALIZE) == HarvestStateMsg::FINALIZING);
static_assert(targetPhase(CycleState::RECONFIRM) == HarvestStateMsg::VALIDATING);
static_assert(
  targetPhase(CycleState::MTC_APPROACH_INSERT) == HarvestStateMsg::APPROACHING);
static_assert(targetPhase(CycleState::ACTUATE_TOOL) == HarvestStateMsg::TOOL_ACTION);
static_assert(targetPhase(CycleState::MTC_RETREAT) == HarvestStateMsg::RETREATING);
static_assert(targetPhase(CycleState::PLAN_READY) == HarvestStateMsg::COMPLETING);
static_assert(targetPhase(CycleState::SUCCEEDED) == HarvestStateMsg::TARGET_SUCCEEDED);
static_assert(targetPhase(CycleState::FAILED) == HarvestStateMsg::TARGET_FAILED);

rclcpp_action::GoalResponse ApproachGraspNode::onActionGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const RunTargetCycle::Goal> goal)
{
  const ScopedTimer timer(get_logger(), "action_goal", &callback_timing_);
  // 运动输出权限绑定 Active 态（A8）：非 Active 一律拒 goal 并给出原因。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    RCLCPP_WARN(
      get_logger(), "拒绝目标请求 %s: %s", goal->target_id.c_str(),
      motion_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // OBSERVE_ONLY 是受理模式之一：只走观察+精化验证段（BT 内 IsObserveOnly
  // 分支短路，不进 MTC/工具/撤离），终局按 PLAN_READY 上报 SUCCEEDED。
  if (goal->target_id.empty() ||
    (goal->mode != RunTargetCycle::Goal::PREVIEW &&
    goal->mode != RunTargetCycle::Goal::OBSERVE_ONLY &&
    goal->mode != RunTargetCycle::Goal::FULL) ||
    running_.load() || contact_recovery_required_.load())
  {
    RCLCPP_WARN(
      get_logger(), "拒绝目标请求 %s: 周期运行中/恢复待确认/请求非法",
      goal->target_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 受理门分模式（阶段 E 残局抬质量能力端）：OBSERVE_ONLY 的 goal 目标是
  // FULL 终局后已被感知计划 complete 的残局目标，selected 已切走或为空，
  // 不再钉死 selected 缓存，改为命中锁定集锚点缓存（confirmed 目标逐帧
  // 由 onTargets 维护）即受理；拒绝日志区分"不在锁定集/锚点缺失"。
  // FULL/PREVIEW 维持原钉死语义（目标必须仍是感知 selected）。
  if (goal->mode == RunTargetCycle::Goal::OBSERVE_ONLY) {
    const auto sample = cache_.lockedTargetGateSample(goal->target_id);
    if (sample.id != goal->target_id) {
      RCLCPP_WARN(
        get_logger(), "拒绝目标请求 %s: 目标不在感知锁定集锚点缓存",
        goal->target_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sample.valid) {
      RCLCPP_WARN(
        get_logger(), "拒绝目标请求 %s: 目标在锁定集但锚点缺失/无效",
        goal->target_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  const auto target = targetSnapshot();
  if (!target || target->id != goal->target_id) {
    RCLCPP_WARN(
      get_logger(), "拒绝目标请求 %s: 缓存目标=%s",
      goal->target_id.c_str(),
      target ? target->id.c_str() : "（无有效锚点）");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ApproachGraspNode::onActionCancel(
  const std::shared_ptr<RunTargetGoalHandle>)
{
  const ScopedTimer timer(get_logger(), "action_cancel", &callback_timing_);
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
    cycle_observe_only_.store(false);
    previewContact(false, trigger_response);
  } else {
    // Action 是自动编排专用入口；手动 Trigger 仍要求每周期单独 arm。
    if (execution_enabled_.load()) {execution_armed_.store(true);}
    // OBSERVE_ONLY：BT 在 FinalizeAndValidate 后经 IsObserveOnly 分支短路，
    // 不进 MTC/工具/撤离段（见 harvest_tree.xml report_or_grasp）。
    cycle_observe_only_.store(goal->mode == RunTargetCycle::Goal::OBSERVE_ONLY);
    // goal 钉死（设计文档第 7 节）：受理到 Prepare 快照之间感知可能切换
    // selected；钉入受理时的目标 ID，btPrepareCycle 发现身份不一致即失败，
    // 由编排按新 selected 重新派发。
    cycle_target_id_ = goal->target_id;
    onStart(std::make_shared<Trigger::Request>(), trigger_response, true);
  }
  if (!trigger_response->success) {
    auto result = std::make_shared<RunTargetCycle::Result>();
    result->outcome = RunTargetCycle::Result::FAILED;
    result->reason = trigger_response->message;
    result->recovery_required = contact_recovery_required_.load();
    // 启动即失败：计时未启动则两数组为空，符合"未经历的阶段不出现"契约。
    fillStageDurations(result);
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
      // A13：CycleState→TargetPhase 投影随反馈下发（此前恒 0/TARGET_IDLE，
      // 编排器批次过程线的目标阶段在周期内停在 IDLE）。
      feedback->state.target_phase = targetPhase(current_state_);
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
  // 阶段耗时埋点：成功/取消/失败终局一律填充已历经阶段（含取消路径）。
  fillStageDurations(result);
  // 线程可 join 后必须兜住 shutdown 竞态下的上报异常，避免 std::terminate。
  try {
    if (outcome == CycleOutcome::SUCCEEDED) {
      result->outcome = RunTargetCycle::Result::SUCCEEDED;
      goal_handle->succeed(result);
    } else if (outcome == CycleOutcome::CANCELED || goal_handle->is_canceling()) {
      // 取消终局显式上报 outcome=CANCELED（2026-08 起替代复用 FAILED）：
      // 编排器据 outcome 判别"操作员跳过"与"暂停/立即取消"语义。
      // recovery 路径不进本分支（终局枚举为 RECOVERY_REQUIRED，走下方 abort）。
      result->outcome = RunTargetCycle::Result::CANCELED;
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
  const ScopedTimer timer(get_logger(), "start_cycle", &callback_timing_);
  // 运动输出权限绑定 Active 态（A8）：start_cycle 与 action 派生的周期共用本入口。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  if (!action_driven) {
    // 手动 Trigger 周期恒为 FULL 语义：清掉上一 action 周期可能残留的
    // observe-only 标志（action 路径由 executeAction 按 goal.mode 显式设置）。
    cycle_observe_only_.store(false);
  }
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
    // 启动前目标检查按周期生效目标取快照（OBSERVE_ONLY=锁定集锚点缓存的
    // goal 目标，其余=感知 selected 缓存；语义见 bt_nodes.cpp
    // cycleTargetSnapshot）。OBSERVE_ONLY 受理时锁定集命中，但受理到启动
    // 之间可能解锁/换批次，此处必须按同一数据源复核。
    const bool observe_only =
      cycle_observe_only_.load() && !cycle_target_id_.empty();
    const auto target = cycleTargetSnapshot();
    if (!target || target->id.empty()) {
      running_.store(false);
      response->success = false;
      response->message = observe_only ?
        "goal 目标在锁定集锚点缓存中无有效锚点（受理后已解锁/换批次）" :
        "没有可用的 selected_target 初始几何";
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
  // 阶段耗时计时随周期真正启动开始（此前一切拒绝路径不计时）。
  startCycleTiming();
  worker_ = std::thread(&ApproachGraspNode::runCycle, this);
  response->success = true;
  response->message = execution_enabled_.load() ? "已启动主动视觉靠近周期" :
    "已启动只规划预览（不会发送运动）";
}

void ApproachGraspNode::onCancel(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "cancel_cycle", &callback_timing_);
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
  // arm 是运动类入口（A8）：非 Active 一律拒绝（含解除 arm——Active 权限关闭时
  // on_deactivate 已自动撤 arm，无需外部再操作）。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = "拒绝 arm 操作: " + motion_reason;
    return;
  }
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
