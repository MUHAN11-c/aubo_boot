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
#include "cycle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <peach_interfaces/msg/failure_code.hpp>
#include <peach_interfaces/msg/harvest_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// === cycle_action.cpp ===
using namespace std::chrono_literals;

namespace peach_manipulation_skills
{
using FailureCode = peach_interfaces::msg::FailureCode;

namespace
{

double axisAngleDeg(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double na = first.norm();
  const double nb = second.norm();
  if (na < 1e-9 || nb < 1e-9) {
    return 180.0;
  }
  const double cosine = std::clamp(first.dot(second) / (na * nb), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / std::acos(-1.0);
}

}  // namespace

// A13：targetPhase 投影字面量与 HarvestState.msg 的 TARGET_* 常量双向钉死
// （防消息常量重排后投影静默漂移）。
using HarvestStateMsg = peach_interfaces::msg::HarvestState;
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

rclcpp_action::GoalResponse ManipulationSkillsNode::onActionGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const ExecuteTarget::Goal> goal)
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
    (goal->mode != ExecuteTarget::Goal::PREVIEW &&
    goal->mode != ExecuteTarget::Goal::OBSERVE_ONLY &&
    goal->mode != ExecuteTarget::Goal::FULL &&
    goal->mode != ExecuteTarget::Goal::PREGRASP_ONLY) ||
    running_.load() || contact_recovery_required_.load())
  {
    RCLCPP_WARN(
      get_logger(), "拒绝目标请求 %s: 周期运行中/恢复待确认/请求非法",
      goal->target_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // FULL/PREVIEW 与 OBSERVE_ONLY 同一受理门：以 ExecuteTarget.goal.target_id
  // 为准，命中锁定集有效锚点即可；未锁定时回退感知 selected 缓存（单目标
  // 手动周期）。编排器选择权在 peach_task_executor，不再要求 selected 字段。
  const auto locked = cache_.lockedTargetGateSample(goal->target_id);
  if (locked.id == goal->target_id && locked.valid) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  const auto target = targetSnapshot();
  if (target && target->id == goal->target_id) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  RCLCPP_WARN(
    get_logger(), "拒绝目标请求 %s: 不在锁定集且缓存目标=%s",
    goal->target_id.c_str(),
    target ? target->id.c_str() : "（无有效锚点）");
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ManipulationSkillsNode::onActionCancel(
  const std::shared_ptr<RunTargetGoalHandle>)
{
  const ScopedTimer timer(get_logger(), "action_cancel", &callback_timing_);
  cancel_requested_.store(true);
  if (move_group_) {move_group_->stop();}
  if (grasp_task_) {grasp_task_->cancel();}
  cache_.notifyAll();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationSkillsNode::onActionAccepted(
  const std::shared_ptr<RunTargetGoalHandle> goal_handle)
{
  // action 执行线程保持可 join：析构时先置取消标志再回收，避免 detach 后
  // 线程在 shutdown 之后访问已销毁成员。同一时刻至多一个周期在运行。
  if (action_thread_.joinable()) {
    action_thread_.join();
  }
  action_thread_ = std::thread([this, goal_handle]() {executeAction(goal_handle);});
}

void ManipulationSkillsNode::executeAction(
  const std::shared_ptr<RunTargetGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto trigger_response = std::make_shared<Trigger::Response>();
  if (goal->mode == ExecuteTarget::Goal::PREVIEW) {
    cycle_observe_only_.store(false);
    cycle_pregrasp_only_.store(false);
    cycle_skip_observation_.store(false);
    previewContact(false, trigger_response);
  } else {
    // Action 是自动编排专用入口；手动 Trigger 仍要求每周期单独 arm。
    if (execution_enabled_.load()) {execution_armed_.store(true);}
    // OBSERVE_ONLY：BT 在 FinalizeAndValidate 后经 IsObserveOnly 分支短路，
    // 不进 MTC/工具/撤离段（见 behavior_tree.xml report_or_grasp）。
    cycle_observe_only_.store(goal->mode == ExecuteTarget::Goal::OBSERVE_ONLY);
    cycle_pregrasp_only_.store(goal->mode == ExecuteTarget::Goal::PREGRASP_ONLY);
    cycle_skip_observation_.store(goal->skip_observation);
    // goal 钉死（ExecuteTarget.goal.target_id）：受理到 Prepare 快照之间感知可能切换
    // selected；钉入受理时的目标 ID，btPrepareCycle 发现身份不一致即失败，
    // 由编排按新 selected 重新派发。
    cycle_target_id_ = goal->target_id;
    cycle_deposit_ok_ = false;
    cycle_deposit_skipped_m8_ = false;
    cycle_deposit_reason_.clear();
    onStart(std::make_shared<Trigger::Request>(), trigger_response, true);
  }
  if (!trigger_response->success) {
    auto result = std::make_shared<ExecuteTarget::Result>();
    result->outcome = ExecuteTarget::Result::FAILED;
    result->reason = trigger_response->message;
    result->recovery_required = contact_recovery_required_.load();
    // 启动即失败：计时未启动则两数组为空，符合"未经历的阶段不出现"契约。
    fillStageDurations(result);
    fillExecuteResults(result);
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
    auto feedback = std::make_shared<ExecuteTarget::Feedback>();
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
  auto result = std::make_shared<ExecuteTarget::Result>();
  result->reason = cycle_result.reason;
  result->recovery_required = cycle_result.recovery_required;
  // 阶段耗时埋点：成功/取消/失败终局一律填充已历经阶段（含取消路径）。
  fillStageDurations(result);
  const bool succeeded = outcome == CycleOutcome::SUCCEEDED;
  const bool canceled =
    outcome == CycleOutcome::CANCELED || goal_handle->is_canceling();
  if (succeeded) {
    result->outcome = ExecuteTarget::Result::SUCCEEDED;
  } else if (canceled) {
    result->outcome = ExecuteTarget::Result::CANCELED;
  } else {
    result->outcome = pending_outcome_.load();
  }
  // outcome_record 必须在最终 outcome 赋值后生成，避免默认 0 污染失败记账。
  fillExecuteResults(result);
  // 线程可 join 后必须兜住 shutdown 竞态下的上报异常，避免 std::terminate。
  try {
    if (succeeded) {
      goal_handle->succeed(result);
    } else if (canceled) {
      // 取消终局显式上报 outcome=CANCELED（2026-08 起替代复用 FAILED）：
      // 编排器据 outcome 判别"操作员跳过"与"暂停/立即取消"语义。
      // recovery 路径不进本分支（终局枚举为 RECOVERY_REQUIRED，走下方 abort）。
      goal_handle->canceled(result);
    } else {
      // abort 路径按 BT 失败点记录的 pending_outcome_ 分级（质量/不可达/失败）。
      goal_handle->abort(result);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "action 终局上报失败（可能正在 shutdown）: %s", error.what());
  }
}

void ManipulationSkillsNode::onStart(
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
    cycle_pregrasp_only_.store(false);
    cycle_skip_observation_.store(false);
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
  pending_outcome_.store(ExecuteTarget::Result::FAILED);
  cycle_action_driven_ = action_driven;
  // 阶段耗时计时随周期真正启动开始（此前一切拒绝路径不计时）。
  startCycleTiming();
  worker_ = std::thread(&ManipulationSkillsNode::runCycle, this);
  response->success = true;
  response->message = execution_enabled_.load() ? "已启动主动视觉靠近周期" :
    "已启动只规划预览（不会发送运动）";
}

void ManipulationSkillsNode::onCancel(
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

void ManipulationSkillsNode::onAcknowledgeRecovery(
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

void ManipulationSkillsNode::onQuery(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  response->success = true;
  response->message = state_json_.dump();
}

void ManipulationSkillsNode::onArm(
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

rclcpp_action::GoalResponse ManipulationSkillsNode::onSurveyGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const SurveyScene::Goal> goal)
{
  (void)goal;
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    RCLCPP_WARN(get_logger(), "拒绝 SurveyScene: %s", motion_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (running_.load() || contact_recovery_required_.load()) {
    RCLCPP_WARN(get_logger(), "拒绝 SurveyScene: 周期占用或待恢复");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationSkillsNode::onSurveyCancel(
  const std::shared_ptr<SurveyGoalHandle>)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationSkillsNode::onSurveyAccepted(
  const std::shared_ptr<SurveyGoalHandle> goal_handle)
{
  if (survey_thread_.joinable()) {
    survey_thread_.join();
  }
  survey_thread_ = std::thread([this, goal_handle]() {executeSurvey(goal_handle);});
}

void ManipulationSkillsNode::executeSurvey(
  const std::shared_ptr<SurveyGoalHandle> goal_handle)
{
  auto result = std::make_shared<SurveyScene::Result>();
  auto response = std::make_shared<Trigger::Response>();
  // 动作入口与 ExecuteTarget 一致：execution.enabled 时自动一次性 arm。
  if (execution_enabled_.load()) {execution_armed_.store(true);}
  onGoToPhotoPose(std::make_shared<Trigger::Request>(), response);
  execution_armed_.store(false);
  result->snapshot_id = last_snapshot_id_;
  result->degraded = !last_target_set_locked_ || last_observation_count_ == 0;
  result->message = response->message;
  result->scene_epoch = 0;
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    return;
  }
  if (!response->success) {
    goal_handle->abort(result);
    return;
  }
  goal_handle->succeed(result);
}

}  // namespace peach_manipulation_skills

// === bt_nodes.cpp ===
namespace peach_manipulation_skills
{
namespace
{
// 跟踪状态枚举 → 中文标签（再确认失败原因文案用；常量为
// PeachTargetObservation.msg 的 tracking_status 枚举，255=缓存未知）。
std::string trackingStatusLabel(uint8_t status)
{
  using Observation = peach_interfaces::msg::PeachTargetObservation;
  switch (status) {
    case Observation::OBSERVED:
      return "OBSERVED";
    case Observation::OCCLUDED:
      return "OCCLUDED(遮挡)";
    case Observation::LOST:
      return "LOST(跟踪丢失)";
    case Observation::INVALID:
      return "INVALID";
    case Observation::OUT_OF_VIEW:
      return "OUT_OF_VIEW(出视野)";
    case Observation::DEPTH_VOID:
      return "DEPTH_VOID(深度空洞)";
    default:
      return "UNKNOWN";
  }
}
}  // namespace
// 数据快照薄壳：统一从 cache_ 取一致性快照，供 BT 节点体与运动接口使用。
QualitySnapshot ManipulationSkillsNode::qualitySnapshot()
{
  return cache_.qualitySnapshot();
}

std::optional<CachedTarget> ManipulationSkillsNode::targetSnapshot()
{
  return cache_.targetSnapshot();
}

// 周期生效目标快照（阶段 E 残局抬质量能力端）。
// 为什么按周期分流而不是在执行接受时把 goal 目标写进 selected 缓存：
// 覆盖 target_ 会污染感知驱动的四源调和（selected 身份/精化/决策缓存的 ID
// 一致性全部以 target_ 为准），周期结束后还会留下"假 selected"干扰后续
// 周期；而锁定集锚点缓存是独立数据源，goal 钉入 ID（cycle_target_id_）在
// 受理时已写好，执行体按"本周期生效目标"取快照即可，零侵入既有调和语义。
// FULL/PREVIEW/手动周期（cycle_observe_only_=false）恒退化为 selected 缓存。
std::optional<CachedTarget> ManipulationSkillsNode::cycleTargetSnapshot()
{
  if (!cycle_target_id_.empty()) {
    auto locked = cache_.lockedTargetSnapshot(cycle_target_id_);
    if (locked) {return locked;}
  }
  return cache_.targetSnapshot();
}

std::vector<Eigen::Vector3d> ManipulationSkillsNode::observedDirectionsSnapshot()
{
  return cache_.observedDirections();
}

std::optional<CachedRefined> ManipulationSkillsNode::refinedSnapshot()
{
  return cache_.refinedSnapshot();
}

std::string ManipulationSkillsNode::graspDecisionTargetSnapshot()
{
  return cache_.graspDecisionTarget();
}

bool ManipulationSkillsNode::waitForNewView(std::size_t previous_views)
{
  return cache_.waitForNewView(previous_views, effectiveFrameWaitS(), cancel_requested_);
}

bool ManipulationSkillsNode::waitForNewStation(std::size_t previous_stations)
{
  return cache_.waitForNewStation(
    previous_stations, effectiveFrameWaitS(), cancel_requested_);
}

bool ManipulationSkillsNode::waitForFreshTarget(double after_s)
{
  return waitForFreshCycleTarget(after_s, effectiveFrameWaitS());
}

bool ManipulationSkillsNode::waitForFreshCycleTarget(
  double after_s, double window_s, bool live_observation_required)
{
  // 数据源随周期生效目标走（同 cycleTargetSnapshot 的分流理由）：
  // OBSERVE_ONLY 等 goal 目标的锁定集锚点新鲜帧，其余等 selected 新鲜帧。
  // live_observation_required=false 仅再确认：已 OBSERVED 则用当前锚点。
  if (!cycle_target_id_.empty()) {
    return cache_.waitForFreshLockedTarget(
      cycle_target_id_, after_s, window_s, cancel_requested_,
      live_observation_required);
  }
  return cache_.waitForFreshTarget(
    after_s, window_s, cancel_requested_, live_observation_required);
}

bool ManipulationSkillsNode::waitForRefined(const std::string & target_id)
{
  // 超时按协议 2.7-FINALIZE 的 T(refined) 帧率自适应（effectiveRefinedWaitS）。
  return cache_.waitForRefined(target_id, effectiveRefinedWaitS(), cancel_requested_);
}

void ManipulationSkillsNode::registerBehaviorTreeNodes()
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
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 之后、MTC 接触段之前的
  // 最后一道验证关（新鲜观测+锚点漂移+摆动平息）。
  bt_factory_.registerSimpleAction(
    "ReconfirmTarget", [this](BT::TreeNode &) {return btReconfirmTarget();});
  bt_factory_.registerSimpleCondition(
    "IsGraspDisabled", [this](BT::TreeNode &) {
      return grasp_enabled_.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  // OBSERVE_ONLY 周期短路条件：FinalizeAndValidate 后直接落 PLAN_READY 终态，
  // 跳过 MTC/工具/撤离段（优先级高于 IsGraspDisabled，见 behavior_tree.xml）。
  bt_factory_.registerSimpleCondition(
    "IsObserveOnly", [this](BT::TreeNode &) {
      return cycle_observe_only_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  bt_factory_.registerSimpleCondition(
    "IsSkipObservation", [this](BT::TreeNode &) {
      return cycle_skip_observation_.load() ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  bt_factory_.registerSimpleCondition(
    "IsPregraspOnly", [this](BT::TreeNode &) {
      return cycle_pregrasp_only_.load() ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  bt_factory_.registerSimpleAction(
    "ReportObserveOnly", [this](BT::TreeNode &) {return btReportObserveOnly();});
  bt_factory_.registerSimpleAction(
    "ReportReadyForGrasp", [this](BT::TreeNode &) {return btReportReady();});
  bt_factory_.registerSimpleAction(
    "MTCApproachAndInsert", [this](BT::TreeNode &) {return btMtcApproachAndInsert();});
  bt_factory_.registerSimpleAction(
    "MovePregrasp", [this](BT::TreeNode &) {return btMovePregrasp();});
  bt_factory_.registerSimpleAction(
    "VerifyPregrasp", [this](BT::TreeNode &) {return btVerifyPregrasp();});
  bt_factory_.registerSimpleAction(
    "HoldPregrasp", [this](BT::TreeNode &) {return btHoldPregrasp();});
  bt_factory_.registerSimpleAction(
    "PlanSleeveAndReverseRetreat", [this](BT::TreeNode &) {
      return btPlanSleeveAndReverseRetreat();
    });
  bt_factory_.registerSimpleAction(
    "SleeveLinear", [this](BT::TreeNode &) {return btSleeveLinear();});
  bt_factory_.registerSimpleAction(
    "VerifyCutHold", [this](BT::TreeNode &) {return btVerifyCutHold();});
  bt_factory_.registerSimpleAction(
    "ActuateTool", [this](BT::TreeNode &) {return btActuateTool();});
  bt_factory_.registerSimpleAction(
    "ActuateCutter", [this](BT::TreeNode &) {return btActuateTool();});
  bt_factory_.registerSimpleAction(
    "VerifyCut", [this](BT::TreeNode &) {return btVerifyCut();});
  bt_factory_.registerSimpleAction(
    "ExecuteReservedReverseRetreat", [this](BT::TreeNode &) {return btMtcRetreat();});
  bt_factory_.registerSimpleAction(
    "MTCRetreat", [this](BT::TreeNode &) {return btMtcRetreat();});
  bt_factory_.registerSimpleAction(
    "ReturnHarvestStow", [this](BT::TreeNode &) {return btReturnHarvestStow();});
  bt_factory_.registerSimpleAction(
    "VerifyHarvestOutcome", [this](BT::TreeNode &) {return btVerifyHarvestOutcome();});
  bt_factory_.registerSimpleAction(
    "DepositToStation", [this](BT::TreeNode &) {return btDepositToStation();});
  bt_factory_.registerSimpleAction(
    "CompleteTarget", [this](BT::TreeNode &) {return btCompleteTarget();});
}

BT::NodeStatus ManipulationSkillsNode::btFailure(const std::string & reason)
{
  bt_failure_reason_ = reason;
  setState(CycleState::FAILED, reason);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ManipulationSkillsNode::btPrepareCycle()
{
  // 周期"黑板"锚定本周期生效目标：OBSERVE_ONLY=锁定集锚点缓存的 goal 目标，
  // 其余=感知 selected（见 cycleTargetSnapshot 注释）。
  cycle_target_ = cycleTargetSnapshot();
  cycle_refined_.reset();
  cycle_candidates_.clear();
  cycle_degraded_grasp_ = false;
  cycle_pregrasp_verified_ = false;
  cycle_sleeve_planned_ = false;
  cycle_cut_command_accepted_ = false;
  cycle_cut_confirmed_ = false;
  cycle_retreat_confirmed_ = false;
  cycle_completion_level_ = 0;
  cycle_failure_code_ = FailureCode::NONE;
  cycle_pregrasp_msg_ = peach_interfaces::msg::PregraspVerification();
  cycle_contact_transaction_id_ = cycle_target_id_ + ":contact";
  tool_actuator_.resetSafe();
  if (!cycle_target_) {
    return btFailure("周期目标（selected/锁定集锚点）在启动后失效");
  }
  // goal 钉死校验（ExecuteTarget.goal.target_id）：action 受理到本快照之间感知若已切换
  // selected，身份不一致即周期失败，由编排按新 selected 重新派发；
  // 手动周期钉入值为空，直接采纳当下快照身份。OBSERVE_ONLY 周期的快照按
  // goal ID 取自锁定集缓存，身份一致由缓存键保证（条目消失走上方空快照
  // 失败分支），本校验恒通过。
  if (!cycle_target_id_.empty() && cycle_target_->id != cycle_target_id_) {
    return btFailure(
      "目标身份变更: goal=" + cycle_target_id_ +
      " 当前 selected=" + cycle_target_->id);
  }
  cycle_target_id_ = cycle_target_->id;
  setState(CycleState::PLAN_OBSERVATION, "行为树生成目标导向主动视点");
  const auto base_from_camera = motion_->lookupTransform(base_frame_, camera_frame_);
  if (!base_from_camera) {
    return btFailure("无法取得当前相机位姿");
  }
  ViewContext view_context;
  view_context.target = cycle_target_->center;
  view_context.current_camera_position = base_from_camera->translation();
  view_context.observed_directions = observedDirectionsSnapshot();
  view_context.bbox_valid = cycle_target_->bbox_valid;
  view_context.bbox_x = cycle_target_->bbox_x;
  view_context.bbox_y = cycle_target_->bbox_y;
  view_context.bbox_w = cycle_target_->bbox_w;
  view_context.bbox_h = cycle_target_->bbox_h;
  view_context.image_width = cycle_target_->image_width;
  view_context.image_height = cycle_target_->image_height;
  view_context.neighbor_centers = cache_.lockedNeighborCenters(cycle_target_id_);
  cycle_candidates_ = view_planner_->generate(view_context);
  publishViewMarkers(cycle_target_->center, cycle_candidates_);
  if (cycle_candidates_.empty()) {
    return btFailure("没有生成可用观察视点");
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btPlanPreview()
{
  // OBSERVE_ONLY 必须真走臂采多视角；plan-only 不得伪装成观察成功。
  if (cycle_observe_only_.load()) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure(
      "observe_only 需要 execution.enabled=true 才能采多视角；当前为只规划预览");
  }
  for (const auto & candidate : cycle_candidates_) {
    if (motion_->planOrMoveCamera(
        candidate.camera_pose, "PTP", false, candidate.label, true))
    {
      cycle_terminal_state_ = CycleState::PLAN_READY;
      cycle_terminal_message_ = "行为树只规划预览成功；未发送任何运动";
      return BT::NodeStatus::SUCCESS;
    }
  }
  return btFailure("所有候选观察位姿均不可规划");
}

BT::NodeStatus ManipulationSkillsNode::btAcquireViews()
{
  // 扫描预算制（2.13-E2 / 2.7-OBSERVE，判定纯核 ScanBudget）：
  //   - 质量优先：完整质量门放行即停，不为凑主动移动次数继续运动；
  //   - 下限保证：质量尚未达标且有效视点未达 min_effective_views_ 时，
  //     不得按预算提前收口；
  //   - 预算自适应：运行预算 = max(scan_time_budget_s_,
  //     budget_cost_margin×移动成本EMA)；
  //     剩余预算按实测 EMA 换不起一个视点即预测性收口，强制 finalize 走降级链；
  //   - maximum_moves_ 仅兜底（候选规划/移动全失败的极端场景）。
  const ScanBudget scan_budget(ScanBudgetConfig{
      maximum_scan_moves_, min_effective_views_, scan_time_budget_s_,
      scan_budget_cost_margin_});
  int moves = 0;
  int effective_views = 0;
  bool budget_exhausted = false;
  std::vector<std::string> attempted;
  const double scan_start_s = now().seconds();
  setState(CycleState::WAIT_FRAME, "当前位采帧，不环绕");
  {
    const std::size_t before_stay = qualitySnapshot().captured_views;
    if (waitForFreshTarget(scan_start_s) && waitForNewView(before_stay)) {
      RCLCPP_INFO(
        get_logger(),
        "当前位已采重建帧，不环绕；基线未过再做最多两次短 PTP");
    } else {
      RCLCPP_WARN(get_logger(), "当前位采帧未完成，将做一次短 PTP 接近");
    }
  }
  while (!cancel_requested_.load()) {
    const GateResult finalize_gate = quality_gate_->readyToFinalize(qualitySnapshot());
    const double elapsed_s = now().seconds() - scan_start_s;
    const ScanVerdict verdict = scan_budget.poll(
      finalize_gate.allowed, moves, effective_views, elapsed_s,
      scan_move_cost_ema_s_);
    if (verdict == ScanVerdict::CONVERGED) {
      break;
    }
    if (verdict == ScanVerdict::BUDGET_EXHAUSTED) {
      budget_exhausted = true;
      RCLCPP_WARN(
        get_logger(),
        "观察预算收口（已耗时 %.1fs / 预算 %.1fs，移动成本EMA %.1fs，"
        "有效视点 %d/%d）：%s，强制 finalize 走降级抓取链",
        elapsed_s, scan_budget.effectiveBudgetS(scan_move_cost_ema_s_),
        scan_move_cost_ema_s_, effective_views, min_effective_views_,
        finalize_gate.reason.c_str());
      break;
    }
    if (verdict == ScanVerdict::MOVES_EXHAUSTED) {
      break;
    }
    const auto current_camera = motion_->lookupTransform(base_frame_, camera_frame_);
    if (!current_camera) {
      return btFailure("扫描中无法取得相机位姿");
    }
    // 每次规划前用缓存中的最新观测锚点：跨视角锚点偏差在近距获得观测后
    // 自动纠偏，避免按拍照位姿的旧锚点把目标指到画面外（stale 主因）。
    // OBSERVE_ONLY 周期取 goal 目标的锁定集锚点（见 cycleTargetSnapshot）。
    const auto latest_target = cycleTargetSnapshot();
    const Eigen::Vector3d scan_center =
      (latest_target && latest_target->id == cycle_target_id_) ?
      latest_target->center : cycle_target_->center;
    ViewContext scan_context;
    scan_context.target = scan_center;
    scan_context.current_camera_position = current_camera->translation();
    scan_context.observed_directions = observedDirectionsSnapshot();
    if (latest_target && latest_target->id == cycle_target_id_) {
      scan_context.bbox_valid = latest_target->bbox_valid;
      scan_context.bbox_x = latest_target->bbox_x;
      scan_context.bbox_y = latest_target->bbox_y;
      scan_context.bbox_w = latest_target->bbox_w;
      scan_context.bbox_h = latest_target->bbox_h;
      scan_context.image_width = latest_target->image_width;
      scan_context.image_height = latest_target->image_height;
    } else if (cycle_target_) {
      scan_context.bbox_valid = cycle_target_->bbox_valid;
      scan_context.bbox_x = cycle_target_->bbox_x;
      scan_context.bbox_y = cycle_target_->bbox_y;
      scan_context.bbox_w = cycle_target_->bbox_w;
      scan_context.bbox_h = cycle_target_->bbox_h;
      scan_context.image_width = cycle_target_->image_width;
      scan_context.image_height = cycle_target_->image_height;
    }
    scan_context.neighbor_centers = cache_.lockedNeighborCenters(cycle_target_id_);
    cycle_candidates_ = view_planner_->generate(scan_context);
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
      setState(
        CycleState::MOVE_TO_VIEW,
        candidate.label + " score=" + std::to_string(candidate.score));
      // 观察只走短 PTP：LIN 失败后的 OMPL 会绕行，后续机位也不再 LIN。
      const std::string planner = "PTP";
      // 移动前记下机位数：到位后同机位连帧不加机位，必须比移动前多一个。
      const std::size_t stations_before_move = observedDirectionsSnapshot().size();
      // 移动成本计时起点：含规划+执行+到位后等帧（预规划预算估计的实测输入）。
      const double move_start_s = now().seconds();
      if (!motion_->planOrMoveCamera(
          candidate.camera_pose, planner, true, candidate.label, false))
      {
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
        // 无新鲜目标观测则本视点必无有效掩膜帧，不再等重建成帧，直接换视点；
        // 本次移动不计有效视点、不进成本 EMA（失败样本会带偏预算估计）。
        RCLCPP_WARN(get_logger(), "视点到达但等待新鲜目标观测超时，换下一视点");
        break;
      }
      // 停走采帧（温室多视：到位静止后再积分）。同机位连帧会加 captured_views
      // 但不加机位；等 view_directions 相对移动前增加，避免 60ms 内收口丢掉下一颗。
      if (!waitForNewStation(stations_before_move)) {
        RCLCPP_WARN(get_logger(), "视点到达但未形成新机位，换下一视点");
        break;
      }
      ++effective_views;
      // 移动+等帧成本 EMA（0.7/0.3，与 trackFrameInterval 同形状）：预算制
      // （2.13-E2）预测"剩余预算能否再换一个有效视点"的实测输入，跨周期保留。
      const double move_cost_s = now().seconds() - move_start_s;
      scan_move_cost_ema_s_ = scan_move_cost_ema_s_ > 0.0 ?
        0.7 * scan_move_cost_ema_s_ + 0.3 * move_cost_s : move_cost_s;
      break;
    }
    if (!moved) {
      // 候选穷尽：质量门已放行时直接收口 finalize（有效视点下限防的是"没看
      // 够就收"，质量达标说明既有覆盖足够）；未放行才按目标不可达跳过。
      if (finalize_gate.allowed) {
        break;
      }
      // 视角规划不可达：批次侧可按 SKIPPED_UNREACHABLE 直接跳过该目标。
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
      return btFailure("剩余候选视点均不可达或规划失败");
    }
  }
  if (cancel_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  const GateResult gate = quality_gate_->readyToFinalize(qualitySnapshot());
  if (!gate.allowed) {
    if (budget_exhausted) {
      // FULL：预算收口带现有覆盖强制 finalize，精化不达标由候选锚点降级抓取。
      // OBSERVE_ONLY：没有精化产物就算失败，不能把重建未绑定/帧不足当成功。
      if (cycle_observe_only_.load()) {
        pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
        return btFailure("观察预算收口但重建未收敛: " + gate.reason);
      }
      return BT::NodeStatus::SUCCESS;
    }
    // 移动次数上限内采集帧不足/不收敛（含有效视点未达下限）：按目标不可达跳过。
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    return btFailure(
      "达到扫描上限仍未收敛（有效视点 " + std::to_string(effective_views) +
      "/" + std::to_string(min_effective_views_) + "）: " + gate.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btFinalizeAndValidate()
{
  setState(CycleState::FINALIZE, "等待重建精化几何（BuildTargetModel）");
  const bool refined_arrived = waitForRefined(cycle_target_id_);
  if (cycle_observe_only_.load()) {
    if (!refined_arrived) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
      return btFailure(
        "observe_only 未等到绑定目标的 TSDF/精化几何: " + cycle_target_id_);
    }
    return BT::NodeStatus::SUCCESS;
  }
  const QualitySnapshot snapshot = qualitySnapshot();
  const GateResult gate = cycle_pregrasp_only_.load() ?
    quality_gate_->readyToApproach(snapshot) :
    quality_gate_->readyToGrasp(snapshot);
  if (!gate.allowed) {
    RCLCPP_WARN(
      get_logger(),
      "grasp quality gate denied: %s (axis_angle_deg=%.2f pregrasp_only=%s)",
      gate.reason.c_str(), snapshot.axis_angle_deg,
      cycle_pregrasp_only_.load() ? "true" : "false");
  }
  const bool grasp_ready = refined_arrived && gate.allowed &&
    (cycle_pregrasp_only_.load() ||
    graspDecisionTargetSnapshot() == cycle_target_id_);
  const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    return btFailure("无法取得 tip 到 tool 的变换");
  }
  if (grasp_ready) {
    cycle_refined_ = refinedSnapshot();
    if (!cycle_refined_) {
      return btFailure("精化位姿数据不存在");
    }
    // 再确认漂移判定必须和后续新鲜观测用同一套锚点定义（感知底/颈中点）。
    // 若拿 TSDF 中点去比单帧检测中点，现场曾把 ~5cm 的定义差当成果实移动，
    // 再把精化入口整包平移，直线插入在错误位置走不完。
    const auto live_anchor = cycleTargetSnapshot();
    cycle_reference_anchor_ = (live_anchor && live_anchor->valid) ?
      live_anchor->center :
      0.5 * (cycle_refined_->bottom + cycle_refined_->neck);
    Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
    entry_tool_pose.translation() = cycle_refined_->entry;
    entry_tool_pose.linear() = ViewPlanner::toolOrientation(
      cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
    cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
    cycle_travel_m_ = insertionTravel(*cycle_refined_);
    const auto quality = qualitySnapshot();
    RCLCPP_INFO(
      get_logger(),
      "接触几何（核对方向） target=%s entry=[%.3f %.3f %.3f] axis=[%.3f %.3f %.3f] "
      "travel=%.3fm axis_angle_deg=%.2f",
      cycle_target_id_.c_str(),
      cycle_refined_->entry.x(), cycle_refined_->entry.y(), cycle_refined_->entry.z(),
      cycle_refined_->axis.x(), cycle_refined_->axis.y(), cycle_refined_->axis.z(),
      cycle_travel_m_, quality.axis_angle_deg);
    if (grasp_hyp_pub_) {
      peach_interfaces::msg::GraspHypothesis hyp;
      hyp.header.stamp = now();
      hyp.header.frame_id = base_frame_;
      hyp.target_id = cycle_target_id_;
      hyp.entry_pose = tf2::toMsg(cycle_entry_tip_pose_);
      hyp.travel_m = static_cast<float>(cycle_travel_m_);
      hyp.rank_score = 1.0f;
      grasp_hyp_pub_->publish(hyp);
    }
    return BT::NodeStatus::SUCCESS;
  }
  pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
  cycle_failure_code_ = FailureCode::DEGRADED_CONTACT_FORBIDDEN;
  return btFailure(
    cycle_pregrasp_only_.load() ?
    ("预抓取缺少融合几何: " + gate.reason) :
    ("GraspDecision.allowed=false，禁止降级接触: " + gate.reason));
}

BT::NodeStatus ManipulationSkillsNode::btReconfirmTarget()
{
  // 抓取前再确认（2.7-RECONFIRM）：FinalizeAndValidate 已产出入口几何
  // （cycle_refined_/cycle_entry_tip_pose_/cycle_travel_m_，本周期"黑板"成员），
  // 但 finalize 耗时必然超过观测新鲜度窗口，接触段前必须用最新观测复核目标
  // 没飘走、没换身份、没在风里持续摆动，再决定放行 MTC。
  //
  // 回退开关（验证期遗留）：allow_stale_anchor=true 时退化为旧"按静态锚点
  // 继续"行为，直接放行；false（默认）时新鲜度由本节点单点把关。
  if (allow_stale_anchor_) {
    RCLCPP_WARN(
      get_logger(),
      "grasp.allow_stale_anchor=true：跳过抓取前再确认，按静态目标锚点继续"
      "（旧行为，验证期遗留）");
    return BT::NodeStatus::SUCCESS;
  }
  if (!cycle_refined_ || !cycle_refined_->valid) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure("再确认无有效入口几何（FinalizeAndValidate 未产出）");
  }
  setState(
    CycleState::RECONFIRM,
    "抓取前再确认：等待新鲜观测复核身份/锚点漂移/摆动平息");
  // plan-while-waiting（2.13-E3）：再确认等待窗口机械臂静止，用当前锚点后台
  // 预规划 MTC 接近/插入任务（只规划不执行）；再确认通过且锚点漂移 ≤
  // grasp.replan_threshold_m 时由 MTC 节点直接执行预规划解，超限才重规划。
  // allow_stale_anchor 旧行为路径在上方已提前返回，不启动预规划。
  preplan_reuse_ = false;
  launchPreplan();
  ReconfirmPolicy policy(ReconfirmConfig{
      reconfirm_tolerance_m_, reconfirm_max_attempts_, false});
  Eigen::Vector3d reference_anchor = cycle_reference_anchor_;
  // 最近一次新鲜观测样本（REFINED 重算要用其锚点/轴）。
  Eigen::Vector3d latest_anchor = reference_anchor;
  Eigen::Vector3d latest_axis = cycle_refined_->axis;
  while (!cancel_requested_.load()) {
    // 单次尝试窗口：实测帧间隔 EMA 自适应（运行时优先，禁硬编码墙钟）；
    // 摆动等平息留在同一窗口预算内，不因摆动帧重开窗口。
    const double window_s = effectiveReconfirmWaitS();
    const double deadline_s = now().seconds() + window_s;
    double after_s = now().seconds();
    ReconfirmDecision decision;
    while (!cancel_requested_.load()) {
      const double remaining_s = deadline_s - now().seconds();
      bool got_fresh = false;
      if (remaining_s > 1e-3) {
        // 再确认不等 received_s 新戳：锁定目标已是 OBSERVED 时用当前锚点
        // 判漂移（记忆锚点帧仍刷新 updated_s / tracking_status）。
        got_fresh = waitForFreshCycleTarget(
          after_s, remaining_s, false);
      }
      const auto latest = got_fresh ? cycleTargetSnapshot() : std::nullopt;
      if (!latest) {
        // 窗口耗尽/目标失效：累计一次超限（摆动持续整个窗口也走本路径）。
        decision = policy.check(ReconfirmSample::exhausted());
        break;
      }
      after_s = latest->updated_s > 0.0 ? latest->updated_s : latest->received_s;
      latest_anchor = latest->center;
      latest_axis = latest->initial_axis;
      ReconfirmSample sample;
      sample.fresh = true;
      sample.identity_ok = latest->id == cycle_target_id_;
      sample.swinging = latest->swinging;
      sample.anchor = latest_anchor;
      sample.axis = latest_axis;
      sample.anchor_drift_m = (latest_anchor - reference_anchor).norm();
      decision = policy.check(sample);
      // PENDING 仅出现在摆动等平息路径：留在本窗口预算内等下一帧复核。
      if (decision.verdict != ReconfirmVerdict::PENDING) {
        break;
      }
      setState(CycleState::RECONFIRM, decision.reason);
    }
    if (cancel_requested_.load()) {
      return BT::NodeStatus::FAILURE;
    }
    if (decision.verdict == ReconfirmVerdict::PASS) {
      // 预规划复用判定（2.13-E3）：先等有界落定（再确认窗口通常长于 MTC
      // 规划，多数情况已 READY；等待上限 planning_time_s_ 是 MTC stage 自身
      // 的规划预算，不会超产）；READY 且最终锚点漂移 ≤ replan_threshold_m_
      // 才复用，否则清场走内联重规划（由 btMtcApproachAndInsert 执行）。
      preplan_slot_.waitReady(planning_time_s_, cancel_requested_);
      std::string reuse_why;
      preplan_reuse_ = !cancel_requested_.load() &&
        preplan_slot_.reusable(latest_anchor, replan_threshold_m_, reuse_why);
      if (preplan_reuse_) {
        RCLCPP_INFO(
          get_logger(), "%s；MTC 将复用再确认窗口的预规划解", decision.reason.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "%s；预规划不复用（%s），MTC 将内联重规划",
          decision.reason.c_str(), reuse_why.c_str());
        settlePreplanBeforeReplan();
      }
      return BT::NodeStatus::SUCCESS;
    }
    if (decision.verdict == ReconfirmVerdict::REFINED) {
      if (!cycle_degraded_grasp_) {
        // 精化路径：多视 TSDF 入口比单帧检测稳，超容差只计一次超限并再开窗，
        // 不把入口平移到跳变锚点。预规划仍对应原精化几何，槽可保留。
        RCLCPP_WARN(
          get_logger(),
          "%s；保留 TSDF 入口，不按单帧平移", decision.reason.c_str());
        setState(
          CycleState::RECONFIRM,
          decision.reason + "；保留精化入口，等观测回到容差");
        continue;
      }
      // 降级路径来自单帧候选：入口必须跟最新锚点走，旧预规划作废。
      preplan_slot_.discard();
      if (nonzeroFinite(latest_axis)) {
        cycle_refined_->axis = latest_axis.normalized();
      }
      cycle_refined_->entry = degradedEntryPoint(
        latest_anchor, cycle_refined_->axis, cycle_travel_m_, fallback_standoff_m_);
      const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
      if (!tip_from_tool) {
        return btFailure("再确认重算入口时无法取得 tip 到 tool 的变换");
      }
      Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
      entry_tool_pose.translation() = cycle_refined_->entry;
      entry_tool_pose.linear() = ViewPlanner::toolOrientation(
        cycle_refined_->axis, cycle_target_->initial_pose.linear().col(0));
      cycle_entry_tip_pose_ = entry_tool_pose * tip_from_tool->inverse();
      reference_anchor = latest_anchor;
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      setState(CycleState::RECONFIRM, decision.reason);
      continue;
    }
    if (decision.verdict == ReconfirmVerdict::PENDING) {
      // 窗口耗尽计一次超限后重开窗口再试（reason 已含累计次数）。
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      setState(CycleState::RECONFIRM, decision.reason);
      continue;
    }
    // ABORT：放弃本目标，周期终局 SKIPPED_QUALITY；reason 附最近跟踪状态，
    // 便于区分"观测窗口耗尽"是出视野/深度空洞/跟踪丢失中的哪一类。
    std::string reason = decision.reason;
    const auto latest = cycleTargetSnapshot();
    if (latest) {
      reason += "；最近跟踪状态=" + trackingStatusLabel(latest->tracking_status);
      reason += " received_s=" + std::to_string(latest->received_s);
      reason += " updated_s=" + std::to_string(latest->updated_s);
    }
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    return btFailure(reason);
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ManipulationSkillsNode::btReportReady()
{
  cycle_terminal_state_ = CycleState::READY_FOR_GRASP;
  cycle_terminal_message_ = cycle_degraded_grasp_ ?
    "grasp.enabled=false，未执行接触；入口来自感知降级锚点" :
    "精化质量通过；grasp.enabled=false，未执行接触动作";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btReportObserveOnly()
{
  // OBSERVE_ONLY 圆满终态：观察+精化验证段已完成，reason 标 observe_only；
  // PLAN_READY 经 terminalOutcome 映射 SUCCEEDED 上报编排器。
  cycle_terminal_state_ = CycleState::PLAN_READY;
  cycle_terminal_message_ =
    "observe_only 周期完成：仅观察与精化验证，未执行靠近/抓取/工具动作";
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btMtcApproachAndInsert()
{
  std::string reason;
  // 运动输出权限（A8）：deactivate 竞态下周期虽已被取消流程拦截，此处再兜底。
  if (!motionOutputAllowed(reason)) {
    return btFailure("MTC 执行被拒绝: " + reason);
  }
  if (!safetyReady(reason)) {
    return btFailure("MTC 执行前机器人安全门失败: " + reason);
  }
  // 环境几何保护区（阶段 F1）：接触段入口点落入任一保护盒（闭区间，含盒
  // 表面）即拒规划，终局 SKIPPED_UNREACHABLE。本检查放在 MTC 规划前单点
  // 执行，天然覆盖全部入口来源——FinalizeAndValidate 精化入口、降级链
  // degradedEntryPoint 入口、ReconfirmTarget 漂移重算入口；静态几何约束
  // 先于目标新鲜度等待判定，快速失败且不消耗观测窗口预算。
  if (cycle_refined_ && cycle_refined_->valid) {
    if (const auto hit = protectedZoneHit(cycle_refined_->entry, protected_zones_)) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
      return btFailure(
        "入口在保护区（盒#" + std::to_string(*hit) + "，入口点 [" +
        std::to_string(cycle_refined_->entry.x()) + ", " +
        std::to_string(cycle_refined_->entry.y()) + ", " +
        std::to_string(cycle_refined_->entry.z()) + "]），拒绝 MTC 规划");
    }
  }
  if (!cycleTargetReady(cycle_target_id_, reason)) {
    if (reason != "selected_target_stale") {
      return btFailure("MTC 执行前安全门失败: " + reason);
    }
    // finalize 耗时必然超过观测新鲜度窗口：先等一窗新鲜观测再复核；
    // 仍不新鲜时——allow_stale_anchor=false（默认）：新鲜度已由 ReconfirmTarget
    // 单点把关，此处不再二次放行 stale，按质量原因跳过；allow_stale_anchor=true
    // （验证期遗留）：按静态果实处置——锚点来自多视融合/身份记忆，MTC 碰撞
    // 检查兜底，继续执行。
    if (waitForFreshTarget(now().seconds()) &&
      cycleTargetReady(cycle_target_id_, reason))
    {
      RCLCPP_INFO(get_logger(), "MTC 前目标观测已刷新，继续执行");
    } else if (!allow_stale_anchor_) {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
      return btFailure(
        "MTC 执行前目标观测仍陈旧（ReconfirmTarget 后二次把关，"
        "allow_stale_anchor=false 不放行 stale）");
    } else {
      RCLCPP_WARN(
        get_logger(), "MTC 前观测仍陈旧，按静态目标锚点继续（已等待复核）");
    }
  }
  setState(
    CycleState::MTC_APPROACH_INSERT,
    cycle_degraded_grasp_ ?
      "MTC（降级）：沿检测轴短程进入；未对轴则先 PTP 到预抓取" :
      "MTC：沿检测轴短程进入；未对轴则先 PTP 到预抓取");
  GraspTaskResult result;
  if (preplan_reuse_) {
    // 预规划复用（2.13-E3）：再确认 PASS 点已判定 READY 且漂移 ≤
    // replan_threshold_m_；直接执行预规划解，省去串行规划段。入口几何自
    // 预规划启动后未变（REFINED 重算已在再确认段清槽，不可能走到这里）。
    preplan_reuse_ = false;
    if (preplan_thread_.joinable()) {
      preplan_thread_.join();
    }
    preplan_slot_.discard();
    setState(CycleState::MTC_APPROACH_INSERT, "MTC 执行预规划解（plan-while-waiting 命中）");
    result = grasp_task_->executePreplannedApproach();
    if (!result.success && !result.execution_started &&
      result.reason.find("无可用预规划解") != std::string::npos)
    {
      // 兜底：槽判定与任务槽不一致（理论上不发生）时退化为内联规划。
      RCLCPP_WARN(get_logger(), "预规划槽命中但任务缺失，退回内联规划");
      result = grasp_task_->approachAndInsert(
        cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_, true);
    }
  } else {
    // 内联规划前清场：防预规划线程仍占用 GraspTask active 槽造成并发。
    settlePreplanBeforeReplan();
    result = grasp_task_->approachAndInsert(
      cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_, true);
  }
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  if (!result.success) {
    // 规划阶段失败说明目标不可达；执行已启动后的失败维持 FAILED 分级。
    pending_outcome_.store(
      result.execution_started ?
      ExecuteTarget::Result::FAILED : ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    return btFailure("MTC 接近/插入失败: " + result.reason);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btActuateTool()
{
  if (cycle_pregrasp_only_.load()) {
    return btFailure("PREGRASP_ONLY 不得 SetIO");
  }
  if (!tool_enabled_.load()) {
    setState(CycleState::ACTUATE_TOOL, "tool.enabled=false，跳过末端 IO");
    cycle_cut_command_accepted_ = false;
    cycle_cut_confirmed_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  setState(CycleState::ACTUATE_TOOL, "触发末端工具剪切");
  std::string reason;
  ToolCommandContext ctx;
  ctx.run_id = cycle_target_id_;
  ctx.target_id = cycle_target_id_;
  ctx.contact_transaction_id = cycle_contact_transaction_id_;
  if (!tool_actuator_.arm(ctx, reason) || !tool_actuator_.sendCut(reason)) {
    const auto retreat = grasp_task_->retreat(
      cycle_refined_->axis, cycle_travel_m_ + mtc_approach_along_axis_m_, true);
    if (retreat.success) {
      contact_recovery_required_.store(false);
    }
    cycle_failure_code_ = FailureCode::CUT_COMMAND_FAILED;
    return btFailure("末端工具失败: " + reason + "；撤离: " + retreat.reason);
  }
  cycle_cut_command_accepted_ = true;
  cycle_completion_level_ = std::max(
    cycle_completion_level_,
    ExecuteTarget::Result::LEVEL_CUT_COMMAND_ACCEPTED);
  std::string feedback_reason;
  if (tool_actuator_.confirmFeedback(false, feedback_reason)) {
    cycle_cut_confirmed_ = true;
    cycle_completion_level_ = std::max(
      cycle_completion_level_, ExecuteTarget::Result::LEVEL_CUT_CONFIRMED);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btMtcRetreat()
{
  setState(CycleState::MTC_RETREAT, "MTC 沿插入反方向保持直线撤离");
  const auto result = grasp_task_->retreat(
    cycle_refined_->axis, cycle_travel_m_ + mtc_approach_along_axis_m_, true);
  if (!result.success) {
    cycle_failure_code_ = FailureCode::RETREAT_FAILED;
    return btFailure("MTC 抓取后撤离失败，需要人工处理: " + result.reason);
  }
  contact_recovery_required_.store(false);
  cycle_retreat_confirmed_ = true;
  cycle_completion_level_ = std::max(
    cycle_completion_level_,
    ExecuteTarget::Result::LEVEL_RETREAT_CONFIRMED);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btMovePregrasp()
{
  setState(CycleState::MTC_APPROACH_INSERT, "PTP 到轴上预抓取");
  if (!cycle_refined_ || !grasp_task_) {
    return btFailure("预抓取无入口几何");
  }
  const auto result = grasp_task_->moveToPregrasp(
    cycle_entry_tip_pose_, cycle_refined_->axis, true);
  if (!result.success) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    cycle_failure_code_ = FailureCode::SLEEVE_PLAN_FAILED;
    return btFailure("到预抓取失败: " + result.reason);
  }
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  cycle_completion_level_ = std::max(
    cycle_completion_level_, ExecuteTarget::Result::LEVEL_NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btVerifyPregrasp()
{
  setState(CycleState::RECONFIRM, "预抓取停稳验证（不 SetIO）");
  if (!cycle_refined_ || !cycle_refined_->valid) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
    cycle_failure_code_ = FailureCode::PREGRASP_RESIDUAL;
    return btFailure("预抓取验证无精化几何");
  }
  cycle_pregrasp_msg_.target_id = cycle_target_id_;
  cycle_pregrasp_msg_.tool_profile_id = "hollow_cylinder_v1";
  if (!execution_enabled_.load()) {
    cycle_pregrasp_verified_ = true;
    cycle_pregrasp_msg_.passed = true;
    cycle_pregrasp_msg_.reason = "execution_disabled_skip_residual";
    cycle_completion_level_ = std::max(
      cycle_completion_level_,
      ExecuteTarget::Result::LEVEL_PREGRASP_VERIFIED);
    return BT::NodeStatus::SUCCESS;
  }
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    const auto first_axis = motion_->lookupTransform(base_frame_, "tool_axis");
    const auto first_mouth = motion_->lookupTransform(base_frame_, "sleeve_mouth");
    const auto first_cut = motion_->lookupTransform(base_frame_, "cutting_plane");
    std::this_thread::sleep_for(200ms);
    const auto second_axis = motion_->lookupTransform(base_frame_, "tool_axis");
    const auto second_mouth = motion_->lookupTransform(base_frame_, "sleeve_mouth");
    const auto second_cut = motion_->lookupTransform(base_frame_, "cutting_plane");
    if (!first_axis || !second_axis || !first_mouth || !second_mouth ||
      !first_cut || !second_cut)
    {
      pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
      cycle_failure_code_ = FailureCode::EXACT_TF_MISSING;
      return btFailure("预抓取缺少 tool_axis/sleeve_mouth/cutting_plane TF");
    }
    const Eigen::Vector3d tool_z = second_axis->linear().col(2);
    const double frames_deg = axisAngleDeg(
      first_axis->linear().col(2), tool_z);
    const Eigen::Vector3d bag_axis = cycle_refined_->axis;
    const double angle = axisAngleDeg(tool_z, bag_axis);
    const Eigen::Vector3d delta_b =
      second_mouth->translation() - cycle_refined_->bottom;
    const Eigen::Vector3d delta_c =
      second_cut->translation() - cycle_refined_->neck;
    const double axial = delta_c.dot(bag_axis);
    const double lateral = std::max(
      (delta_b - delta_b.dot(bag_axis) * bag_axis).norm(),
      (delta_c - axial * bag_axis).norm());
    const bool consistent = frames_deg < 1.5;
    const bool passed = consistent && angle <= 2.0 && lateral <= 0.003;
    cycle_pregrasp_msg_.frames_consistent = consistent;
    cycle_pregrasp_msg_.correction_count = attempt;
    cycle_pregrasp_msg_.axis_angle_deg = static_cast<float>(angle);
    cycle_pregrasp_msg_.lateral_error_m = static_cast<float>(lateral);
    cycle_pregrasp_msg_.axial_error_m = static_cast<float>(axial);
    cycle_pregrasp_msg_.needs_correction = (!passed) && consistent;
    cycle_pregrasp_msg_.passed = passed;
    if (passed) {
      cycle_pregrasp_verified_ = true;
      cycle_pregrasp_msg_.reason = "pregrasp_verified";
      cycle_completion_level_ = std::max(
        cycle_completion_level_,
        ExecuteTarget::Result::LEVEL_PREGRASP_VERIFIED);
      return BT::NodeStatus::SUCCESS;
    }
    if (cycle_pregrasp_msg_.needs_correction && attempt < 2 && grasp_task_) {
      setState(CycleState::RECONFIRM, "预抓取短修正（停—看，不 SetIO）");
      const auto fix = grasp_task_->moveToPregrasp(
        cycle_entry_tip_pose_, cycle_refined_->axis, true);
      if (!fix.success) {
        break;
      }
      continue;
    }
    break;
  }
  if (cycle_pregrasp_only_.load()) {
    cycle_pregrasp_msg_.reason = "pregrasp_residual_hold";
    cycle_pregrasp_msg_.failure_code = FailureCode::PREGRASP_RESIDUAL;
    cycle_completion_level_ = std::max(
      cycle_completion_level_,
      ExecuteTarget::Result::LEVEL_PREGRASP_VERIFIED);
    RCLCPP_WARN(
      get_logger(),
      "PREGRASP_ONLY 残差未过门仍停在预抓取 angle=%.2fdeg lateral=%.4fm",
      cycle_pregrasp_msg_.axis_angle_deg,
      cycle_pregrasp_msg_.lateral_error_m);
    return BT::NodeStatus::SUCCESS;
  }
  pending_outcome_.store(ExecuteTarget::Result::SKIPPED_QUALITY);
  cycle_failure_code_ = FailureCode::PREGRASP_RESIDUAL;
  cycle_pregrasp_msg_.failure_code = FailureCode::PREGRASP_RESIDUAL;
  cycle_pregrasp_msg_.reason = "pregrasp_residual";
  return btFailure("预抓取残差未过门");
}

BT::NodeStatus ManipulationSkillsNode::btHoldPregrasp()
{
  // PREGRASP_ONLY 定位门：停在预抓取，不 PTP harvest_stow、不套入、不 SetIO。
  // execution 开过则保持/置 recovery，调度 ACK 前不得 Survey / 下一颗。
  setState(
    CycleState::RECOVERY_REQUIRED,
    "PREGRASP_ONLY：停在预抓取，不回 harvest_stow，未套入未 SetIO");
  if (execution_enabled_.load()) {
    contact_recovery_required_.store(true);
  }
  RCLCPP_INFO(
    get_logger(),
    "PREGRASP_ONLY 停在预抓取；ACK 后再 Survey / 派下一颗");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btPlanSleeveAndReverseRetreat()
{
  setState(CycleState::PREVIEW_CONTACT_PLANNING, "预规划套入与反向撤退");
  if (!cycle_refined_ || !grasp_task_) {
    return btFailure("套入预规划无几何");
  }
  const auto result = grasp_task_->previewFullContact(
    cycle_entry_tip_pose_, cycle_refined_->axis, cycle_travel_m_);
  if (!result.success) {
    pending_outcome_.store(ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    cycle_failure_code_ = FailureCode::SLEEVE_PLAN_FAILED;
    return btFailure("套入/撤退预规划失败: " + result.reason);
  }
  cycle_sleeve_planned_ = true;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btSleeveLinear()
{
  setState(CycleState::MTC_APPROACH_INSERT, "沿轴 LIN 套入");
  if (!cycle_sleeve_planned_ || !grasp_task_ || !cycle_refined_) {
    return btFailure("套入前未完成正反向预规划");
  }
  const auto result = grasp_task_->sleeveLinear(
    cycle_refined_->axis, cycle_travel_m_, true);
  if (result.execution_started) {
    contact_recovery_required_.store(true);
  }
  if (!result.success) {
    pending_outcome_.store(
      result.execution_started ?
      ExecuteTarget::Result::FAILED : ExecuteTarget::Result::SKIPPED_UNREACHABLE);
    cycle_failure_code_ = FailureCode::SLEEVE_PLAN_FAILED;
    return btFailure("沿轴套入失败: " + result.reason);
  }
  cycle_completion_level_ = std::max(
    cycle_completion_level_,
    ExecuteTarget::Result::LEVEL_SLEEVE_COMPLETED);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btVerifyCutHold()
{
  setState(CycleState::ACTUATE_TOOL, "切割保持位确认");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btVerifyCut()
{
  if (!tool_enabled_.load()) {
    cycle_cut_confirmed_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  if (!cycle_cut_command_accepted_) {
    pending_outcome_.store(ExecuteTarget::Result::FAILED);
    cycle_failure_code_ = FailureCode::CUT_COMMAND_FAILED;
    return btFailure("无 CUT_COMMAND_ACCEPTED");
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btReturnHarvestStow()
{
  setState(CycleState::MTC_RETREAT, "返回 harvest_stow");
  if (!motion_) {
    return btFailure("MoveIt 尚未初始化");
  }
  const std::string stow = harvest_stow_named_target_.empty() ?
    photo_pose_named_target_ : harvest_stow_named_target_;
  std::string message;
  if (!motion_->goToPhotoPose(stow, execution_enabled_.load(), message)) {
    return btFailure("返回 harvest_stow 失败: " + message);
  }
  contact_recovery_required_.store(false);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btVerifyHarvestOutcome()
{
  const bool harvest_ok = cycle_cut_confirmed_ && cycle_retreat_confirmed_;
  if (tool_enabled_.load() && grasp_enabled_.load() && !harvest_ok) {
    pending_outcome_.store(ExecuteTarget::Result::FAILED);
    cycle_failure_code_ = cycle_retreat_confirmed_ ?
      FailureCode::CUT_FEEDBACK_TIMEOUT : FailureCode::RETREAT_FAILED;
    return btFailure("切断或撤退未确认，不得记采摘成功");
  }
  if (harvest_ok) {
    cycle_completion_level_ = ExecuteTarget::Result::LEVEL_HARVEST_CONFIRMED;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btDepositToStation()
{
  setState(CycleState::MTC_RETREAT, "转移到卸果站或跳过（M8 未标定）");
  if (deposit_pose_named_target_.empty()) {
    cycle_deposit_ok_ = false;
    cycle_deposit_skipped_m8_ = true;
    cycle_deposit_reason_ = "pending_m8_unload_pose";
    return BT::NodeStatus::SUCCESS;
  }
  if (!motion_) {
    cycle_deposit_ok_ = false;
    cycle_deposit_skipped_m8_ = false;
    cycle_deposit_reason_ = "MoveIt 尚未初始化";
    return btFailure("卸果转移失败: " + cycle_deposit_reason_);
  }
  std::string message;
  const bool ok = motion_->goToPhotoPose(
    deposit_pose_named_target_, execution_enabled_.load(), message);
  cycle_deposit_ok_ = ok;
  cycle_deposit_skipped_m8_ = false;
  cycle_deposit_reason_ = message;
  if (!ok) {
    return btFailure("卸果转移失败: " + message);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ManipulationSkillsNode::btCompleteTarget()
{
  // 账本由 peach_task_executor 按 ExecuteTarget 终态写入；此处只结束行为树。
  cycle_terminal_state_ = CycleState::SUCCEEDED;
  if (cycle_pregrasp_only_.load()) {
    cycle_terminal_message_ =
      "PREGRASP_ONLY：停在预抓取，未套入未 SetIO；ACK 后再 Survey";
  } else if (cycle_cut_confirmed_ && cycle_retreat_confirmed_) {
    cycle_terminal_message_ = "切断与撤退均已确认";
  } else {
    cycle_terminal_message_ =
      "行为树完成；tool.enabled=false 或未宣称采摘成功";
  }
  return BT::NodeStatus::SUCCESS;
}

void ManipulationSkillsNode::publishViewMarkers(
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

}  // namespace peach_manipulation_skills
