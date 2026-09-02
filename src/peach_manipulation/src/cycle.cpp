// 功能：ExecuteTarget / SurveyScene 的接受、执行、取消，以及周期状态投影；
// 运动阶段授权矩阵（authorizeStage）的唯一实现。不实现阶段函数（见 stages.cpp）。
#include "peach_manipulation/cycle.hpp"

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
#include <peach_interfaces/msg/harvest_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace peach_manipulation
{
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

// 运动阶段授权矩阵（execution_authority.hpp）：一切运动执行入口最终收敛到
// 本判定。公共 = Active ∧ robotReady ∧ !cancel；TRANSIT/PREGRASP 叠加
// execution_enabled；CONTACT 叠加 grasp_enabled ∧ GraspDecision 复检；
// TOOL 再叠加 tool_enabled。
bool ManipulationSkillsNode::authorizeStage(
  const CycleContext & ctx, MotionStage stage, std::string & why)
{
  if (!motionOutputAllowed(why)) {
    return false;
  }
  if (!safetyReady(why)) {
    return false;
  }
  if (cancel_requested_.load()) {
    why = "周期已请求取消";
    return false;
  }
  if (stage == MotionStage::TRANSIT || stage == MotionStage::PREGRASP) {
    if (!execution_enabled_.load()) {
      why = "execution.enabled=false（只规划预览，不得执行运动）";
      return false;
    }
    return true;
  }
  if (!grasp_enabled_.load()) {
    why = "grasp.enabled=false（接触未使能）";
    return false;
  }
  if (graspDecisionTargetSnapshot() != ctx.target_id ||
    !qualitySnapshot().grasp_allowed)
  {
    why = "GraspDecision 复检未通过（allowed=false 或目标不符）";
    return false;
  }
  if (stage == MotionStage::TOOL && !tool_enabled_.load()) {
    why = "tool.enabled=false";
    return false;
  }
  return true;
}

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
  // OBSERVE_ONLY 是受理模式之一：只走观察+精化验证段（执行器内 observe_only
  // 分支短路，不进接触/工具/撤离），终局按 PLAN_READY 上报 SUCCEEDED。
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
  // 手动周期）。编排器选择权在 peach_executor，不再要求 selected 字段。
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
  requestCancelAll();
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
  // 周期上下文：PREVIEW 模式走预览入口（不是周期，不创建 ctx，终局按空
  // 上下文默认值填充）；其余模式受理即创建并钉 goal 身份，action 线程自持
  // shared_ptr——后续周期整体丢弃本份也不影响本次终局读取。
  std::shared_ptr<CycleContext> ctx;
  if (goal->mode == ExecuteTarget::Goal::PREVIEW) {
    previewContact(false, trigger_response);
  } else {
    // Action 是自动编排专用入口；手动 Trigger 仍要求每周期单独 arm。
    if (execution_enabled_.load()) {execution_armed_.store(true);}
    ctx = std::make_shared<CycleContext>();
    ctx->target_id = goal->target_id;
    ctx->observe_only = goal->mode == ExecuteTarget::Goal::OBSERVE_ONLY;
    ctx->pregrasp_only = goal->mode == ExecuteTarget::Goal::PREGRASP_ONLY;
    ctx->skip_observation = goal->skip_observation;
    ctx->action_driven = true;
    cycle_ = ctx;
    onStart(std::make_shared<Trigger::Request>(), trigger_response, true);
  }
  if (!trigger_response->success) {
    auto result = std::make_shared<ExecuteTarget::Result>();
    result->outcome = ExecuteTarget::Result::FAILED;
    result->reason = trigger_response->message;
    result->recovery_required = contact_recovery_required_.load();
    // 启动即失败：计时未启动则两数组为空，符合"未经历的阶段不出现"契约。
    fillStageDurations(result);
    fillExecuteResults(result, ctx.get());
    goal_handle->abort(result);
    return;
  }

  while (rclcpp::ok() && running_.load()) {
    if (goal_handle->is_canceling()) {
      requestCancelAll();
    }
    auto feedback = std::make_shared<ExecuteTarget::Feedback>();
    feedback->state.target_id = goal->target_id;
    feedback->state.action_active = running_.load();
    feedback->state.execution_enabled = execution_enabled_.load();
    feedback->state.grasp_enabled = grasp_enabled_.load();
    feedback->state.tool_enabled = tool_enabled_.load();
    feedback->state.recovery_required = contact_recovery_required_.load();
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
  cycle_result.recovery_required = contact_recovery_required_.load();
  const CycleOutcome outcome = cycle_result.outcome;
  auto result = std::make_shared<ExecuteTarget::Result>();
  result->reason = cycle_result.reason;
  // 旗标与终局解耦：PREGRASP_ONLY 到位是 SUCCEEDED，仍须 ACK 才 Survey。
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
  fillExecuteResults(result, ctx.get());
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
      // abort 路径按阶段失败点记录的 pending_outcome_ 分级（质量/不可达/失败）。
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
    // 手动 Trigger 周期恒为 FULL 语义：全新上下文（目标空、三旗标全 false），
    // 不残留上一 action 周期的 goal 钉死身份与模式旗标。
    cycle_ = std::make_shared<CycleContext>();
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
    // goal 目标，其余=感知 selected 缓存；语义见 stages.cpp
    // cycleTargetSnapshot）。OBSERVE_ONLY 受理时锁定集命中，但受理到启动
    // 之间可能解锁/换批次，此处必须按同一数据源复核。
    const bool observe_only =
      cycle_->observe_only && !cycle_->target_id.empty();
    const auto target = cycleTargetSnapshot(cycle_->target_id);
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
  // 每周期开始重置终局分级（阶段失败点按需覆盖）。
  pending_outcome_.store(ExecuteTarget::Result::FAILED);
  // 阶段耗时计时随周期真正启动开始（此前一切拒绝路径不计时）。
  startCycleTiming();
  // worker 按 shared_ptr 持有周期上下文：周期消亡即整体丢弃，钉残留不可能。
  worker_ = std::thread(
    [this, ctx = cycle_]() {executeCycle(*ctx);});
  response->success = true;
  response->message = execution_enabled_.load() ? "已启动主动视觉靠近周期" :
    "已启动只规划预览（不会发送运动）";
}

void ManipulationSkillsNode::onCancel(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "cancel_cycle", &callback_timing_);
  requestCancelAll();
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
  // 取消与 ExecuteTarget 同纪律：除置取消标志外，停 MoveIt/MTC 当前执行并
  // 唤醒等待（否则拍照位运动会继续走完，周期侧等待也不退场）。
  requestCancelAll();
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
  std::string snapshot_before;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_before = last_snapshot_id_;
  }
  onGoToPhotoPose(std::make_shared<Trigger::Request>(), response);
  execution_armed_.store(false);
  if (response->success && execution_enabled_.load()) {
    // 到位后等一帧新快照再填 result：到位瞬间读到的常是移动前的旧帧
    // （snapshot_id 未变）。窗口有界 2×等帧超时，超时即用旧值；50ms 切片
    // 轮询，取消/关停立即退场（沿用 cache_ 等待纪律）。
    const double deadline_s = now().seconds() + 2.0 * effectiveFrameWaitS();
    while (rclcpp::ok() && !goal_handle->is_canceling() &&
      !cancel_requested_.load())
    {
      bool changed = false;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        changed = last_snapshot_id_ != snapshot_before && !last_snapshot_id_.empty();
      }
      if (changed || now().seconds() >= deadline_s) {
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    result->snapshot_id = last_snapshot_id_;
    result->degraded = !last_target_set_locked_ || last_observation_count_ == 0;
  }
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

}  // namespace peach_manipulation
