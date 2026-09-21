// 功能：ExecuteTarget / SurveyScene 的接受、执行、取消，以及周期状态投影；
// 运动阶段授权矩阵（authorizeStage）的唯一实现。不实现阶段函数（见 stages.cpp）。
#include "peach_arm/manipulation_skills_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <future>
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

#include "peach_arm/model_contract.hpp"
#include "peach_arm/plan_contract.hpp"

using namespace std::chrono_literals;

namespace peach_arm
{
using FailureCode = peach_interfaces::msg::FailureCode;
// A13：targetPhase 投影字面量与 HarvestState.msg 的 TARGET_* 常量双向钉死
// （防消息常量重排后投影静默漂移）。同款钉死（M3a/M3c）：纯核
// plan_contract.hpp / stage_denial.hpp 不引 ROS 消息，其钉死值与 IDL 常量
// 在此双向锁定。
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
static_assert(
  FailureCode::PLAN_MISMATCH == 20u,
  "plan_contract.hpp ExecutePlanGate 的失败码须与 FailureCode.PLAN_MISMATCH 一致");
static_assert(
  ExecuteTarget::Result::FAILED == kOutcomeFailed,
  "stage_denial.hpp kOutcomeFailed 须与 ExecuteTarget.Result.FAILED 一致");
static_assert(
  ExecuteTarget::Result::SKIPPED_QUALITY == kOutcomeSkippedQuality,
  "stage_denial.hpp kOutcomeSkippedQuality 须与 ExecuteTarget.Result.SKIPPED_QUALITY 一致");

// 运动阶段授权矩阵（cycle_support.hpp）：一切运动执行入口最终收敛到
// 本判定。公共 = Active ∧ robotReady ∧ !cancel；TRANSIT/PREGRASP 叠加
// execution_enabled；CONTACT 叠加 grasp_enabled ∧ 接触许可复检（清洁重写轮
// 双路：goal.clearance 令牌优先——只验新鲜度与 allowed，不重算几何；旧
// 客户端未填令牌时回退 GraspDecision 话题快照）；TOOL 再叠加 tool_enabled。
// denial（M3c）输出拒因分类：令牌/许可「过期」（valid_until / model_stamp
// 超窗）= EXPIRED——可重派；其余（含令牌 allowed=false 的明确不允许）
// = DENIED；通过 = ALLOWED。requireStageAuthority 据此分级终局。
bool ManipulationSkillsNode::authorizeStage(
  const CycleContext & ctx, MotionStage stage, std::string & why,
  StageDenial & denial)
{
  denial = StageDenial::DENIED;
  if (stage == MotionStage::TRANSIT || stage == MotionStage::PREGRASP) {
    if (authorizeTransit(why)) {
      denial = StageDenial::ALLOWED;
      return true;
    }
    return false;
  }
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
  if (!grasp_enabled_.load()) {
    why = "grasp.enabled=false（接触未使能）";
    return false;
  }
  if (ctx.clearance_present) {
    if (!ctx.clearance_allowed) {
      why = "接触许可令牌 allowed=false";
      return false;
    }
    if (ctx.clearance_target_id.empty() || ctx.clearance_target_id != ctx.target_id) {
      why = "接触许可令牌目标不符（未绑定或非当前目标）";
      return false;
    }
    // valid_until 冻结有效期（GraspDecision 源头心跳不续签；零值=生产端
    // 未提供，退回 model_stamp 新鲜度单门）。
    if (ctx.clearance_valid_until.nanoseconds() > 0 &&
      now().seconds() > ctx.clearance_valid_until.seconds())
    {
      why = "接触许可令牌过期（valid_until）";
      denial = StageDenial::EXPIRED;
      return false;
    }
    if (ctx.clearance_fresh_window_s > 0.0) {
      const double age_s = now().seconds() - ctx.clearance_model_stamp.seconds();
      if (age_s < -0.5 || age_s > ctx.clearance_fresh_window_s) {
        why = "接触许可令牌过期（model_stamp 超窗）";
        denial = StageDenial::EXPIRED;
        return false;
      }
    }
    // 令牌只覆盖 allowed/绑定/有效期；档位门仍在（stages.cpp 另有兜底，
    // 此处前置保持授权矩阵单点）。
    if (stage == MotionStage::TOOL && !tool_enabled_.load()) {
      why = "tool.enabled=false";
      return false;
    }
    denial = StageDenial::ALLOWED;
    return true;
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
  denial = StageDenial::ALLOWED;
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
  if (goal->mode == ExecuteTarget::Goal::FULL ||
    goal->mode == ExecuteTarget::Goal::PREGRASP_ONLY)
  {
    ModelIdentity identity;
    identity.run_id = goal->run_id;
    identity.scene_epoch = goal->scene_epoch;
    identity.target_id = goal->target_id;
    identity.model_revision = goal->model_revision;
    identity.tool_profile_id = goal->tool_profile_id;
    identity.calibration_revision = goal->calibration_revision;
    identity.config_revision = goal->config_revision;
    if (!identityComplete(identity)) {
      RCLCPP_WARN(
        get_logger(), "拒绝目标请求 %s: 身份元组不完整",
        goal->target_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  // FULL/PREVIEW 与 OBSERVE_ONLY 同一受理门：以 ExecuteTarget.goal.target_id
  // 为准，命中锁定集有效锚点即可；未锁定时回退感知 selected 缓存（单目标
  // 手动周期）。编排器选择权在 peach_supervisor，不再要求 selected 字段。
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
  // 有界回收（W13-B）：onActionGoal 在 running_ 时已拒单，正常路径旧线程
  // 此刻只剩终局上报，join 立即返回；若旧线程卡死（如 MoveIt 内部长阻塞、
  // 无视取消标志），无限 join 会把 executor 回调吊死——经 packaged_task
  // future 有界等 2 s，超时 WARN 后 detach 放行新周期（detach 只是放弃
  // 回收、不是放弃取消，线程仍受取消标志约束；析构的 joinable 检查自然
  // 跳过已 detach 线程）。
  if (action_thread_.joinable()) {
    if (action_thread_done_.valid() &&
      action_thread_done_.wait_for(2s) == std::future_status::ready)
    {
      action_thread_.join();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "上一 ExecuteTarget 执行线程 2s 内未退场，放弃 join 改为 detach；"
        "线程仍受取消标志约束，请排查卡死原因");
      action_thread_.detach();
    }
  }
  std::packaged_task<void(std::shared_ptr<RunTargetGoalHandle>)> task(
    [this](std::shared_ptr<RunTargetGoalHandle> handle) {
      executeAction(handle);
    });
  action_thread_done_ = task.get_future();
  action_thread_ = std::thread(std::move(task), goal_handle);
}

void ManipulationSkillsNode::executeAction(
  const std::shared_ptr<RunTargetGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto trigger_response = std::make_shared<Trigger::Response>();
  // M3a：每 goal 复位受理期拒单码，防上一 goal 的码泄入本次终局组装。
  pending_accept_failure_code_ = 0;
  // 周期上下文：PREVIEW 模式走预览入口（不是周期，不创建 ctx，终局按空
  // 上下文默认值填充）；其余模式受理即创建并钉 goal 身份，action 线程自持
  // shared_ptr——后续周期整体丢弃本份也不影响本次终局读取。
  std::shared_ptr<CycleContext> ctx;
  auto fill_plan = [this](const ExecuteTarget::Goal & goal_msg, bool require_joints) {
      ContactPlan plan;
      plan.plan_id = goal_msg.plan_id;
      plan.scene_epoch = goal_msg.scene_epoch;
      plan.require_start_joints = require_joints;
      plan.model.run_id = goal_msg.run_id;
      plan.model.scene_epoch = goal_msg.scene_epoch;
      plan.model.target_id = goal_msg.target_id;
      plan.model.model_revision = goal_msg.model_revision;
      plan.model.tool_profile_id = goal_msg.tool_profile_id;
      plan.model.calibration_revision = goal_msg.calibration_revision;
      plan.model.config_revision = goal_msg.config_revision;
      if (require_joints && move_group_) {
        plan.start_joints = move_group_->getCurrentJointValues();
      }
      return plan;
    };
  if (goal->mode == ExecuteTarget::Goal::PREVIEW) {
    // G2：预览绑定只由 PREVIEW 模式 goal 写入。OBSERVE_ONLY / PREGRASP_ONLY /
    // FULL 一律不写（观察是采数据不是计划预览；observe goal 在模型建好前
    // 本就带不了三修订，旧「observe 转记绑定」会让保守档 FULL 必拒且
    // last_preview_valid_ 无复位点）。手动 preview→FULL 链路（本写入 +
    // 下方全字段比对）校验强度不变。
    last_preview_plan_ = fill_plan(*goal, true);
    last_preview_valid_ = !goal->plan_id.empty();
    previewContact(false, trigger_response);
  } else {
    bool plan_ok = true;
    if (last_preview_valid_ && !goal->plan_id.empty()) {
      ContactPlan execute = fill_plan(
        *goal, last_preview_plan_.require_start_joints);
      const ExecutePlanGate gate = executePlanGate(
        true, true, last_preview_plan_, execute, 0.05);
      if (!gate.pass) {
        plan_ok = false;
        // M3a：受理期拒单码经 pending 成员带入 Result 组装（此路径 ctx 尚
        // 未创建；gate.failure_code=20=PLAN_MISMATCH，static_assert 与 IDL 钉死）。
        pending_accept_failure_code_ = gate.failure_code;
        trigger_response->success = false;
        trigger_response->message = "plan_id mismatch: preview != execute";
      }
    }
    if (plan_ok) {
      // Action 是唯一周期入口（自动编排）：受理即自动 arm（手动 Trigger
      // 类入口须另行 set_execution_armed）。
      if (execution_enabled_.load()) {execution_armed_.store(true);}
      ctx = std::make_shared<CycleContext>();
      ctx->target_id = goal->target_id;
      ctx->observe_only = goal->mode == ExecuteTarget::Goal::OBSERVE_ONLY;
      ctx->pregrasp_only = goal->mode == ExecuteTarget::Goal::PREGRASP_ONLY;
      ctx->skip_observation = goal->skip_observation;
      ctx->action_driven = true;
      // 清洁重写轮：goal.profile 优先（PREGRASP_HOLD≈PREGRASP_ONLY、
      // FULL≈FULL）；旧客户端不填 profile（0=PREGRASP_HOLD 与 PREGRASP_ONLY
      // 语义衔接，mode 仍各自赋值，行为不变）。接触许可令牌填写即启用
      // CONTACT/TOOL 级令牌复检路径（authorizeStage 双路）。
      if (goal->profile == ExecuteTarget::Goal::PROFILE_FULL) {
        ctx->pregrasp_only = false;
      } else if (goal->profile == ExecuteTarget::Goal::PROFILE_PREGRASP_HOLD) {
        ctx->pregrasp_only = true;
      }
      if (goal->clearance.model_stamp.sec > 0 ||
        goal->clearance.model_stamp.nanosec > 0)
      {
        ctx->clearance_present = true;
        ctx->clearance_allowed = goal->clearance.allowed;
        ctx->clearance_target_id = goal->clearance.target_id;
        ctx->clearance_valid_until = rclcpp::Time(goal->clearance.valid_until);
        ctx->clearance_model_stamp = rclcpp::Time(goal->clearance.model_stamp);
        ctx->clearance_fresh_window_s = effectiveTargetMaxAgeS();
      }
      last_checkpoint_.store(0);
      cycle_ = ctx;
      onStart(trigger_response);
    }
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
    // M1：受理即拒的终局同样收口取消旗标（周期未启动，running_=false 恒真）。
    clearCancelFlagIfIdle();
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
      // 清洁重写轮：阶段检查点随反馈下发（stages.cpp 到达即 mark）。
      feedback->checkpoint = last_checkpoint_.load();
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
  // G2 复位：FULL / PREGRASP_ONLY 周期终局（成功/失败/取消）清预览绑定。
  // 二者是消费预览比对的执行周期（调度 _cmd_full 二选一，PREGRASP_ONLY 是
  // 现行默认干跑档）；周期已终局，绑定跨目标残留只会把下一颗误拒。
  // PREVIEW 模式不进本路径（绑定刚写入）；受理即拒（plan mismatch）在上方
  // 早退分支，保留绑定让修正后的 FULL 仍受全字段比对约束；OBSERVE_ONLY
  // 不写绑定也无须清（观察不是计划预览）。
  if (goal->mode == ExecuteTarget::Goal::FULL ||
    goal->mode == ExecuteTarget::Goal::PREGRASP_ONLY)
  {
    last_preview_valid_ = false;
    last_preview_plan_ = ContactPlan{};
  }
  // M1：周期终局（worker 已落终态、取消不再向周期内传播）收口取消旗标，
  // 一次单果取消/skip 不得把后续一切 MoveTo/观察拒之门外。
  clearCancelFlagIfIdle();
}

void ManipulationSkillsNode::onStart(const Trigger::Response::SharedPtr & response)
{
  const ScopedTimer timer(get_logger(), "start_cycle", &callback_timing_);
  // 运动输出权限绑定 Active 态（A8）：action 派生周期唯一启动入口。
  // M3b：启动拒绝优先落既有词表码（下方 recovery / 锚点失效两支）；无 ctx
  // 的调用安全跳过（Result 组装经 fillExecuteResults 读 ctx->failure_code）。
  const auto set_failure = [this](uint32_t code) {
      if (cycle_) {cycle_->failure_code = code;}
    };
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  if (!move_group_) {
    // M3b：词表无对应码（MoveIt 未初始化 / 周期占用 / 未 arm 三支同此），
    // 保持 failure_code=0 由 reason 传达；新增枚举值须动 IDL，本轮不做
    // （TODO：FailureCode 词表扩充轮补 NOT_ARMED / MOVEIT_UNAVAILABLE 类码）。
    response->success = false;
    response->message = "MoveIt 尚未初始化";
    return;
  }
  if (contact_recovery_required_.load()) {
    response->success = false;
    response->message =
      "上一周期可能停在接触区；现场人工撤离并确认后调用 acknowledge_recovery";
    set_failure(FailureCode::RECOVERY_REQUIRED);
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
      // M3b：与 stagePrepareCycle 同条件同码（周期目标锚点失效=观察失败）。
      set_failure(FailureCode::OBSERVE_FAILED);
      return;
    }
  }
  if (worker_.joinable()) {
    // M2（W13-B 同款有界回收）：正常路径旧 worker 已落终态（finish 置
    // running_=false 后线程即将退场），join 立即返回；卡死（MoveIt 内部
    // 长阻塞、无视取消标志）时经 packaged_task future 有界等 2s，超时
    // WARN 后 detach 放行新周期——放弃回收不等于放弃取消，线程仍受取消
    // 标志约束；析构的 joinable 检查自然跳过已 detach 线程。
    if (worker_done_.valid() &&
      worker_done_.wait_for(2s) == std::future_status::ready)
    {
      worker_.join();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "上一周期 worker 线程 2s 内未退场，放弃 join 改为 detach；"
        "线程仍受取消标志约束，请排查卡死原因");
      worker_.detach();
    }
  }
  cancel_requested_.store(false);
  // 每周期开始重置终局分级（阶段失败点按需覆盖）。
  pending_outcome_.store(ExecuteTarget::Result::FAILED);
  // 阶段耗时计时随周期真正启动开始（此前一切拒绝路径不计时）。
  startCycleTiming();
  // worker 按 shared_ptr 持有周期上下文：周期消亡即整体丢弃，钉残留不可能。
  std::packaged_task<void()> cycle_task(
    [this, ctx = cycle_]() {executeCycle(*ctx);});
  worker_done_ = cycle_task.get_future();
  worker_ = std::thread(std::move(cycle_task));
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
  // M2（W13-B 同款有界回收）：本回调在默认互斥组——旧 survey 线程卡死时
  // 裸 join 会把 ACK/取消/订阅一并吊死；经 packaged_task future 有界等 2s，
  // 超时 WARN 后 detach 放行新 survey（放弃回收≠放弃取消，线程仍受取消
  // 标志约束；析构的 joinable 检查自然跳过已 detach 线程）。
  if (survey_thread_.joinable()) {
    if (survey_thread_done_.valid() &&
      survey_thread_done_.wait_for(2s) == std::future_status::ready)
    {
      survey_thread_.join();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "上一 SurveyScene 执行线程 2s 内未退场，放弃 join 改为 detach；"
        "线程仍受取消标志约束，请排查卡死原因");
      survey_thread_.detach();
    }
  }
  std::packaged_task<void(std::shared_ptr<SurveyGoalHandle>)> survey_task(
    [this](std::shared_ptr<SurveyGoalHandle> handle) {executeSurvey(handle);});
  survey_thread_done_ = survey_task.get_future();
  survey_thread_ = std::thread(std::move(survey_task), goal_handle);
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
  // 场景纪元无真实源接入：SurveyScene 结果恒填 0（BeginScene 世代尚未
  // 回传给本节点）；接入场景纪元源时改此单点，勿在别处复填。
  result->scene_epoch = 0;
  // M1：survey 终局收口取消旗标（周期不在运行即清；语义见声明处注释）。
  clearCancelFlagIfIdle();
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

}  // namespace peach_arm
