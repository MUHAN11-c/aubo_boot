// 功能：MoveTo 动作服务端（清洁重写轮 2b）——视点/拍照位/命名位/位姿移动；
// TRANSIT 级授权公共段（authorizeStage 的 TRANSIT/PREGRASP 分支与 Survey/MoveTo
// 共用）；操作台使能广播订阅（/peach/batch/enables，无发布者时本地参数权威）；
// 阶段检查点记账（markCheckpoint，ExecuteTarget 反馈随行下发）。
// 取消 = requestCancelAll（透传 abort + RobotMoveStop），不 resume 原轨迹。
#include "peach_arm/manipulation_skills_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <peach_interfaces/msg/failure_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

namespace peach_arm
{
using FailureCode = peach_interfaces::msg::FailureCode;

bool ManipulationSkillsNode::authorizeTransit(std::string & why)
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
  if (!execution_enabled_.load()) {
    why = "execution.enabled=false（只规划预览，不得执行运动）";
    return false;
  }
  return true;
}

void ManipulationSkillsNode::onEnables(
  const peach_interfaces::msg::Enables::SharedPtr message)
{
  // 意图源=大脑广播（阶段 3 起 SetEnables 服务后发）。旧栈无发布者时
  // 本地参数保持唯一权威——本订阅静默，行为零变化。
  if (!enables_external_) {
    enables_external_ = true;
    RCLCPP_INFO(
      get_logger(), "使能切换到操作台广播源（/peach/batch/enables）");
  }
  enables_last_beat_ = std::chrono::steady_clock::now();
  execution_enabled_.store(message->execution);
  grasp_enabled_.store(message->grasp);
  tool_enabled_.store(message->tool);
  RCLCPP_INFO(
    get_logger(), "操作台使能: execution=%d grasp=%d tool=%d（%s）",
    message->execution ? 1 : 0, message->grasp ? 1 : 0,
    message->tool ? 1 : 0, message->reason.c_str());
  publishState();
}

void ManipulationSkillsNode::checkEnablesHeartbeat()
{
  // 缺心跳=故障（AGENTS 第 2 章 DEFAULT 3）：操作台广播断流超时后不得
  // 永久保持最后值——回落本地参数权威，使能链重新由本节点参数决定。
  // 与 onEnables 同在默认互斥回调组，enables_external_ 无需原子。
  if (!enables_external_ || enables_heartbeat_timeout_s_ <= 0.0) {
    return;
  }
  const double age_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - enables_last_beat_).count();
  if (age_s <= enables_heartbeat_timeout_s_) {
    return;
  }
  enables_external_ = false;
  const auto params = param_listener_->get_params();
  execution_enabled_.store(params.execution.enabled);
  grasp_enabled_.store(params.grasp.enabled);
  tool_enabled_.store(params.tool.enabled);
  if (!execution_enabled_.load()) {
    execution_armed_.store(false);
  }
  RCLCPP_WARN(
    get_logger(),
    "操作台使能广播超时（%.1fs 无心跳），回落本地参数权威: "
    "execution=%d grasp=%d tool=%d",
    age_s, params.execution.enabled ? 1 : 0, params.grasp.enabled ? 1 : 0,
    params.tool.enabled ? 1 : 0);
  publishState();
}

void ManipulationSkillsNode::markCheckpoint(uint8_t checkpoint, const char * where)
{
  // CK_* 单调前进：重复/回退到达不回拨（反馈消费者只看最新到达档）。
  uint8_t previous = last_checkpoint_.load();
  while (checkpoint > previous &&
    !last_checkpoint_.compare_exchange_weak(previous, checkpoint))
  {
  }
  if (checkpoint > 0 && previous < checkpoint) {
    RCLCPP_INFO(
      get_logger(), "检查点 %u（%s）", static_cast<unsigned>(checkpoint), where);
  }
}

rclcpp_action::GoalResponse ManipulationSkillsNode::onMoveToGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const MoveToAction::Goal> goal)
{
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    RCLCPP_WARN(get_logger(), "拒绝 MoveTo: %s", motion_reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // 周期独占：接触周期或 Survey 运行中不接受视点/命名位移动（阶段 3 起
  // 由大脑编排保证不并发；本门兜底）。
  if (running_.load() || contact_recovery_required_.load()) {
    RCLCPP_WARN(get_logger(), "拒绝 MoveTo: 周期占用或待恢复");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->kind == MoveToAction::Goal::KIND_NAMED) {
    if (goal->named_target.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝 MoveTo: KIND_NAMED 未填命名目标");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  if (goal->kind == MoveToAction::Goal::KIND_POSE) {
    if (goal->pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝 MoveTo: KIND_POSE 未填 frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  // KIND_JOINTS 本轮预留：关节目标移动现阶段只服务接触段（ExecuteTarget
  // 内部 staging），独立下发待阶段 3 视点规划需要时再接。
  RCLCPP_WARN(
    get_logger(), "拒绝 MoveTo: KIND_JOINTS 预留未实现（kind=%u）",
    static_cast<unsigned>(goal->kind));
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ManipulationSkillsNode::onMoveToCancel(
  const std::shared_ptr<MoveToGoalHandle>)
{
  // 与 ExecuteTarget/Survey 同纪律：停 MoveIt 当前执行并唤醒等待。
  requestCancelAll();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationSkillsNode::onMoveToAccepted(
  const std::shared_ptr<MoveToGoalHandle> goal_handle)
{
  if (move_to_thread_.joinable()) {
    move_to_thread_.join();
  }
  move_to_thread_ = std::thread(
    [this, goal_handle]() {executeMoveTo(goal_handle);});
}

void ManipulationSkillsNode::executeMoveTo(
  const std::shared_ptr<MoveToGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToAction::Result>();
  auto feedback = std::make_shared<MoveToAction::Feedback>();
  const auto publish_feedback = [this, &goal_handle, &feedback]() {
      feedback->progress = feedback->phase ==
        MoveToAction::Feedback::PHASE_SETTLING ? 1.0 : 0.5;
      try {
        goal_handle->publish_feedback(feedback);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "MoveTo 反馈上报失败（可能正在 shutdown）: %s",
          error.what());
      }
    };
  feedback->phase = MoveToAction::Feedback::PHASE_PLANNING;
  publish_feedback();
  // 受理后执行前复核 TRANSIT 授权（取消/使能可能已变化）。
  std::string why;
  if (!motion_) {
    result->arrived = false;
    result->failure_code = 0;
    result->detail = "MoveIt 尚未初始化";
    goal_handle->abort(result);
    return;
  }
  if (!authorizeTransit(why)) {
    result->arrived = false;
    result->failure_code = FailureCode::DECISION_REJECTED;
    result->detail = why;
    goal_handle->abort(result);
    return;
  }
  if (goal->speed_scaling > 1.0e-6) {
    // 速度档本轮回退部署值（planOrMoveTip 内部按 config 覆盖）；
    // 非默认值打 WARN 提示未生效，阶段 3 视点规划需要时再开。
    RCLCPP_WARN(
      get_logger(), "MoveTo speed_scaling=%g 本轮未生效（部署档为准）",
      goal->speed_scaling);
  }
  feedback->phase = MoveToAction::Feedback::PHASE_MOVING;
  publish_feedback();
  bool ok = false;
  std::string message;
  if (goal->kind == MoveToAction::Goal::KIND_NAMED) {
    // goToPhotoPose 对任意命名目标通用：拍照位额外带原路返程逻辑。
    ok = motion_->goToPhotoPose(goal->named_target, true, message);
  } else {  // KIND_POSE（受理门已保证 frame_id 非空）
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(goal->pose.pose, target_pose);
    if (goal->pose.header.frame_id != base_frame_) {
      const auto to_base = motion_->lookupTransform(
        base_frame_, goal->pose.header.frame_id);
      if (!to_base) {
        result->arrived = false;
        result->failure_code = FailureCode::EXACT_TF_MISSING;
        result->detail = "MoveTo 位姿换系缺 TF: " + goal->pose.header.frame_id +
          " → " + base_frame_;
        goal_handle->abort(result);
        return;
      }
      target_pose = (*to_base) * target_pose;
    }
    // lin_only=只 LIN（观察短移/直连兜底，失败不回退）；否则 PTP（失败按
    // motion 接口默认回退策略）。camera_frame=true：pose 为相机光学位姿，
    // 走 planOrMoveCamera（内部经 TF 换算到 tip）——supervisor 视点规划用。
    const std::string planner = goal->lin_only ? "LIN" : "PTP";
    if (goal->camera_frame) {
      ok = motion_->planOrMoveCamera(
        target_pose, planner, true,
        goal->lin_only ? "move_to_camera_lin" : "move_to_camera_ptp",
        !goal->lin_only);
    } else {
      ok = motion_->planOrMoveTip(
        target_pose, planner, true,
        goal->lin_only ? "move_to_lin" : "move_to_ptp",
        !goal->lin_only);
    }
    message = ok ? "到位" : "移动失败（见节点日志）";
  }
  if (goal_handle->is_canceling() || cancel_requested_.load()) {
    result->arrived = false;
    result->failure_code = FailureCode::RECOVERY_REQUIRED;
    result->detail = "MoveTo 已取消（透传 abort + RobotMoveStop）";
    try {
      goal_handle->canceled(result);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "MoveTo 终局上报失败（可能正在 shutdown）: %s",
        error.what());
    }
    return;
  }
  feedback->phase = MoveToAction::Feedback::PHASE_SETTLING;
  publish_feedback();
  result->arrived = ok;
  result->failure_code = ok ? FailureCode::NONE : FailureCode::SLEEVE_PLAN_FAILED;
  result->detail = message;
  try {
    if (ok) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "MoveTo 终局上报失败（可能正在 shutdown）: %s",
      error.what());
  }
}

}  // namespace peach_arm
