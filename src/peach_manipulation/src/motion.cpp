// 功能：节点侧运动入口与 MoveIt 运动接口实现（拍照位 PTP、观察 LIN、预览接近/
// 接触服务、tip/camera 规划执行、TF 查询）。真实下发前过注入的安全门
// （TRANSIT 级底座；CONTACT/TOOL 级在阶段函数与 GraspTask 门加查）。
#include "peach_manipulation/motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>

#include "peach_manipulation/manipulation_skills_node.hpp"
#include "peach_manipulation/eigen_conversions.hpp"
#include "peach_manipulation/grasp_geometry.hpp"
#include "peach_manipulation/trajectory_guard.hpp"

using namespace std::chrono_literals;

namespace peach_manipulation
{

void ManipulationSkillsNode::onRobotStatus(
  const aubo_msgs::msg::RobotStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  robot_status_ = *message;
  robot_status_received_ = now();
  robot_status_valid_ = true;
}

double ManipulationSkillsNode::insertionTravel(const CachedRefined & refined) const
{
  // TCP 是工具圆柱前端面圆心，也就是物理剪切点。精化给出入口到剪切
  // 参考的行程时优先采用；否则按袋颈减颈部余量回退。
  double travel = refined.suggested_travel_m;
  const bool has_refined_geometry =
    refined.entry.allFinite() &&
    refined.bottom.allFinite() &&
    refined.neck.allFinite() &&
    refined.axis.allFinite() && refined.axis.norm() > 1.0e-9 &&
    (refined.neck - refined.bottom).norm() > 1.0e-6;
  if (travel <= 1.0e-6 && has_refined_geometry) {
    const Eigen::Vector3d axis = refined.axis.normalized();
    travel = (refined.neck - refined.entry).dot(axis) - neck_margin_m_;
  }
  return std::clamp(travel, minimum_travel_m_, maximum_travel_m_);
}

bool ManipulationSkillsNode::safetyReady(std::string & reason)
{
  RobotStatusSample sample;
  {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    sample.received = robot_status_valid_;
    sample.received_s = robot_status_received_.seconds();
    sample.e_stopped = robot_status_.e_stopped != 0;
    sample.in_error = robot_status_.in_error != 0;
    sample.drives_powered = robot_status_.drives_powered != 0;
    sample.motion_possible = robot_status_.motion_possible != 0;
  }
  return safety_gate_->robotReady(sample, reason);
}

bool ManipulationSkillsNode::cycleTargetReady(
  const std::string & target_id, std::string & reason)
{
  // 安全门样本按周期生效目标取数（同 stages.cpp cycleTargetSnapshot 的
  // 分流理由）：钉入 ID 的周期取 goal 目标的锁定集锚点样本（目标门判其
  // 身份/有效性/新鲜度），其余周期取感知 selected 样本。
  const TargetGateSample sample =
    !target_id.empty() ?
    cache_.lockedTargetGateSample(target_id) : cache_.targetGateSample();
  return safety_gate_->targetReady(sample, target_id, reason);
}

bool ManipulationSkillsNode::commandToolClose()
{
  // 工具 IO 收敛到授权矩阵 TOOL 级（Active ∧ robotReady ∧ !cancel ∧
  // execution ∧ grasp ∧ GraspDecision 复检 ∧ tool.enabled）。周期身份取自
  // ToolActuator 当前事务（arm 在 sendCut 前已写入周期 goal 目标）。
  CycleContext probe;
  probe.target_id = tool_actuator_.context().target_id;
  std::string why;
  if (!authorizeStage(probe, MotionStage::TOOL, why)) {
    setState(CycleState::FAILED, "工具 IO 被拒绝: " + why, probe.target_id);
    return false;
  }
  if (!tool_io_client_->wait_for_service(
      std::chrono::duration<double>(service_timeout_s_)))
  {
    setState(CycleState::FAILED, "末端工具 set_io 服务不可用");
    return false;
  }
  auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  request->fun = static_cast<int8_t>(tool_io_fun_);
  request->pin = static_cast<int8_t>(tool_io_pin_);
  request->state = static_cast<float>(tool_close_state_);
  auto future = tool_io_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) !=
    std::future_status::ready || !future.get()->success)
  {
    setState(CycleState::FAILED, "末端工具关闭命令失败");
    return false;
  }
  return true;
}

void ManipulationSkillsNode::onPreviewApproachInsert(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "preview_approach_insert", &callback_timing_);
  previewContact(false, response);
}

void ManipulationSkillsNode::onPreviewFullContact(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "preview_full_contact", &callback_timing_);
  previewContact(true, response);
}

void ManipulationSkillsNode::previewContact(
  bool include_retreat, Trigger::Response::SharedPtr response)
{
  // 接触轨迹预览是运动类入口（A8）：即便只规划不执行，也非 Active 不放行。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    response->success = false;
    response->message = "已有规划或执行周期正在运行";
    return;
  }
  // 预览周期同样在每周期开始时重置终局分级。
  pending_outcome_.store(ExecuteTarget::Result::FAILED);
  // 预览周期（含 action PREVIEW 模式）同样计时：approach_insert 段覆盖
  // PREVIEW_CONTACT_PLANNING（见 cycle_support.hpp 阶段投影注释）。
  startCycleTiming();
  const auto finish = [this, &response](
    bool success, CycleState state, const std::string & message)
    {
      // 先落终态再解除 running，保证 action 侧读到的一定是终态而非中间态。
      setState(state, message);
      running_.store(false);
      publishState();
      response->success = success;
      response->message = message;
    };

  const GateResult gate = quality_gate_->readyToPreviewContact(qualitySnapshot());
  const auto target = targetSnapshot();
  const auto refined = refinedSnapshot();
  if (!gate.allowed) {
    finish(false, CycleState::PREVIEW_FAILED, "接触轨迹预览质量门失败: " + gate.reason);
    return;
  }
  if (!target || !refined || target->id != refined->id ||
    graspDecisionTargetSnapshot() != target->id)
  {
    finish(false, CycleState::PREVIEW_FAILED, "接触轨迹预览目标 ID 或精化几何不一致");
    return;
  }

  // preview 隔离（重构协议阶段 B）：预览不是周期，绝不得创建/读写周期上下文
  // （周期身份钉在 ctx，由 action 受理/stagePrepareCycle 管理）——否则预览过
  // 的目标 ID 会污染后续手动周期：stagePrepareCycle 的 goal 钉死校验会把
  // "新 selected ≠ 预览残留 ID"误判为目标身份变更而失败。本函数内一律用局部
  // target/refined。
  Eigen::Isometry3d entry_tool_pose = entryToolPose(
    refined->entry, refined->axis, target->initial_pose.linear().col(0));
  const auto tip_from_tool = motion_->lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    finish(false, CycleState::PREVIEW_FAILED, "无法取得 tip 到 tool 的变换");
    return;
  }
  const Eigen::Isometry3d entry_tip_pose =
    entry_tool_pose * tip_from_tool->inverse();
  const double travel = insertionTravel(*refined);

  setState(
    CycleState::PREVIEW_CONTACT_PLANNING,
    include_retreat ? "MTC 只规划：到入口、直线插入、同轴撤离" :
    "MTC 只规划：到入口、直线插入");
  const GraspTaskResult result = include_retreat ?
    grasp_task_->previewFullContact(entry_tip_pose, refined->axis, travel) :
    grasp_task_->approachAndInsert(entry_tip_pose, refined->axis, travel);
  if (!result.success) {
    finish(false, CycleState::PREVIEW_FAILED, "MTC 接触轨迹预览失败: " + result.reason);
    return;
  }
  finish(
    true, CycleState::PREVIEW_READY,
    include_retreat ?
    "完整接触轨迹已发布到 RViz；仅规划，未发送任何运动" :
    "入口与插入轨迹已发布到 RViz；仅规划，未发送任何运动");
}

// 选果级 TCP IK 预检：请求当入口，换成与 MovePregrasp 同一停位再 setFromIK
// （后撤 mtc_approach_along_axis_m + alignFrameZ 保留当前 TCP 滚转）。只答能否，
// 不规划、不占周期互斥、不绑 Active；无解由 executor SELECT 过滤。
void ManipulationSkillsNode::onCheckReachability(
  const CheckReachability::Request::SharedPtr request,
  CheckReachability::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "check_reachability", &callback_timing_);
  response->reachable.resize(request->tcp_poses.size(), false);
  response->error_codes.resize(request->tcp_poses.size(), "");
  if (!move_group_) {
    response->message = "moveit_unavailable";
    for (auto & code : response->error_codes) {
      code = "moveit_unavailable";
    }
    return;
  }
  const auto robot_model = move_group_->getRobotModel();
  const auto group = robot_model->getJointModelGroup(planning_group_);
  if (group == nullptr) {
    response->message = "unknown_planning_group";
    for (auto & code : response->error_codes) {
      code = "invalid_group";
    }
    return;
  }
  moveit::core::RobotState seed = *move_group_->getCurrentState();
  const double timeout_s = request->timeout_s > 0.0 ?
    std::min(request->timeout_s, 1.0) : 0.1;
  Eigen::Isometry3d current_tip = Eigen::Isometry3d::Identity();
  bool have_current_tip = false;
  if (robot_model->hasLinkModel(tip_frame_)) {
    current_tip = seed.getGlobalLinkTransform(tip_frame_);
    have_current_tip = current_tip.matrix().allFinite();
  }
  for (std::size_t i = 0; i < request->tcp_poses.size(); ++i) {
    const auto & stamped = request->tcp_poses[i];
    geometry_msgs::msg::Pose pose = stamped.pose;
    // 非模型系先经 TF 转换（一般请求即 base_link==模型系）
    if (!stamped.header.frame_id.empty() &&
      stamped.header.frame_id != robot_model->getModelFrame())
    {
      try {
        auto tf = tf_buffer_.lookupTransform(
          robot_model->getModelFrame(), stamped.header.frame_id,
          tf2::TimePointZero);
        tf2::doTransform(pose, pose, tf);
      } catch (const tf2::TransformException & error) {
        response->error_codes[i] = "invalid_frame";
        continue;
      }
    }
    Eigen::Isometry3d target;
    tf2::fromMsg(pose, target);
    if (have_current_tip) {
      target = pregraspFromEntryKeepRoll(
        target, current_tip.linear(), params_.moveit.mtc_approach_along_axis_m);
    }
    moveit::core::RobotState state = seed;
    response->reachable[i] = state.setFromIK(
      group, target, tip_frame_, timeout_s);
    if (!response->reachable[i]) {
      response->error_codes[i] = "no_ik";
    }
  }
}

// 全局拍照位姿：批次编排器在发现/复扫轮次开始前调用，把机械臂送到 SRDF
// 命名状态（默认 global_photo_pose）。同步规划（execution 使能时含执行）。
// A7 起三个长规划服务独占 planning_callback_group_；规划期间只组内排队。
void ManipulationSkillsNode::onGoToPhotoPose(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  const ScopedTimer timer(get_logger(), "go_to_photo_pose", &callback_timing_);
  // 运动输出权限绑定 Active 态（A8）：非 Active 拒绝并给出明确原因。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    response->success = false;
    response->message = motion_reason;
    return;
  }
  if (!motion_) {
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
    response->message = "已有靠近/抓取周期正在运行，不能移动拍照位姿";
    return;
  }
  // 提前返回路径统一经 finish 解除占用并刷新状态投影。
  const auto finish = [this, &response](bool success, const std::string & message) {
      running_.store(false);
      publishState();
      response->success = success;
      response->message = message;
    };
  std::string message;
  const bool success = motion_->goToPhotoPose(
    photo_pose_named_target_, execution_enabled_.load(), message);
  finish(success, message);
}

MoveItMotionInterface::MoveItMotionInterface(
  moveit::planning_interface::MoveGroupInterface * move_group,
  tf2_ros::Buffer * tf_buffer,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  MoveItMotionConfig config,
  std::function<bool(std::string &)> safety_gate,
  std::function<void(const std::string &)> safety_block_hook)
: move_group_(move_group),
  tf_buffer_(tf_buffer),
  logger_(std::move(logger)),
  clock_(std::move(clock)),
  config_(std::move(config)),
  safety_gate_(std::move(safety_gate)),
  safety_block_hook_(std::move(safety_block_hook))
{
}

std::optional<Eigen::Isometry3d> MoveItMotionInterface::lookupTransform(
  const std::string & target, const std::string & source)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero, 1s);
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(
      logger_, "TF %s <- %s 不可用: %s",
      target.c_str(), source.c_str(), error.what());
    return std::nullopt;
  }
}

bool MoveItMotionInterface::planOrMoveTip(
  const Eigen::Isometry3d & tip_pose,
  const std::string & planner_id,
  bool execute,
  const std::string & label,
  bool allow_fallback)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = config_.base_frame;
  target.header.stamp = clock_->now();
  target.pose = eigenToPose(tip_pose);
  move_group_->setStartStateToCurrentState();
  move_group_->setPlanningPipelineId(config_.pilz_pipeline);
  move_group_->setPlannerId(planner_id);
  move_group_->setPlanningTime(config_.observe_planning_time_s);
  move_group_->setNumPlanningAttempts(config_.observe_planning_attempts);
  move_group_->allowReplanning(false);
  move_group_->setMaxVelocityScalingFactor(config_.transit_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.transit_acceleration_scaling);
  move_group_->setPoseTarget(target, config_.tip_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS &&
    planner_id == "LIN" && allow_fallback)
  {
    RCLCPP_WARN(
      logger_, "%s 的 LIN 规划失败，改用 Pilz PTP（仅当调用方显式允许）",
      label.c_str());
    move_group_->setPlanningPipelineId(config_.pilz_pipeline);
    move_group_->setPlannerId("PTP");
    result = move_group_->plan(plan);
  }
  move_group_->clearPoseTargets();
  move_group_->setPlanningTime(config_.default_planning_time_s);
  move_group_->setNumPlanningAttempts(config_.default_planning_attempts);
  move_group_->allowReplanning(true);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  // 观察禁止大关节绕行（看行程，不按时长）。接触/观察短移不走 PTP 兜底。
  const TrajectoryGuardLimits camera_limits{
    config_.observe_max_duration_s,
    config_.observe_max_total_joint_travel_rad,
    config_.observe_max_single_joint_travel_rad};
  const auto guard = inspectApproachTrajectory(
    plan.trajectory.joint_trajectory, camera_limits);
  if (!guard.allowed) {
    RCLCPP_WARN(
      logger_, "%s 拒绝绕行轨迹: %s", label.c_str(), guard.reason.c_str());
    return false;
  }
  if (!execute) {
    return true;
  }
  std::string safety_reason;
  if (!safety_gate_(safety_reason)) {
    if (safety_block_hook_) {
      safety_block_hook_("执行前安全门失败: " + safety_reason);
    }
    return false;
  }
  // 目标身份/新鲜度不在运动层判定：由阶段执行器 + SafetyGate 单点决策，
  // 避免与再确认的 stale 放行/记忆锚点获取性移动策略互相否决。
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveItMotionInterface::planOrMoveCamera(
  const Eigen::Isometry3d & camera_pose,
  const std::string & planner_id,
  bool execute,
  const std::string & label,
  bool allow_fallback)
{
  const auto tip_from_camera = lookupTransform(config_.tip_frame, config_.camera_frame);
  if (!tip_from_camera) {
    return false;
  }
  const Eigen::Isometry3d tip_pose = camera_pose * tip_from_camera->inverse();
  return planOrMoveTip(
    tip_pose, planner_id, execute, label, allow_fallback);
}

bool MoveItMotionInterface::goToPhotoPose(
  const std::string & named_target, bool execute, std::string & message)
{
  std::string reason;
  if (!safety_gate_(reason)) {
    message = "拍照位姿安全门未通过: " + reason;
    return false;
  }
  move_group_->setStartStateToCurrentState();
  if (!move_group_->setNamedTarget(named_target)) {
    message = "SRDF 中不存在命名状态: " + named_target;
    return false;
  }
  // 先试 Pilz PTP 管线（点到点关节空间），失败回退 OMPL（同 planOrMoveTip 风格）。
  move_group_->setPlanningPipelineId(config_.pilz_pipeline);
  move_group_->setPlannerId("PTP");
  move_group_->setPlanningTime(config_.photo_planning_time_s);
  move_group_->setNumPlanningAttempts(std::max(1, config_.default_planning_attempts));
  move_group_->setMaxVelocityScalingFactor(config_.transit_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.transit_acceleration_scaling);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "拍照位姿 Pilz PTP 规划失败(%s)，回退 OMPL（仍须过行程护栏）",
      moveit::core::errorCodeToString(result).c_str());
    move_group_->setPlanningPipelineId(config_.fallback_pipeline);
    move_group_->setPlannerId("");
    result = move_group_->plan(plan);
  }
  move_group_->setPlanningTime(config_.default_planning_time_s);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    message = "拍照位姿规划失败: " + moveit::core::errorCodeToString(result);
    return false;
  }
  const TrajectoryGuardLimits photo_limits{
    config_.transit_max_duration_s,
    config_.transit_max_total_joint_travel_rad,
    config_.transit_max_single_joint_travel_rad};
  const auto guard = inspectApproachTrajectory(
    plan.trajectory.joint_trajectory, photo_limits);
  if (!guard.allowed) {
    message = "拍照位姿拒绝绕行轨迹: " + guard.reason;
    RCLCPP_WARN(logger_, "%s", message.c_str());
    return false;
  }
  if (!execute) {
    if (!atNamedTarget(named_target, message)) {
      return false;
    }
    message = "拍照位姿规划成功；当前关节已在命名状态（execution 未使能）";
    return true;
  }
  // 执行前复核安全门：规划耗时数秒，期间现场可能拍急停。
  if (!safety_gate_(reason)) {
    message = "拍照位姿执行前安全门失败: " + reason;
    return false;
  }
  result = move_group_->execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    message = "拍照位姿执行失败: " + moveit::core::errorCodeToString(result);
    return false;
  }
  if (!atNamedTarget(named_target, message)) {
    return false;
  }
  message = "已到达全局拍照位姿";
  return true;
}

bool MoveItMotionInterface::atNamedTarget(
  const std::string & named_target, std::string & message)
{
  if (move_group_ == nullptr) {
    message = "photo_pose_mismatch: MoveIt 尚未初始化";
    return false;
  }
  const auto robot_model = move_group_->getRobotModel();
  const auto * group = robot_model->getJointModelGroup(move_group_->getName());
  if (group == nullptr) {
    message = "photo_pose_mismatch: unknown_planning_group";
    return false;
  }
  const moveit::core::RobotState current = *move_group_->getCurrentState();
  moveit::core::RobotState named = current;
  if (!named.setToDefaultValues(group, named_target)) {
    message = "photo_pose_mismatch: SRDF 中不存在命名状态: " + named_target;
    return false;
  }
  std::ostringstream detail;
  bool ok = true;
  for (const std::string & joint_name : group->getActiveJointModelNames()) {
    const double delta = std::abs(
      current.getVariablePosition(joint_name) -
      named.getVariablePosition(joint_name));
    const double velocity = std::abs(current.getVariableVelocity(joint_name));
    if (delta > config_.photo_pose_joint_tolerance_rad) {
      ok = false;
      detail << ' ' << joint_name << " dq=" << delta;
    }
    if (std::isfinite(velocity) &&
      velocity > config_.photo_pose_max_joint_vel_rad_s)
    {
      ok = false;
      detail << ' ' << joint_name << " vel=" << velocity;
    }
  }
  if (!ok) {
    message = "photo_pose_mismatch:" + detail.str();
    return false;
  }
  return true;
}

}  // namespace peach_manipulation
