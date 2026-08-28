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
#include "motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// === motion_interface.cpp ===
namespace peach_manipulation_skills
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
  // 安全门样本按周期生效目标取数（同 bt_nodes.cpp cycleTargetSnapshot 的
  // 分流理由）：OBSERVE_ONLY 周期取 goal 目标的锁定集锚点样本（目标门
  // 判其身份/有效性/新鲜度），其余周期取感知 selected 样本。
  const TargetGateSample sample =
    !cycle_target_id_.empty() ?
    cache_.lockedTargetGateSample(target_id) : cache_.targetGateSample();
  return safety_gate_->targetReady(sample, target_id, reason);
}

bool ManipulationSkillsNode::commandToolClose()
{
  // 工具 IO 是运动类输出（A8）：非 Active 拒绝（纵深防御——正常路径下周期
  // 根本不会在非 Active 启动，此处兜底 deactivate 竞态）。
  std::string motion_reason;
  if (!motionOutputAllowed(motion_reason)) {
    setState(CycleState::FAILED, "工具 IO 被拒绝: " + motion_reason);
    return false;
  }
  if (!tool_enabled_.load()) {
    setState(CycleState::FAILED, "grasp.enabled=true 但 tool.enabled=false");
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
  // PREVIEW_CONTACT_PLANNING（见 stage_timing.hpp 投影注释）。
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

  // preview 隔离（重构协议阶段 B）：预览不是周期，绝不得写 cycle_target_id_
  // （周期身份钉，由 action 受理/btPrepareCycle 管理）——否则预览过的目标 ID
  // 会污染后续手动周期：btPrepareCycle 的 goal 钉死校验会把"新 selected ≠
  // 预览残留 ID"误判为目标身份变更而失败。本函数内一律用局部 target/refined。
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = refined->entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    refined->axis, target->initial_pose.linear().col(0));
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
    grasp_task_->approachAndInsert(entry_tip_pose, refined->axis, travel, false);
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

// 全局拍照位姿：批次编排器在发现/复扫轮次开始前调用，把机械臂送到 SRDF
// 命名状态（默认 global_photo_pose）。与 previewContact 同为 executor 回调内
// 同步规划（execution 使能时含执行），单次调用可能占用数秒——A7 起三个长
// 规划服务（preview_approach_insert / preview_full_contact / go_to_photo_pose）
// 独占 planning_callback_group_（独立互斥组），规划期间只组内排队，默认组的
// 订阅/快捷服务/action 回调照常调度。规划/执行体在 MotionInterfaceBase 实现
// 内，本回调只保留周期互斥、recovery 守卫与响应投影。
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

}  // namespace peach_manipulation_skills

// === motion_interface_impl.cpp ===
using namespace std::chrono_literals;

namespace peach_manipulation_skills
{

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
      logger_, "%s 的 LIN 规划失败，改用 Pilz PTP（禁止 OMPL 绕行）",
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
  // 观察禁止大关节绕行：OMPL/长 PTP 曾走出数秒到数十秒。
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
  // 目标身份/新鲜度不在运动层判定：由 BT + SafetyGate 单点决策（flow.md），
  // 避免与 BT 的 stale 放行/记忆锚点获取性移动策略互相否决。
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

bool MoveItMotionInterface::planOrMoveTool(
  const Eigen::Isometry3d & tool_pose,
  const std::string & planner_id,
  const std::string & label)
{
  const auto tip_from_tool = lookupTransform(config_.tip_frame, config_.tool_frame);
  if (!tip_from_tool) {
    return false;
  }
  return planOrMoveTip(
    tool_pose * tip_from_tool->inverse(), planner_id, true, label, false);
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
    message = "拍照位姿规划成功；仅规划（execution 未使能）";
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
  message = "已到达全局拍照位姿";
  return true;
}

}  // namespace peach_manipulation_skills
