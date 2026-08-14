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
// 运动接口：TF 查询、tip/camera/tool 规划执行、Trigger 调用、工具 IO，
// 以及接触轨迹预览与 go_to_photo_pose 的规划执行体（含执行前安全门复核）。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "approach_grasp_node_impl.hpp"
#include "eigen_conversions.hpp"

using namespace std::chrono_literals;

namespace peach_approach_grasp
{

void ApproachGraspNode::onRobotStatus(
  const aubo_msgs::msg::RobotStatus::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(robot_mutex_);
  robot_status_ = *message;
  robot_status_received_ = now();
  robot_status_valid_ = true;
}

double ApproachGraspNode::insertionTravel(const CachedRefined & refined) const
{
  // suggested_travel_m 由感知/重建端结合工具几何给出，是跨包行程契约。
  // 仅为兼容旧记录或异常消息，字段无效时才按几何距离与颈部余量回退。
  double travel = refined.suggested_travel_m;
  if (!std::isfinite(travel) || travel <= 0.0) {
    travel = (refined.neck - refined.entry).norm() - neck_margin_m_;
  }
  return std::clamp(travel, minimum_travel_m_, maximum_travel_m_);
}

bool ApproachGraspNode::safetyReady(std::string & reason)
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

bool ApproachGraspNode::cycleTargetReady(
  const std::string & target_id, std::string & reason)
{
  return safety_gate_->targetReady(cache_.targetGateSample(), target_id, reason);
}

std::optional<Eigen::Isometry3d> ApproachGraspNode::lookupTransform(
  const std::string & target, const std::string & source)
{
  try {
    const auto transform = tf_buffer_.lookupTransform(
      target, source, tf2::TimePointZero, 1s);
    return tf2::transformToEigen(transform.transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(
      get_logger(), "TF %s <- %s 不可用: %s",
      target.c_str(), source.c_str(), error.what());
    return std::nullopt;
  }
}

bool ApproachGraspNode::planOrMoveTip(
  const Eigen::Isometry3d & tip_pose,
  const std::string & planner_id,
  bool execute,
  const std::string & label,
  bool allow_fallback)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = base_frame_;
  target.header.stamp = now();
  target.pose = eigenToPose(tip_pose);
  move_group_->setStartStateToCurrentState();
  move_group_->setPlanningPipelineId(pilz_pipeline_);
  move_group_->setPlannerId(planner_id);
  move_group_->setMaxVelocityScalingFactor(transit_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(transit_acceleration_scaling_);
  move_group_->setPoseTarget(target, tip_frame_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS &&
    planner_id == "LIN" && allow_fallback)
  {
    RCLCPP_WARN(
      get_logger(), "%s 的 LIN 规划失败，回退 OMPL PTP", label.c_str());
    move_group_->setPlanningPipelineId(fallback_pipeline_);
    move_group_->setPlannerId("");
    result = move_group_->plan(plan);
  }
  move_group_->clearPoseTargets();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  if (!execute) {
    return true;
  }
  std::string safety_reason;
  if (!safetyReady(safety_reason)) {
    setState(CycleState::FAILED, "执行前安全门失败: " + safety_reason);
    return false;
  }
  // 目标身份/新鲜度不在运动层判定：由 BT 层单点决策（设计文档第 7 节），
  // 避免与 BT 的 stale 放行/记忆锚点获取性移动策略互相否决。
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool ApproachGraspNode::planOrMoveCamera(
  const Eigen::Isometry3d & camera_pose,
  const std::string & planner_id,
  bool execute,
  const std::string & label,
  bool allow_fallback)
{
  const auto tip_from_camera = lookupTransform(tip_frame_, camera_frame_);
  if (!tip_from_camera) {
    return false;
  }
  const Eigen::Isometry3d tip_pose = camera_pose * tip_from_camera->inverse();
  return planOrMoveTip(
    tip_pose, planner_id, execute, label, allow_fallback);
}

bool ApproachGraspNode::planOrMoveTool(
  const Eigen::Isometry3d & tool_pose,
  const std::string & planner_id,
  const std::string & label)
{
  const auto tip_from_tool = lookupTransform(tip_frame_, tool_frame_);
  if (!tip_from_tool) {
    return false;
  }
  return planOrMoveTip(
    tool_pose * tip_from_tool->inverse(), planner_id, true, label, false);
}

bool ApproachGraspNode::callTrigger(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const std::string & label,
  bool required)
{
  if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_s_))) {
    if (required) {
      setState(CycleState::FAILED, label + " 服务不可用");
    }
    return false;
  }
  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) !=
    std::future_status::ready)
  {
    if (required) {
      setState(CycleState::FAILED, label + " 服务超时");
    }
    return false;
  }
  if (!future.get()->success) {
    if (required) {
      setState(CycleState::FAILED, label + " 拒绝请求");
    }
    return false;
  }
  return true;
}

bool ApproachGraspNode::commandToolClose()
{
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

void ApproachGraspNode::onPreviewApproachInsert(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  previewContact(false, response);
}

void ApproachGraspNode::onPreviewFullContact(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  previewContact(true, response);
}

void ApproachGraspNode::previewContact(
  bool include_retreat, Trigger::Response::SharedPtr response)
{
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    response->success = false;
    response->message = "已有规划或执行周期正在运行";
    return;
  }
  // 预览周期同样在每周期开始时重置终局分级。
  pending_outcome_.store(RunTargetCycle::Result::FAILED);
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

  cycle_target_id_ = target->id;
  Eigen::Isometry3d entry_tool_pose = Eigen::Isometry3d::Identity();
  entry_tool_pose.translation() = refined->entry;
  entry_tool_pose.linear() = ViewPlanner::toolOrientation(
    refined->axis, target->initial_pose.linear().col(0));
  const auto tip_from_tool = lookupTransform(tip_frame_, tool_frame_);
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
// 同步规划（execution 使能时含执行），单次调用可能占用 executor 数秒——
// 沿用本包既有同步模式，不引入新的线程模型；默认互斥回调组下并发的其他
// 服务调用会排队，属预期行为。
void ApproachGraspNode::onGoToPhotoPose(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
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
  std::string reason;
  if (!safetyReady(reason)) {
    finish(false, "拍照位姿安全门未通过: " + reason);
    return;
  }
  move_group_->setStartStateToCurrentState();
  if (!move_group_->setNamedTarget(photo_pose_named_target_)) {
    finish(false, "SRDF 中不存在命名状态: " + photo_pose_named_target_);
    return;
  }
  // 先试 Pilz PTP 管线（点到点关节空间），失败回退 OMPL（同 planOrMoveTip 风格）。
  move_group_->setPlanningPipelineId(pilz_pipeline_);
  move_group_->setPlannerId("PTP");
  move_group_->setMaxVelocityScalingFactor(transit_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(transit_acceleration_scaling_);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(
      get_logger(), "拍照位姿 Pilz PTP 规划失败(%s)，回退 OMPL",
      moveit::core::errorCodeToString(result).c_str());
    move_group_->setPlanningPipelineId(fallback_pipeline_);
    move_group_->setPlannerId("");
    result = move_group_->plan(plan);
  }
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    finish(false, "拍照位姿规划失败: " + moveit::core::errorCodeToString(result));
    return;
  }
  if (!execution_enabled_.load()) {
    finish(true, "拍照位姿规划成功；仅规划（execution 未使能）");
    return;
  }
  // 执行前复核安全门：规划耗时数秒，期间现场可能拍急停。
  if (!safetyReady(reason)) {
    finish(false, "拍照位姿执行前安全门失败: " + reason);
    return;
  }
  result = move_group_->execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    finish(false, "拍照位姿执行失败: " + moveit::core::errorCodeToString(result));
    return;
  }
  finish(true, "已到达全局拍照位姿");
}

}  // namespace peach_approach_grasp
