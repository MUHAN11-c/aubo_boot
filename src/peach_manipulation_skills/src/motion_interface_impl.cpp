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
// 运动接口默认实现 MoveItMotionInterface（注册名 moveit_motion）的方法体。
// 与节点侧薄壳（motion_interface.cpp）分离，使本编译单元只依赖
// MoveIt/TF/rclcpp，可被契约测试直接链接（工厂 inline 实例化需要构造函数
// 符号）。行为与原 ApproachGraspNode 成员函数内联实现逐行等价。
#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "eigen_conversions.hpp"
#include "motion_interface_impl.hpp"

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
  move_group_->setMaxVelocityScalingFactor(config_.transit_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.transit_acceleration_scaling);
  move_group_->setPoseTarget(target, config_.tip_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS &&
    planner_id == "LIN" && allow_fallback)
  {
    RCLCPP_WARN(
      logger_, "%s 的 LIN 规划失败，回退 OMPL PTP", label.c_str());
    move_group_->setPlanningPipelineId(config_.fallback_pipeline);
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
  if (!safety_gate_(safety_reason)) {
    if (safety_block_hook_) {
      safety_block_hook_("执行前安全门失败: " + safety_reason);
    }
    return false;
  }
  // 目标身份/新鲜度不在运动层判定：由 BT 层单点决策（设计文档第 7 节），
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
  move_group_->setMaxVelocityScalingFactor(config_.transit_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.transit_acceleration_scaling);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "拍照位姿 Pilz PTP 规划失败(%s)，回退 OMPL",
      moveit::core::errorCodeToString(result).c_str());
    move_group_->setPlanningPipelineId(config_.fallback_pipeline);
    move_group_->setPlannerId("");
    result = move_group_->plan(plan);
  }
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    message = "拍照位姿规划失败: " + moveit::core::errorCodeToString(result);
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
