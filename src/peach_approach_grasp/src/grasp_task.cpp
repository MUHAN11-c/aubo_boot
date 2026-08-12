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
#include "peach_approach_grasp/grasp_task.hpp"

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>

#include <exception>
#include <sstream>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace peach_approach_grasp
{
namespace mtc = moveit::task_constructor;

namespace
{
geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}
}  // namespace

GraspTask::GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config)
: node_(std::move(node)), config_(std::move(config))
{
}

GraspTask::~GraspTask() = default;

GraspTaskResult GraspTask::approachAndInsert(
  const Eigen::Isometry3d & entry_wrist_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m,
  bool execute)
{
  auto task = std::make_unique<mtc::Task>("peach_approach_insert");
  task->loadRobotModel(node_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));

  // 室外枝叶环境先用 OMPL 搜索无碰路径到入口；接触段禁止回退 OMPL，必须保持轴向直线。
  auto free_space = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, config_.free_space_pipeline, config_.free_space_planner);
  free_space->setMaxVelocityScalingFactor(config_.velocity_scaling);
  free_space->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  auto move_to_entry = std::make_unique<mtc::stages::MoveTo>(
    "collision-aware move to refined entry", free_space);
  move_to_entry->setGroup(config_.planning_group);
  move_to_entry->setIKFrame(config_.wrist_frame);
  move_to_entry->setTimeout(config_.planning_time_s);
  geometry_msgs::msg::PoseStamped entry;
  entry.header.frame_id = config_.base_frame;
  entry.header.stamp = node_->now();
  entry.pose = toPose(entry_wrist_pose);
  move_to_entry->setGoal(entry);
  task->add(std::move(move_to_entry));

  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setStepSize(config_.cartesian_step_m);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  cartesian->setPrecision(precision);
  cartesian->setMaxVelocityScalingFactor(config_.velocity_scaling);
  cartesian->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  auto insert = std::make_unique<mtc::stages::MoveRelative>(
    "guarded linear insertion", cartesian);
  insert->setGroup(config_.planning_group);
  insert->setIKFrame(config_.wrist_frame);
  insert->setMinMaxDistance(insertion_distance_m, insertion_distance_m);
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = config_.base_frame;
  direction.vector.x = insertion_axis.normalized().x();
  direction.vector.y = insertion_axis.normalized().y();
  direction.vector.z = insertion_axis.normalized().z();
  insert->setDirection(direction);
  task->add(std::move(insert));
  return planAndMaybeExecute(std::move(task), execute, config_.approach_execution_gate);
}

GraspTaskResult GraspTask::previewFullContact(
  const Eigen::Isometry3d & entry_wrist_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  auto task = std::make_unique<mtc::Task>("peach_full_contact_preview");
  task->loadRobotModel(node_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));

  auto free_space = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, config_.free_space_pipeline, config_.free_space_planner);
  free_space->setMaxVelocityScalingFactor(config_.velocity_scaling);
  free_space->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  auto move_to_entry = std::make_unique<mtc::stages::MoveTo>(
    "collision-aware move to refined entry", free_space);
  move_to_entry->setGroup(config_.planning_group);
  move_to_entry->setIKFrame(config_.wrist_frame);
  move_to_entry->setTimeout(config_.planning_time_s);
  geometry_msgs::msg::PoseStamped entry;
  entry.header.frame_id = config_.base_frame;
  entry.header.stamp = node_->now();
  entry.pose = toPose(entry_wrist_pose);
  move_to_entry->setGoal(entry);
  task->add(std::move(move_to_entry));

  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setStepSize(config_.cartesian_step_m);
  // 完整预览在插入末端立即反向撤离，TOTG 不支持这种 180 度折返。
  // 此接口永不执行，仅需保留几何路径供 MTC/RViz 展示，因此关闭阶段时间参数化。
  cartesian->setTimeParameterization(nullptr);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  cartesian->setPrecision(precision);
  cartesian->setMaxVelocityScalingFactor(config_.velocity_scaling);
  cartesian->setMaxAccelerationScalingFactor(config_.acceleration_scaling);

  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = config_.base_frame;
  direction.vector.x = insertion_axis.normalized().x();
  direction.vector.y = insertion_axis.normalized().y();
  direction.vector.z = insertion_axis.normalized().z();
  auto insert = std::make_unique<mtc::stages::MoveRelative>(
    "guarded linear insertion", cartesian);
  insert->setGroup(config_.planning_group);
  insert->setIKFrame(config_.wrist_frame);
  insert->setMinMaxDistance(insertion_distance_m, insertion_distance_m);
  insert->setDirection(direction);
  task->add(std::move(insert));

  geometry_msgs::msg::Vector3Stamped reverse = direction;
  reverse.vector.x *= -1.0;
  reverse.vector.y *= -1.0;
  reverse.vector.z *= -1.0;
  auto retreat = std::make_unique<mtc::stages::MoveRelative>(
    "linear retreat along insertion path", cartesian);
  retreat->setGroup(config_.planning_group);
  retreat->setIKFrame(config_.wrist_frame);
  retreat->setMinMaxDistance(insertion_distance_m, insertion_distance_m);
  retreat->setDirection(reverse);
  task->add(std::move(retreat));

  // 此接口没有 execute 参数，结构上保证预览服务不能下发轨迹。
  return planAndMaybeExecute(std::move(task), false, {});
}

GraspTaskResult GraspTask::retreat(
  const Eigen::Vector3d & insertion_axis,
  double retreat_distance_m,
  bool execute)
{
  auto task = std::make_unique<mtc::Task>("peach_linear_retreat");
  task->loadRobotModel(node_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));

  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setStepSize(config_.cartesian_step_m);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  cartesian->setPrecision(precision);
  cartesian->setMaxVelocityScalingFactor(config_.velocity_scaling);
  cartesian->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  auto retreat = std::make_unique<mtc::stages::MoveRelative>(
    "linear retreat along insertion path", cartesian);
  retreat->setGroup(config_.planning_group);
  retreat->setIKFrame(config_.wrist_frame);
  retreat->setMinMaxDistance(retreat_distance_m, retreat_distance_m);
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = config_.base_frame;
  direction.vector.x = -insertion_axis.normalized().x();
  direction.vector.y = -insertion_axis.normalized().y();
  direction.vector.z = -insertion_axis.normalized().z();
  retreat->setDirection(direction);
  task->add(std::move(retreat));
  return planAndMaybeExecute(std::move(task), execute, config_.retreat_execution_gate);
}

GraspTaskResult GraspTask::planAndMaybeExecute(
  std::unique_ptr<mtc::Task> task,
  bool execute,
  const std::function<bool(std::string &)> & execution_gate)
{
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    const auto result = active->plan(config_.max_solutions);
    if (result != moveit::core::MoveItErrorCode::SUCCESS || active->solutions().empty()) {
      std::ostringstream details;
      if (active->explainFailure(details) && !details.str().empty()) {
        std::string message = details.str();
        while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
          message.pop_back();
        }
        output.reason = "MTC planning failed: " + message;
      } else {
        output.reason = "MTC planning failed";
      }
    } else {
      active->introspection().publishSolution(*active->solutions().front());
      if (!execute) {
        output.success = true;
        output.reason = "MTC plan ready";
      } else if (execution_gate && !execution_gate(output.reason)) {
        output.reason = "execution gate rejected: " + output.reason;
      } else {
        output.execution_started = true;
        const auto execute_result = active->execute(*active->solutions().front());
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
          output.success = true;
          output.reason = "MTC execution succeeded";
        } else {
          output.reason = "MTC execution failed";
        }
      }
    }
  } catch (const std::exception & error) {
    output.reason = error.what();
  }
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_.reset();
  }
  return output;
}

void GraspTask::cancel()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (active_task_) {
    active_task_->preempt();
  }
}

}  // namespace peach_approach_grasp
