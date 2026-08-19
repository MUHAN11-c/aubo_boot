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
#include <memory>
#include <sstream>
#include <string>
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

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makeFreeSpaceSolver() const
{
  auto solver = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, config_.free_space_pipeline, config_.free_space_planner);
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  return solver;
}

std::shared_ptr<mtc::solvers::CartesianPath> GraspTask::makeCartesianSolver() const
{
  auto solver = std::make_shared<mtc::solvers::CartesianPath>();
  solver->setStepSize(config_.cartesian_step_m);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  solver->setPrecision(precision);
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  return solver;
}

std::unique_ptr<mtc::stages::MoveTo> GraspTask::makeMoveToEntry(
  const std::shared_ptr<mtc::solvers::PipelinePlanner> & solver,
  const Eigen::Isometry3d & entry_tip_pose) const
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(
    "collision-aware move to refined entry", solver);
  stage->setGroup(config_.planning_group);
  stage->setIKFrame(config_.tip_frame);
  stage->setTimeout(config_.planning_time_s);
  geometry_msgs::msg::PoseStamped entry;
  entry.header.frame_id = config_.base_frame;
  entry.header.stamp = node_->now();
  entry.pose = toPose(entry_tip_pose);
  stage->setGoal(entry);
  return stage;
}

std::unique_ptr<mtc::stages::MoveRelative> GraspTask::makeLinearMove(
  const std::string & label,
  const std::shared_ptr<mtc::solvers::CartesianPath> & solver,
  const Eigen::Vector3d & direction, double distance_m) const
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(label, solver);
  stage->setGroup(config_.planning_group);
  stage->setIKFrame(config_.tip_frame);
  stage->setMinMaxDistance(distance_m, distance_m);
  geometry_msgs::msg::Vector3Stamped stamped;
  stamped.header.frame_id = config_.base_frame;
  const Eigen::Vector3d unit = direction.normalized();
  stamped.vector.x = unit.x();
  stamped.vector.y = unit.y();
  stamped.vector.z = unit.z();
  stage->setDirection(stamped);
  return stage;
}

std::unique_ptr<mtc::Task> GraspTask::makeApproachInsertTask(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  auto task = std::make_unique<mtc::Task>(task_name);
  task->loadRobotModel(node_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));

  // 室外枝叶环境先用 OMPL 搜索无碰路径到入口；接触段禁止回退 OMPL，必须保持轴向直线。
  task->add(makeMoveToEntry(makeFreeSpaceSolver(), entry_tip_pose));
  task->add(
    makeLinearMove(
      "guarded linear insertion", makeCartesianSolver(), insertion_axis,
      insertion_distance_m));
  return task;
}

GraspTaskResult GraspTask::approachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m,
  bool execute)
{
  return planAndMaybeExecute(
    makeApproachInsertTask(
      "peach_approach_insert", entry_tip_pose, insertion_axis,
      insertion_distance_m),
    execute, config_.approach_execution_gate);
}

GraspTaskResult GraspTask::preplanApproachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  // 预规划（2.13-E3）：与 approachAndInsert(execute=false) 同结构，但成功后
  // 任务连同解移入预规划槽保留（供 executePreplannedApproach 复用），失败丢弃。
  auto task = makeApproachInsertTask(
    "peach_approach_insert_preplan", entry_tip_pose, insertion_axis,
    insertion_distance_m);
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    output = planTaskOnly(active);
  } catch (const std::exception & error) {
    output.reason = error.what();
  }
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (output.success) {
      preplanned_task_ = std::move(active_task_);
    } else {
      active_task_.reset();
    }
  }
  return output;
}

GraspTaskResult GraspTask::executePreplannedApproach()
{
  std::unique_ptr<mtc::Task> task;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task = std::move(preplanned_task_);
    preplanned_task_.reset();
  }
  GraspTaskResult output;
  if (!task) {
    output.reason = "无可用预规划解（未预规划或已丢弃）";
    return output;
  }
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    // 执行期间移回 active 槽：cancel() 的 preempt 语义与内联规划路径一致。
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  try {
    output = executeSolution(active, config_.approach_execution_gate);
  } catch (const std::exception & error) {
    output.reason = error.what();
  }
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_.reset();
  }
  return output;
}

void GraspTask::discardPreplanned()
{
  std::lock_guard<std::mutex> lock(task_mutex_);
  preplanned_task_.reset();
}

GraspTaskResult GraspTask::previewFullContact(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  auto task = std::make_unique<mtc::Task>("peach_full_contact_preview");
  task->loadRobotModel(node_);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));

  task->add(makeMoveToEntry(makeFreeSpaceSolver(), entry_tip_pose));

  auto cartesian = makeCartesianSolver();
  // 完整预览在插入末端立即反向撤离，TOTG 不支持这种 180 度折返。
  // 此接口永不执行，仅需保留几何路径供 MTC/RViz 展示，因此关闭阶段时间参数化。
  cartesian->setTimeParameterization(nullptr);
  // 插入与撤离共用同一个 solver 实例（同一 MTC 解内保持配置一致）。
  task->add(
    makeLinearMove(
      "guarded linear insertion", cartesian, insertion_axis, insertion_distance_m));
  task->add(
    makeLinearMove(
      "linear retreat along insertion path", cartesian, -insertion_axis,
      insertion_distance_m));

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

  task->add(
    makeLinearMove(
      "linear retreat along insertion path", makeCartesianSolver(), -insertion_axis,
      retreat_distance_m));
  return planAndMaybeExecute(std::move(task), execute, config_.retreat_execution_gate);
}

GraspTaskResult GraspTask::planTaskOnly(mtc::Task * active)
{
  GraspTaskResult output;
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
    return output;
  }
  active->introspection().publishSolution(*active->solutions().front());
  output.success = true;
  output.reason = "MTC plan ready";
  return output;
}

GraspTaskResult GraspTask::executeSolution(
  mtc::Task * active,
  const std::function<bool(std::string &)> & execution_gate)
{
  GraspTaskResult output;
  if (execution_gate && !execution_gate(output.reason)) {
    output.reason = "execution gate rejected: " + output.reason;
    return output;
  }
  output.execution_started = true;
  const auto execute_result = active->execute(*active->solutions().front());
  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
    output.success = true;
    output.reason = "MTC execution succeeded";
  } else {
    output.reason = "MTC execution failed";
  }
  return output;
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
    output = planTaskOnly(active);
    if (output.success && execute) {
      output = executeSolution(active, execution_gate);
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
