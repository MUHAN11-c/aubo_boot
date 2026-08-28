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
#include "peach_manipulation_skills/grasp_task.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "peach_manipulation_skills/trajectory_guard.hpp"

namespace peach_manipulation_skills
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

double tipToEntryDistanceM(
  const GraspTaskConfig & config, const Eigen::Isometry3d & entry)
{
  if (!config.lookup_current_tip) {
    return 1.0e9;
  }
  const auto start = config.lookup_current_tip();
  if (!start) {
    return 1.0e9;
  }
  return (start->translation() - entry.translation()).norm();
}

struct ApproachSplit
{
  bool need_ptp{true};
  double lin_to_entry_m{0.0};
  double lateral_m{0.0};
  double axial_m{0.0};
  double align_deg{180.0};
};

Eigen::Isometry3d pregraspTipPose(
  const Eigen::Isometry3d & entry, const Eigen::Vector3d & axis, double standoff_m)
{
  Eigen::Isometry3d pose = entry;
  pose.translation() -= axis.normalized() * standoff_m;
  return pose;
}

ApproachSplit classifyApproach(
  const GraspTaskConfig & config,
  const Eigen::Isometry3d & entry,
  const Eigen::Vector3d & insertion_axis)
{
  ApproachSplit out;
  const Eigen::Vector3d axis = insertion_axis.normalized();
  out.lin_to_entry_m = config.approach_along_axis_m;
  if (!config.lookup_current_tip) {
    return out;
  }
  const auto start = config.lookup_current_tip();
  if (!start) {
    return out;
  }
  const Eigen::Vector3d delta = entry.translation() - start->translation();
  out.axial_m = delta.dot(axis);
  out.lateral_m = (delta - out.axial_m * axis).norm();
  const Eigen::Vector3d tip_z = start->linear().col(2);
  const double cosine = std::clamp(tip_z.dot(axis), -1.0, 1.0);
  out.align_deg = std::acos(cosine) * (180.0 / std::acos(-1.0));
  const bool aligned = out.align_deg <= config.approach_max_align_deg;
  const bool on_line = out.lateral_m <= config.approach_max_lateral_m;
  const bool short_axial =
    out.axial_m >= -0.02 &&
    out.axial_m <= config.approach_along_axis_m + 0.02 &&
    out.axial_m <= config.approach_cartesian_max_distance_m;
  out.need_ptp = !(aligned && on_line && short_axial);
  if (!out.need_ptp) {
    out.lin_to_entry_m = std::max(0.0, out.axial_m);
  }
  if (out.lin_to_entry_m < 0.005) {
    out.lin_to_entry_m = 0.0;
  }
  return out;
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
  // 1.0 在末步常因离散/自碰只到 29/30（真机 0.9667）。0.95 仍拒绝半程插入。
  solver->setMinFraction(config_.cartesian_min_fraction);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  solver->setPrecision(precision);
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  return solver;
}

std::unique_ptr<mtc::stages::MoveTo> GraspTask::makeMoveToEntry(
  const std::shared_ptr<mtc::solvers::PipelinePlanner> & solver,
  const Eigen::Isometry3d & entry_tip_pose,
  const std::string & label) const
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, solver);
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

void GraspTask::syncKeepoutCollisionObjects() const
{
  moveit::planning_interface::PlanningSceneInterface scene;
  if (!published_keepout_ids_.empty()) {
    scene.removeCollisionObjects(published_keepout_ids_);
    published_keepout_ids_.clear();
  }
  if (config_.protected_zones.empty()) {
    return;
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.reserve(config_.protected_zones.size());
  std::size_t index = 0;
  for (const auto & zone : config_.protected_zones) {
    moveit_msgs::msg::CollisionObject object;
    object.id = "peach_keepout_" + std::to_string(index++);
    object.header.frame_id = config_.base_frame;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {
      zone.max.x() - zone.min.x(),
      zone.max.y() - zone.min.y(),
      zone.max.z() - zone.min.z()};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.5 * (zone.min.x() + zone.max.x());
    pose.position.y = 0.5 * (zone.min.y() + zone.max.y());
    pose.position.z = 0.5 * (zone.min.z() + zone.max.z());
    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    objects.push_back(object);
    published_keepout_ids_.push_back(object.id);
  }
  scene.applyCollisionObjects(objects);
}

void GraspTask::appendPtpToPose(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & target_tip_pose) const
{
  sequence.add(
    makeMoveToEntry(
      makeFreeSpaceSolver(), target_tip_pose, "ptp to on-axis pregrasp"));
}

void GraspTask::appendAlongAxisMove(
  mtc::SerialContainer & sequence,
  const Eigen::Vector3d & insertion_axis,
  double along_axis_m,
  const std::string & label) const
{
  if (along_axis_m <= 0.005) {
    return;
  }
  sequence.add(
    makeLinearMove(label, makeCartesianSolver(), insertion_axis, along_axis_m));
}

std::unique_ptr<mtc::SerialContainer> GraspTask::makeApproachInsertSequence(
  const Eigen::Vector3d & insertion_axis,
  double along_axis_m,
  double insertion_distance_m) const
{
  auto sequence = std::make_unique<mtc::SerialContainer>("approach and insert");
  appendAlongAxisMove(
    *sequence, insertion_axis, along_axis_m, "along-axis approach to entry");
  sequence->add(
    makeLinearMove(
      "guarded linear insertion", makeCartesianSolver(), insertion_axis,
      insertion_distance_m));
  return sequence;
}

std::unique_ptr<mtc::Task> GraspTask::makeTaskShell(const std::string & task_name) const
{
  auto task = std::make_unique<mtc::Task>(task_name);
  task->loadRobotModel(node_);
  task->setProperty("group", config_.planning_group);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));
  return task;
}

std::unique_ptr<mtc::Task> GraspTask::makeApproachInsertTask(
  const std::string & task_name,
  const Eigen::Vector3d & insertion_axis,
  double along_axis_m,
  double insertion_distance_m)
{
  auto task = makeTaskShell(task_name);
  task->add(
    makeApproachInsertSequence(
      insertion_axis, along_axis_m, insertion_distance_m));
  return task;
}

std::unique_ptr<mtc::Task> GraspTask::makeApproachOnlyTask(
  const std::string & task_name,
  const Eigen::Isometry3d & target_tip_pose)
{
  auto task = makeTaskShell(task_name);
  auto sequence = std::make_unique<mtc::SerialContainer>("approach to pregrasp");
  appendPtpToPose(*sequence, target_tip_pose);
  task->add(std::move(sequence));
  return task;
}

std::unique_ptr<mtc::Task> GraspTask::makeInsertOnlyTask(
  const std::string & task_name,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  auto task = makeTaskShell(task_name);
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
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  const Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  RCLCPP_INFO(
    node_->get_logger(),
    "接近沿检测轴：直线距入口 %.3fm 轴向 %.3fm 侧向 %.3fm 姿态夹角 %.1f° "
    "沿轴LIN %.3fm %s",
    tipToEntryDistanceM(config_, entry_tip_pose),
    split.axial_m, split.lateral_m, split.align_deg, split.lin_to_entry_m,
    split.need_ptp ? "先PTP到轴上预抓取" : "已对轴，只走短程LIN");
  if (split.need_ptp) {
    auto to_pregrasp = planAndMaybeExecute(
      makeApproachOnlyTask("peach_approach_pregrasp", pregrasp),
      execute, config_.approach_execution_gate, true, 0U);
    if (!to_pregrasp.success) {
      to_pregrasp.reason = "到轴上预抓取失败: " + to_pregrasp.reason;
      return to_pregrasp;
    }
    if (!execute) {
      auto along = planAndMaybeExecute(
        makeApproachInsertTask(
          "peach_along_axis_insert", insertion_axis, split.lin_to_entry_m,
          insertion_distance_m),
        false, {}, false, 0U);
      if (!along.success) {
        to_pregrasp.success = false;
        to_pregrasp.reason = "已规划到预抓取，沿轴进入未过: " + along.reason;
        return to_pregrasp;
      }
      to_pregrasp.reason = "已规划到预抓取并沿轴进入（仅规划）";
      return to_pregrasp;
    }
  }
  auto along = planAndMaybeExecute(
    makeApproachInsertTask(
      "peach_along_axis_insert", insertion_axis, split.lin_to_entry_m,
      insertion_distance_m),
    execute, config_.approach_execution_gate, true,
    split.lin_to_entry_m > 0.005 ? 1U : 0U);
  if (along.success || along.execution_started) {
    if (along.success) {
      return along;
    }
  } else if (execute && split.lin_to_entry_m > 0.005) {
    RCLCPP_WARN(
      node_->get_logger(),
      "沿轴进入+插入一体失败（%s），改分步沿轴再插入",
      along.reason.c_str());
    auto to_entry = planAndMaybeExecute(
      makeInsertOnlyTask(
        "peach_along_axis_to_entry", insertion_axis, split.lin_to_entry_m),
      true, config_.approach_execution_gate, false, 0U);
    if (!to_entry.success) {
      to_entry.reason = "沿轴到入口失败: " + to_entry.reason;
      return to_entry;
    }
    along = planAndMaybeExecute(
      makeInsertOnlyTask(
        "peach_linear_insert", insertion_axis, insertion_distance_m),
      true, config_.approach_execution_gate, false, 0U);
    if (along.success) {
      return along;
    }
  } else if (!execute) {
    return along;
  }
  if (along.success) {
    return along;
  }
  if (!execute) {
    return along;
  }
  const auto back = retreat(insertion_axis, insertion_distance_m, true);
  along.execution_started = true;
  along.success = false;
  along.reason = "已到抓取入口但插入未完成: " + along.reason +
    "；撤离: " + back.reason;
  return along;
}

GraspTaskResult GraspTask::preplanOneTask(
  std::unique_ptr<mtc::Task> task, std::size_t guard_skip_tail)
{
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    output = planTaskOnly(active, true, guard_skip_tail);
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

GraspTaskResult GraspTask::preplanApproachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  const Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  const std::size_t skip_tail = split.lin_to_entry_m > 0.005 ? 1U : 0U;
  if (!split.need_ptp) {
    return preplanOneTask(
      makeApproachInsertTask(
        "peach_approach_insert_preplan", insertion_axis, split.lin_to_entry_m,
        insertion_distance_m),
      skip_tail);
  }
  auto task = makeTaskShell("peach_approach_insert_preplan");
  auto sequence = std::make_unique<mtc::SerialContainer>("pregrasp then along-axis");
  appendPtpToPose(*sequence, pregrasp);
  appendAlongAxisMove(
    *sequence, insertion_axis, split.lin_to_entry_m, "along-axis approach to entry");
  sequence->add(
    makeLinearMove(
      "guarded linear insertion", makeCartesianSolver(), insertion_axis,
      insertion_distance_m));
  task->add(std::move(sequence));
  return preplanOneTask(std::move(task), skip_tail);
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
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  const Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  auto task = makeTaskShell("peach_full_contact_preview");
  auto cartesian = makeCartesianSolver();
  cartesian->setTimeParameterization(nullptr);
  auto contact = std::make_unique<mtc::SerialContainer>("preview contact");
  if (split.need_ptp) {
    appendPtpToPose(*contact, pregrasp);
  }
  const double sleeve_m = split.lin_to_entry_m + insertion_distance_m;
  contact->add(
    makeLinearMove(
      "sleeve linear along bag axis", cartesian, insertion_axis, sleeve_m));
  contact->add(
    makeLinearMove(
      "linear retreat along insertion path", cartesian, -insertion_axis,
      sleeve_m));
  task->add(std::move(contact));
  const std::size_t skip_tail = 2U;
  return planAndMaybeExecute(std::move(task), false, {}, true, skip_tail);
}

GraspTaskResult GraspTask::moveToPregrasp(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  bool execute)
{
  const Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  return planAndMaybeExecute(
    makeApproachOnlyTask("peach_move_pregrasp", pregrasp),
    execute, config_.approach_execution_gate, true, 0U);
}

GraspTaskResult GraspTask::sleeveLinear(
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m,
  bool execute)
{
  const double sleeve_m =
    config_.approach_along_axis_m + insertion_distance_m;
  return planAndMaybeExecute(
    makeInsertOnlyTask(
      "peach_sleeve_linear", insertion_axis, sleeve_m),
    execute, config_.approach_execution_gate, true, 0U);
}

GraspTaskResult GraspTask::retreat(
  const Eigen::Vector3d & insertion_axis,
  double retreat_distance_m,
  bool execute)
{
  auto task = makeTaskShell("peach_linear_retreat");
  task->add(
    makeLinearMove(
      "linear retreat along insertion path", makeCartesianSolver(), -insertion_axis,
      retreat_distance_m));
  return planAndMaybeExecute(std::move(task), execute, config_.retreat_execution_gate);
}

GraspTaskResult GraspTask::planTaskOnly(
  mtc::Task * active, bool guard_approach, std::size_t guard_skip_tail)
{
  syncKeepoutCollisionObjects();
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
  if (guard_approach) {
    moveit_task_constructor_msgs::msg::Solution solution;
    active->solutions().front()->toMsg(solution);
    std::vector<trajectory_msgs::msg::JointTrajectory> approach_parts;
    approach_parts.reserve(solution.sub_trajectory.size());
    for (const auto & sub : solution.sub_trajectory) {
      const auto & trajectory = sub.trajectory.joint_trajectory;
      if (!trajectory.joint_names.empty() && !trajectory.points.empty()) {
        approach_parts.push_back(trajectory);
      }
    }
    if (guard_skip_tail > 0U && approach_parts.size() > guard_skip_tail) {
      approach_parts.resize(approach_parts.size() - guard_skip_tail);
    }
    if (approach_parts.empty()) {
      output.reason = "MTC short-path guard rejected: 缺少接近轨迹";
      return output;
    }
    const TrajectoryGuardLimits limits{
      config_.approach_max_duration_s,
      config_.approach_max_total_joint_travel_rad,
      config_.approach_max_single_joint_travel_rad};
    const auto report = inspectApproachTrajectories(approach_parts, limits);
    RCLCPP_INFO(
      node_->get_logger(),
      "MTC 接近短路径审查: segments=%zu "
      "allowed=%s points=%zu duration=%.3fs "
      "joint_total=%.3frad joint_max=%.3frad (%s)",
      approach_parts.size(),
      report.allowed ? "true" : "false", report.point_count,
      report.duration_s, report.total_joint_travel_rad,
      report.max_single_joint_travel_rad, report.reason.c_str());
    if (!report.allowed) {
      output.reason = "MTC short-path guard rejected: " + report.reason;
      return output;
    }
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
  const std::function<bool(std::string &)> & execution_gate,
  bool guard_approach,
  std::size_t guard_skip_tail)
{
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    output = planTaskOnly(active, guard_approach, guard_skip_tail);
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

}  // namespace peach_manipulation_skills
