// 功能：MTC 接触。到预抓取只走 Pilz LIN / CIRC；沿轴套入与撤退。PTP 不用于接触。
// 刀具 IO 不在此文件（阶段执行器 stages.cpp / ToolActuator）。
#include "peach_manipulation/grasp_task.hpp"
#include "peach_manipulation/grasp_geometry.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.hpp>
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
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "peach_manipulation/trajectory_guard.hpp"
#include "peach_manipulation/eigen_conversions.hpp"
#include "peach_manipulation/math_utils.hpp"
#include "peach_manipulation/orientation_gate.hpp"

namespace peach_manipulation
{
namespace mtc = moveit::task_constructor;

namespace
{
// 当前 TCP 到入口点的距离；拿不到当前 TCP 视为无穷远（后续分档按不可用拒）。
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
  bool need_lin{true};
  bool need_align{true};
  enum class Kind { Skip, Lin, LinAlignThenLin, CircThenLin, Blocked } kind{
    Kind::Blocked};
  double lin_to_entry_m{0.0};
  double lateral_m{0.0};
  double axial_m{0.0};
  double align_deg{180.0};
  double sweep_deg{0.0};
  double radius_m{0.0};
  std::string blocked_reason{"无约束笛卡尔接近"};
};

const char * approachKindName(ApproachSplit::Kind kind)
{
  switch (kind) {
    case ApproachSplit::Kind::Skip:
      return "skip";
    case ApproachSplit::Kind::Lin:
      return "LIN";
    case ApproachSplit::Kind::LinAlignThenLin:
      return "LIN-align+LIN";
    case ApproachSplit::Kind::CircThenLin:
      return "CIRC+LIN";
    case ApproachSplit::Kind::Blocked:
      return "blocked";
  }
  return "blocked";
}

// 预抓取位姿 = 入口沿 −axis 后撤 standoff_m，姿态与入口一致（套入同姿态直线进）。
Eigen::Isometry3d pregraspTipPose(
  const Eigen::Isometry3d & entry, const Eigen::Vector3d & axis, double standoff_m)
{
  Eigen::Isometry3d pose = entry;
  pose.translation() -= axis.normalized() * standoff_m;
  return pose;
}

// 线段 AB 不进入以 center 为球心、radius 为半径的开球（预抓取球）。
bool segmentClearsBall(
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b,
  const Eigen::Vector3d & center,
  double radius)
{
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  if (len2 < 1.0e-12) {
    return (a - center).norm() + 1.0e-9 >= radius;
  }
  const double t = std::clamp((center - a).dot(ab) / len2, 0.0, 1.0);
  return (a + t * ab - center).norm() + 0.005 >= radius;
}

// 接近分档（只出结论，不规划）：直线不穿预抓取球 → LIN（未对轴先 LIN 原地
// 转 Z）；直线穿球且等半径短弧条件满足 → CIRC 再沿轴 LIN；弦长超上限或 CIRC
// 不可行 → Blocked（skipped_unreachable，不改 PTP）。判据细节见各分支注释。
ApproachSplit classifyApproach(
  const GraspTaskConfig & config,
  const Eigen::Isometry3d & entry,
  const Eigen::Vector3d & insertion_axis)
{
  ApproachSplit out;
  const Eigen::Vector3d axis = insertion_axis.normalized();
  out.lin_to_entry_m = config.approach_along_axis_m;
  if (!config.lookup_current_tip) {
    out.blocked_reason = "无当前 TCP，无法分档笛卡尔接近";
    return out;
  }
  const auto start = config.lookup_current_tip();
  if (!start) {
    out.blocked_reason = "无当前 TCP，无法分档笛卡尔接近";
    return out;
  }
  const Eigen::Vector3d delta = entry.translation() - start->translation();
  out.axial_m = delta.dot(axis);
  out.lateral_m = (delta - out.axial_m * axis).norm();
  const Eigen::Vector3d tip_z = start->linear().col(2);
  out.align_deg = angleBetweenDeg(tip_z, axis);
  const bool aligned = out.align_deg <= config.approach_max_align_deg;
  const bool on_line = out.lateral_m <= config.approach_max_lateral_m;
  const bool short_axial =
    out.axial_m >= -0.02 &&
    out.axial_m <= config.approach_along_axis_m + 0.02 &&
    out.axial_m <= config.approach_cartesian_max_distance_m;
  out.need_align = !aligned;
  out.need_lin = !(aligned && on_line && short_axial);
  if (!out.need_lin) {
    out.kind = ApproachSplit::Kind::Skip;
    out.lin_to_entry_m = std::max(0.0, out.axial_m);
    if (out.lin_to_entry_m < 0.005) {
      out.lin_to_entry_m = 0.0;
    }
    return out;
  }
  if (out.lin_to_entry_m < 0.005) {
    out.lin_to_entry_m = 0.0;
  }

  const Eigen::Vector3d radial = start->translation() - entry.translation();
  out.radius_m = radial.norm();
  if (out.radius_m > 1.0e-6) {
    out.sweep_deg = angleBetweenDeg(radial, -axis);
  }

  const Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry, axis, config.approach_along_axis_m);
  const bool lin_clears = segmentClearsBall(
    start->translation(), pregrasp.translation(),
    entry.translation(), config.approach_along_axis_m);
  const double chord_m =
    (start->translation() - pregrasp.translation()).norm();
  if (chord_m > config.approach_cartesian_max_distance_m) {
    out.blocked_reason = "笛卡尔弦长超过上限，不改 PTP";
    return out;
  }
  // 直线不穿预抓取球：LIN 约束 TCP。未齐则先原地 LIN 转 Z，再直线平移。
  if (lin_clears) {
    out.kind = aligned ? ApproachSplit::Kind::Lin :
      ApproachSplit::Kind::LinAlignThenLin;
    return out;
  }
  // CIRC：直线会穿球时，等半径短弧是约束下的最短路径（Pilz 取劣弧，<180°）。
  // 后撤≈0 时预抓取与入口重合，球退化，CIRC 圆心即目标点，不走。
  const double arc_m =
    out.radius_m * out.sweep_deg * (std::acos(-1.0) / 180.0);
  const bool has_approach_ball = config.approach_along_axis_m >= 0.005;
  const bool circ_ok =
    has_approach_ball &&
    out.radius_m + 0.02 >= config.approach_along_axis_m &&
    out.sweep_deg >= 5.0 &&
    out.sweep_deg < 90.0 &&
    arc_m <= config.approach_cartesian_max_distance_m + 0.05;
  if (circ_ok) {
    out.kind = ApproachSplit::Kind::CircThenLin;
    return out;
  }
  out.blocked_reason = "直线穿预抓取球且无法 CIRC，不改 PTP";
  return out;
}

// 关节轨迹逐点 FK 成 TCP 点列，供笛卡尔绕行审查（inspectCartesianDetour）。
// 关节名/维度与模型对不上返回空；调用方拿不到点列按拒发处理，不跳过审查。
std::vector<CartesianWaypoint> tcpPathFromJoints(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & tip_frame,
  const std::vector<trajectory_msgs::msg::JointTrajectory> & parts)
{
  std::vector<CartesianWaypoint> points;
  if (!model || !model->hasLinkModel(tip_frame)) {
    return points;
  }
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const auto & trajectory : parts) {
    for (const auto & point : trajectory.points) {
      if (point.positions.size() != trajectory.joint_names.size()) {
        return {};
      }
      for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
        if (!model->hasJointModel(trajectory.joint_names[i])) {
          return {};
        }
        state.setVariablePosition(
          trajectory.joint_names[i], point.positions[i]);
      }
      state.updateLinkTransforms();
      const Eigen::Vector3d p =
        state.getGlobalLinkTransform(tip_frame).translation();
      points.push_back({p.x(), p.y(), p.z()});
    }
  }
  return points;
}
}  // namespace

GraspTask::GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config)
: node_(std::move(node)), config_(std::move(config))
{
}

GraspTask::~GraspTask() = default;

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makePilzSolver(
  const std::string & planner_id) const
{
  auto solver = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, config_.free_space_pipeline, planner_id);
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  return solver;
}

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makeLinSolver() const
{
  return makePilzSolver(config_.free_space_planner);
}

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makeCircSolver() const
{
  return makePilzSolver("CIRC");
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
  entry.pose = eigenToPose(entry_tip_pose);
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

void GraspTask::appendLinToPose(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & target_tip_pose,
  const std::string & label,
  bool gate_orientation) const
{
  auto stage = makeMoveToEntry(makeLinSolver(), target_tip_pose, label);
  // 只在起点已对轴时挂门：ValidateSolution 验含起点的路点，未齐起点会
  // INVALID_MOTION_PLAN。未齐用 LIN-align+LIN，第二段再挂门。
  if (gate_orientation) {
    stage->setPathConstraints(makeOrientationGate(
      config_.tip_frame, config_.base_frame, target_tip_pose,
      config_.approach_max_align_deg, "entry_orientation_gate"));
  }
  sequence.add(std::move(stage));
}

void GraspTask::appendCircToPose(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & target_tip_pose,
  const Eigen::Vector3d & center,
  const std::string & label) const
{
  auto stage = makeMoveToEntry(makeCircSolver(), target_tip_pose, label);
  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x = center.x();
  center_pose.position.y = center.y();
  center_pose.position.z = center.z();
  center_pose.orientation.w = 1.0;
  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = config_.base_frame;
  pos_constraint.link_name = config_.tip_frame;
  pos_constraint.constraint_region.primitive_poses.resize(1);
  pos_constraint.constraint_region.primitive_poses[0] = center_pose;
  pos_constraint.weight = 1.0;
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "center";
  path_constraints.position_constraints.push_back(pos_constraint);
  stage->setPathConstraints(std::move(path_constraints));
  sequence.add(std::move(stage));
}

void GraspTask::appendApproachToPregrasp(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis) const
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  Eigen::Isometry3d pregrasp = pregraspTipPose(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  std::optional<Eigen::Isometry3d> current;
  if (config_.lookup_current_tip) {
    current = config_.lookup_current_tip();
  }
  const bool have_current = current && current->translation().allFinite() &&
    current->linear().allFinite();
  if (have_current) {
    pregrasp.linear() = alignFrameZ(current->linear(), insertion_axis);
  }
  if (split.kind == ApproachSplit::Kind::Skip ||
    split.kind == ApproachSplit::Kind::Blocked)
  {
    return;
  }
  if (split.kind == ApproachSplit::Kind::Lin) {
    appendLinToPose(sequence, pregrasp, "lin to on-axis pregrasp", true);
    return;
  }
  if (split.kind == ApproachSplit::Kind::LinAlignThenLin && have_current) {
    Eigen::Isometry3d aligned = *current;
    aligned.linear() = pregrasp.linear();
    appendLinToPose(sequence, aligned, "lin align tool z", false);
    appendLinToPose(sequence, pregrasp, "lin to on-axis pregrasp", true);
    return;
  }
  if (split.kind == ApproachSplit::Kind::CircThenLin) {
    const Eigen::Vector3d axis = insertion_axis.normalized();
    Eigen::Isometry3d circ_goal = pregrasp;
    circ_goal.translation() =
      entry_tip_pose.translation() - axis * split.radius_m;
    appendCircToPose(
      sequence, circ_goal, entry_tip_pose.translation(), "circ onto bag axis");
    if ((circ_goal.translation() - pregrasp.translation()).norm() > 0.005) {
      appendLinToPose(sequence, pregrasp, "lin to on-axis pregrasp", true);
    }
  }
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
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis)
{
  auto task = makeTaskShell(task_name);
  auto sequence = std::make_unique<mtc::SerialContainer>("approach to pregrasp");
  appendApproachToPregrasp(*sequence, entry_tip_pose, insertion_axis);
  task->add(std::move(sequence));
  return task;
}

GraspTaskResult GraspTask::planToPregrasp(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  bool execute)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  if (split.kind == ApproachSplit::Kind::Blocked) {
    GraspTaskResult blocked;
    blocked.reason = split.blocked_reason;
    return blocked;
  }
  return planAndMaybeExecute(
    makeApproachOnlyTask(task_name, entry_tip_pose, insertion_axis),
    execute, config_.approach_execution_gate, true, 0U);
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

// 只规划（PREVIEW / preview Trigger 专用）：接近分档后一次装配
// 「到预抓取 + 沿轴插入」的 plan-only 预览。执行路径已删——生产周期走
// moveToPregrasp（阶段执行器）+ previewFullContact 预验证 + sleeveLinear。
GraspTaskResult GraspTask::approachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  RCLCPP_INFO(
    node_->get_logger(),
    "接近：距入口 %.3fm 轴向 %.3fm 侧向 %.3fm 夹角 %.1f° 扫角 %.1f° "
    "半径 %.3fm 沿轴LIN %.3fm 原语 %s",
    tipToEntryDistanceM(config_, entry_tip_pose),
    split.axial_m, split.lateral_m, split.align_deg, split.sweep_deg,
    split.radius_m, split.lin_to_entry_m, approachKindName(split.kind));
  if (split.need_lin) {
    auto to_pregrasp = planToPregrasp(
      "peach_approach_pregrasp", entry_tip_pose, insertion_axis, false);
    if (!to_pregrasp.success) {
      to_pregrasp.reason = "到轴上预抓取失败: " + to_pregrasp.reason;
      return to_pregrasp;
    }
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
  return planAndMaybeExecute(
    makeApproachInsertTask(
      "peach_along_axis_insert", insertion_axis, split.lin_to_entry_m,
      insertion_distance_m),
    false, config_.approach_execution_gate, true,
    split.lin_to_entry_m > 0.005 ? 1U : 0U);
}

GraspTaskResult GraspTask::previewFullContact(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  if (split.kind == ApproachSplit::Kind::Blocked) {
    GraspTaskResult blocked;
    blocked.reason = split.blocked_reason;
    return blocked;
  }
  auto task = makeTaskShell("peach_full_contact_preview");
  auto cartesian = makeCartesianSolver();
  cartesian->setTimeParameterization(nullptr);
  auto contact = std::make_unique<mtc::SerialContainer>("preview contact");
  if (split.need_lin) {
    appendApproachToPregrasp(*contact, entry_tip_pose, insertion_axis);
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
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  RCLCPP_INFO(
    node_->get_logger(),
    "接近：距入口 %.3fm 轴向 %.3fm 侧向 %.3fm 夹角 %.1f° 扫角 %.1f° "
    "半径 %.3fm 沿轴LIN %.3fm 原语 %s",
    tipToEntryDistanceM(config_, entry_tip_pose),
    split.axial_m, split.lateral_m, split.align_deg, split.sweep_deg,
    split.radius_m, split.lin_to_entry_m, approachKindName(split.kind));
  if (!split.need_lin) {
    GraspTaskResult already;
    already.success = true;
    already.reason = "already on-axis at pregrasp";
    return already;
  }
  return planToPregrasp(
    "peach_move_pregrasp", entry_tip_pose, insertion_axis, execute);
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
    const bool check_cartesian =
      config_.approach_max_detour_ratio > 0.0 ||
      config_.approach_max_chord_deviation_m > 0.0 ||
      config_.approach_max_recede_m > 0.0;
    if (check_cartesian) {
      const auto tcp = tcpPathFromJoints(
        active->getRobotModel(), config_.tip_frame, approach_parts);
      if (tcp.size() < 2U) {
        output.reason = "MTC short-path guard rejected: 无法 FK 笛卡尔审查";
        return output;
      }
      const CartesianDetourLimits cart{
        config_.approach_max_detour_ratio,
        config_.approach_max_chord_deviation_m,
        config_.approach_max_recede_m};
      const auto cart_report = inspectCartesianDetour(tcp, cart);
      RCLCPP_INFO(
        node_->get_logger(),
        "MTC 接近笛卡尔审查: allowed=%s path=%.3fm chord=%.3fm "
        "ratio=%.2f max_dev=%.3fm recede=%.3fm (%s)",
        cart_report.allowed ? "true" : "false", cart_report.path_m,
        cart_report.chord_m, cart_report.detour_ratio, cart_report.max_dev_m,
        cart_report.max_recede_m, cart_report.reason.c_str());
      if (!cart_report.allowed) {
        output.reason = "MTC short-path guard rejected: " + cart_report.reason;
        return output;
      }
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

}  // namespace peach_manipulation
