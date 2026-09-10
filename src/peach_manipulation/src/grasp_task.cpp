// 功能：MTC 接触。到预抓取按官方抓取管线：GenerateGraspPose 风格滚转采样
// → Fallbacks(Pilz LIN, staging PTP 转移, CartesianPath)。沿轴套入与撤退。
// 接触段（预抓取→入口→插入→撤离）不用 PTP/OMPL；Pilz PTP 只作自由空间
// staging 转移（确定性关节插值，非 OMPL Connect）。刀具 IO 不在此文件。
#include "peach_manipulation/grasp_task.hpp"
#include "peach_manipulation/grasp_geometry.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
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

namespace peach_manipulation
{
namespace mtc = moveit::task_constructor;

namespace
{
const char * approachKindName(ApproachSplit::Kind kind)
{
  switch (kind) {
    case ApproachSplit::Kind::SKIP:
      return "skip";
    case ApproachSplit::Kind::LIN:
      return "LIN";
    case ApproachSplit::Kind::LIN_ALIGN_THEN_LIN:
      return "LIN-align+LIN";
    case ApproachSplit::Kind::CIRC_THEN_LIN:
      return "CIRC+LIN";
    case ApproachSplit::Kind::BLOCKED:
      return "blocked";
  }
  return "blocked";
}

// 接近分档结果一行日志（approachAndInsert / moveToPregrasp 共用）；
// 距入口用 split 内的 TCP 快照，不再触发第二次 TF 查询。
void logApproachSplit(
  const rclcpp::Logger & logger,
  const Eigen::Isometry3d & entry, const ApproachSplit & split)
{
  const double dist_m = split.current_tip ?
    (split.current_tip->translation() - entry.translation()).norm() :
    1.0e9;
  RCLCPP_INFO(
    logger,
    "接近：距入口 %.3fm 轴向 %.3fm 侧向 %.3fm 夹角 %.1f° 扫角 %.1f° "
    "半径 %.3fm 沿轴LIN %.3fm 刀口滚转 %.0f° 原语 %s",
    dist_m,
    split.axial_m, split.lateral_m, split.align_deg, split.sweep_deg,
    split.radius_m, split.lin_to_entry_m, split.tool_roll_rad * 180.0 / kPi,
    approachKindName(split.kind));
}

// MTC GenerateGraspPose 风格：优先小角 0, ±30 … ±150, 180。
std::vector<double> toolRollsRad()
{
  std::vector<double> rolls;
  rolls.reserve(12U);
  rolls.push_back(0.0);
  for (int step = 1; step <= 6; ++step) {
    const double rad = static_cast<double>(step) * kPi / 6.0;
    rolls.push_back(rad);
    if (step < 6) {
      rolls.push_back(-rad);
    }
  }
  return rolls;
}

ApproachSplit withToolRoll(ApproachSplit split, double roll_rad)
{
  split.tool_roll_rad = roll_rad;
  if (std::abs(roll_rad) > 1.0e-6 &&
    split.kind == ApproachSplit::Kind::LIN)
  {
    split.kind = ApproachSplit::Kind::LIN_ALIGN_THEN_LIN;
  }
  return split;
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
  out.current_tip = start;
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
    out.kind = ApproachSplit::Kind::SKIP;
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

  const Eigen::Isometry3d pregrasp = pregraspAlongAxis(
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
  // 直线不穿预抓取球：先原地对齐再平移，避免沿弦 slerp 拧腕导致
  // IK 跳支 / camera_body 撞 wrist1（mock 1757 单段 LIN ValidateSolution）。
  // 夹角已经很小才单段 LIN（姿态门挂在整段上）。
  if (lin_clears) {
    const bool already_square = out.align_deg <= 2.0;
    out.kind = already_square ? ApproachSplit::Kind::LIN :
      ApproachSplit::Kind::LIN_ALIGN_THEN_LIN;
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
    out.kind = ApproachSplit::Kind::CIRC_THEN_LIN;
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

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makePtpSolver() const
{
  return makePilzSolver("PTP");
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

void GraspTask::appendCartesianToPose(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & target_tip_pose,
  const std::string & label) const
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(label, makeCartesianSolver());
  stage->setGroup(config_.planning_group);
  stage->setIKFrame(config_.tip_frame);
  stage->setTimeout(config_.planning_time_s);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = config_.base_frame;
  goal.header.stamp = node_->now();
  goal.pose = eigenToPose(target_tip_pose);
  stage->setGoal(goal);
  sequence.add(std::move(stage));
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
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split) const
{
  Eigen::Isometry3d pregrasp = pregraspAlongAxis(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m);
  // 复用 classifyApproach 的 TCP 快照：不再二次查询（保持分档/装配一致）。
  const std::optional<Eigen::Isometry3d> & current = split.current_tip;
  const bool have_current = current && current->translation().allFinite() &&
    current->linear().allFinite();
  if (have_current) {
    pregrasp.linear() = alignFrameZRolled(
      current->linear(), insertion_axis, split.tool_roll_rad);
  }
  if (split.kind == ApproachSplit::Kind::SKIP ||
    split.kind == ApproachSplit::Kind::BLOCKED)
  {
    return;
  }
  if (split.kind == ApproachSplit::Kind::LIN) {
    appendLinToPose(sequence, pregrasp, "lin to on-axis pregrasp", true);
    return;
  }
  if (split.kind == ApproachSplit::Kind::LIN_ALIGN_THEN_LIN && have_current) {
    Eigen::Isometry3d aligned = *current;
    aligned.linear() = pregrasp.linear();
    appendLinToPose(sequence, aligned, "lin align tool z", false);
    appendLinToPose(sequence, pregrasp, "lin to on-axis pregrasp", true);
    return;
  }
  if (split.kind == ApproachSplit::Kind::CIRC_THEN_LIN) {
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
  // MTC 解显示默认关闭：滚转扫描每个候选解都会实时发 RViz，被护栏拒掉
  // 的候选在臂动之前反复闪跳（轨迹抖动）。事后 enable 不会重建发布器，
  // 故整体关闭；轨迹可视化走 observability TCP Path 与 RobotState。
  task->enableIntrospection(false);
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
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split)
{
  auto task = makeTaskShell(task_name);
  auto sequence = std::make_unique<mtc::SerialContainer>("approach to pregrasp");
  appendApproachToPregrasp(*sequence, entry_tip_pose, insertion_axis, split);
  task->add(std::move(sequence));
  return task;
}

std::unique_ptr<mtc::Task> GraspTask::makeCartesianApproachTask(
  const std::string & task_name,
  const Eigen::Isometry3d & target_tip_pose,
  const std::optional<Eigen::Isometry3d> & staging_tip_pose) const
{
  auto task = makeTaskShell(task_name);
  auto sequence = std::make_unique<mtc::SerialContainer>("cartesian approach");
  if (staging_tip_pose &&
    (staging_tip_pose->translation() - target_tip_pose.translation()).norm() >
    0.005)
  {
    appendCartesianToPose(
      *sequence, *staging_tip_pose, "cartesian interpolate to staging");
    appendLinToPose(*sequence, target_tip_pose, "lin to on-axis pregrasp", true);
  } else {
    appendCartesianToPose(
      *sequence, target_tip_pose, "cartesian interpolate to pregrasp");
  }
  task->add(std::move(sequence));
  return task;
}

// staging 转移级：Pilz PTP（确定性关节插值，最近构型关节目标）到轴上
// staging，再 LIN 沿轴到预抓取。直线 LIN / CIRC 都失败时的自由空间转移——
// 接近段保持笛卡尔；OMPL 仍不引入（1740 绕行由护栏与确定性 PTP 共同排除）。
std::unique_ptr<mtc::Task> GraspTask::makeStagingTransitTask(
  const std::string & task_name,
  const Eigen::Isometry3d & pregrasp_tip_pose,
  const Eigen::Isometry3d & staging_tip_pose,
  const std::map<std::string, double> & staging_joints)
{
  auto task = makeTaskShell(task_name);
  auto sequence =
    std::make_unique<mtc::SerialContainer>("staging transit to pregrasp");
  auto ptp = std::make_unique<mtc::stages::MoveTo>(
    "ptp to axis staging", makePtpSolver());
  ptp->setGroup(config_.planning_group);
  ptp->setIKFrame(config_.tip_frame);
  ptp->setTimeout(config_.planning_time_s);
  ptp->setGoal(staging_joints);
  sequence->add(std::move(ptp));
  // 直连变体（staging==预抓取）：PTP 已到目标，跳过退化的零距 LIN。
  if ((staging_tip_pose.translation() - pregrasp_tip_pose.translation()).norm() >
    0.005)
  {
    appendLinToPose(*sequence, pregrasp_tip_pose, "lin to on-axis pregrasp", true);
  }
  task->add(std::move(sequence));
  return task;
}


GraspTaskResult GraspTask::planToPregrasp(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split,
  bool execute)
{
  // 最近距离接近的三层（重写，非 fallback 补丁链）：
  //   1. LIN / LIN-align+LIN / CIRC（官方笛卡尔原语，短弦/已对轴最快路径）；
  //   2. 弦上直线跟踪（IK 解序列化：任意弦的零绕行构造性解，自适应位姿）；
  //   3. staging PTP 兜底（关节空间弧，过转移级门；仅直线真不可达时）。
  // 接触段不用 OMPL；Pilz PTP 只作 2/3 级的关节空间执行器（确定性插值）。
  // CIRC 失败不得改直线（会穿预抓取球）；直线跟踪与 staging 终段沿轴接近
  // 不受此限。BLOCKED（直弦穿球）跳过第 1 级。
  const bool blocked = split.kind == ApproachSplit::Kind::BLOCKED;
  GraspTaskResult last;
  last.reason = blocked ? split.blocked_reason : "无接近解";
  for (const double roll : blocked ? std::vector<double>() : toolRollsRad()) {
    const ApproachSplit rolled = withToolRoll(split, roll);
    auto result = planAndMaybeExecute(
      makeApproachOnlyTask(task_name, entry_tip_pose, insertion_axis, rolled),
      execute, config_.approach_execution_gate, true, 0U);
    if (result.success || result.execution_started) {
      if (std::abs(roll) > 1.0e-6) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Pilz 接近通过：刀口滚转 %.0f°", roll * 180.0 / kPi);
      }
      return result;
    }
    last = result;
  }
  if (tryStompApproach(
      task_name, entry_tip_pose, insertion_axis, split, execute, last))
  {
    return last;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "STOMP 接近未成（%s），降级 staging PTP 兜底（非最短路径）",
    last.reason.c_str());
  if (tryStagingTransit(
      task_name, entry_tip_pose, insertion_axis, split, execute, last))
  {
    return last;
  }
  return last;
}

// STOMP 接近（轨迹优化，最近距离主路径）：以「当前→预抓取」为问题交给
// STOMP（Jazzy moveit_planners_stomp）——关节空间线性种子 + 代价=碰撞/
// 平滑/控制，产出**最贴近直线**的无碰轨迹。直弦穿过自碰/IK 边界带时
// （笛卡尔与关节插值都物理不可行），STOMP 给出最小局部外凸，绕行幅度
// 由代价函数压到最小，且对任意目标位姿自适应（无特例分支）。
std::unique_ptr<mtc::Task> GraspTask::makeStompApproachTask(
  const std::string & task_name,
  const Eigen::Isometry3d & pregrasp_tip_pose,
  const std::map<std::string, double> * goal_joints) const
{
  auto task = makeTaskShell(task_name);
  auto solver = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, "stomp", "");
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  auto stage = std::make_unique<mtc::stages::MoveTo>(
    "stomp near-straight approach", solver);
  stage->setGroup(config_.planning_group);
  stage->setIKFrame(config_.tip_frame);
  stage->setTimeout(5.0);  // STOMP 迭代比 Pilz 慢，给足单段预算
  if (goal_joints != nullptr && !goal_joints->empty()) {
    // 关节目标优先：位姿目标的内嵌单次 IK 在边界位姿会 INVALID_GOAL_
    // CONSTRAINTS；select_goal_joints 的多种子最近构型已验证可解。
    stage->setGoal(*goal_joints);
  } else {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = config_.base_frame;
    goal.header.stamp = node_->now();
    goal.pose = eigenToPose(pregrasp_tip_pose);
    stage->setGoal(goal);
  }
  auto sequence =
    std::make_unique<mtc::SerialContainer>("stomp approach to pregrasp");
  sequence->add(std::move(stage));
  task->add(std::move(sequence));
  return task;
}

bool GraspTask::tryStompApproach(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split,
  bool execute,
  GraspTaskResult & last)
{
  if (!split.current_tip) {
    return false;
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  const Eigen::Isometry3d pregrasp0 = pregraspAlongAxis(
    entry_tip_pose, axis, config_.approach_along_axis_m);
  const double chord_m =
    (split.current_tip->translation() - pregrasp0.translation()).norm();
  if (chord_m > config_.approach_cartesian_max_distance_m) {
    last.reason = "弦长超过笛卡尔上限，不走 STOMP 接近";
    return false;
  }
  // 关节目标优先：select_goal_joints 内部已做 12 滚转 × 2 种子最近构型
  // 扫描，其解在位姿边界处比 MoveTo 内嵌单次 IK 可靠得多。
  if (config_.select_goal_joints) {
    Eigen::Isometry3d keep_roll = pregrasp0;
    keep_roll.linear() = alignFrameZ(split.current_tip->linear(), axis);
    const auto candidate = config_.select_goal_joints(keep_roll);
    if (candidate && !candidate->joints.empty()) {
      RCLCPP_INFO(
        node_->get_logger(),
        "STOMP 轨迹优化接近（关节目标，最近构型）：弦长 %.2fm", chord_m);
      auto result = planAndMaybeExecute(
        makeStompApproachTask(
          task_name + "_stomp", keep_roll, &candidate->joints),
        execute, config_.approach_execution_gate, true, 0U, true);
      if (result.success || result.execution_started) {
        last = result;
        return true;
      }
      last = result;
    }
  }
  for (const double roll : toolRollsRad()) {
    Eigen::Isometry3d goal = pregrasp0;
    goal.linear() =
      alignFrameZRolled(split.current_tip->linear(), axis, roll);
    auto result = planAndMaybeExecute(
      makeStompApproachTask(task_name + "_stomp", goal, nullptr),
      execute, config_.approach_execution_gate, true, 0U, true);
    if (result.success || result.execution_started) {
      last = result;
      return true;
    }
    last = result;
  }
  last.reason = "STOMP 接近全部滚转失败: " + last.reason;
  return false;
}


bool GraspTask::tryStagingTransit(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split,
  bool execute,
  GraspTaskResult & last)
{
  if (config_.approach_staging_standoff_m <= 0.005 || !split.current_tip ||
    !config_.select_goal_joints)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "staging 转移不可用：standoff=%.3f current_tip=%s select_goal_joints=%s",
      config_.approach_staging_standoff_m, split.current_tip ? "有" : "无",
      config_.select_goal_joints ? "有" : "无");
    return false;
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  const Eigen::Isometry3d pregrasp0 = pregraspAlongAxis(
    entry_tip_pose, axis, config_.approach_along_axis_m);
  Eigen::Isometry3d staging0 = pregrasp0;
  staging0.translation() -= axis * config_.approach_staging_standoff_m;
  staging0.linear() = alignFrameZ(split.current_tip->linear(), axis);
  const auto candidate = config_.select_goal_joints(staging0);
  if (!candidate || candidate->joints.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "staging 转移无最近构型 IK 候选（12 滚转 × 2 种子均无解）");
    last.reason = "staging 转移无最近构型 IK 候选（Pilz: " + last.reason + "）";
    return false;
  }
  Eigen::Isometry3d staging = staging0;
  staging.linear() = candidate->pose.linear();
  Eigen::Isometry3d pregrasp = pregrasp0;
  pregrasp.linear() = candidate->pose.linear();
  if (!segmentClearsBall(
      split.current_tip->translation(), staging.translation(),
      entry_tip_pose.translation(), config_.approach_along_axis_m))
  {
    last.reason = "staging 转移直线穿预抓取球，不改 PTP";
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "直线跟踪与 LIN 均未成（%s），staging PTP 转移兜底（关节空间弧，"
    "过转移级门；仅直线不可达时使用）",
    last.reason.c_str());
  auto result = planAndMaybeExecute(
    makeStagingTransitTask(
      task_name + "_staging", pregrasp, staging, candidate->joints),
    execute, config_.approach_execution_gate, true, 0U, true);
  if (result.success || result.execution_started) {
    last = result;
    return true;
  }
  // 近伸展边界：staging→预抓取的最后一小段 LIN 会因路径 IK 抖动失败，而两
  // 端 IK 均可达。此时以最近构型 PTP 直达到预抓取（不经笛卡尔直线），仍受
  // 转移级笛卡尔门+关节门约束；直线穿预抓取球则不放行（1740 式绕行另由门拒）。
  if (config_.approach_staging_standoff_m > 0.005 &&
    config_.select_goal_joints &&
    segmentClearsBall(
      split.current_tip->translation(), pregrasp0.translation(),
      entry_tip_pose.translation(), config_.approach_along_axis_m))
  {
    Eigen::Isometry3d pregrasp = pregrasp0;
    pregrasp.linear() = alignFrameZ(split.current_tip->linear(), axis);
    const auto direct = config_.select_goal_joints(pregrasp);
    if (direct && !direct->joints.empty()) {
      RCLCPP_INFO(
        node_->get_logger(),
        "staging 转移 LIN 未过，直连 PTP 到预抓取（最近构型，过转移级门）");
      result = planAndMaybeExecute(
        makeStagingTransitTask(
          task_name + "_direct", pregrasp, pregrasp, direct->joints),
        execute, config_.approach_execution_gate, true, 0U, true);
      last = result;
      return result.success || result.execution_started;
    }
  }
  last = result;
  return false;
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
  logApproachSplit(node_->get_logger(), entry_tip_pose, split);
  if (split.need_lin) {
    auto to_pregrasp = planToPregrasp(
      "peach_approach_pregrasp", entry_tip_pose, insertion_axis, split, false);
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
  if (split.kind == ApproachSplit::Kind::BLOCKED) {
    GraspTaskResult blocked;
    blocked.reason = split.blocked_reason;
    return blocked;
  }
  auto task = makeTaskShell("peach_full_contact_preview");
  auto cartesian = makeCartesianSolver();
  cartesian->setTimeParameterization(nullptr);
  auto contact = std::make_unique<mtc::SerialContainer>("preview contact");
  if (split.need_lin) {
    appendApproachToPregrasp(*contact, entry_tip_pose, insertion_axis, split);
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
  // 接近护栏只审接近段：SKIP（已对轴停在预抓取，moveToPregrasp 成功后的
  // 常态）时 task 只有 sleeve+retreat 两段，skip_tail=2 剥不掉，回退门对
  // 「插入再原路撤出」恒触发（start==goal 使 chord≈0、最深处=插入深度，
  // 必超 max_recede_m）。无接近段即无接近护栏可审；sleeve/retreat 执行段
  // 在 sleeveLinear/retreat 里各自过门。
  const std::size_t skip_tail = 2U;
  return planAndMaybeExecute(
    std::move(task), false, {}, split.need_lin, skip_tail);
}

GraspTaskResult GraspTask::moveToPregrasp(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  bool execute)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis);
  logApproachSplit(node_->get_logger(), entry_tip_pose, split);
  if (!split.need_lin) {
    GraspTaskResult already;
    already.success = true;
    already.reason = "already on-axis at pregrasp";
    return already;
  }
  return planToPregrasp(
    "peach_move_pregrasp", entry_tip_pose, insertion_axis, split, execute);
}

GraspTaskResult GraspTask::moveToPregraspViaCorridor(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  bool execute)
{
  const auto current = config_.lookup_current_tip ?
    config_.lookup_current_tip() : std::nullopt;
  if (!current) {
    GraspTaskResult out;
    out.reason = "接近回退失败: 无当前 TCP";
    return out;
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  Eigen::Isometry3d pregrasp0 = pregraspAlongAxis(
    entry_tip_pose, axis, config_.approach_along_axis_m);
  std::optional<Eigen::Isometry3d> staging0;
  if (config_.approach_staging_standoff_m > 0.005) {
    Eigen::Isometry3d pose = pregrasp0;
    pose.translation() -= axis * config_.approach_staging_standoff_m;
    staging0 = pose;
  }
  const Eigen::Vector3d line_goal = staging0 ?
    staging0->translation() : pregrasp0.translation();
  if (!segmentClearsBall(
      current->translation(), line_goal, entry_tip_pose.translation(),
      std::max(0.005, config_.approach_along_axis_m)))
  {
    GraspTaskResult out;
    out.reason = "笛卡尔回退直线穿预抓取球，不改 PTP";
    return out;
  }
  GraspTaskResult last;
  last.reason = "笛卡尔回退失败";
  for (const double roll : toolRollsRad()) {
    Eigen::Isometry3d pregrasp = pregrasp0;
    pregrasp.linear() = alignFrameZRolled(current->linear(), axis, roll);
    std::optional<Eigen::Isometry3d> staging;
    if (staging0) {
      Eigen::Isometry3d pose = *staging0;
      pose.linear() = pregrasp.linear();
      staging = pose;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "CartesianPath 回退: 刀口滚转 %.0f°%s（不走 PTP/OMPL）",
      roll * 180.0 / kPi,
      staging ? "→staging→轴向 LIN" : "");
    auto result = planAndMaybeExecute(
      makeCartesianApproachTask(
        "peach_move_pregrasp_cartesian", pregrasp, staging),
      execute, config_.approach_execution_gate, true, 0U);
    if (result.success || result.execution_started) {
      return result;
    }
    last = result;
  }
  return last;
}

std::vector<Eigen::Isometry3d> GraspTask::approachCorridorWaypoints(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis)
{
  const auto current = config_.lookup_current_tip ?
    config_.lookup_current_tip() : std::nullopt;
  if (!current) {
    return {};
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  Eigen::Isometry3d pregrasp = pregraspAlongAxis(
    entry_tip_pose, axis, config_.approach_along_axis_m);
  pregrasp.linear() = alignFrameZ(current->linear(), axis);
  if (config_.approach_staging_standoff_m > 0.005) {
    Eigen::Isometry3d staging = pregrasp;
    staging.translation() -= axis * config_.approach_staging_standoff_m;
    auto points = cartesianLineSamples(*current, staging, 1U);
    auto tail = cartesianLineSamples(staging, pregrasp, 0U);
    points.insert(points.end(), tail.begin(), tail.end());
    return points;
  }
  return cartesianLineSamples(*current, pregrasp, 1U);
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
  mtc::Task * active, bool guard_approach, std::size_t guard_skip_tail,
  bool staging_guard)
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
      // staging 转移级专用笛卡尔门（关节行程门不变）：转移 PTP 是关节空间
      // 弧，偏离/回退天然大于直连 LIN；上限仍须拒绝 1740 无约束 PTP。
      const CartesianDetourLimits cart = staging_guard ?
        CartesianDetourLimits{
          config_.staging_max_detour_ratio,
          config_.staging_max_chord_deviation_m,
          config_.staging_max_recede_m} :
        CartesianDetourLimits{
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
  // MTC 解显示保持关闭（makeTaskShell 注释）：事后 enable 不会重建
  // introspection 发布器，发布不到总线；轨迹可视化以 observability
  // TCP Path 与 RViz RobotState 为准。
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
  std::size_t guard_skip_tail,
  bool staging_guard)
{
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    output = planTaskOnly(active, guard_approach, guard_skip_tail, staging_guard);
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
