// 功能：MTC 接触。接近主路径 = 斜直线（面内一跳到预抓取下方轴上）+
// + 沿轴垂直进入；斜插 keep-roll 对轴）；规划失败才
// PTP staging 兜底。已对轴/已在袋底侧的短修正走直连 LIN。沿轴套入与撤退；
// 返程倒放同一接近轨迹。
// G/under 单弦档已删（photo→G 弦 fraction 0.41–0.73，2026-09-10）。
// 刀具 IO 不在此文件。
#include "peach_arm/grasp_task.hpp"
#include "execution_guard.hpp"
#include "peach_arm/acm_policy.hpp"
#include "peach_arm/grasp_geometry.hpp"

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
#include <atomic>
#include <chrono>
#include <future>
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
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "peach_arm/trajectory_guard.hpp"
#include "peach_arm/eigen_conversions.hpp"
#include "peach_arm/math_utils.hpp"

namespace peach_arm
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
    case ApproachSplit::Kind::STAGING:
      return "staging";
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
    split.radius_m, split.lin_to_entry_m,
        split.tool_roll_rad * 180.0 / static_cast<double>(EIGEN_PI),
    approachKindName(split.kind));
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

// 接近分档：主路径 STAGING=PTP 关节空间+沿轴垂直进入。SKIP=已对轴
// 停在预抓取；LIN=已在果下方（s≤0）且直连工具扫掠不触果胶囊、弦长在限内
// 的短修正（含预抓取残差修正这类小位移）。其余一律 STAGING；无当前 TCP
// 才 BLOCKED。
ApproachSplit classifyApproach(
  const GraspTaskConfig & config,
  const Eigen::Isometry3d & entry,
  const Eigen::Vector3d & insertion_axis,
  const FruitCapsule & fruit)
{
  ApproachSplit out;
  const Eigen::Vector3d axis = insertion_axis.normalized();
  out.lin_to_entry_m = config.approach_along_axis_m;
  if (!config.lookup_current_tip) {
    out.blocked_reason = "无当前 TCP，无法分档接近";
    return out;
  }
  const auto start = config.lookup_current_tip();
  if (!start) {
    out.blocked_reason = "无当前 TCP，无法分档接近";
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

  // LIN 档资格：起点已在果下方（s≤0，锚果底平面）且直连工具扫掠不触
  // 果实胶囊、弦长在限内。
  Eigen::Isometry3d pregrasp = pregraspAlongAxis(
    entry, axis, config.approach_along_axis_m);
  pregrasp.linear() = alignFrameZ(start->linear(), axis);
  double start_s = 0.0;
  double start_r = 0.0;
  axialRadial(start->translation(), fruit, start_s, start_r);
  (void)start_r;
  const double chord_m =
    (start->translation() - pregrasp.translation()).norm();
  const bool lin_eligible =
    start_s <= 0.0 &&
    !toolSweepHitsFruit(
      *start, pregrasp, fruit, config.tool_body_length_m,
      config.tool_body_radius_m) &&
    chord_m <= config.approach_cartesian_max_distance_m;
  if (lin_eligible) {
    const bool already_square = out.align_deg <= 2.0;
    out.kind = already_square ? ApproachSplit::Kind::LIN :
      ApproachSplit::Kind::LIN_ALIGN_THEN_LIN;
    return out;
  }
  out.kind = ApproachSplit::Kind::STAGING;
  return out;
}


// 关节轨迹逐点 FK 成 TCP 点列（PF-4/W5-11：单次 FK，按段切分返回）。
// 返回值 per_part[i] = 第 i 段的 TCP 点列（段序=parts 序）；笛卡尔绕行/
// 姿态审查的全量点列 = 顺序拼接（同一批 FK 结果，不二次正运动学）。
// 关节名/维度与模型对不上返回空；调用方拿不到点列按拒发处理，不跳过审查。
std::vector<std::vector<CartesianWaypoint>> tcpPathsFromJointsPerPart(
  const moveit::core::RobotModelConstPtr & model,
  const std::string & tip_frame,
  const std::vector<trajectory_msgs::msg::JointTrajectory> & parts)
{
  std::vector<std::vector<CartesianWaypoint>> per_part;
  per_part.reserve(parts.size());
  if (!model || !model->hasLinkModel(tip_frame)) {
    return {};
  }
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const auto & trajectory : parts) {
    std::vector<CartesianWaypoint> segment;
    segment.reserve(trajectory.points.size());
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
      const Eigen::Isometry3d tip = state.getGlobalLinkTransform(tip_frame);
      const Eigen::Vector3d p = tip.translation();
      const Eigen::Quaterniond q(tip.linear());
      segment.push_back({p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()});
    }
    per_part.push_back(std::move(segment));
  }
  return per_part;
}
}  // namespace

GraspTask::GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config)
: node_(std::move(node)), config_(std::move(config)),
  retiring_(std::make_unique<RetireBucket>())
{
}

GraspTask::~GraspTask()
{
  retiring_->dispose();
}

void GraspTask::setContactAcm(const std::string & target_id, ContactAcmStage stage)
{
  pending_acm_target_id_ = target_id;
  pending_acm_stage_ = stage;
}

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makePilzSolver(
  const std::string & planner_id,
  double velocity_scaling,
  double acceleration_scaling) const
{
  auto solver = std::make_shared<mtc::solvers::PipelinePlanner>(
    node_, config_.free_space_pipeline, planner_id);
  const double vel =
    velocity_scaling > 0.0 ? velocity_scaling : config_.velocity_scaling;
  const double acc = acceleration_scaling > 0.0 ?
    acceleration_scaling : std::min(1.0, vel * 2.0);
  solver->setMaxVelocityScalingFactor(vel);
  solver->setMaxAccelerationScalingFactor(std::min(1.0, acc));
  return solver;
}

std::shared_ptr<mtc::solvers::PipelinePlanner> GraspTask::makePtpSolver() const
{
  return makePilzSolver("PTP");
}

std::shared_ptr<mtc::solvers::CartesianPath> GraspTask::makeCartesianSolver(
  double velocity_scaling) const
{
  auto solver = std::make_shared<mtc::solvers::CartesianPath>();
  solver->setStepSize(config_.cartesian_step_m);
  // 1.0 在末步常因离散/自碰只到 29/30（真机 0.9667）。0.95 仍拒绝半程插入。
  solver->setMinFraction(config_.cartesian_min_fraction);
  moveit::core::CartesianPrecision precision;
  precision.translational = config_.cartesian_precision_m;
  solver->setPrecision(precision);
  const double scaling =
    velocity_scaling > 0.0 ? velocity_scaling : config_.velocity_scaling;
  solver->setMaxVelocityScalingFactor(scaling);
  solver->setMaxAccelerationScalingFactor(
    std::min(1.0, scaling * 2.0));  // 近果低档时加速度同步压低
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
  applyToolOctomapExemption(scene);
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

// ③层工具豁免合并实现（W5-5，原整图/轮内两份 90% 重复）：
//   - 整图：工具链（tool.links 档案）× <octomap> = allowed；
//   - per_target_id 非空（PerTarget 策略）：接触阶段 指定目标对象 × 接触
//     连杆（tool.contact_links 档案，经 acmAllows 阶段策略过滤）。
// MoveIt setPlanningSceneDiffMsg 在 ACM entry_names 非空时用消息矩阵
// **整表替换** SRDF 相邻豁免，不能只发子方阵：先 GetPlanningScene 取现行
// ACM，setEntry 后回写全表。对象名是保留名 "<octomap>"
// （planning_scene.cpp OCTOMAP_NS），不是 "octomap"。GetPlanningScene 走
// 独立短命节点，避免在技能 planning callback group 上 wait 同源服务死锁。
void GraspTask::applyOctomapExemptionImpl(
  const rclcpp::Logger & logger,
  moveit::planning_interface::PlanningSceneInterface & scene,
  const std::vector<std::string> & tool_links,
  const std::vector<std::string> & contact_tool_links,
  const std::string & per_target_id,
  ContactAcmStage per_target_stage)
{
  static std::atomic<int> fetch_seq{0};
  const std::string helper_name =
    "peach_octomap_acm_" + std::to_string(fetch_seq.fetch_add(1));
  auto helper = std::make_shared<rclcpp::Node>(helper_name);
  auto client = helper->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      logger, "get_planning_scene 不可用，跳过工具×octomap ACM 豁免");
    return;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = client->async_send_request(request);
  const auto spin_rc = rclcpp::spin_until_future_complete(
    helper, future, std::chrono::seconds(2));
  if (spin_rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      logger, "读取现行 ACM 超时，跳过工具×octomap 豁免（避免整表替换）");
    return;
  }
  const auto response = future.get();
  if (!response || response->scene.allowed_collision_matrix.entry_names.empty()) {
    RCLCPP_WARN(
      logger, "现行 ACM 为空，跳过工具×octomap 豁免（避免冲掉 SRDF）");
    return;
  }
  collision_detection::AllowedCollisionMatrix acm(
    response->scene.allowed_collision_matrix);
  if (allowToolVersusWholeOctomap()) {
    for (const auto & link : tool_links) {
      acm.setEntry(link, "<octomap>", true);
    }
  }
  // 接触阶段只对接触连杆 × 指定目标对象放行；默认不豁免整张 octomap。
  if (!per_target_id.empty()) {
    for (const auto & link : contact_tool_links) {
      if (acmAllows(per_target_id, link, per_target_stage, contact_tool_links)) {
        acm.setEntry(link, per_target_id, true);
      }
    }
  }
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  acm.getMessage(diff.allowed_collision_matrix);
  scene.applyPlanningScene(diff);
}

void GraspTask::applyWholeOctomapToolExemption(
  const rclcpp::Logger & logger,
  moveit::planning_interface::PlanningSceneInterface & scene,
  const std::vector<std::string> & tool_links)
{
  applyOctomapExemptionImpl(
    logger, scene, tool_links, {}, std::string(), ContactAcmStage::Transit);
}

void GraspTask::applyToolOctomapExemption(
  moveit::planning_interface::PlanningSceneInterface & scene,
  OctomapExemptionPolicy policy) const
{
  applyOctomapExemptionImpl(
    node_ ? node_->get_logger() : rclcpp::get_logger("peach_arm"), scene,
    config_.tool_links, config_.contact_tool_links,
    policy == OctomapExemptionPolicy::PerTarget ? pending_acm_target_id_ :
    std::string(), pending_acm_stage_);
}

void GraspTask::appendLinToPose(
  mtc::SerialContainer & sequence,
  const Eigen::Isometry3d & target_tip_pose,
  const std::string & label,
  bool gate_orientation,
  double velocity_scaling,
  double acceleration_scaling) const
{
  auto stage = makeMoveToEntry(
    makePilzSolver(
      config_.free_space_planner, velocity_scaling, acceleration_scaling),
    target_tip_pose, label);
  // 只在起点已对轴时挂门：ValidateSolution 验含起点的路点，未齐起点会
  // INVALID_MOTION_PLAN。未齐用 LIN-align+LIN，第二段再挂门。
  if (gate_orientation) {
    stage->setPathConstraints(makeOrientationGate(
      config_.tip_frame, config_.base_frame, target_tip_pose,
      config_.approach_max_align_deg, "entry_orientation_gate"));
  }
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
  // STAGING 档主路径走 tryStagingTransit（v4：PTP+垂直入冠+沿轴）。
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
    makeLinearMove(
      label, makeCartesianSolver(config_.approach_near_velocity_scaling),
      insertion_axis, along_axis_m));
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
      "guarded linear insertion",
      makeCartesianSolver(config_.approach_near_velocity_scaling), insertion_axis,
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

// 主路径序列本体（v4）：Pilz PTP（确定性关节插值，正常转移速度档）落到
// 中段点正下方（垂直线上、树冠外），世界垂直 LIN 上行入冠到中段点
// （伸进果树里），再沿轴 LIN 对轴进入预抓取。PTP 弧与两段 LIN 都过
// 果实胶囊 FK 审查与关节行程护栏（staging_guard）。
std::unique_ptr<mtc::SerialContainer> GraspTask::makeStagingSequence(
  const Eigen::Isometry3d & pregrasp_tip_pose,
  const Eigen::Isometry3d & mid_tip_pose,
  const Eigen::Isometry3d & staging_tip_pose,
  const std::map<std::string, double> & staging_joints,
  const std::string & label) const
{
  auto sequence = std::make_unique<mtc::SerialContainer>(label);
  auto ptp = std::make_unique<mtc::stages::MoveTo>(
    "ptp below canopy entry", makePtpSolver());
  ptp->setGroup(config_.planning_group);
  ptp->setIKFrame(config_.tip_frame);
  ptp->setTimeout(config_.planning_time_s);
  ptp->setGoal(staging_joints);
  sequence->add(std::move(ptp));
  if ((staging_tip_pose.translation() - mid_tip_pose.translation()).norm() >
    0.005)
  {
    appendLinToPose(
      *sequence, mid_tip_pose, "vertical canopy entry lin", true);
  }
  if ((mid_tip_pose.translation() - pregrasp_tip_pose.translation()).norm() >
    0.005)
  {
    appendLinToPose(
      *sequence, pregrasp_tip_pose, "lin to on-axis pregrasp", true);
  }
  return sequence;
}

std::unique_ptr<mtc::Task> GraspTask::makeStagingTransitTask(
  const std::string & task_name,
  const Eigen::Isometry3d & pregrasp_tip_pose,
  const Eigen::Isometry3d & mid_tip_pose,
  const Eigen::Isometry3d & staging_tip_pose,
  const std::map<std::string, double> & staging_joints)
{
  auto task = makeTaskShell(task_name);
  task->add(makeStagingSequence(
    pregrasp_tip_pose, mid_tip_pose, staging_tip_pose, staging_joints,
    "staging transit to pregrasp"));
  return task;
}

GraspTaskResult GraspTask::planToPregrasp(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split,
  const FruitCapsule & fruit,
  bool execute)
{
  pending_fruit_ = fruit;
  inspect_fruit_ = true;
  GraspTaskResult last;
  last.reason = split.kind == ApproachSplit::Kind::BLOCKED ?
    split.blocked_reason : "无接近解";
  if (split.kind == ApproachSplit::Kind::BLOCKED) {
    inspect_fruit_ = false;
    return last;
  }
  // 主档：PTP 关节空间到轴上 staging + 沿轴垂直直线进入（keep-roll 单次
  // IK，无候选扫描/降速档）。不满足即失败收口（2026-09-23 定型三版）。
  // LIN 档起点已在果下方，走短修正直连。
  if (tryStagingTransit(
      task_name, entry_tip_pose, insertion_axis, split, execute, last))
  {
    inspect_fruit_ = false;
    return last;
  }
  // 已在果下方且直连工具扫掠不触果胶囊的短修正走 LIN（预抓取残差修正
  // 等）——这是几何分档，不是兜底链。G/under 单弦档已删（文件头数据）。
  if (split.kind == ApproachSplit::Kind::LIN ||
    split.kind == ApproachSplit::Kind::LIN_ALIGN_THEN_LIN)
  {
    if (tryRolledApproach(
        task_name, entry_tip_pose, insertion_axis, split, split.kind, execute,
        last))
    {
      inspect_fruit_ = false;
      return last;
    }
  }
  inspect_fruit_ = false;
  return last;
}

bool GraspTask::tryRolledApproach(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  ApproachSplit split,
  ApproachSplit::Kind kind,
  bool execute,
  GraspTaskResult & last)
{
  split.kind = kind;
  for (const double roll : toolRollsRad()) {
    ApproachSplit rolled = withToolRoll(split, roll);
    rolled.kind = kind;
    if (kind == ApproachSplit::Kind::LIN &&
      (std::abs(roll) > 1.0e-6 || split.need_align))
    {
      rolled.kind = ApproachSplit::Kind::LIN_ALIGN_THEN_LIN;
    }
    auto result = planAndMaybeExecute(
      makeApproachOnlyTask(task_name, entry_tip_pose, insertion_axis, rolled),
      execute, config_.approach_execution_gate, true, 0U);
    if (result.success || result.execution_started) {
      if (std::abs(roll) > 1.0e-6) {
        RCLCPP_INFO(
          node_->get_logger(),
          "接近通过（%s）：刀口滚转 %.0f°",
          approachKindName(rolled.kind), roll * 180.0 / static_cast<double>(EIGEN_PI));
      }
      last = result;
      return true;
    }
    last = result;
  }
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
  // 接近主路径（唯一路径，2026-09-23 定型四版）：Pilz PTP 关节空间落到
  // 中段点正下方（世界垂直线上、树冠外，滚转梯子 IK），世界垂直 LIN 上行
  // 入冠到中段点（伸进果树里），再沿轴 LIN 对轴进入预抓取。PTP 弧过
  // 果实胶囊审查（staging_guard）。无候选扫描/降速档；失败即收口带码。
  if (!split.current_tip || !config_.staging_ik) {
    last.reason = "PTP 接近不可用：无当前 TCP 或 IK 钩子缺席";
    return false;
  }
  const Eigen::Vector3d axis = insertion_axis.normalized();
  // 梯子先在 roll-0 几何上试 IK；命中滚转后用 stagingWaypoints 重建三
  // 路点姿态（PTP 落点与两段 LIN 目标同滚转族，20° 门可过，审查 P1-2）。
  const Eigen::Isometry3d probe_pose = [&] {
      Eigen::Isometry3d p = pregraspAlongAxis(
        entry_tip_pose, axis, config_.approach_along_axis_m);
      p.translation() -= axis * config_.approach_final_axial_m;
      p.translation() -= Eigen::Vector3d::UnitZ() *
        config_.approach_canopy_entry_m;
      p.linear() = alignFrameZ(split.current_tip->linear(), axis);
      return p;
    }();
  const auto ik = config_.staging_ik(probe_pose);
  if (!ik) {
    last.reason = "入冠落点滚转梯子 IK 无解";
    return false;
  }
  const auto [joints, roll] = *ik;
  const auto w = stagingWaypoints(
    entry_tip_pose, axis, split.current_tip->linear(), roll,
    config_.approach_along_axis_m, config_.approach_final_axial_m,
    config_.approach_canopy_entry_m);
  RCLCPP_INFO(
    node_->get_logger(),
    "PTP 接近（主路径）：关节空间到中段点正下方 + 垂直入冠 %.3fm + 沿轴 %.3fm（滚转 %.0f°）",
    config_.approach_canopy_entry_m, config_.approach_final_axial_m,
    roll * 180.0 / static_cast<double>(EIGEN_PI));
  auto result = planAndMaybeExecute(
    makeStagingTransitTask(
      task_name + "_staging", w.pregrasp, w.mid, w.staging, joints),
    execute, config_.approach_execution_gate, true, 0U, true);
  if (result.success || result.execution_started) {
    last = result;
    return true;
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
      "guarded linear insertion",
      makeCartesianSolver(config_.approach_near_velocity_scaling), insertion_axis,
      insertion_distance_m));
  return task;
}

// 只规划（PREVIEW / preview Trigger 专用）：接近分档后一次装配
// 「到预抓取 + 沿轴插入」的 plan-only 预览。执行路径已删——生产周期走
// moveToPregrasp（阶段执行器）+ previewFullContact 预验证 + sleeveLinear。
GraspTaskResult GraspTask::approachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m,
  const FruitCapsule & fruit)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis, fruit);
  logApproachSplit(node_->get_logger(), entry_tip_pose, split);
  if (split.need_lin) {
    auto to_pregrasp = planToPregrasp(
      "peach_approach_pregrasp", entry_tip_pose, insertion_axis, split, fruit,
      false);
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
  double insertion_distance_m,
  const FruitCapsule & fruit)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis, fruit);
  if (split.kind == ApproachSplit::Kind::BLOCKED) {
    GraspTaskResult blocked;
    blocked.reason = split.blocked_reason;
    return blocked;
  }
  auto task = makeTaskShell("peach_full_contact_preview");
  auto cartesian = makeCartesianSolver(config_.approach_near_velocity_scaling);
  cartesian->setTimeParameterization(nullptr);
  auto contact = std::make_unique<mtc::SerialContainer>("preview contact");
  pending_fruit_ = fruit;
  inspect_fruit_ = split.need_lin;
  if (split.need_lin) {
    if (split.kind == ApproachSplit::Kind::STAGING) {
      // 预览与执行同一形状：PTP 关节空间 + 沿轴垂直进入（keep-roll 单次 IK）。
      if (!split.current_tip || !config_.staging_ik) {
        inspect_fruit_ = false;
        GraspTaskResult out;
        out.reason = "预览：PTP 接近不可用（无当前 TCP / IK 钩子缺席）";
        return out;
      }
      const Eigen::Vector3d axis = insertion_axis.normalized();
      Eigen::Isometry3d probe_pose = pregraspAlongAxis(
        entry_tip_pose, axis, config_.approach_along_axis_m);
      probe_pose.translation() -= axis * config_.approach_final_axial_m;
      probe_pose.translation() -= Eigen::Vector3d::UnitZ() *
        config_.approach_canopy_entry_m;
      probe_pose.linear() = alignFrameZ(split.current_tip->linear(), axis);
      const auto ik = config_.staging_ik(probe_pose);
      if (!ik) {
        inspect_fruit_ = false;
        GraspTaskResult out;
        out.reason = "预览：入冠落点滚转梯子 IK 无解";
        return out;
      }
      const auto [joints, roll] = *ik;
      const auto w = stagingWaypoints(
        entry_tip_pose, axis, split.current_tip->linear(), roll,
        config_.approach_along_axis_m, config_.approach_final_axial_m,
        config_.approach_canopy_entry_m);
      contact->add(makeStagingSequence(
        w.pregrasp, w.mid, w.staging, joints, "staging transit to pregrasp"));
    } else {
      appendApproachToPregrasp(*contact, entry_tip_pose, insertion_axis, split);
    }
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
  auto result = planAndMaybeExecute(
    std::move(task), false, {}, split.need_lin, skip_tail, true,
    false);
  inspect_fruit_ = false;
  return result;
}

GraspTaskResult GraspTask::moveToPregrasp(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const FruitCapsule & fruit,
  bool execute)
{
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis, fruit);
  logApproachSplit(node_->get_logger(), entry_tip_pose, split);
  if (!split.need_lin) {
    GraspTaskResult already;
    already.success = true;
    already.reason = "already on-axis at pregrasp";
    return already;
  }
  return planToPregrasp(
    "peach_move_pregrasp", entry_tip_pose, insertion_axis, split, fruit,
    execute);
}

GraspTaskResult GraspTask::sleeveLinear(
  const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m,
  bool execute)
{
  inspect_fruit_ = false;
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
  inspect_fruit_ = false;
  auto task = makeTaskShell("peach_linear_retreat");
  task->add(
    makeLinearMove(
      "linear retreat along insertion path",
      makeCartesianSolver(config_.approach_near_velocity_scaling),
      -insertion_axis, retreat_distance_m));
  return planAndMaybeExecute(std::move(task), execute, config_.retreat_execution_gate);
}

GraspTaskResult GraspTask::planTaskOnly(
  mtc::Task * active, bool guard_approach, std::size_t guard_skip_tail,
  bool staging_guard, std::size_t max_solutions, bool cartesian_per_part)
{
  syncKeepoutCollisionObjects();
  GraspTaskResult output;
  const std::size_t solutions =
    max_solutions == 0U ? config_.max_solutions : max_solutions;
  const auto result = active->plan(solutions);
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
    // PF-4（W5-11）：FK 只做一次（全量按段切分）；逐段果实胶囊审查与全量
    // 笛卡尔/姿态审查复用同一批点列，不再对每段重复正运动学。
    const auto tcp_per_part = tcpPathsFromJointsPerPart(
      active->getRobotModel(), config_.tip_frame, approach_parts);
    std::vector<CartesianWaypoint> tcp;
    {
      std::size_t total = 0U;
      for (const auto & segment : tcp_per_part) {
        total += segment.size();
      }
      tcp.reserve(total);
      for (const auto & segment : tcp_per_part) {
        tcp.insert(tcp.end(), segment.begin(), segment.end());
      }
    }
    if (inspect_fruit_) {
      // 逐段审查（①+②层）：staging 转移首段（PTP 弧）只查工具有限圆柱×
      // 果实胶囊接触——拍照位本就在果上方，锚定起点的反爬门会把关节弧
      // 2–4 cm 自然拱高误判绕行；其后各段（轴向 LIN/直连 LIN）另查反爬
      // （TCP 的 s 不得超过本段起点 max(s,0)+2 cm，锚定果底平面），工具
      // 姿态取自 FK 四元数。套入/撤退不走本审查（任务性穿果）。
      bool fruit_allowed = true;
      std::string fruit_reason;
      for (std::size_t part = 0; part < approach_parts.size(); ++part) {
        const auto & part_points = tcp_per_part[part];
        if (part_points.size() < 2U) {
          fruit_allowed = false;
          fruit_reason = "无法 FK 果实胶囊审查";
          break;
        }
        const bool audit_climb = !(staging_guard && part == 0U);
        const auto fruit_report = inspectToolVsFruit(
          part_points, pending_fruit_, audit_climb, config_.tool_body_length_m,
          config_.tool_body_radius_m);
        RCLCPP_INFO(
          node_->get_logger(),
          "MTC 接近果实胶囊审查(seg%zu%s): allowed=%s "
          "min_clearance=%.3fm (%s)", part,
          audit_climb ? "" : " 转移",
          fruit_report.allowed ? "true" : "false",
          std::isfinite(fruit_report.min_clearance_m) ?
          fruit_report.min_clearance_m : 999.0,
          fruit_report.reason.c_str());
        if (!fruit_report.allowed) {
          fruit_allowed = false;
          fruit_reason = fruit_report.reason;
          break;
        }
      }
      if (!fruit_allowed) {
        output.reason = "MTC short-path guard rejected: " + fruit_reason;
        return output;
      }
    }
    const bool check_cartesian = staging_guard ?
      (config_.staging_max_detour_ratio > 0.0 ||
      config_.staging_max_chord_deviation_m > 0.0 ||
      config_.staging_max_recede_m > 0.0) :
      (config_.approach_max_detour_ratio > 0.0 ||
      config_.approach_max_chord_deviation_m > 0.0 ||
      config_.approach_max_recede_m > 0.0);
    if (check_cartesian) {
      const CartesianDetourLimits cart = (staging_guard && !cartesian_per_part) ?
        CartesianDetourLimits{
        config_.staging_max_detour_ratio,
        config_.staging_max_chord_deviation_m,
        config_.staging_max_recede_m} :
      CartesianDetourLimits{
        config_.approach_max_detour_ratio,
        config_.approach_max_chord_deviation_m,
        config_.approach_max_recede_m};
      auto reject_cart = [&](const CartesianDetourReport & cart_report) {
          RCLCPP_INFO(
          node_->get_logger(),
          "MTC 接近笛卡尔审查%s: allowed=%s path=%.3fm chord=%.3fm "
          "ratio=%.2f max_dev=%.3fm recede=%.3fm (%s)",
          cartesian_per_part ? "(逐段)" : "",
          cart_report.allowed ? "true" : "false", cart_report.path_m,
          cart_report.chord_m, cart_report.detour_ratio, cart_report.max_dev_m,
          cart_report.max_recede_m, cart_report.reason.c_str());
          return !cart_report.allowed;
        };
      if (cartesian_per_part) {
        for (std::size_t part = 0; part < tcp_per_part.size(); ++part) {
          if (tcp_per_part[part].size() < 2U) {
            output.reason = "MTC short-path guard rejected: 无法 FK 笛卡尔审查";
            return output;
          }
          const auto cart_report = inspectCartesianDetour(tcp_per_part[part], cart);
          if (reject_cart(cart_report)) {
            output.reason = "MTC short-path guard rejected: " + cart_report.reason;
            return output;
          }
        }
      } else {
        if (tcp.size() < 2U) {
          output.reason = "MTC short-path guard rejected: 无法 FK 笛卡尔审查";
          return output;
        }
        // staging 转移级专用笛卡尔门（关节行程门不变）：转移 PTP 是关节空间
        // 弧，偏离/回退天然大于直连 LIN；上限仍须拒绝 1740 无约束 PTP。
        const auto cart_report = inspectCartesianDetour(tcp, cart);
        if (reject_cart(cart_report)) {
          output.reason = "MTC short-path guard rejected: " + cart_report.reason;
          return output;
        }
      }
    }
    if (config_.approach_max_tcp_rotation_deg > 0.0) {
      if (tcp.size() < 2U) {
        output.reason = "MTC short-path guard rejected: 无法 FK 姿态审查";
        return output;
      }
      const auto ori_report = inspectTcpOrientationTravel(
        tcp, config_.approach_max_tcp_rotation_deg,
        config_.approach_tcp_rotation_slack_deg);
      RCLCPP_INFO(
        node_->get_logger(),
        "MTC 接近姿态审查: allowed=%s max_from_start=%.1fdeg (%s)",
        ori_report.allowed ? "true" : "false",
        ori_report.max_from_start_deg, ori_report.reason.c_str());
      if (!ori_report.allowed) {
        output.reason = "MTC short-path guard rejected: " + ori_report.reason;
        return output;
      }
    }
    planned_approach_parts_ = approach_parts;
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
  // MTC Task::execute 同步阻塞且不暴露 stop：move_group TEM 异常时永久堵
  // 死动作通道。有界守卫先到先收；超时经 execution_stop（节点侧 MGI::stop）
  // 兜底，弃等线程移交 retiring_。闭包只持任务裸指针——弃等时由调用方
  // （planAndMaybeExecute）把 active_task_ release() 泄漏给线程，正常收口
  // 则线程已 join、reset 安全（审查 P0-1 修复）。
  retiring_->reap();
  auto task_raw = active;
  auto stop_fn = config_.execution_stop;
  auto timeout_s = config_.execute_timeout_s;
  auto logger = node_->get_logger();
  bool abandoned = false;
  const auto execute_result = runBoundedExecute(
    [task_raw] {return task_raw->execute(*task_raw->solutions().front());},
    stop_fn, nullptr, timeout_s, logger, *retiring_, &abandoned);
  if (abandoned) {
    task_abandoned_ = true;
    output.reason = "MTC execution timeout (stop issued; task handed off)";
    return output;
  }
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
  bool staging_guard,
  bool cartesian_per_part)
{
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  try {
    if (guard_approach) {
      planned_approach_parts_.clear();
    }
    output = planTaskOnly(
      active, guard_approach, guard_skip_tail, staging_guard,
      execute ? 1U : 0U, cartesian_per_part);
    if (output.success && execute) {
      output = executeSolution(active, execution_gate);
    }
  } catch (const std::exception & error) {
    output.reason = error.what();
  }
  if (guard_approach) {
    if (output.success && execute && output.execution_started) {
      last_approach_parts_ = planned_approach_parts_;
    } else if (execute && output.execution_started) {
      last_approach_parts_.clear();
    }
  }
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (task_abandoned_) {
      // 弃等线程仍在执行该任务：release() 移交所有权（进程生命周期泄漏
      // 一例换通道可用），不得 reset 双删（审查 P0-1）。
      static_cast<void>(active_task_.release());
      task_abandoned_ = false;
    } else {
      active_task_.reset();
    }
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

trajectory_msgs::msg::JointTrajectory GraspTask::lastApproachTrajectory() const
{
  return concatJointTrajectories(last_approach_parts_);
}

}  // namespace peach_arm
