// 功能：两段接近。直接转移至视觉预抓取后方净空点，再一条沿轴套入。
// 视觉几何不改；预验证完整套入/撤退；返程倒放实际成功的转移轨迹。
#include "peach_arm/grasp_task.hpp"
#include "execution_guard.hpp"
#include "joint_line_ptp.hpp"
#include "trajectory_audit.hpp"
#include "peach_arm/acm_policy.hpp"
#include "peach_arm/grasp_geometry.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <cmath>
#include <exception>
#include <memory>
#include <moveit/utils/moveit_error_code.hpp>
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
    case ApproachSplit::Kind::STAGING:
      return "PTP-clearance";
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
  return split;
}

ApproachSplit classifyApproach(
  const GraspTaskConfig & config, const Eigen::Isometry3d & entry,
  const Eigen::Vector3d & insertion_axis, const FruitCapsule & fruit)
{
  ApproachSplit out;
  out.kind = ApproachSplit::Kind::BLOCKED;
  const auto geometry = trajectoryApproachGeometry(
    entry, insertion_axis, config.approach_along_axis_m, config.approach_clearance_m);
  if (!geometry) {
    out.blocked_reason = "轨迹净空几何无效";
    return out;
  }
  out.lin_to_entry_m = geometry->sleeve_distance_m;
  if (!config.lookup_current_tip) {return out;}
  const auto start = config.lookup_current_tip();
  if (!start || !start->matrix().allFinite()) {return out;}
  out.current_tip = start;
  const Eigen::Vector3d axis = insertion_axis.normalized();
  const Eigen::Vector3d delta = entry.translation() - start->translation();
  out.axial_m = delta.dot(axis);
  out.lateral_m = (delta - out.axial_m * axis).norm();
  out.align_deg = angleBetweenDeg(start->linear().col(2), axis);
  out.need_align = out.align_deg > 2.0;
  const double precision = std::isfinite(config.cartesian_precision_m) &&
    config.cartesian_precision_m > 0.0 ? config.cartesian_precision_m : 0.001;
  const double chord = (start->translation() - geometry->transfer.translation()).norm();
  // SKIP 只认同一净空停点，不把视觉预抓取/入口等轴上其它位置当作已到达。
  out.need_lin = chord > precision || out.need_align;
  if (!out.need_lin) {out.kind = ApproachSplit::Kind::SKIP; return out;}
  const Eigen::Vector3d radial = start->translation() - entry.translation();
  out.radius_m = radial.norm();
  if (out.radius_m > 1.0e-6) {out.sweep_deg = angleBetweenDeg(radial, -axis);}
  auto target = geometry->transfer;
  target.linear() = alignFrameZ(start->linear(), axis);
  double start_s = 0.0, start_r = 0.0;
  axialRadial(start->translation(), fruit, start_s, start_r);
  const bool lin_eligible = !out.need_align && start_s <= 0.0 &&
    chord <= config.approach_cartesian_max_distance_m &&
    !toolSweepHitsFruit(*start, target, fruit, config.tool_body_length_m,
        config.tool_body_radius_m);
  out.kind = lin_eligible ? ApproachSplit::Kind::LIN : ApproachSplit::Kind::STAGING;
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

// MTC 独占一套模型/IK loader，避免与 MGI 的可变 KDL solver 并发共用。
// Task 获得指向 model 的别名 shared_ptr；即使执行弃等后 Task 被移交，
// loader 仍由同一控制块保活。model 先于 loader 析构。
struct GraspTask::RobotModelResources
{
  explicit RobotModelResources(const rclcpp::Node::SharedPtr & node)
  : loader(node), model(loader.getModel()) {}

  robot_model_loader::RobotModelLoader loader;
  moveit::core::RobotModelConstPtr model;
};

GraspTask::GraspTask(rclcpp::Node::SharedPtr node, GraspTaskConfig config)
: node_(std::move(node)), config_(std::move(config)),
  model_resources_(std::make_shared<RobotModelResources>(node_)),
  retiring_(std::make_unique<RetireBucket>())
{
  contact_logger_ = config_.contact_logger;
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

void GraspTask::setTargetContext(const std::string & target_id)
{
  target_context_ = target_id;
}

std::string GraspTask::tctx() const
{
  return target_context_.empty() ?
         std::string() :
         "[" + target_context_ + "] ";
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
  rclcpp::Parameter plugins;
  if (!node_ || config_.free_space_pipeline != "pilz_industrial_motion_planner" ||
    !node_->get_parameter(config_.free_space_pipeline + ".planning_plugins", plugins) ||
    plugins.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY ||
    plugins.as_string_array() !=
    std::vector<std::string>{"pilz_industrial_motion_planner/CommandPlanner"})
  {
    return makePilzSolver("PTP");
  }
  // 新增相位点也先经过PTP原几何门。验证失败保留原时间参数，整条MTC
  // 仍按原流程再次核几何/时长、授权及取消；不省略候选或返程段。
  auto gate = [config = config_, fruit = pending_fruit_, inspect_fruit = inspect_fruit_](
    const robot_trajectory::RobotTrajectory & path) {
      moveit_msgs::msg::RobotTrajectory msg;
      path.getRobotTrajectoryMsg(msg);
      std::string reason;
      if (!fitsExecutionTimeout(plannedDuration(msg), config.execute_timeout_s, reason) ||
        !inspectApproachTrajectory(msg.joint_trajectory, {
        config.approach_max_duration_s, config.approach_max_total_joint_travel_rad,
        config.approach_max_single_joint_travel_rad}).allowed) {return false;}
      const auto parts = tcpPathsFromJointsPerPart(
        path.getRobotModel(), config.tip_frame, {msg.joint_trajectory});
      if (parts.size() != 1 || parts.front().size() < 2) {return false;}
      const auto & tcp = parts.front();
      return (!inspect_fruit || inspectToolVsFruit(
        tcp, fruit, false, config.tool_body_length_m, config.tool_body_radius_m).allowed) &&
             inspectCartesianDetour(tcp, {
        config.staging_max_detour_ratio, config.staging_max_chord_deviation_m,
        config.staging_max_recede_m}).allowed &&
             inspectTcpOrientationTravel(tcp, config.approach_max_tcp_rotation_deg,
        config.approach_tcp_rotation_slack_deg).allowed;
    };
  auto solver = std::make_shared<JointLinePtpPlanner>(
    node_, config_.free_space_pipeline, std::move(gate));
  solver->setMaxVelocityScalingFactor(config_.velocity_scaling);
  solver->setMaxAccelerationScalingFactor(std::min(1.0, config_.velocity_scaling * 2.0));
  return solver;
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

std::unique_ptr<mtc::stages::MoveTo> GraspTask::makeCartesianMoveToPoint(
  const std::string & label, const Eigen::Vector3d & target) const
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(
    label, makeCartesianSolver(config_.approach_near_velocity_scaling));
  stage->setGroup(config_.planning_group);
  stage->setIKFrame(config_.tip_frame);
  stage->setTimeout(config_.planning_time_s);
  // PointStamped 保留该段实际起点姿态，整条轴向路径不新增刀口旋转。
  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = config_.base_frame;
  point.point.x = target.x();
  point.point.y = target.y();
  point.point.z = target.z();
  stage->setGoal(point);
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
//   - 整图：场景障碍豁免条目（acm_policy::obstacleExemptionEntries，
//     2026-09-29 避障只为保护相机：豁免清单×{<octomap>}∪obstacle_object_ids，
//     guard 开关控制相机连杆是否受查）；
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
  const ObstacleExemptionSpec & obstacle_spec,
  const std::vector<std::string> & contact_tool_links,
  const std::string & per_target_id,
  ContactAcmStage per_target_stage,
  const std::function<bool()> & still_current)
{
  if (still_current && !still_current()) {return;}
  static std::atomic<int> fetch_seq{0};
  const std::string helper_name =
    "peach_octomap_acm_" + std::to_string(fetch_seq.fetch_add(1));
  auto helper = std::make_shared<rclcpp::Node>(helper_name);
  auto client = helper->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      logger, "get_planning_scene 不可用，跳过场景障碍 ACM 豁免");
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
      logger, "读取现行 ACM 超时，跳过场景障碍豁免（避免整表替换）");
    return;
  }
  const auto response = future.get();
  if (!response || response->scene.allowed_collision_matrix.entry_names.empty()) {
    RCLCPP_WARN(
      logger, "现行 ACM 为空，跳过场景障碍豁免（避免冲掉 SRDF）");
    return;
  }
  collision_detection::AllowedCollisionMatrix acm(
    response->scene.allowed_collision_matrix);
  if (allowToolVersusWholeOctomap()) {
    for (const auto & [link, object] : obstacleExemptionEntries(obstacle_spec)) {
      acm.setEntry(link, object, true);
    }
    // guard 从 false 重开时，现行 ACM 已有相机豁免；必须显式写回 false。
    // 只写本包管理的 guard_links × 障碍域，不修改 SRDF 或其他对象的条目。
    auto guard_spec = obstacle_spec;
    guard_spec.exempt_links = obstacle_spec.guard_links;
    guard_spec.guard_links.clear();
    for (const auto & [link, object] : obstacleExemptionEntries(guard_spec)) {
      acm.setEntry(link, object, !obstacle_spec.guard_enabled);
    }
  }
  // 指定目标 × 接触连杆显式写入当前阶段策略，离开接触阶段撤销旧 true。
  if (!per_target_id.empty()) {
    for (const auto & link : contact_tool_links) {
      acm.setEntry(
        link, per_target_id, acmAllows(per_target_id, link, per_target_stage, contact_tool_links));
    }
  }
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  acm.getMessage(diff.allowed_collision_matrix);
  // 服务等待期间可能已改参/停用；旧后台轮次写前再次复检。
  // 这只能阻止尚未派发的请求，不能撤回已发送的 DDS 服务调用。
  if (still_current && !still_current()) {return;}
  const bool applied = scene.applyPlanningScene(diff);
  // 09-29 真相机轮勘定：成功必须留痕——此前静默成功/静默失败都无日志，
  // 起点碰撞排查时无法区分「条目没写」与「写了被覆盖」。
  const size_t obstacle_entries = allowToolVersusWholeOctomap() ?
    obstacleExemptionEntries(obstacle_spec).size() : 0U;
  if (applied) {
    RCLCPP_INFO(
      logger, "场景障碍 ACM 豁免已应用: %zu 条（含 <octomap> 保留名）",
      obstacle_entries);
  } else {
    RCLCPP_WARN(
      logger, "场景障碍 ACM 豁免写入失败（applyPlanningScene 返回 false）");
  }
}

void GraspTask::applyWholeOctomapToolExemption(
  const rclcpp::Logger & logger,
  moveit::planning_interface::PlanningSceneInterface & scene,
  const std::vector<std::string> & tool_links,
  const ObstacleExemptionSpec & obstacle_spec,
  const std::function<bool()> & still_current)
{
  ObstacleExemptionSpec spec = obstacle_spec;
  spec.exempt_links = tool_links;
  applyOctomapExemptionImpl(
    logger, scene, tool_links, spec, {}, std::string(), ContactAcmStage::Transit, still_current);
}

void GraspTask::applyToolOctomapExemption(
  moveit::planning_interface::PlanningSceneInterface & scene,
  OctomapExemptionPolicy policy) const
{
  applyOctomapExemptionImpl(
    node_ ? node_->get_logger() : rclcpp::get_logger("peach_arm"), scene,
    config_.tool_links,
    ObstacleExemptionSpec{
      config_.tool_links, config_.obstacle_guard_links,
      config_.obstacle_object_ids, config_.obstacle_guard_enabled},
    config_.contact_tool_links,
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
  // INVALID_MOTION_PLAN。未齐由直接 PTP 完成姿态变化。
  if (gate_orientation) {
    stage->setPathConstraints(makeOrientationGate(
      config_.tip_frame, config_.base_frame, target_tip_pose,
      config_.approach_max_align_deg, "entry_orientation_gate"));
  }
  sequence.add(std::move(stage));
}

void GraspTask::appendApproachToPregrasp(
  mtc::SerialContainer & sequence, const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis, const ApproachSplit & split) const
{
  if (split.kind != ApproachSplit::Kind::LIN || !split.current_tip) {return;}
  const auto geometry = trajectoryApproachGeometry(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m, config_.approach_clearance_m);
  if (!geometry) {throw std::invalid_argument("invalid approach clearance geometry");}
  auto target = geometry->transfer;
  target.linear() = alignFrameZRolled(split.current_tip->linear(), insertion_axis,
      split.tool_roll_rad);
  appendLinToPose(sequence, target, "direct lin to trajectory clearance", true);
}

std::unique_ptr<mtc::Task> GraspTask::makeTaskShell(const std::string & task_name) const
{
  // 必须从构造时关闭。先默认启动 introspection 再关闭，会在它的 executor
  // 尚未开始 spin 时 cancel + join；Jazzy spin 会重新置 true，可能堵至 shutdown。
  // 被护栏拒掉的滚转候选也不应发成执行轨迹；可视化走实际 TCP Path/RobotState。
  auto task = std::make_unique<mtc::Task>(task_name, false);
  task->setRobotModel(moveit::core::RobotModelConstPtr(
    model_resources_, model_resources_->model.get()));
  task->setProperty("group", config_.planning_group);
  task->add(std::make_unique<mtc::stages::CurrentState>("current robot state"));
  if (!planning_start_override_.empty()) {
    auto start = std::make_unique<mtc::stages::ModifyPlanningScene>("preview named start only");
    start->setCallback([joints = planning_start_override_](const auto & scene, const auto &) {
        scene->getCurrentStateNonConst().setVariablePositions(joints);
        scene->getCurrentStateNonConst().update();
    });
    task->add(std::move(start));
  }
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

std::unique_ptr<mtc::Task> GraspTask::makePtpTransferTask(
  const std::string & task_name, const std::map<std::string, double> & joints)
{
  auto task = makeTaskShell(task_name);
  auto ptp = std::make_unique<mtc::stages::MoveTo>("direct ptp to trajectory clearance",
      makePtpSolver());
  ptp->setGroup(config_.planning_group);
  ptp->setIKFrame(config_.tip_frame);
  ptp->setTimeout(config_.planning_time_s);
  ptp->setGoal(joints);
  task->add(std::move(ptp));
  return task;
}

GraspTaskResult GraspTask::preflightContact(
  std::unique_ptr<mtc::Task> task, const TrajectoryApproachGeometry & geometry,
  bool has_transfer, bool ptp_transfer, bool include_retreat)
{
  auto contact = std::make_unique<mtc::SerialContainer>("preflight full sleeve and retreat");
  // 只改规划任务的局部 scene，按实际 Sleeve/Retreat 阶段切换同一目标 ACM。
  // 绝不为了预验证把转移阶段的全局 ACM 改成允许接触。
  if (!pending_acm_target_id_.empty()) {
    auto sleeve_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("preview sleeve ACM");
    sleeve_acm->allowCollisions(pending_acm_target_id_, config_.contact_tool_links, true);
    contact->add(std::move(sleeve_acm));
  }
  contact->add(makeCartesianMoveToPoint("single axial sleeve", geometry.sleeve_end.translation()));
  if (include_retreat) {
    if (!pending_acm_target_id_.empty()) {
      auto retreat_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("preview retreat ACM");
      retreat_acm->allowCollisions(pending_acm_target_id_, config_.contact_tool_links, false);
      contact->add(std::move(retreat_acm));
    }
    contact->add(makeCartesianMoveToPoint("single axial retreat", geometry.transfer.translation()));
  }
  task->add(std::move(contact));
  std::vector<Eigen::Vector3d> endpoints;
  if (has_transfer) {endpoints.push_back(geometry.transfer.translation());}
  endpoints.push_back(geometry.sleeve_end.translation());
  if (include_retreat) {endpoints.push_back(geometry.transfer.translation());}
  return planAndMaybeExecute(
    std::move(task), false, {}, has_transfer, include_retreat ? 2U : 1U,
    ptp_transfer, false, true, endpoints);
}

GraspTaskResult GraspTask::planToPregrasp(
  const std::string & task_name,
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const ApproachSplit & split,
  const FruitCapsule & fruit,
  bool execute,
  double insert_travel_m, bool contact_preview, bool include_retreat)
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
  const bool direct_lin = split.kind == ApproachSplit::Kind::LIN;
  bool exceeds_lin_budget = false;
  // 仅实际Pilz LIN的执行模式可用平移限速证明整个直连goal不可能过门。
  // 各滚转共享同一位置弦；不必先做套入IK、加载管线并规划后才拒绝超时。
  // 缺参数、不同插件/原语或无效输入保留原搜索；完整接触预检同样检查预算。
  // MoveTo对URDF link使用该link的原点约束；附着物/子帧则可能折算到
  // 带固定偏置的母link，转动会改变两者弦长，不能直接使用tip平移下界。
  if (direct_lin && (execute || contact_preview) && node_ && split.current_tip &&
    model_resources_ && model_resources_->model &&
    model_resources_->model->hasLinkModel(config_.tip_frame) &&
    config_.free_space_pipeline == "pilz_industrial_motion_planner" &&
    config_.free_space_planner == "LIN" &&
    std::isfinite(config_.execute_timeout_s) && config_.execute_timeout_s > 0.0 &&
    std::isfinite(config_.approach_along_axis_m) && config_.approach_along_axis_m >= 0.0 &&
    entry_tip_pose.matrix().allFinite() && split.current_tip->matrix().allFinite() &&
    insertion_axis.allFinite() && insertion_axis.norm() > 1.0e-9)
  {
    rclcpp::Parameter plugins, velocity;
    if (node_->get_parameter(config_.free_space_pipeline + ".planning_plugins", plugins) &&
      plugins.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY &&
      plugins.as_string_array() ==
      std::vector<std::string>{"pilz_industrial_motion_planner/CommandPlanner"} &&
      node_->get_parameter("robot_description_planning.cartesian_limits.max_trans_vel", velocity) &&
      velocity.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      const auto geometry = trajectoryApproachGeometry(
        entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
          config_.approach_clearance_m);
      const double minimum_s = geometry ? cartesianTranslationLowerBound(
        (geometry->transfer.translation() - split.current_tip->translation()).norm(),
        velocity.as_double(), config_.velocity_scaling) : 0.0;
      exceeds_lin_budget = minimum_s > config_.execute_timeout_s;
      if (exceeds_lin_budget) {
        RCLCPP_INFO(
          contactLogger(), "%s跳过直连LIN：仅平移必要时间下界 %.3fs > 执行预算 %.3fs",
          tctx().c_str(), minimum_s, config_.execute_timeout_s);
      }
    }
  }
  // 几何与预算允许已齐直连时先验证完整 LIN。许可拒绝、取消或
  // 已经运动仍由原候选执行路径收口，不得因规划失败继续换路径。
  if (direct_lin && !exceeds_lin_budget) {
    if (tryRolledApproach(
        task_name, entry_tip_pose, insertion_axis, split, split.kind, execute,
        last, insert_travel_m, contact_preview, include_retreat))
    {
      inspect_fruit_ = false;
      return last;
    }
  }
  if (!last.execution_started && !last.retry_forbidden) {
    tryStagingTransit(
      task_name, entry_tip_pose, insertion_axis, split, execute, last,
      insert_travel_m, contact_preview, include_retreat);
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
  GraspTaskResult & last,
  double insert_travel_m, bool contact_preview, bool include_retreat)
{
  split.kind = kind;
  const auto generation = cancel_generation_.load();
  int consecutive_lin_plan_fails = 0;
  for (const double roll : toolRollsRad()) {
    if (!rclcpp::ok() || cancel_generation_.load() != generation) {
      last.retry_forbidden = true;
      last.reason = "approach candidate search canceled";
      return true;
    }
    if (insert_travel_m > 1.0e-6 && planning_start_override_.empty() &&
      config_.sleeve_roll_feasible &&
      !config_.sleeve_roll_feasible(roll, entry_tip_pose, insert_travel_m))
    {
      RCLCPP_INFO(
        contactLogger(),
        "%s跳过接近滚转 %.0f°：该档套入直线未走满",
        tctx().c_str(), roll * 180.0 / static_cast<double>(EIGEN_PI));
      continue;
    }
    ApproachSplit rolled = withToolRoll(split, roll);
    const double roll_deg = roll * 180.0 / static_cast<double>(EIGEN_PI);
    rolled.kind = kind;
    const auto started = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      node_->get_logger(), "%s接近候选开始: kind=%s roll=%.0fdeg",
      tctx().c_str(), approachKindName(rolled.kind), roll_deg);
    if (insert_travel_m > 1.0e-6 && (execute || contact_preview)) {
      const auto geometry = trajectoryApproachGeometry(
        entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
        config_.approach_clearance_m, insert_travel_m);
      if (!geometry) {last.reason = "invalid contact geometry"; return false;}
      auto check = preflightContact(
        makeApproachOnlyTask(task_name + "_preflight", entry_tip_pose, insertion_axis, rolled),
        *geometry, true, false, include_retreat);
      if (cancel_generation_.load() != generation) {
        check.success = false; check.retry_forbidden = true;
      }
      if (!check.success) {
        last = check;
        if (check.retry_forbidden) {return true;}
        continue;
      }
      if (contact_preview) {last = check; return true;}
    }
    const auto transfer_geometry = trajectoryApproachGeometry(
      entry_tip_pose, insertion_axis, config_.approach_along_axis_m, config_.approach_clearance_m);
    if (!transfer_geometry) {last.reason = "invalid clearance geometry"; return false;}
    auto task = makeApproachOnlyTask(task_name, entry_tip_pose, insertion_axis, rolled);
    RCLCPP_INFO(
      node_->get_logger(), "%s接近候选任务已建立: elapsed=%.3fs",
      tctx().c_str(), std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count());
    auto result = planAndMaybeExecute(
      std::move(task),
      execute, config_.approach_execution_gate, true, 0U, false, false, false,
      {transfer_geometry->transfer.translation()});
    RCLCPP_INFO(
      node_->get_logger(), "%s接近候选结束: roll=%.0fdeg elapsed=%.3fs success=%s reason=%s",
      tctx().c_str(), roll_deg,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
      result.success ? "true" : "false", result.reason.c_str());
    if (result.success || result.execution_started || result.retry_forbidden) {
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
    ++consecutive_lin_plan_fails;
    if (abandonRolledLinSearch(consecutive_lin_plan_fails)) {
      RCLCPP_WARN(
        contactLogger(),
        "%s笛卡尔 LIN 连续 %d 档规划失败（最近: %s），停止滚转搜索，改走 PTP",
        tctx().c_str(), consecutive_lin_plan_fails, result.reason.c_str());
      break;
    }
  }
  return false;
}

bool GraspTask::tryStagingTransit(
  const std::string & task_name, const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis, const ApproachSplit & split,
  bool execute, GraspTaskResult & last, double insert_travel_m, bool contact_preview,
  bool include_retreat)
{
  if (!split.current_tip || !config_.staging_ik) {
    last.reason = "PTP 净空转移不可用：无当前 TCP 或 IK 钩子";
    return false;
  }
  const auto geometry = trajectoryApproachGeometry(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
    config_.approach_clearance_m, insert_travel_m);
  if (!geometry) {last.reason = "轨迹净空几何无效"; return false;}
  auto probe_pose = geometry->transfer;
  probe_pose.linear() = alignFrameZ(split.current_tip->linear(), insertion_axis);
  const std::vector<Eigen::Isometry3d> axial_goals{
    insert_travel_m > 1.0e-6 ? geometry->sleeve_end : geometry->transfer};
  const auto generation = cancel_generation_.load();
  for (const auto & roll_group : groupStagingRolls(toolRollsRad())) {
    if (!rclcpp::ok() || cancel_generation_.load() != generation) {
      last.retry_forbidden = true; last.reason = "clearance candidate search canceled";
      return false;
    }
    const auto candidates = config_.staging_ik(probe_pose, axial_goals, roll_group,
        planning_start_override_);
    for (const auto & candidate : candidates) {
      if (!rclcpp::ok() || cancel_generation_.load() != generation) {
        last.retry_forbidden = true; last.reason = "clearance candidate search canceled";
        return false;
      }
      RCLCPP_INFO(contactLogger(),
        "%s直接PTP净空候选 roll=%.0fdeg clearance=%.3fm joint_cost=%.3f",
        tctx().c_str(), candidate.roll_rad * 180.0 / static_cast<double>(EIGEN_PI),
        config_.approach_clearance_m, candidate.joint_cost);
      if (insert_travel_m > 1.0e-6 && (execute || contact_preview)) {
        auto check = preflightContact(makePtpTransferTask(task_name + "_preflight",
            candidate.joints),
          *geometry, true, true, include_retreat);
        if (cancel_generation_.load() != generation) {
          check.success = false; check.retry_forbidden = true;
        }
        if (!check.success) {
          last = check;
          if (check.retry_forbidden) {return false;}
          continue;
        }
        if (contact_preview) {last = check; return true;}
      }
      auto result = planAndMaybeExecute(
        makePtpTransferTask(task_name + "_ptp", candidate.joints), execute,
        config_.approach_execution_gate, true, 0U, true, false, false,
        {geometry->transfer.translation()});
      last = result;
      if (!retryStagingCandidate(result.success, result.execution_started,
          cancel_generation_.load() != generation, result.retry_forbidden))
      {
        return result.success || result.execution_started || result.retry_forbidden;
      }
    }
  }
  if (last.reason.empty()) {last.reason = "直接转移到轨迹净空点无合法解";}
  return false;
}

// 只规划（PREVIEW / preview Trigger 专用）：接近分档后一次装配
// 「到预抓取 + 沿轴插入」的 plan-only 预览。执行路径已删——生产周期走
// moveToPregrasp（阶段执行器）+ previewFullContact 预验证 + sleeveLinear。
GraspTaskResult GraspTask::approachAndInsert(
  const Eigen::Isometry3d & entry_tip_pose, const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m, const FruitCapsule & fruit)
{
  return previewFullContact(entry_tip_pose, insertion_axis, insertion_distance_m, fruit, true,
      false);
}

GraspTaskResult GraspTask::previewFullContact(
  const Eigen::Isometry3d & entry_tip_pose, const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m, const FruitCapsule & fruit, bool /* prepare_execution */,
  bool include_retreat, const std::string & start_named)
{
  struct RestorePlanningStart
  {
    std::map<std::string, double> & current;
    std::map<std::string, double> saved;
    ~RestorePlanningStart() {current = std::move(saved);}
  } restore{planning_start_override_, planning_start_override_};
  planning_start_override_.clear();
  auto classify_config = config_;
  if (!start_named.empty()) {
    const auto model = model_resources_->model;
    const auto * group = model->getJointModelGroup(config_.planning_group);
    moveit::core::RobotState start(model);
    start.setToDefaultValues();
    if (group == nullptr || !start.setToDefaultValues(group, start_named)) {
      GraspTaskResult rejected; rejected.reason = "预验证拍照命名起点无效"; return rejected;
    }
    start.update();
    for (const auto & name : group->getVariableNames()) {
      planning_start_override_[name] = start.getVariablePosition(name);
    }
    const auto tip = start.getGlobalLinkTransform(config_.tip_frame);
    classify_config.lookup_current_tip = [tip]() {return std::optional<Eigen::Isometry3d>(tip);};
  }
  const auto geometry = trajectoryApproachGeometry(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
    config_.approach_clearance_m, insertion_distance_m);
  const auto split = classifyApproach(classify_config, entry_tip_pose, insertion_axis, fruit);
  if (!geometry || insertion_distance_m <= 0.0 || split.kind == ApproachSplit::Kind::BLOCKED) {
    GraspTaskResult result; result.reason = "接触预验证缺少有效净空几何/行程/当前TCP"; return result;
  }
  if (split.need_lin) {
    return planToPregrasp("peach_full_contact_preview", entry_tip_pose, insertion_axis,
      split, fruit, false, insertion_distance_m, true, include_retreat);
  }
  inspect_fruit_ = false;
  return preflightContact(makeTaskShell("peach_axial_contact_preview"),
    *geometry, false, false, include_retreat);
}

GraspTaskResult GraspTask::moveToPregrasp(
  const Eigen::Isometry3d & entry_tip_pose,
  const Eigen::Vector3d & insertion_axis,
  const FruitCapsule & fruit,
  bool execute,
  double insert_travel_m)
{
  if (execute) {
    // 一次新的接近拥有独立执行历史；被拒候选/上个目标不能成为本次返程。
    last_approach_parts_.clear();
  }
  const ApproachSplit split =
    classifyApproach(config_, entry_tip_pose, insertion_axis, fruit);
  logApproachSplit(node_->get_logger(), entry_tip_pose, split);
  if (!split.need_lin) {
    GraspTaskResult already;
    already.success = true;
    already.reason = "already at trajectory clearance";
    return already;
  }
  return planToPregrasp(
    "peach_move_pregrasp", entry_tip_pose, insertion_axis, split, fruit,
    execute, insert_travel_m);
}

GraspTaskResult GraspTask::sleeveLinear(
  const Eigen::Isometry3d & entry_tip_pose, const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m, bool execute,
  const std::function<bool(std::string &)> & execution_gate)
{
  inspect_fruit_ = false;
  const auto geometry = trajectoryApproachGeometry(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
    config_.approach_clearance_m, insertion_distance_m);
  const auto current = config_.lookup_current_tip ? config_.lookup_current_tip() : std::nullopt;
  if (!geometry || !current || !current->matrix().allFinite() ||
    (current->translation() - geometry->transfer.translation()).norm() >
    config_.cartesian_precision_m)
  {
    GraspTaskResult rejected;
    rejected.reason = "套入起点未在轨迹净空点 A 的笛卡尔精度内";
    return rejected;
  }
  auto task = makeTaskShell("peach_sleeve_linear");
  task->add(makeCartesianMoveToPoint("single axial sleeve", geometry->sleeve_end.translation()));
  return planAndMaybeExecute(
    std::move(task), execute, execution_gate, true, 0U, false, false, false,
    {geometry->sleeve_end.translation()});
}

GraspTaskResult GraspTask::retreat(
  const Eigen::Isometry3d & entry_tip_pose, const Eigen::Vector3d & insertion_axis,
  double insertion_distance_m, bool execute)
{
  inspect_fruit_ = false;
  const auto geometry = trajectoryApproachGeometry(
    entry_tip_pose, insertion_axis, config_.approach_along_axis_m,
    config_.approach_clearance_m, insertion_distance_m);
  if (!geometry) {
    GraspTaskResult rejected; rejected.reason = "撤退净空几何无效"; return rejected;
  }
  auto task = makeTaskShell("peach_linear_retreat");
  task->add(makeCartesianMoveToPoint("single axial retreat", geometry->transfer.translation()));
  return planAndMaybeExecute(std::move(task), execute, config_.retreat_execution_gate,
    false, 0U, false, false, false, {geometry->transfer.translation()});
}

GraspTaskResult GraspTask::planTaskOnly(
  mtc::Task * active, bool guard_approach, std::size_t guard_skip_tail,
  bool staging_guard, TimingCheck timing, bool cartesian_per_part,
  const std::vector<Eigen::Vector3d> & endpoint_targets)
{
  syncKeepoutCollisionObjects();
  GraspTaskResult output;
  const std::size_t solutions = timing == TimingCheck::WHOLE_GOAL ? 1U : config_.max_solutions;
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
  moveit_task_constructor_msgs::msg::Solution solution;
  if (guard_approach || timing != TimingCheck::NONE || !endpoint_targets.empty()) {
    active->solutions().front()->toMsg(solution);
  }
  if (!endpoint_targets.empty()) {
    std::vector<const trajectory_msgs::msg::JointTrajectory *> paths;
    for (const auto & sub : solution.sub_trajectory) {
      if (!sub.trajectory.joint_trajectory.points.empty()) {
        paths.push_back(&sub.trajectory.joint_trajectory);
      }
    }
    if (paths.size() != endpoint_targets.size()) {
      output.reason = "轨迹终点验证段数不符";
      return output;
    }
    for (std::size_t i = 0; i < paths.size(); ++i) {
      moveit::core::RobotState end(active->getRobotModel());
      end.setToDefaultValues();
      end.setVariablePositions(paths[i]->joint_names, paths[i]->points.back().positions);
      end.update();
      const auto & tip = end.getGlobalLinkTransform(config_.tip_frame);
      if (!tip.matrix().allFinite() ||
        (tip.translation() - endpoint_targets[i]).norm() > config_.cartesian_precision_m)
      {
        output.reason = "轨迹未到达预期净空/套入终点（拒绝部分笛卡尔路径）";
        return output;
      }
    }
  }
  if (timing != TimingCheck::NONE) {
    // MTC 按 sub_trajectory 顺序执行，场景差分项没有运动段。
    std::chrono::nanoseconds total{}, last{};
    std::size_t moves = 0;
    for (const auto & sub : solution.sub_trajectory) {
      const auto & traj = sub.trajectory;
      if (traj.joint_trajectory.points.empty() && traj.multi_dof_joint_trajectory.points.empty()) {
        continue;
      }
      last = plannedDuration(traj);
      total += last;
      ++moves;
      if (timing == TimingCheck::CONTACT_LEGS &&
        (last.count() <= 0 || !fitsExecutionTimeout(last, config_.execute_timeout_s,
          output.reason)))
      {
        output.reason = "首次运动前分段预验证: " + output.reason;
        return output;
      }
    }
    if (timing == TimingCheck::WHOLE_GOAL) {
      if (!fitsExecutionTimeout(total, config_.execute_timeout_s, output.reason)) {return output;}
    } else if (moves == 0U) {
      output.reason = "缺少有时间参数的接触轨迹";
      return output;
    }
  }

  if (guard_approach) {
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
          if (!cart_report.allowed) {
            RCLCPP_WARN(
              contactLogger(),
              "%s拒绝大弧 PTP：ratio=%.2f max_dev=%.3fm recede=%.3fm "
              "门=%.2f/%.3fm/%.3fm，试下一 IK 候选",
              tctx().c_str(), cart_report.detour_ratio, cart_report.max_dev_m,
              cart_report.max_recede_m, cart.max_detour_ratio,
              cart.max_chord_deviation_m, cart.max_recede_m);
          }
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
    output.retry_forbidden = true;
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
  const auto audit_target = target_context_;
  bool abandoned = false;
  const auto execute_result = runBoundedExecute(
    [task_raw, audit_target] {
      const auto & solution = *task_raw->solutions().front();
      if (!trajectoryAuditEnabled()) {return task_raw->execute(solution);}
      const AuditedTaskSolution audited(
        solution, task_raw->getRobotModel(), task_raw->name(), audit_target);
      return task_raw->execute(audited);
    },
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
    RCLCPP_WARN(
      node_->get_logger(), "%sMTC 执行失败: %s", tctx().c_str(),
      moveit::core::errorCodeToString(execute_result).c_str());
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
  bool cartesian_per_part, bool prepare_contact_execution,
  const std::vector<Eigen::Vector3d> & endpoint_targets)
{
  const auto generation = cancel_generation_.load();
  mtc::Task * active = nullptr;
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_task_ = std::move(task);
    active = active_task_.get();
  }
  GraspTaskResult output;
  const auto plan_started = std::chrono::steady_clock::now();
  try {
    if (guard_approach) {
      planned_approach_parts_.clear();
    }
    output = planTaskOnly(
      active, guard_approach, guard_skip_tail, staging_guard,
      execute ? TimingCheck::WHOLE_GOAL :
      (prepare_contact_execution ? TimingCheck::CONTACT_LEGS : TimingCheck::NONE),
      cartesian_per_part, endpoint_targets);
    RCLCPP_INFO(
      node_->get_logger(), "%sMTC规划检查已返回: elapsed=%.3fs success=%s",
      tctx().c_str(), std::chrono::duration<double>(
        std::chrono::steady_clock::now() - plan_started).count(),
      output.success ? "true" : "false");
    if (cancel_generation_.load() != generation) {
      output.success = false;
      output.retry_forbidden = true;
      output.reason = "MTC planning canceled";
    } else if (output.success && execute) {
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
      const auto reset_started = std::chrono::steady_clock::now();
      RCLCPP_DEBUG(node_->get_logger(), "%sMTC任务开始释放", tctx().c_str());
      active_task_.reset();
      RCLCPP_INFO(
        node_->get_logger(), "%sMTC任务已释放: elapsed=%.3fs",
        tctx().c_str(), std::chrono::duration<double>(
          std::chrono::steady_clock::now() - reset_started).count());
    }
  }
  return output;
}

void GraspTask::cancel()
{
  cancel_generation_.fetch_add(1);
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (active_task_) {
    active_task_->preempt();
  }
}

std::vector<trajectory_msgs::msg::JointTrajectory> GraspTask::lastApproachTrajectories() const
{
  return last_approach_parts_;
}

}  // namespace peach_arm
