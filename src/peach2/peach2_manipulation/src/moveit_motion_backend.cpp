#include "moveit_motion_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "moveit/robot_state/robot_state.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "peach2_end_effector/failure_codes.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace peach2_manipulation
{

namespace failure = peach2_end_effector::failure;
using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

namespace
{

double clamp_scaling(double value)
{
  if (!std::isfinite(value)) {
    return 0.01;
  }
  return std::clamp(value, 0.001, 1.0);
}

JointTrajectory from_msg(const trajectory_msgs::msg::JointTrajectory & msg)
{
  JointTrajectory out;
  out.joint_names = msg.joint_names;
  out.points.reserve(msg.points.size());
  for (const auto & p : msg.points) {
    TrajectoryPoint point;
    point.time_from_start_s = rclcpp::Duration(p.time_from_start).seconds();
    point.positions = p.positions;
    point.velocities = p.velocities;
    point.accelerations = p.accelerations;
    out.points.push_back(std::move(point));
  }
  return out;
}

trajectory_msgs::msg::JointTrajectory to_msg(const JointTrajectory & trajectory)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = trajectory.joint_names;
  msg.points.reserve(trajectory.points.size());
  for (const auto & p : trajectory.points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = p.positions;
    point.velocities = p.velocities;
    point.accelerations = p.accelerations;
    point.time_from_start = rclcpp::Duration::from_seconds(p.time_from_start_s);
    msg.points.push_back(std::move(point));
  }
  return msg;
}

}  // namespace

uint32_t plan_failure_code(int32_t moveit_error_code, PlanKind kind)
{
  using E = moveit_msgs::msg::MoveItErrorCodes;
  switch (moveit_error_code) {
    case E::NO_IK_SOLUTION:
      return failure::PLAN_NO_IK;
    case E::GOAL_IN_COLLISION:
    case E::START_STATE_IN_COLLISION:
      return failure::PLAN_COLLISION;
    default:
      return kind == PlanKind::LINEAR ? failure::PLAN_CARTESIAN_INCOMPLETE : failure::PLAN_FAILED;
  }
}

MoveItMotionBackend::MoveItMotionBackend(rclcpp::Node::SharedPtr node, MoveItBackendConfig config)
: node_(std::move(node)), config_(std::move(config)), logger_(node_->get_logger())
{
  if (config_.cartesian_max_trans_vel_mps <= 0.0) {
    throw std::invalid_argument("cartesian_max_trans_vel_mps must be > 0");
  }
  mgi_ = std::make_shared<MoveGroupInterface>(
    node_, config_.group, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(config_.wait_for_servers_s));
  if (!mgi_->getRobotModel()) {
    throw std::runtime_error("robot model unavailable");
  }
  mgi_->setEndEffectorLink(config_.tip_link);
  mgi_->setPoseReferenceFrame(config_.base_frame);
  exec_client_ = rclcpp_action::create_client<ExecuteTrajectory>(node_, "execute_trajectory");
  validity_client_ =
    node_->create_client<moveit_msgs::srv::GetStateValidity>("check_state_validity");
  if (!exec_client_->wait_for_action_server(
      std::chrono::duration<double>(config_.wait_for_servers_s)))
  {
    throw std::runtime_error("move_group execute_trajectory action unavailable");
  }
}

MoveItMotionBackend::~MoveItMotionBackend()
{
  cancel_active_goal();
}

PlanResult MoveItMotionBackend::plan(const PlanRequest & request)
{
  std::lock_guard<std::mutex> lock(mgi_mutex_);
  PlanResult result;
  auto & group = *mgi_;
  group.clearPoseTargets();
  double velocity = request.velocity_scaling;
  if (request.kind == PlanKind::LINEAR) {
    group.setPlanningPipelineId(config_.linear_pipeline);
    group.setPlannerId(config_.linear_planner_id);
    velocity = request.linear_speed_mps / config_.cartesian_max_trans_vel_mps;
  } else {
    group.setPlanningPipelineId(config_.free_pipeline);
    group.setPlannerId(config_.free_planner_id);
  }
  group.setMaxVelocityScalingFactor(clamp_scaling(velocity));
  group.setMaxAccelerationScalingFactor(clamp_scaling(request.acceleration_scaling));
  group.setPlanningTime(config_.planning_time_s);
  group.setNumPlanningAttempts(std::max(1, config_.planning_attempts));

  if (request.start_joints) {
    auto current = group.getCurrentState(1.0);
    moveit::core::RobotState start =
      current ? *current : moveit::core::RobotState(group.getRobotModel());
    if (!current) {
      start.setToDefaultValues();
    }
    const auto * jmg = start.getJointModelGroup(config_.group);
    if (!jmg || jmg->getVariableCount() != request.start_joints->size()) {
      result.failure_code = failure::PLAN_FAILED;
      result.reason = "start_joints_size_mismatch";
      return result;
    }
    start.setJointGroupPositions(jmg, *request.start_joints);
    start.update();
    group.setStartState(start);
  } else {
    group.setStartStateToCurrentState();
  }

  if (request.kind == PlanKind::NAMED) {
    if (!group.setNamedTarget(request.named_target)) {
      result.failure_code = failure::PLAN_FAILED;
      result.reason = "unknown_named_target:" + request.named_target;
      return result;
    }
  } else {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = config_.base_frame;
    goal.pose = tf2::toMsg(request.tcp_goal);
    if (!group.setPoseTarget(goal, config_.tip_link)) {
      result.failure_code = failure::PLAN_FAILED;
      result.reason = "pose_target_rejected";
      return result;
    }
  }

  MoveGroupInterface::Plan plan;
  const auto rc = group.plan(plan);
  group.clearPoseTargets();
  if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
    result.failure_code = plan_failure_code(rc.val, request.kind);
    result.reason = request.label + ":" + moveit::core::errorCodeToString(rc);
    return result;
  }
  JointTrajectory trajectory = from_msg(plan.trajectory.joint_trajectory);
  std::optional<std::vector<double>> start = request.start_joints;
  if (!start && trajectory.points.size() < 2U) {
    // Pilz / time parameterization collapse start == goal to one point; check it against the
    // real start instead of trusting the point alone.
    if (auto current = group.getCurrentState(1.0)) {
      if (const auto * jmg = current->getJointModelGroup(config_.group)) {
        std::vector<double> q;
        current->copyJointGroupPositions(jmg, q);
        start = std::move(q);
      }
    }
  }
  return finalize_plan(
    std::move(trajectory), start, request.kind, request.label, config_.at_goal_tolerance_rad,
    request.collapse_at_goal);
}

ExecResult MoveItMotionBackend::execute(
  const JointTrajectory & trajectory, const ExecOptions & options)
{
  ExecResult result;
  if (trajectory.points.size() < 2U) {
    result.failure_code = failure::EXEC_FAILED;
    result.reason = "empty_trajectory";
    return result;
  }
  ExecuteTrajectory::Goal goal;
  goal.trajectory.joint_trajectory = to_msg(trajectory);
  goal.trajectory.joint_trajectory.header.frame_id = config_.base_frame;

  auto goal_future = exec_client_->async_send_goal(goal);
  if (goal_future.wait_for(2s) != std::future_status::ready) {
    result.failure_code = failure::EXEC_FAILED;
    result.reason = "execute_goal_no_response";
    return result;
  }
  auto handle = goal_future.get();
  if (!handle) {
    result.failure_code = failure::EXEC_FAILED;
    result.reason = "execute_goal_rejected";
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ = handle;
  }
  auto result_future = exec_client_->async_get_result(handle);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(options.timeout_s);
  std::optional<Abort> abort;
  bool timed_out = false;
  while (result_future.wait_for(20ms) != std::future_status::ready) {
    if (options.abort_probe) {
      abort = options.abort_probe();
      if (abort) {
        break;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      timed_out = true;
      break;
    }
  }
  if (abort || timed_out) {
    cancel_active_goal();
    {
      std::lock_guard<std::mutex> lock(mgi_mutex_);
      mgi_->stop();
    }
    if (result_future.wait_for(std::chrono::duration<double>(config_.stop_grace_s)) !=
      std::future_status::ready)
    {
      RCLCPP_ERROR(
        logger_, "execute_trajectory did not acknowledge the cancel within %.1f s",
        config_.stop_grace_s);
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
    if (abort) {
      result.failure_code = abort->failure_code;
      result.reason = abort->reason;
    } else {
      result.failure_code = failure::EXEC_TIMEOUT;
      result.reason = "execute_timeout";
    }
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  const auto wrapped = result_future.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
    wrapped.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    result.ok = true;
    return result;
  }
  result.failure_code = failure::EXEC_FAILED;
  result.reason = "execute_error:" +
    std::to_string(wrapped.result ? wrapped.result->error_code.val : 0);
  if (wrapped.result && wrapped.result->error_code.val ==
    moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
  {
    result.failure_code = failure::EXEC_TIMEOUT;
  }
  return result;
}

bool MoveItMotionBackend::validate(const JointTrajectory & trajectory, std::string * why)
{
  auto fail = [why](const std::string & reason) {
      if (why) {
        *why = reason;
      }
      return false;
    };
  if (trajectory.points.empty()) {
    return fail("empty_trajectory");
  }
  if (!validity_client_->service_is_ready()) {
    return fail("state_validity_unavailable");
  }
  const std::size_t n = trajectory.points.size();
  const std::size_t stride = static_cast<std::size_t>(std::max(1, config_.validate_stride));
  for (std::size_t i = 0; i < n; i += stride) {
    const std::size_t index = (i + stride >= n) ? n - 1 : i;
    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    request->group_name = config_.group;
    request->robot_state.joint_state.name = trajectory.joint_names;
    request->robot_state.joint_state.position = trajectory.points[index].positions;
    request->robot_state.is_diff = true;
    auto future = validity_client_->async_send_request(request);
    if (future.wait_for(1s) != std::future_status::ready) {
      validity_client_->remove_pending_request(future);
      return fail("state_validity_timeout");
    }
    if (!future.get()->valid) {
      return fail("waypoint_in_collision:" + std::to_string(index));
    }
    if (index == n - 1) {
      break;
    }
  }
  return true;
}

std::optional<std::vector<double>> MoveItMotionBackend::current_joints()
{
  std::lock_guard<std::mutex> lock(mgi_mutex_);
  auto state = mgi_->getCurrentState(1.0);
  if (!state) {
    return std::nullopt;
  }
  const auto * jmg = state->getJointModelGroup(config_.group);
  if (!jmg) {
    return std::nullopt;
  }
  std::vector<double> joints;
  state->copyJointGroupPositions(jmg, joints);
  return joints;
}

std::optional<Eigen::Isometry3d> MoveItMotionBackend::current_tcp()
{
  std::lock_guard<std::mutex> lock(mgi_mutex_);
  auto state = mgi_->getCurrentState(1.0);
  if (!state) {
    return std::nullopt;
  }
  if (!state->knowsFrameTransform(config_.base_frame) ||
    !state->knowsFrameTransform(config_.tip_link))
  {
    return std::nullopt;
  }
  return state->getFrameTransform(config_.base_frame).inverse() *
         state->getFrameTransform(config_.tip_link);
}

void MoveItMotionBackend::stop()
{
  cancel_active_goal();
  std::unique_lock<std::mutex> lock(mgi_mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    mgi_->stop();
  }
}

void MoveItMotionBackend::cancel_active_goal()
{
  GoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    handle = active_goal_;
  }
  if (handle && exec_client_) {
    try {
      exec_client_->async_cancel_goal(handle);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "cancel execute_trajectory failed: %s", e.what());
    }
  }
}

}  // namespace peach2_manipulation
