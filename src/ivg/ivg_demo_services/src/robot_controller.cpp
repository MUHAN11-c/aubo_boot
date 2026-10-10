// RobotController 实现 — 源自 aubo_boot demo_driver（2026-09-30 移植，Jazzy 适配）。
// 适配点见头文件注释；笛卡尔规划统一走非弃用 computeCartesianPath 重载。
#include "ivg_demo_services/robot_controller.hpp"

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <chrono>
#include <cmath>
#include <thread>

namespace ivg_demo_services
{

RobotController::RobotController(rclcpp::Node * owner, const std::string & planning_group)
: node_(owner), planning_group_(planning_group)
{
  // 两阶段初始化：MoveGroupInterface 构造需要 owner->shared_from_this()，
  // 而 enable_shared_from_this 的 weak_ptr 在 shared_ptr 构造完成后才可用。
  if (node_->has_parameter("io_simulated")) {
    io_simulated_ = node_->get_parameter("io_simulated").as_bool();
  }
  if (node_->has_parameter("home_target")) {
    home_target_ = node_->get_parameter("home_target").as_string();
  }
  // 本仓 moveit 默认管线=pilz（v4d）；Pilz 建上下文必须显式 planner_id（PTP/LIN/CIRC），
  // 缺省 PTP。aubo_boot 旧栈默认 OMPL 不需要——移植适配点。
  if (node_->has_parameter("planning_pipeline")) {
    planning_pipeline_ = node_->get_parameter("planning_pipeline").as_string();
  }
  if (node_->has_parameter("planner_id")) {
    planner_id_ = node_->get_parameter("planner_id").as_string();
  }
}

bool RobotController::init()
{
  if (move_group_) {return true;}
  if (!node_) {return false;}
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_->shared_from_this(), planning_group_);
    move_group_->allowReplanning(true);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    setEndEffectorLink(move_group_->getEndEffectorLink());
    // 启动即订阅 /joint_states：否则首次查询才建订阅，DDS 发现超 1s 会
    // "Failed to fetch current robot state"（冷启动竞态，实测可 SIGSEGV）
    move_group_->startStateMonitor(5.0);
    io_client_ = node_->create_client<aubo_msgs::srv::SetIO>("/aubo_io_controller/set_io");
    return true;
  } catch (...) {
    return false;
  }
}

// ═════════════════════ 运动 ═════════════════════

/// plan/move 前统一套用规划管线与规划器（pilz 需显式 planner_id）
void RobotController::applyPlannerSelection()
{
  if (!move_group_) {return;}
  if (!planning_pipeline_.empty()) {
    move_group_->setPlanningPipelineId(planning_pipeline_);
  }
  if (!planner_id_.empty()) {
    move_group_->setPlannerId(planner_id_);
  }
}

bool RobotController::moveToHome(float vel, float acc)
{
  if (!move_group_) {return false;}
  move_group_->setStartStateToCurrentState();
  setVelocityScaling(vel);
  setAccelerationScaling(acc);
  applyPlannerSelection();
  move_group_->setNamedTarget(home_target_);
  const auto result = move_group_->move();
  return result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToJoints(
  const std::array<double, 6> & joints, float vel, float acc)
{
  if (!move_group_) {return false;}
  move_group_->setStartStateToCurrentState();
  setVelocityScaling(vel);
  setAccelerationScaling(acc);
  applyPlannerSelection();
  move_group_->setJointValueTarget(std::vector<double>(joints.begin(), joints.end()));
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group_->plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {return false;}
  const auto execute_result = move_group_->execute(plan);
  return execute_result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToPose(
  const geometry_msgs::msg::Pose & target, float vel, float acc)
{
  if (!move_group_) {return false;}
  move_group_->setStartStateToCurrentState();
  setVelocityScaling(vel);
  setAccelerationScaling(acc);
  applyPlannerSelection();
  move_group_->setPoseTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToPosition(double x, double y, double z, float vel, float acc)
{
  auto target = getCurrentPose();
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;
  return moveToPose(target, vel, acc);
}

bool RobotController::moveCartesianZ(double offset_m, float vel, float acc)
{
  return moveCartesianPath({{'z', offset_m}}, vel, acc);
}

bool RobotController::moveCartesianPath(
  const std::vector<CartesianSegment> & segments, float vel, float acc)
{
  if (!move_group_ || segments.empty()) {return false;}
  setVelocityScaling(vel);
  setAccelerationScaling(acc);

  auto waypoints = buildSegmentWaypoints(currentPoseInternal(), segments, z_min_limit_);
  if (waypoints.empty()) {return false;}

  for (int attempt = 0; attempt < max_retries_; ++attempt) {
    moveit_msgs::msg::RobotTrajectory traj;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step_, traj, true, &error_code);
    if (fraction < 0.99) {
      if (attempt < max_retries_ - 1) {
        std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
      }
      continue;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = traj;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return false;
}

bool RobotController::moveCartesianStraight(
  const geometry_msgs::msg::Pose & target, float vel, float acc)
{
  if (!move_group_) {return false;}
  setVelocityScaling(vel);
  setAccelerationScaling(acc);

  // 手动生成 slerp waypoints，保证最短旋转路径
  auto start = move_group_->getCurrentPose(eef_link_).pose;
  int steps = 20;
  auto waypoints = interpolateCartesian(start, target, steps);

  bool success = false;
  for (int attempt = 0; attempt < max_retries_; ++attempt) {
    moveit_msgs::msg::RobotTrajectory traj;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    double fraction = move_group_->computeCartesianPath(
      waypoints, 0.002, traj, true, &error_code);
    if (fraction < 0.99) {
      RCLCPP_WARN(
        node_->get_logger(), "CartesianStraight 规划 fraction=%.3f (第%d/%d次), 重试...",
        fraction, attempt + 1, max_retries_);
      if (attempt < max_retries_ - 1) {
        std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
      }
      continue;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = traj;
    success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {break;}
    RCLCPP_WARN(
      node_->get_logger(), "CartesianStraight 执行失败 (第%d/%d次), 重试...",
      attempt + 1, max_retries_);
    if (attempt < max_retries_ - 1) {
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
    }
  }
  return success;
}

bool RobotController::executeCartesianPath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, float vel, float acc)
{
  if (!move_group_ || waypoints.empty()) {return false;}
  setVelocityScaling(vel);
  setAccelerationScaling(acc);

  for (int attempt = 0; attempt < max_retries_; ++attempt) {
    moveit_msgs::msg::RobotTrajectory traj;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    // 0.002 步长防止 MoveIt 跳过密集 waypoints（与 aubo_boot 一致）
    double fraction = move_group_->computeCartesianPath(
      waypoints, 0.002, traj, true, &error_code);
    if (fraction < 0.95) {
      RCLCPP_WARN(
        node_->get_logger(), "executeCartesianPath 规划 fraction=%.3f (第%d/%d次), 重试...",
        fraction, attempt + 1, max_retries_);
      if (attempt < max_retries_ - 1) {
        std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
      }
      continue;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = traj;
    bool ok = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (ok) {return true;}
    RCLCPP_WARN(
      node_->get_logger(), "executeCartesianPath 执行失败 (第%d/%d次), 重试...",
      attempt + 1, max_retries_);
    if (attempt < max_retries_ - 1) {
      std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
    }
  }
  return false;
}

// ═════════════════════ IO ═════════════════════

bool RobotController::setGripper(int pin, bool open)
{
  if (io_simulated_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "setGripper(pin=%d, open=%d): io_simulated 旁路生效（未下发真实 IO）", pin, open ? 1 : 0);
    return true;
  }
  if (!io_client_) {
    RCLCPP_WARN(node_->get_logger(), "setGripper: io_client_ 未初始化");
    return false;
  }
  if (!io_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(), "setGripper: /aubo_io_controller/set_io 3s 未就绪");
    return false;
  }
  auto req = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  req->fun = aubo_msgs::srv::SetIO::Request::FUN_SET_ROBOT_BOARD_USER_DO;
  req->pin = pin;
  req->state = open ? 1.0f : 0.0f;
  auto future = io_client_->async_send_request(req);
  return future.wait_for(std::chrono::seconds(10)) == std::future_status::ready &&
         future.get()->success;
}

bool RobotController::setQuickSwap(int pin, bool lock)
{
  return setGripper(pin, lock);
}

// ═════════════════════ 查询 ═════════════════════

geometry_msgs::msg::Pose RobotController::getCurrentPose()
{
  if (!move_group_) {
    RCLCPP_WARN(node_->get_logger(), "getCurrentPose: move_group_ 未初始化");
    return geometry_msgs::msg::Pose();
  }
  return move_group_->getCurrentPose(eef_link_).pose;
}

std::vector<double> RobotController::getCurrentJoints()
{
  if (!move_group_) {
    RCLCPP_WARN(node_->get_logger(), "getCurrentJoints: move_group_ 未初始化");
    return {};
  }
  auto state = move_group_->getCurrentState();
  if (!state) {
    // CSM 冷启动竞态时返回空指针；判空防 SIGSEGV（上层报失败而非崩溃）
    RCLCPP_WARN(node_->get_logger(), "getCurrentJoints: 当前状态不可用（无 /joint_states）");
    return {};
  }
  std::vector<double> jv;
  state->copyJointGroupPositions(
    state->getRobotModel()->getJointModelGroup(move_group_->getName()), jv);
  return jv;
}

geometry_msgs::msg::Pose RobotController::jointsToPose(const std::array<double, 6> & joints)
{
  if (!move_group_) {
    RCLCPP_WARN(node_->get_logger(), "jointsToPose: move_group_ 未初始化");
    return geometry_msgs::msg::Pose();
  }
  auto robot_model = move_group_->getRobotModel();
  auto jmg = robot_model->getJointModelGroup(move_group_->getName());
  moveit::core::RobotState state(robot_model);
  state.setJointGroupPositions(jmg, std::vector<double>(joints.begin(), joints.end()));
  state.update();
  const auto & t = state.getGlobalLinkTransform(eef_link_);
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

std::string RobotController::getEndEffectorLink() const
{
  return eef_link_;
}

void RobotController::setEndEffectorLink(const std::string & link)
{
  eef_link_ = link;
  if (move_group_ && !link.empty()) {
    move_group_->setEndEffectorLink(link);
  }
}

// ═════════════════════ 配置 ═════════════════════

void RobotController::setVelocityScaling(float v)
{
  if (move_group_) {move_group_->setMaxVelocityScalingFactor(v);}
}

void RobotController::setAccelerationScaling(float a)
{
  if (move_group_) {move_group_->setMaxAccelerationScalingFactor(a);}
}

geometry_msgs::msg::Pose RobotController::currentPoseInternal()
{
  if (!move_group_) {return geometry_msgs::msg::Pose();}
  return move_group_->getCurrentPose(eef_link_).pose;
}

}  // namespace ivg_demo_services
