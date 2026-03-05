/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 *
 * Movel 服务：笛卡尔直线运动。依赖 move_group 与 robot_description。
 *
 * 测试命令（需先启动含 move_group + movel_server 的 launch，如 aubo_moveit_pure_ros2 + demo_driver_services 或 graspnet_demo）：
 *
 *   # 单行调用（目标 xyz 单位 m，姿态四元数 xyzw；velocity/acceleration_factor 0~1，缺省会用默认值）
 *   ros2 service call /movel demo_interface/srv/Movel "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_factor: 0.15, acceleration_factor: 0.1}"
 *
 *   # 仅改 z 高度示例
 *   ros2 service call /movel demo_interface/srv/Movel "{target_pose: {position: {x: 0.35, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_factor: 0.2, acceleration_factor: 0.1}"
 *
 * 查看服务类型：
 *   ros2 service type /movel
 *   ros2 interface show demo_interface/srv/Movel
 */

#include "demo_driver/movel_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

namespace demo_driver
{

static constexpr float kDefaultVelocityFactor = 0.15f;
static constexpr float kDefaultAccelerationFactor = 0.1f;
static constexpr double kMinFractionSuccess = 0.99;

MovelServer::MovelServer(const rclcpp::NodeOptions& options)
: Node("movel_server_node", options),
  planning_group_name_("manipulator"),
  base_frame_("base_link"),
  end_effector_link_(""),
  compute_cartesian_path_service_name_("move_group/compute_cartesian_path"),
  cartesian_max_step_(0.002)
{
  this->declare_parameter("planning_group_name", std::string("manipulator"));
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("compute_cartesian_path_service", std::string("move_group/compute_cartesian_path"));
  this->declare_parameter("cartesian_max_step", 0.002);

  this->get_parameter("planning_group_name", planning_group_name_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("compute_cartesian_path_service", compute_cartesian_path_service_name_);
  this->get_parameter("cartesian_max_step", cartesian_max_step_);

  movel_service_ = this->create_service<demo_interface::srv::Movel>(
    "/movel",
    std::bind(&MovelServer::movelCallback, this, std::placeholders::_1, std::placeholders::_2));

  compute_cartesian_path_client_ =
    this->create_client<moveit_msgs::srv::GetCartesianPath>(compute_cartesian_path_service_name_);

  RCLCPP_INFO(this->get_logger(), "MovelServer: 服务 /movel 已创建, GetCartesianPath 客户端: %s",
    compute_cartesian_path_service_name_.c_str());
}

MovelServer::~MovelServer() {}

bool MovelServer::wait_for_robot_description(int timeout_seconds)
{
  auto start_time = std::chrono::steady_clock::now();
  int check_count = 0;

  while (rclcpp::ok()) {
    try {
      auto test_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
      std::string test_frame = test_move_group->getPlanningFrame();
      if (!test_frame.empty()) {
        return true;
      }
    } catch (const std::exception& e) {
      check_count++;
      if (check_count % 10 == 0) {
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(this->get_logger(), "MovelServer: 等待 robot_description... (已等待 %ld 秒)", elapsed_seconds);
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds) {
      RCLCPP_ERROR(this->get_logger(), "MovelServer: 等待超时 (%d 秒)", timeout_seconds);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return false;
}

bool MovelServer::initialize(int max_retries, int retry_delay_seconds)
{
  if (!wait_for_robot_description(30)) {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (int attempt = 1; attempt <= max_retries; ++attempt) {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
      planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
      move_group_->setPoseReferenceFrame(base_frame_);
      end_effector_link_ = move_group_->getEndEffectorLink();
      std::string planning_frame = move_group_->getPlanningFrame();
      RCLCPP_INFO(this->get_logger(), "[Movel] 初始化 规划组=%s | 参考系=%s | 末端=%s",
        planning_group_name_.c_str(), planning_frame.c_str(), end_effector_link_.c_str());
      return true;
    } catch (const std::exception& e) {
      if (attempt < max_retries) {
        std::this_thread::sleep_for(std::chrono::seconds(retry_delay_seconds));
      } else {
        RCLCPP_ERROR(this->get_logger(), "MovelServer: MoveIt 接口初始化失败，已尝试 %d 次", max_retries);
        return false;
      }
    }
  }
  return false;
}

void MovelServer::movelCallback(
  const std::shared_ptr<demo_interface::srv::Movel::Request> req,
  std::shared_ptr<demo_interface::srv::Movel::Response> res)
{
  float velocity_factor = req->velocity_factor;
  float acceleration_factor = req->acceleration_factor;
  if (velocity_factor <= 0.f || velocity_factor > 1.f) {
    velocity_factor = kDefaultVelocityFactor;
  }
  if (acceleration_factor <= 0.f || acceleration_factor > 1.f) {
    acceleration_factor = kDefaultAccelerationFactor;
  }

  RCLCPP_INFO(this->get_logger(), "[Movel] 目标 (%.3f, %.3f, %.3f) vel=%.2f acc=%.2f",
    req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z,
    velocity_factor, acceleration_factor);

  int32_t error_code = 0;
  std::string message;
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(movel_mutex_);
    success = movel(req->target_pose, velocity_factor, acceleration_factor, error_code, message);
  }
  res->success = success;
  res->error_code = error_code;
  res->message = message;

  if (success) {
    RCLCPP_INFO(this->get_logger(), "[Movel] 响应 成功");
  } else {
    RCLCPP_WARN(this->get_logger(), "[Movel] 响应 失败 error_code=%d | %s", error_code, message.c_str());
  }
}

bool MovelServer::movel(
  const geometry_msgs::msg::Pose& target_pose,
  float velocity_factor,
  float acceleration_factor,
  int32_t& error_code,
  std::string& message)
{
  if (!move_group_) {
    error_code = static_cast<int32_t>(MovelErrorCode::MOVEIT_NOT_INITIALIZED);
    message = "MoveIt 接口未初始化";
    return false;
  }

  try {
    // Retry getCurrentState (joint_states may have stamp 0 at startup; allow time for valid state)
    moveit::core::RobotStatePtr current_state;
    const int kMaxGetStateRetries = 5;
    const auto kRetryDelay = std::chrono::milliseconds(500);
    for (int attempt = 0; attempt < kMaxGetStateRetries; ++attempt) {
      current_state = move_group_->getCurrentState();
      if (current_state) {
        break;
      }
      if (attempt < kMaxGetStateRetries - 1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[Movel] getCurrentState attempt %d/%d failed, retrying...", attempt + 1,
                             kMaxGetStateRetries);
        rclcpp::sleep_for(kRetryDelay);
      }
    }
    if (!current_state) {
      error_code = static_cast<int32_t>(MovelErrorCode::GET_CURRENT_STATE_FAILED);
      message = "无法获取当前机器人状态（请检查 /joint_states 发布与 use_sim_time 一致性）";
      return false;
    }

    const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(planning_group_name_);
    if (!jmg) {
      error_code = static_cast<int32_t>(MovelErrorCode::JOINT_MODEL_GROUP_FAILED);
      message = "无法获取关节模型组";
      return false;
    }

    if (end_effector_link_.empty()) {
      error_code = static_cast<int32_t>(MovelErrorCode::END_EFFECTOR_LINK_EMPTY);
      message = "末端执行器 link 为空";
      return false;
    }

    current_state->update();
    const Eigen::Isometry3d& ee_transform = current_state->getGlobalLinkTransform(end_effector_link_);
    Eigen::Vector3d pos = ee_transform.translation();
    Eigen::Quaterniond quat(ee_transform.rotation());

    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = pos.x();
    current_pose.position.y = pos.y();
    current_pose.position.z = pos.z();
    current_pose.orientation.x = quat.x();
    current_pose.orientation.y = quat.y();
    current_pose.orientation.z = quat.z();
    current_pose.orientation.w = quat.w();

    moveit_msgs::msg::RobotState start_state_msg;
    moveit::core::robotStateToRobotStateMsg(*current_state, start_state_msg, true);

    if (!compute_cartesian_path_client_->wait_for_service(std::chrono::seconds(5))) {
      error_code = static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_SERVICE_UNAVAILABLE);
      message = "GetCartesianPath 服务不可用: " + compute_cartesian_path_service_name_;
      return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
    request->header.frame_id = move_group_->getPlanningFrame();
    request->header.stamp = this->now();
    request->start_state = start_state_msg;
    request->group_name = planning_group_name_;
    request->link_name = end_effector_link_;
    request->waypoints.push_back(current_pose);
    request->waypoints.push_back(target_pose);
    request->max_step = cartesian_max_step_;
    request->jump_threshold = 0.0;
    request->prismatic_jump_threshold = 0.0;
    request->revolute_jump_threshold = 0.0;
    request->avoid_collisions = true;

    auto result_future = compute_cartesian_path_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      error_code = static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_CALL_TIMEOUT);
      message = "GetCartesianPath 调用超时或失败";
      return false;
    }

    auto response = result_future.get();
    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_code = response->error_code.val;
      message = "GetCartesianPath 规划失败, error_code=" + std::to_string(response->error_code.val);
      return false;
    }

    if (response->fraction < kMinFractionSuccess) {
      error_code = static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_FRACTION_INCOMPLETE);
      message = "笛卡尔路径仅完成 " + std::to_string(static_cast<int>(response->fraction * 100)) + "%";
      return false;
    }

    if (response->solution.joint_trajectory.points.empty()) {
      error_code = static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_EMPTY_TRAJECTORY);
      message = "GetCartesianPath 返回空轨迹";
      return false;
    }

    move_group_->setMaxVelocityScalingFactor(velocity_factor);
    move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = response->solution;

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error_code = exec_result.val;
      message = "轨迹执行失败, error_code=" + std::to_string(exec_result.val);
      return false;
    }

    error_code = static_cast<int32_t>(MovelErrorCode::SUCCESS);
    message = "笛卡尔直线运动完成";
    return true;
  } catch (const std::exception& e) {
    error_code = static_cast<int32_t>(MovelErrorCode::EXCEPTION);
    message = std::string("异常: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "[Movel] %s", message.c_str());
    return false;
  }
}

void MovelServer::spin()
{
  rclcpp::spin(this->shared_from_this());
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  try {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", false);
    auto server = std::make_shared<demo_driver::MovelServer>(node_options);

    if (!server->initialize(10, 2)) {
      RCLCPP_ERROR(rclcpp::get_logger("movel_server_node"), "MoveIt 接口初始化失败");
      rclcpp::shutdown();
      return 1;
    }

    // Spin SingleThreadedExecutor so the current state monitor can receive joint_states
    executor.add_node(server);
    std::thread([&executor]() { executor.spin(); }).detach();
    /* Otherwise robot with zeros joint_states */
    rclcpp::sleep_for(std::chrono::seconds(1));
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("movel_server_node"), "movel_server_node 异常: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
