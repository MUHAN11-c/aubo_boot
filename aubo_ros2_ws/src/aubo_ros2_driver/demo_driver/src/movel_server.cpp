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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

namespace demo_driver
{

namespace
{
void setMovelError(int32_t& error_code, std::string& message, int32_t code, std::string msg)
{
  error_code = code;
  message = std::move(msg);
}

void setMovelSuccess(int32_t& error_code, std::string& message)
{
  error_code = static_cast<int32_t>(MovelErrorCode::SUCCESS);
  message = "笛卡尔直线运动完成";
}
}  // namespace

static constexpr float kDefaultVelocityFactor = 0.15f;
static constexpr float kDefaultAccelerationFactor = 0.1f;
static constexpr double kMinFractionSuccess = 0.99;

MovelServer::MovelServer(const rclcpp::NodeOptions& options)
: Node("movel_server_node", options),
  planning_group_name_("manipulator"),
  base_frame_("base_link"),
  end_effector_link_(""),
  cartesian_max_step_(0.02),
  use_request_orientation_(false)
{
  this->declare_parameter("planning_group_name", std::string("manipulator"));
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("cartesian_max_step", 0.02);  // 步长越大插值点越少，有时可避开奇异/限位导致的部分完成；可 launch 覆盖
  this->declare_parameter("use_request_orientation", false);  // false=仅用请求位置、姿态保持当前，易提高完成率

  this->get_parameter("planning_group_name", planning_group_name_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("cartesian_max_step", cartesian_max_step_);
  this->get_parameter("use_request_orientation", use_request_orientation_);

  movel_service_ = this->create_service<demo_interface::srv::Movel>(
    "/movel",
    std::bind(&MovelServer::movelCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "MovelServer: 服务 /movel 已创建（笛卡尔路径使用 MoveGroupInterface::computeCartesianPath，与 moveit2_tutorials 一致）");
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

bool MovelServer::initialize(int /* max_retries */, int /* retry_delay_seconds */)
{
  if (!wait_for_robot_description(30)) {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  try {
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
    moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

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
    RCLCPP_ERROR(this->get_logger(), "MovelServer: MoveIt 接口初始化失败: %s", e.what());
    return false;
  }
}

void MovelServer::movelCallback(
  const std::shared_ptr<demo_interface::srv::Movel::Request> req,
  std::shared_ptr<demo_interface::srv::Movel::Response> res)
{
  const std::string& move_axis = req->move_axis;
  if (move_axis != "x" && move_axis != "y" && move_axis != "z") {
    res->success = false;
    res->error_code = static_cast<int32_t>(MovelErrorCode::INVALID_MOVE_AXIS);
    res->message = "move_axis 必须为 x、y 或 z";
    RCLCPP_WARN(this->get_logger(), "[Movel] 响应 失败 move_axis 非法: \"%s\"", move_axis.c_str());
    return;
  }

  float velocity_factor = req->velocity_factor;
  float acceleration_factor = req->acceleration_factor;
  if (velocity_factor <= 0.f || velocity_factor > 1.f) {
    velocity_factor = kDefaultVelocityFactor;
  }
  if (acceleration_factor <= 0.f || acceleration_factor > 1.f) {
    acceleration_factor = kDefaultAccelerationFactor;
  }

  RCLCPP_INFO(this->get_logger(), "[Movel] 目标 (%.3f, %.3f, %.3f) move_axis=%s vel=%.2f acc=%.2f",
    req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z,
    move_axis.c_str(), velocity_factor, acceleration_factor);

  int32_t error_code = 0;
  std::string message;
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(movel_mutex_);
    success = movel(req->target_pose, velocity_factor, acceleration_factor, move_axis, error_code, message);
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
  const std::string& move_axis,
  int32_t& error_code,
  std::string& message)
{
  if (!move_group_) {
    setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::MOVEIT_NOT_INITIALIZED), "MoveIt 接口未初始化");
    return false;
  }

  try {
    moveit::core::RobotStatePtr current_state;
    const double kGetStateTimeout = 10.0;
    if (!moveit_cpp_ || !moveit_cpp_->getCurrentState(current_state, kGetStateTimeout) || !current_state) {
      setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::GET_CURRENT_STATE_FAILED),
                    "无法获取当前机器人状态（请检查 /joint_states 发布与 use_sim_time 一致性）");
      return false;
    }

    const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(planning_group_name_);
    if (!jmg) {
      setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::JOINT_MODEL_GROUP_FAILED), "无法获取关节模型组");
      return false;
    }

    if (end_effector_link_.empty()) {
      setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::END_EFFECTOR_LINK_EMPTY), "末端执行器 link 为空");
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

    // 当前状态打印日志
    std::vector<double> joint_positions;
    current_state->copyJointGroupPositions(jmg, joint_positions);
    std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
    std::stringstream ss;
    ss << "关节(rad): ";
    for (size_t i = 0; i < joint_names.size() && i < joint_positions.size(); ++i)
      ss << joint_names[i] << "=" << std::fixed << std::setprecision(4) << joint_positions[i] << " ";
    RCLCPP_INFO(this->get_logger(), "[Movel] 当前状态 %s", ss.str().c_str());
    RCLCPP_INFO(this->get_logger(), "[Movel] 当前末端(%s) xyz=[%.4f, %.4f, %.4f] xyzw=[%.4f, %.4f, %.4f, %.4f]",
                end_effector_link_.c_str(), pos.x(), pos.y(), pos.z(),
                quat.x(), quat.y(), quat.z(), quat.w());

    move_group_->setStartState(*current_state);
    // 单轴移动：仅 move_axis 取 target，其余两轴保持 current
    geometry_msgs::msg::Pose waypoint_pose;
    waypoint_pose.position.x = (move_axis == "x") ? target_pose.position.x : current_pose.position.x;
    waypoint_pose.position.y = (move_axis == "y") ? target_pose.position.y : current_pose.position.y;
    waypoint_pose.position.z = (move_axis == "z") ? target_pose.position.z : current_pose.position.z;
    if (use_request_orientation_) {
      waypoint_pose.orientation = target_pose.orientation;
    } else {
      waypoint_pose.orientation = current_pose.orientation;
    }
    RCLCPP_INFO(this->get_logger(), "[Movel] move_axis=%s waypoint xyz=[%.4f, %.4f, %.4f]",
                move_axis.c_str(), waypoint_pose.position.x, waypoint_pose.position.y, waypoint_pose.position.z);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(waypoint_pose);

    move_group_->setPlanningTime(5.0);  // 规划超时 5 秒

    // #region agent log
    {
      std::ofstream f("/home/mu/IVG2.0/.cursor/debug-5074e2.log", std::ios::app);
      if (f) {
        f << std::fixed << std::setprecision(6)
          << "{\"sessionId\":\"5074e2\",\"hypothesisId\":\"H1,H2,H4\",\"location\":\"movel_server.cpp:before_computeCartesianPath\",\"message\":\"movel inputs\",\"data\":{"
          << "\"target_xyz\":[" << target_pose.position.x << "," << target_pose.position.y << "," << target_pose.position.z << "],"
          << "\"current_xyz\":[" << current_pose.position.x << "," << current_pose.position.y << "," << current_pose.position.z << "],"
          << "\"cartesian_max_step\":" << cartesian_max_step_ << ","
          << "\"waypoints_count\":" << waypoints.size() << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
      }
    }
    // #endregion

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    moveit_msgs::msg::MoveItErrorCodes path_error;
    double fraction = move_group_->computeCartesianPath(
        waypoints, cartesian_max_step_, 0.0, trajectory_msg, true, &path_error);

    // #region agent log
    {
      std::ofstream f("/home/mu/IVG2.0/.cursor/debug-5074e2.log", std::ios::app);
      if (f) {
        size_t npt = trajectory_msg.joint_trajectory.points.size();
        f << std::fixed << std::setprecision(6)
          << "{\"sessionId\":\"5074e2\",\"hypothesisId\":\"H1,H3,H5\",\"location\":\"movel_server.cpp:after_computeCartesianPath\",\"message\":\"movel result\",\"data\":{"
          << "\"fraction\":" << fraction << ",\"path_error_val\":" << path_error.val << ",\"trajectory_points\":" << npt << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
      }
    }
    // #endregion

    if (path_error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      setMovelError(error_code, message, path_error.val, "笛卡尔路径规划失败, error_code=" + std::to_string(path_error.val));
      return false;
    }
    if (fraction < kMinFractionSuccess) {
      setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_FRACTION_INCOMPLETE),
                    "笛卡尔路径仅完成 " + std::to_string(static_cast<int>(fraction * 100)) + "%");
      return false;
    }
    if (trajectory_msg.joint_trajectory.points.empty()) {
      setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::CARTESIAN_PATH_EMPTY_TRAJECTORY), "笛卡尔路径返回空轨迹");
      return false;
    }

    move_group_->setMaxVelocityScalingFactor(velocity_factor);
    move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_msg;

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      setMovelError(error_code, message, exec_result.val, "轨迹执行失败, error_code=" + std::to_string(exec_result.val));
      return false;
    }

    setMovelSuccess(error_code, message);
    return true;
  } catch (const std::exception& e) {
    setMovelError(error_code, message, static_cast<int32_t>(MovelErrorCode::EXCEPTION), std::string("异常: ") + e.what());
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
  // 多线程：回调内会 wait_for_service()，若用单线程会阻塞 executor 导致服务发现无法完成
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  try {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    // 不在此处写死 use_sim_time，由 launch 传入，与 move_group / joint_states 一致，否则 CurrentStateMonitor 会认为时间戳过期
    auto server = std::make_shared<demo_driver::MovelServer>(node_options);

    if (!server->initialize(10, 2)) {
      RCLCPP_ERROR(rclcpp::get_logger("movel_server_node"), "MoveIt 接口初始化失败");
      rclcpp::shutdown();
      return 1;
    }

    executor.add_node(server);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("movel_server_node"), "movel_server_node 异常: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
