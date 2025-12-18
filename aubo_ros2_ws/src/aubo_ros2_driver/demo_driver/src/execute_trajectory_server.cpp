/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/execute_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
ExecuteTrajectoryServer::ExecuteTrajectoryServer()
    : Node("execute_trajectory_server_node")
    , planning_group_name_("manipulator_e5")
    , base_frame_("base_link")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator_e5"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化 MoveIt 接口
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
    
    try
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_name_);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        // 设置参考坐标系
        move_group_->setPoseReferenceFrame(base_frame_);
        
        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized successfully");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interfaces: %s", e.what());
        throw;
    }

    // 初始化服务服务器
    execute_trajectory_service_ = this->create_service<demo_interface::srv::ExecuteTrajectory>(
        "/execute_trajectory",
        std::bind(&ExecuteTrajectoryServer::executeTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ExecuteTrajectoryServer initialized, service '/execute_trajectory' is ready");
}

ExecuteTrajectoryServer::~ExecuteTrajectoryServer()
{
}

/**
 * @brief 服务回调函数
 */
void ExecuteTrajectoryServer::executeTrajectoryCallback(
    const std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Request> req,
    std::shared_ptr<demo_interface::srv::ExecuteTrajectory::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received execute_trajectory request");
    RCLCPP_INFO(this->get_logger(), "Trajectory has %zu waypoints", req->trajectory.points.size());

    // 验证轨迹
    if (req->trajectory.points.empty())
    {
        res->success = false;
        res->error_code = -1;
        res->message = "Trajectory is empty";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    if (req->trajectory.joint_names.empty())
    {
        res->success = false;
        res->error_code = -2;
        res->message = "Trajectory joint names are empty";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 执行轨迹
    int32_t error_code = 0;
    std::string message;
    bool success = executeTrajectory(req->trajectory, error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Trajectory execution failed: %s (error_code: %d)", message.c_str(), error_code);
    }
}

/**
 * @brief 执行轨迹
 */
bool ExecuteTrajectoryServer::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                int32_t& error_code,
                                                std::string& message)
{
    if (!move_group_)
    {
        error_code = -100;
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 将 trajectory_msgs::JointTrajectory 转换为 MoveIt 的 RobotTrajectory
        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
        robot_trajectory_msg.joint_trajectory = trajectory;

        // 创建 MoveIt 轨迹对象
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = robot_trajectory_msg;

        // 验证轨迹点数量
        if (plan.trajectory_.joint_trajectory.points.empty())
        {
            error_code = -101;
            message = "Converted trajectory is empty";
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Executing trajectory with %zu waypoints", 
                 plan.trajectory_.joint_trajectory.points.size());

        // 执行轨迹
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(plan);

        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            return false;
        }

        // 成功
        error_code = 0;
        message = "Successfully executed trajectory with " + 
                  std::to_string(plan.trajectory_.joint_trajectory.points.size()) + " waypoints";
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = -200;
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void ExecuteTrajectoryServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver
