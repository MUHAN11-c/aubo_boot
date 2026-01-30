/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_speed_factor_server.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <cmath>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
SetSpeedFactorServer::SetSpeedFactorServer()
    : Node("set_speed_factor_server_node")
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
    set_speed_factor_service_ = this->create_service<demo_interface::srv::SetSpeedFactor>(
        "/set_speed_factor",
        std::bind(&SetSpeedFactorServer::setSpeedFactorCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "SetSpeedFactorServer initialized, service '/set_speed_factor' is ready");
}

SetSpeedFactorServer::~SetSpeedFactorServer()
{
}

/**
 * @brief 服务回调函数
 */
void SetSpeedFactorServer::setSpeedFactorCallback(
    const std::shared_ptr<demo_interface::srv::SetSpeedFactor::Request> req,
    std::shared_ptr<demo_interface::srv::SetSpeedFactor::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received set_speed_factor request: velocity_factor = %.2f", req->velocity_factor);

    // 验证输入参数
    if (req->velocity_factor < 0.0 || req->velocity_factor > 1.0)
    {
        res->success = false;
        res->message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 设置速度因子
    std::string message;
    bool success = setSpeedFactor(req->velocity_factor, message);

    res->success = success;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Set speed factor succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set speed factor failed: %s", message.c_str());
    }
}

/**
 * @brief 设置速度因子
 */
bool SetSpeedFactorServer::setSpeedFactor(float velocity_factor, std::string& message)
{
    if (!move_group_)
    {
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 设置速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        
        message = "Successfully set velocity factor to " + std::to_string(velocity_factor);
        RCLCPP_INFO(this->get_logger(), "Velocity factor set to %.2f", velocity_factor);
        return true;
    }
    catch (const std::exception& e)
    {
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void SetSpeedFactorServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver
