/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_robot_enable_server.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器和服务服务器
 */
SetRobotEnableServer::SetRobotEnableServer()
    : Node("set_robot_enable_server_node")
    , robot_control_topic_("robot_control")
{
    // 获取参数
    this->declare_parameter("robot_control_topic", std::string("robot_control"));
    this->get_parameter("robot_control_topic", robot_control_topic_);

    // 初始化发布器
    robot_control_pub_ = this->create_publisher<std_msgs::msg::String>(robot_control_topic_, 10);

    // 等待发布器连接
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 初始化服务服务器
    set_robot_enable_service_ = this->create_service<demo_interface::srv::SetRobotEnable>(
        "/set_robot_enable",
        std::bind(&SetRobotEnableServer::setRobotEnableCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "SetRobotEnableServer initialized, service '/set_robot_enable' is ready");
    RCLCPP_INFO(this->get_logger(), "Using robot control topic: %s", robot_control_topic_.c_str());
}

SetRobotEnableServer::~SetRobotEnableServer()
{
}

/**
 * @brief 服务回调函数
 */
void SetRobotEnableServer::setRobotEnableCallback(
    const std::shared_ptr<demo_interface::srv::SetRobotEnable::Request> req,
    std::shared_ptr<demo_interface::srv::SetRobotEnable::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received set_robot_enable request: enable = %s", req->enable ? "true" : "false");

    // 设置机器人使能状态
    int32_t error_code = 0;
    std::string message;
    bool success = setRobotEnable(req->enable, error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Set robot enable succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set robot enable failed: %s (error_code: %d)", message.c_str(), error_code);
    }
}

/**
 * @brief 设置机器人使能状态
 */
bool SetRobotEnableServer::setRobotEnable(bool enable, int32_t& error_code, std::string& message)
{
    try
    {
        auto control_msg = std::make_shared<std_msgs::msg::String>();

        if (enable)
        {
            // 使能机器人：发送 powerOn 命令
            control_msg->data = "powerOn";
            RCLCPP_INFO(this->get_logger(), "Sending powerOn command to enable robot");
        }
        else
        {
            // 禁用机器人：发送 powerOff 命令
            control_msg->data = "powerOff";
            RCLCPP_INFO(this->get_logger(), "Sending powerOff command to disable robot");
        }

        // 发布控制命令
        if (robot_control_pub_->get_subscription_count() == 0)
        {
            error_code = -1;
            message = "No subscribers to robot control topic: " + robot_control_topic_ + 
                     ". Make sure aubo_driver is running.";
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
            return false;
        }

        robot_control_pub_->publish(*control_msg);
        
        // 等待命令发送
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 验证消息是否发送成功
        if (robot_control_pub_->get_subscription_count() > 0)
        {
            error_code = 0;
            message = "Successfully sent " + std::string(enable ? "enable" : "disable") + 
                     " command to robot";
            return true;
        }
        else
        {
            error_code = -2;
            message = "Failed to send command: subscriber disconnected";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        error_code = -100;
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void SetRobotEnableServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver
