/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/robot_io_status_publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器和订阅器
 */
RobotIOStatusPublisher::RobotIOStatusPublisher()
    : Node("robot_io_status_publisher_node")
    , io_states_topic_("/aubo_driver/io_states")
    , base_frame_("base_link")
    , is_connected_(false)
{
    // 获取参数
    this->declare_parameter("io_states_topic", std::string("/aubo_driver/io_states"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("io_states_topic", io_states_topic_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化发布器
    robot_io_status_pub_ = this->create_publisher<demo_interface::msg::RobotIOStatus>("/robot_io_status", 10);

    // 初始化订阅器
    io_states_sub_ = this->create_subscription<aubo_msgs::msg::IOStates>(
        io_states_topic_, 10,
        std::bind(&RobotIOStatusPublisher::ioStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotIOStatusPublisher initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", io_states_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: /robot_io_status");
}

RobotIOStatusPublisher::~RobotIOStatusPublisher()
{
}

/**
 * @brief IO状态回调函数
 * @param msg IO状态消息
 */
void RobotIOStatusPublisher::ioStatesCallback(const aubo_msgs::msg::IOStates::SharedPtr msg)
{
    // 标记为已连接
    is_connected_ = true;

    // 发布IO状态
    publishIOStatus(*msg);
}

/**
 * @brief 发布IO状态消息
 * @param io_state aubo IO状态消息
 */
void RobotIOStatusPublisher::publishIOStatus(const aubo_msgs::msg::IOStates& io_state)
{
    auto status_msg = std::make_shared<demo_interface::msg::RobotIOStatus>();

    // 设置消息头
    status_msg->header.stamp = this->now();
    status_msg->header.frame_id = base_frame_;

    // 转换数字输入状态
    status_msg->digital_inputs.clear();
    for (size_t i = 0; i < io_state.digital_in_states.size(); ++i)
    {
        status_msg->digital_inputs.push_back(io_state.digital_in_states[i].state);
    }

    // 转换数字输出状态
    status_msg->digital_outputs.clear();
    for (size_t i = 0; i < io_state.digital_out_states.size(); ++i)
    {
        status_msg->digital_outputs.push_back(io_state.digital_out_states[i].state);
    }

    // 转换模拟输入状态
    status_msg->analog_inputs.clear();
    for (size_t i = 0; i < io_state.analog_in_states.size(); ++i)
    {
        status_msg->analog_inputs.push_back(io_state.analog_in_states[i].state);
    }

    // 转换模拟输出状态
    status_msg->analog_outputs.clear();
    for (size_t i = 0; i < io_state.analog_out_states.size(); ++i)
    {
        status_msg->analog_outputs.push_back(io_state.analog_out_states[i].state);
    }

    // 转换工具IO状态
    status_msg->tool_io_status.digital_inputs.clear();
    status_msg->tool_io_status.digital_outputs.clear();
    status_msg->tool_io_status.analog_inputs.clear();
    status_msg->tool_io_status.analog_outputs.clear();

    // 工具数字IO状态 - 使用 flag_states（如果可用）
    // 注意：IOStates.msg 中没有专门的 tool_io_states 字段
    // flag_states 可能包含工具相关的标志状态
    for (size_t i = 0; i < io_state.flag_states.size(); ++i)
    {
        // flag_states 中的 state 字段表示状态
        status_msg->tool_io_status.digital_inputs.push_back(io_state.flag_states[i].state);
    }

    // 工具模拟输入状态 - IOStates.msg 中没有此字段
    // 如果需要工具模拟输入，可能需要从其他字段获取或留空
    // status_msg->tool_io_status.analog_inputs 保持为空

    // 设置连接状态
    status_msg->is_connected = is_connected_;

    // 发布消息
    robot_io_status_pub_->publish(*status_msg);
}

/**
 * @brief 主循环，保持节点运行
 */
void RobotIOStatusPublisher::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver
