/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/robot_io_status_publisher.h"
#include <ros/ros.h>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器和订阅器
 */
RobotIOStatusPublisher::RobotIOStatusPublisher()
    : nh_()
    , private_nh_("~")
    , io_states_topic_("/aubo_driver/io_states")
    , base_frame_("base_link")
    , is_connected_(false)
{
    // 获取参数
    private_nh_.param("io_states_topic", io_states_topic_, std::string("/aubo_driver/io_states"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));

    // 初始化发布器
    robot_io_status_pub_ = nh_.advertise<demo_interface::RobotIOStatus>("/demo_robot_io_status", 10);

    // 初始化订阅器
    io_states_sub_ = nh_.subscribe(io_states_topic_, 10,
                                   &RobotIOStatusPublisher::ioStatesCallback, this);

    ROS_INFO("RobotIOStatusPublisher initialized Publishing to: /robot_io_status");
}

RobotIOStatusPublisher::~RobotIOStatusPublisher()
{
}

/**
 * @brief IO状态回调函数
 * @param msg IO状态消息
 */
void RobotIOStatusPublisher::ioStatesCallback(const aubo_msgs::IOState::ConstPtr& msg)
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
void RobotIOStatusPublisher::publishIOStatus(const aubo_msgs::IOState& io_state)
{
    demo_interface::RobotIOStatus status_msg;

    // 设置消息头
    status_msg.header.stamp = ros::Time::now();
    status_msg.header.frame_id = base_frame_;

    // 转换数字输入状态
    status_msg.digital_inputs.clear();
    for (size_t i = 0; i < io_state.digital_in_states.size(); ++i)
    {
        status_msg.digital_inputs.push_back(io_state.digital_in_states[i].state);
    }

    // 转换数字输出状态
    status_msg.digital_outputs.clear();
    for (size_t i = 0; i < io_state.digital_out_states.size(); ++i)
    {
        status_msg.digital_outputs.push_back(io_state.digital_out_states[i].state);
    }

    // 转换模拟输入状态
    status_msg.analog_inputs.clear();
    for (size_t i = 0; i < io_state.analog_in_states.size(); ++i)
    {
        status_msg.analog_inputs.push_back(io_state.analog_in_states[i].state);
    }

    // 转换模拟输出状态
    status_msg.analog_outputs.clear();
    for (size_t i = 0; i < io_state.analog_out_states.size(); ++i)
    {
        status_msg.analog_outputs.push_back(io_state.analog_out_states[i].state);
    }

    // 转换工具IO状态
    status_msg.tool_io_status.digital_inputs.clear();
    status_msg.tool_io_status.digital_outputs.clear();
    status_msg.tool_io_status.analog_inputs.clear();
    status_msg.tool_io_status.analog_outputs.clear();

    // 工具数字IO状态
    for (size_t i = 0; i < io_state.tool_io_states.size(); ++i)
    {
        // 根据 flag 判断是输入还是输出
        if (io_state.tool_io_states[i].flag == 0)  // 输入
        {
            status_msg.tool_io_status.digital_inputs.push_back(io_state.tool_io_states[i].state);
        }
        else  // 输出
        {
            status_msg.tool_io_status.digital_outputs.push_back(io_state.tool_io_states[i].state);
        }
    }

    // 工具模拟输入状态
    for (size_t i = 0; i < io_state.tool_ai_states.size(); ++i)
    {
        status_msg.tool_io_status.analog_inputs.push_back(io_state.tool_ai_states[i].state);
    }

    // 设置连接状态
    status_msg.is_connected = is_connected_;

    // 发布消息
    robot_io_status_pub_.publish(status_msg);
}

/**
 * @brief 主循环，保持节点运行
 */
void RobotIOStatusPublisher::spin()
{
    ros::spin();
}

} // namespace demo_driver

