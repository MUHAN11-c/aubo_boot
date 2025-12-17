/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/read_robot_io_server.h"
#include <ros/ros.h>
#include <algorithm>
#include <cctype>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化订阅器和服务服务器
 */
ReadRobotIOServer::ReadRobotIOServer()
    : nh_()
    , private_nh_("~")
    , io_state_received_(false)
    , io_states_topic_("/aubo_driver/io_states")
{
    // 获取参数
    private_nh_.param("io_states_topic", io_states_topic_, std::string("/aubo_driver/io_states"));

    // 初始化订阅器
    io_states_sub_ = nh_.subscribe(io_states_topic_, 10,
                                   &ReadRobotIOServer::ioStatesCallback, this);

    // 初始化服务服务器
    read_robot_io_service_ = nh_.advertiseService("/read_robot_io",
                                                   &ReadRobotIOServer::readRobotIOCallback, this);

    ROS_INFO("ReadRobotIOServer initialized, service '/read_robot_io' is ready");
    ROS_INFO("Subscribing to: %s", io_states_topic_.c_str());
}

ReadRobotIOServer::~ReadRobotIOServer()
{
}

/**
 * @brief IO状态回调函数
 * @param msg IO状态消息
 */
void ReadRobotIOServer::ioStatesCallback(const aubo_msgs::IOState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(io_state_mutex_);
    current_io_state_ = *msg;  // 保存当前IO状态
    io_state_received_ = true;  // 标记已收到IO状态
}

/**
 * @brief 服务回调函数
 * @param req 服务请求
 * @param res 服务响应
 * @return 成功返回true
 */
bool ReadRobotIOServer::readRobotIOCallback(demo_interface::ReadRobotIO::Request& req,
                                           demo_interface::ReadRobotIO::Response& res)
{
    ROS_DEBUG("Received read_robot_io request");
    ROS_DEBUG("IO type: %s, index: %d", req.io_type.c_str(), req.io_index);

    // 验证输入参数
    if (req.io_index < 0)
    {
        res.success = false;
        res.value = 0.0;
        res.message = "Invalid io_index, must be >= 0";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    // 读取机器人IO
    double value = 0.0;
    std::string message;
    bool success = readRobotIO(req.io_type, req.io_index, value, message);

    res.success = success;
    res.value = value;
    res.message = message;

    if (success)
    {
        ROS_DEBUG("Read robot IO succeeded: %s, value: %.2f", message.c_str(), value);
    }
    else
    {
        ROS_WARN("Read robot IO failed: %s", message.c_str());
    }

    return true;
}

/**
 * @brief 读取机器人IO
 * @param io_type IO类型
 * @param io_index IO点索引
 * @param value 输出的值
 * @param message 输出消息
 * @return 成功返回true
 */
bool ReadRobotIOServer::readRobotIO(const std::string& io_type,
                                    int32_t io_index,
                                    double& value,
                                    std::string& message)
{
    std::lock_guard<std::mutex> lock(io_state_mutex_);

    // 检查是否已收到IO状态
    if (!io_state_received_)
    {
        message = "IO state not received yet. Make sure aubo_driver is running and publishing IO states.";
        return false;
    }

    // 将IO类型字符串转换为小写以便比较
    std::string io_type_lower = io_type;
    std::transform(io_type_lower.begin(), io_type_lower.end(), io_type_lower.begin(),
                  [](unsigned char c) { return std::tolower(c); });

    try
    {
        if (io_type_lower == "digital_input" || io_type_lower == "digitalinput")
        {
            // 数字输入
            if (io_index < static_cast<int32_t>(current_io_state_.digital_in_states.size()))
            {
                value = current_io_state_.digital_in_states[io_index].state ? 1.0 : 0.0;
                message = "Successfully read digital input pin " + std::to_string(io_index) + 
                         " = " + (value != 0.0 ? "HIGH" : "LOW");
                return true;
            }
            else
            {
                message = "Digital input index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.digital_in_states.size() - 1);
                return false;
            }
        }
        else if (io_type_lower == "digital_output" || io_type_lower == "digitaloutput")
        {
            // 数字输出
            if (io_index < static_cast<int32_t>(current_io_state_.digital_out_states.size()))
            {
                value = current_io_state_.digital_out_states[io_index].state ? 1.0 : 0.0;
                message = "Successfully read digital output pin " + std::to_string(io_index) + 
                         " = " + (value != 0.0 ? "HIGH" : "LOW");
                return true;
            }
            else
            {
                message = "Digital output index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.digital_out_states.size() - 1);
                return false;
            }
        }
        else if (io_type_lower == "analog_input" || io_type_lower == "analoginput")
        {
            // 模拟输入
            if (io_index < static_cast<int32_t>(current_io_state_.analog_in_states.size()))
            {
                value = static_cast<double>(current_io_state_.analog_in_states[io_index].state);
                message = "Successfully read analog input pin " + std::to_string(io_index) + 
                         " = " + std::to_string(value);
                return true;
            }
            else
            {
                message = "Analog input index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.analog_in_states.size() - 1);
                return false;
            }
        }
        else if (io_type_lower == "analog_output" || io_type_lower == "analogoutput")
        {
            // 模拟输出
            if (io_index < static_cast<int32_t>(current_io_state_.analog_out_states.size()))
            {
                value = static_cast<double>(current_io_state_.analog_out_states[io_index].state);
                message = "Successfully read analog output pin " + std::to_string(io_index) + 
                         " = " + std::to_string(value);
                return true;
            }
            else
            {
                message = "Analog output index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.analog_out_states.size() - 1);
                return false;
            }
        }
        else if (io_type_lower == "tool_io" || io_type_lower == "toolio")
        {
            // 工具数字IO
            if (io_index < static_cast<int32_t>(current_io_state_.tool_io_states.size()))
            {
                value = current_io_state_.tool_io_states[io_index].state ? 1.0 : 0.0;
                std::string io_mode = (current_io_state_.tool_io_states[io_index].flag == 0) ? 
                                     "input" : "output";
                message = "Successfully read tool IO pin " + std::to_string(io_index) + 
                         " (" + io_mode + ") = " + (value != 0.0 ? "HIGH" : "LOW");
                return true;
            }
            else
            {
                message = "Tool IO index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.tool_io_states.size() - 1);
                return false;
            }
        }
        else if (io_type_lower == "tool_analog_input" || io_type_lower == "toolanaloginput")
        {
            // 工具模拟输入
            if (io_index < static_cast<int32_t>(current_io_state_.tool_ai_states.size()))
            {
                value = static_cast<double>(current_io_state_.tool_ai_states[io_index].state);
                message = "Successfully read tool analog input pin " + std::to_string(io_index) + 
                         " = " + std::to_string(value);
                return true;
            }
            else
            {
                message = "Tool analog input index " + std::to_string(io_index) + 
                         " out of range. Available: 0-" + 
                         std::to_string(current_io_state_.tool_ai_states.size() - 1);
                return false;
            }
        }
        else
        {
            message = "Invalid io_type: " + io_type + 
                     ". Valid types: digital_input, digital_output, analog_input, "
                     "analog_output, tool_io, tool_analog_input";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        message = std::string("Exception occurred: ") + e.what();
        ROS_ERROR("%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void ReadRobotIOServer::spin()
{
    ros::spin();
}

} // namespace demo_driver

