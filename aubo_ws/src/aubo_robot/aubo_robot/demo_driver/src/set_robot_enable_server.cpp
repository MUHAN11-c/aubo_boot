/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_robot_enable_server.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/duration.h>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器和服务服务器
 */
SetRobotEnableServer::SetRobotEnableServer()
    : nh_()
    , private_nh_("~")
    , robot_control_topic_("robot_control")
{
    // 获取参数
    private_nh_.param("robot_control_topic", robot_control_topic_, std::string("robot_control"));

    // 初始化发布器
    robot_control_pub_ = nh_.advertise<std_msgs::String>(robot_control_topic_, 10);

    // 等待发布器连接
    ros::Duration(0.5).sleep();

    // 初始化服务服务器
    set_robot_enable_service_ = nh_.advertiseService("/demo_driver/set_robot_enable",
                                                     &SetRobotEnableServer::setRobotEnableCallback, this);

    ROS_INFO("SetRobotEnableServer initialized, service '/demo_driver/set_robot_enable' is ready");
    ROS_INFO("Using robot control topic: %s", robot_control_topic_.c_str());
}

SetRobotEnableServer::~SetRobotEnableServer()
{
}

/**
 * @brief 服务回调函数
 * @param req 服务请求
 * @param res 服务响应
 * @return 成功返回true
 */
bool SetRobotEnableServer::setRobotEnableCallback(demo_interface::SetRobotEnable::Request& req,
                                                  demo_interface::SetRobotEnable::Response& res)
{
    ROS_INFO("Received set_robot_enable request: enable = %s", req.enable ? "true" : "false");

    // 设置机器人使能状态
    int32_t error_code = 0;
    std::string message;
    bool success = setRobotEnable(req.enable, error_code, message);

    res.success = success;
    res.error_code = error_code;
    res.message = message;

    if (success)
    {
        ROS_INFO("Set robot enable succeeded: %s", message.c_str());
    }
    else
    {
        ROS_WARN("Set robot enable failed: %s (error_code: %d)", message.c_str(), error_code);
    }

    return true;
}

/**
 * @brief 设置机器人使能状态
 * @param enable 使能状态（true为使能，false为禁用）
 * @param error_code 输出错误代码
 * @param message 输出消息
 * @return 成功返回true
 */
bool SetRobotEnableServer::setRobotEnable(bool enable, int32_t& error_code, std::string& message)
{
    try
    {
        std_msgs::String control_msg;

        if (enable)
        {
            // 使能机器人：发送 powerOn 命令
            control_msg.data = "powerOn";
            ROS_INFO("Sending powerOn command to enable robot");
        }
        else
        {
            // 禁用机器人：发送 powerOff 命令
            control_msg.data = "powerOff";
            ROS_INFO("Sending powerOff command to disable robot");
        }

        // 发布控制命令
        if (robot_control_pub_.getNumSubscribers() == 0)
        {
            error_code = -1;
            message = "No subscribers to robot control topic: " + robot_control_topic_ + 
                     ". Make sure aubo_driver is running.";
            ROS_WARN("%s", message.c_str());
            return false;
        }

        robot_control_pub_.publish(control_msg);
        
        // 等待命令发送
        ros::Duration(0.1).sleep();

        // 验证消息是否发送成功
        if (robot_control_pub_.getNumSubscribers() > 0)
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
        ROS_ERROR("%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void SetRobotEnableServer::spin()
{
    ros::spin();
}

} // namespace demo_driver

