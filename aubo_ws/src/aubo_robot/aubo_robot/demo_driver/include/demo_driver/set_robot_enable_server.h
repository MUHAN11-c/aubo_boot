/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_

#include <ros/ros.h>
#include <demo_interface/SetRobotEnable.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>

namespace demo_driver
{

/**
 * @brief 设置机器人使能状态服务服务器类
 * 提供设置机器人使能/禁用的服务
 */
class SetRobotEnableServer
{
public:
    SetRobotEnableServer();  // 构造函数
    ~SetRobotEnableServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // 发布器
    ros::Publisher robot_control_pub_;  // 发布机器人控制命令

    // 服务服务器
    ros::ServiceServer set_robot_enable_service_;  // 设置机器人使能服务

    // 服务回调函数
    bool setRobotEnableCallback(demo_interface::SetRobotEnable::Request& req,
                                demo_interface::SetRobotEnable::Response& res);

    // 辅助函数
    bool setRobotEnable(bool enable, int32_t& error_code, std::string& message);

    // 参数
    std::string robot_control_topic_;  // 机器人控制话题名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_

