/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/set_robot_enable.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 设置机器人使能状态服务服务器类
 * 提供设置机器人使能/禁用的服务
 */
class SetRobotEnableServer : public rclcpp::Node
{
public:
    SetRobotEnableServer();  // 构造函数
    ~SetRobotEnableServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // 发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_control_pub_;  // 发布机器人控制命令

    // 服务服务器
    rclcpp::Service<demo_interface::srv::SetRobotEnable>::SharedPtr set_robot_enable_service_;  // 设置机器人使能服务

    // 服务回调函数
    void setRobotEnableCallback(
        const std::shared_ptr<demo_interface::srv::SetRobotEnable::Request> req,
        std::shared_ptr<demo_interface::srv::SetRobotEnable::Response> res);

    // 辅助函数
    bool setRobotEnable(bool enable, int32_t& error_code, std::string& message);

    // 参数
    std::string robot_control_topic_;  // 机器人控制话题名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_ROBOT_ENABLE_SERVER_H_
