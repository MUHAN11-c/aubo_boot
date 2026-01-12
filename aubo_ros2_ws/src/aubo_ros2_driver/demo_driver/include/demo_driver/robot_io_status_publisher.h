/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_
#define DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <aubo_msgs/msg/io_states.hpp>
#include <demo_interface/msg/robot_io_status.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 机器人IO状态发布器类
 * 用于订阅 aubo_driver 的IO状态并发布到 /robot_io_status 话题
 */
class RobotIOStatusPublisher : public rclcpp::Node
{
public:
    RobotIOStatusPublisher();  // 构造函数
    ~RobotIOStatusPublisher(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // 发布器
    rclcpp::Publisher<demo_interface::msg::RobotIOStatus>::SharedPtr robot_io_status_pub_;  // 发布机器人IO状态消息

    // 订阅器
    rclcpp::Subscription<aubo_msgs::msg::IOStates>::SharedPtr io_states_sub_;  // 订阅 aubo_driver 的IO状态

    // 回调函数
    void ioStatesCallback(const aubo_msgs::msg::IOStates::SharedPtr msg);

    // 辅助函数
    void publishIOStatus(const aubo_msgs::msg::IOStates& io_state);

    // 参数
    std::string io_states_topic_;  // IO状态话题名称
    std::string base_frame_;       // 基础坐标系名称
    bool is_connected_;           // IO接口连接状态
};

} // namespace demo_driver

#endif // DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_
