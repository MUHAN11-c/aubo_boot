/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/msg/io_states.hpp>
#include <ivg_interfaces/srv/read_robot_io.hpp>
#include <string>
#include <mutex>
#include <memory>

namespace demo_driver
{

/**
 * @brief 读取机器人IO服务服务器类
 * 提供读取机器人IO状态的服务，支持数字输入/输出、模拟输入/输出和工具IO
 */
class ReadRobotIOServer : public rclcpp::Node
{
public:
    ReadRobotIOServer();  // 构造函数
    ~ReadRobotIOServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // 订阅器
    rclcpp::Subscription<ivg_interfaces::msg::IOStates>::SharedPtr io_states_sub_;  // 订阅 aubo_driver 的IO状态

    // 服务服务器
    rclcpp::Service<ivg_interfaces::srv::ReadRobotIO>::SharedPtr read_robot_io_service_;  // 读取机器人IO服务

    // 回调函数
    void ioStatesCallback(const ivg_interfaces::msg::IOStates::SharedPtr msg);

    // 服务回调函数
    void readRobotIOCallback(
        const std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Request> req,
        std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Response> res);

    // 辅助函数
    bool readRobotIO(const std::string& io_type,
                    int32_t io_index,
                    double& value,
                    std::string& message);

    // 成员变量
    ivg_interfaces::msg::IOStates current_io_state_;  // 当前IO状态
    bool io_state_received_;                // 是否已收到IO状态
    std::mutex io_state_mutex_;             // IO状态互斥锁

    // 参数
    std::string io_states_topic_;  // IO状态话题名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_
