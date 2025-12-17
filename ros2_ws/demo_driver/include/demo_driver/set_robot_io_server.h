/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/set_robot_io.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <string>
#include <memory>

namespace demo_driver
{

/**
 * @brief 设置机器人IO服务服务器类
 * 提供设置机器人IO状态的服务，支持数字输入/输出、模拟输入/输出和工具IO
 */
class SetRobotIOServer : public rclcpp::Node
{
public:
    SetRobotIOServer();  // 构造函数
    ~SetRobotIOServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // 服务客户端
    rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr aubo_set_io_client_;  // aubo_driver的IO设置服务客户端

    // 服务服务器
    rclcpp::Service<demo_interface::srv::SetRobotIO>::SharedPtr set_robot_io_service_;  // 设置机器人IO服务

    // 服务回调函数
    void setRobotIOCallback(
        const std::shared_ptr<demo_interface::srv::SetRobotIO::Request> req,
        std::shared_ptr<demo_interface::srv::SetRobotIO::Response> res);

    // 辅助函数
    bool setRobotIO(const std::string& io_type,
                   int32_t io_index,
                   double value,
                   int32_t& error_code,
                   std::string& message);

    // 参数
    std::string aubo_set_io_service_name_;  // aubo_driver的IO设置服务名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_
