/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include "demo_driver/robot_status_publisher.h"

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动机器人状态发布器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);

    try
    {
        // 创建机器人状态发布器对象
        auto publisher = std::make_shared<demo_driver::RobotStatusPublisher>();
        RCLCPP_INFO(publisher->get_logger(), "Robot Status Publisher node started");
        // 进入主循环
        rclcpp::spin(publisher);
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("robot_status_publisher_node"), 
                    "Exception in robot_status_publisher_node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
