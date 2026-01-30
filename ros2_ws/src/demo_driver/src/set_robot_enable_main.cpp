/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include "demo_driver/set_robot_enable_server.h"

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动设置机器人使能状态服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);

    try
    {
        // 创建设置机器人使能状态服务服务器对象
        auto server = std::make_shared<demo_driver::SetRobotEnableServer>();
        RCLCPP_INFO(server->get_logger(), "Set Robot Enable Server node started");
        // 进入主循环
        rclcpp::spin(server);
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("set_robot_enable_server_node"), 
                    "Exception in set_robot_enable_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
