/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include "demo_driver/execute_trajectory_server.h"
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动执行轨迹服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建执行轨迹服务服务器对象
        auto server = std::make_shared<demo_driver::ExecuteTrajectoryServer>();
        RCLCPP_INFO(server->get_logger(), "Execute Trajectory Server node started");
        
        executor.add_node(server);
        // 进入主循环
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("execute_trajectory_server_node"), 
                    "Exception in execute_trajectory_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
