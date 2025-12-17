/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <ros/ros.h>
#include "demo_driver/robot_status_publisher.h"

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动机器人状态发布器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "robot_status_publisher_node");

    try
    {
        // 创建机器人状态发布器对象
        demo_driver::RobotStatusPublisher publisher;
        ROS_INFO("Robot Status Publisher node started");
        // 进入主循环
        publisher.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        ROS_ERROR("Exception in robot_status_publisher_node: %s", e.what());
        return 1;
    }

    return 0;
}

