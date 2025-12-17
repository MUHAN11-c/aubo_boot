/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <ros/ros.h>
#include "demo_driver/set_robot_io_server.h"

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动设置机器人IO服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "set_robot_io_server_node");

    try
    {
        // 创建设置机器人IO服务服务器对象
        demo_driver::SetRobotIOServer server;
        ROS_INFO("Set Robot IO Server node started");
        // 进入主循环
        server.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        ROS_ERROR("Exception in set_robot_io_server_node: %s", e.what());
        return 1;
    }

    return 0;
}

