/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include <ros/ros.h>
#include "demo_driver/get_current_state_server.h"
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动获取当前状态服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "get_current_state_server_node");
    
    // 启动异步spinner（MoveIt 需要）
    ros::AsyncSpinner spinner(2);
    spinner.start();

    try
    {
        // 创建获取当前状态服务服务器对象
        demo_driver::GetCurrentStateServer server;
        ROS_INFO("Get Current State Server node started");
        // 进入主循环
        server.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        ROS_ERROR("Exception in get_current_state_server_node: %s", e.what());
        return 1;
    }

    spinner.stop();
    return 0;
}

