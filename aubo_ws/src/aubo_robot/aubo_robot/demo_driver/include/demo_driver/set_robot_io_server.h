/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_

#include <ros/ros.h>
#include <demo_interface/SetRobotIO.h>
#include <aubo_msgs/SetIO.h>
#include <string>

namespace demo_driver
{

/**
 * @brief 设置机器人IO服务服务器类
 * 提供设置机器人IO状态的服务，支持数字输入/输出、模拟输入/输出和工具IO
 */
class SetRobotIOServer
{
public:
    SetRobotIOServer();  // 构造函数
    ~SetRobotIOServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // 服务客户端
    ros::ServiceClient aubo_set_io_client_;  // aubo_driver的IO设置服务客户端

    // 服务服务器
    ros::ServiceServer set_robot_io_service_;  // 设置机器人IO服务

    // 服务回调函数
    bool setRobotIOCallback(demo_interface::SetRobotIO::Request& req,
                           demo_interface::SetRobotIO::Response& res);

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

