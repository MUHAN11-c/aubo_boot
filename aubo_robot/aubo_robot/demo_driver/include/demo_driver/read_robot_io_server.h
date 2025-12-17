/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_

#include <ros/ros.h>
#include <aubo_msgs/IOState.h>
#include <demo_interface/ReadRobotIO.h>
#include <string>
#include <mutex>

namespace demo_driver
{

/**
 * @brief 读取机器人IO服务服务器类
 * 提供读取机器人IO状态的服务，支持数字输入/输出、模拟输入/输出和工具IO
 */
class ReadRobotIOServer
{
public:
    ReadRobotIOServer();  // 构造函数
    ~ReadRobotIOServer(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // 订阅器
    ros::Subscriber io_states_sub_;  // 订阅 aubo_driver 的IO状态

    // 服务服务器
    ros::ServiceServer read_robot_io_service_;  // 读取机器人IO服务

    // 回调函数
    void ioStatesCallback(const aubo_msgs::IOState::ConstPtr& msg);

    // 服务回调函数
    bool readRobotIOCallback(demo_interface::ReadRobotIO::Request& req,
                            demo_interface::ReadRobotIO::Response& res);

    // 辅助函数
    bool readRobotIO(const std::string& io_type,
                    int32_t io_index,
                    double& value,
                    std::string& message);

    // 成员变量
    aubo_msgs::IOState current_io_state_;  // 当前IO状态
    bool io_state_received_;                // 是否已收到IO状态
    std::mutex io_state_mutex_;             // IO状态互斥锁

    // 参数
    std::string io_states_topic_;  // IO状态话题名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_READ_ROBOT_IO_SERVER_H_

