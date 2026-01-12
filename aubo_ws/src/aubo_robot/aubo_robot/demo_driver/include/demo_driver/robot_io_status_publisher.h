/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_
#define DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_

#include <ros/ros.h>
#include <aubo_msgs/IOState.h>
#include <demo_interface/RobotIOStatus.h>
#include <string>

namespace demo_driver
{

/**
 * @brief 机器人IO状态发布器类
 * 用于订阅 aubo_driver 的IO状态并发布到 /robot_io_status 话题
 */
class RobotIOStatusPublisher
{
public:
    RobotIOStatusPublisher();  // 构造函数
    ~RobotIOStatusPublisher(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // 发布器
    ros::Publisher robot_io_status_pub_;  // 发布机器人IO状态消息

    // 订阅器
    ros::Subscriber io_states_sub_;  // 订阅 aubo_driver 的IO状态

    // 回调函数
    void ioStatesCallback(const aubo_msgs::IOState::ConstPtr& msg);

    // 辅助函数
    void publishIOStatus(const aubo_msgs::IOState& io_state);

    // 参数
    std::string io_states_topic_;  // IO状态话题名称
    std::string base_frame_;       // 基础坐标系名称
    bool is_connected_;           // IO接口连接状态
};

} // namespace demo_driver

#endif // DEMO_DRIVER_ROBOT_IO_STATUS_PUBLISHER_H_

