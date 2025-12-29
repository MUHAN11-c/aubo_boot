/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_
#define DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>
#include <demo_interface/RobotStatus.h>
#include <aubo_msgs/GetFK.h>
#include <string>
#include <vector>

namespace demo_driver
{

/**
 * @brief 机器人状态发布器类
 * 用于收集机器人状态信息并发布到 /robot_status 话题
 */
class RobotStatusPublisher
{
public:
    RobotStatusPublisher();  // 构造函数
    ~RobotStatusPublisher(); // 析构函数

    void spin();  // 主循环，保持节点运行

private:
    // ROS 节点句柄
    ros::NodeHandle nh_;           // 公共节点句柄
    ros::NodeHandle private_nh_;  // 私有节点句柄（用于参数）

    // 发布器
    ros::Publisher robot_status_pub_;  // 发布机器人状态消息

    // 订阅器
    ros::Subscriber joint_states_sub_;              // 订阅关节状态
    ros::Subscriber industrial_robot_status_sub_;    // 订阅工业机器人状态
    ros::Subscriber trajectory_execution_sub_;      // 订阅轨迹执行事件

    // 服务客户端
    ros::ServiceClient fk_client_;  // 正运动学计算服务客户端

    // 回调函数
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);  // 关节状态回调
    void industrialRobotStatusCallback(const industrial_msgs::RobotStatus::ConstPtr& msg);  // 工业机器人状态回调
    void trajectoryExecutionCallback(const std_msgs::String::ConstPtr& msg);  // 轨迹执行事件回调

    // 辅助函数
    bool getForwardKinematics(const std::vector<double>& joint_positions, 
                              geometry_msgs::Pose& cartesian_pose);  // 计算正运动学（获取笛卡尔位置）
    std::string getPlanningStatus();  // 获取规划状态
    void publishRobotStatus();  // 发布机器人状态消息

    // 成员变量
    sensor_msgs::JointState current_joint_states_;           // 当前关节状态
    industrial_msgs::RobotStatus current_industrial_status_; // 当前工业机器人状态
    bool joint_states_received_;      // 是否已收到关节状态
    bool industrial_status_received_; // 是否已收到工业机器人状态
    std::string planning_status_;     // 规划状态（idle/planning/executing/error）
    ros::Timer status_timer_;         // 状态发布定时器
    
    // 定时器回调
    void statusTimerCallback(const ros::TimerEvent& event);  // 定时器回调，定期发布状态

    // 参数
    double publish_rate_;              // 发布频率（Hz）
    std::string base_frame_;           // 基础坐标系名称
    std::string planning_group_name_;  // 规划组名称
};

} // namespace demo_driver

#endif // DEMO_DRIVER_ROBOT_STATUS_PUBLISHER_H_

