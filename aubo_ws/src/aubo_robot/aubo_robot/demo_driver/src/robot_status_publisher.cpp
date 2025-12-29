/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/robot_status_publisher.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{

/**
 * @brief 构造函数，初始化发布器、订阅器和服务客户端
 */
RobotStatusPublisher::RobotStatusPublisher()
    : nh_()
    , private_nh_("~")
    , joint_states_received_(false)
    , industrial_status_received_(false)
    , planning_status_("idle")
    , publish_rate_(10.0)
    , base_frame_("base_link")
    , planning_group_name_("manipulator_e5")
{
    // 获取参数
    private_nh_.param("publish_rate", publish_rate_, 10.0);  // 发布频率
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));  // 基础坐标系
    private_nh_.param("planning_group_name", planning_group_name_, std::string("manipulator_e5"));  // 规划组名称
    
    // 获取发布话题名称（默认为 /demo_robot_status，避免与 aubo_driver 的 /robot_status 冲突）
    std::string robot_status_topic = "/demo_robot_status";
    private_nh_.param("robot_status_topic", robot_status_topic, std::string("/demo_robot_status"));

    // 初始化发布器
    robot_status_pub_ = nh_.advertise<demo_interface::RobotStatus>(robot_status_topic, 10);

    // 初始化订阅器
    joint_states_sub_ = nh_.subscribe("joint_states", 10, 
                                      &RobotStatusPublisher::jointStatesCallback, this);  // 订阅关节状态
    industrial_robot_status_sub_ = nh_.subscribe("robot_status", 10,
                                                  &RobotStatusPublisher::industrialRobotStatusCallback, this);  // 订阅工业机器人状态
    trajectory_execution_sub_ = nh_.subscribe("trajectory_execution_event", 10,
                                               &RobotStatusPublisher::trajectoryExecutionCallback, this);  // 订阅轨迹执行事件

    // 初始化服务客户端
    fk_client_ = nh_.serviceClient<aubo_msgs::GetFK>("/aubo_driver/get_fk");  // 正运动学服务

    // 等待服务可用
    if (!fk_client_.waitForExistence(ros::Duration(5.0)))
    {
        ROS_WARN("FK service not available, cartesian position may not be accurate");
    }

    // 初始化定时器，用于定期发布状态
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &RobotStatusPublisher::statusTimerCallback, this);

    ROS_INFO("RobotStatusPublisher initialized /demo_robot_status topic");
}

RobotStatusPublisher::~RobotStatusPublisher()
{
}

/**
 * @brief 关节状态回调函数
 * @param msg 关节状态消息
 */
void RobotStatusPublisher::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_states_ = *msg;  // 保存当前关节状态
    joint_states_received_ = true;  // 标记已收到关节状态
}

/**
 * @brief 工业机器人状态回调函数
 * @details 该函数接收来自 aubo_driver 发布的 robot_status 话题消息
 *          状态值在 aubo_driver.cpp 的 timerCallback 函数中设置（第185-191行）：
 *          - mode.val: 运行模式 (orpeStatus)
 *          - e_stopped.val: 急停状态 (softEmergency || emergency_stopped_)
 *          - drives_powered.val: 驱动器上电状态 (armPowerStatus)
 *          - motion_possible.val: 是否可以运动 (!start_move_)
 *          - in_motion.val: 是否在运动中 (start_move_)
 *          - in_error.val: 保护性停止状态 (protective_stopped_)
 *          - error_code: 错误代码 (singularityOverSpeedAlarm)
 * 
 *          注意：motion_possible 的逻辑是 motion_possible.val = (int)(!start_move_)
 *                - start_move_ = false → motion_possible = true (可以接受新的运动命令)
 *                - start_move_ = true  → motion_possible = false (正在运动，不能接受新命令)
 * @param msg 工业机器人状态消息（来自 /robot_status 话题）
 */
void RobotStatusPublisher::industrialRobotStatusCallback(const industrial_msgs::RobotStatus::ConstPtr& msg)
{
    current_industrial_status_ = *msg;  // 保存当前工业机器人状态
    industrial_status_received_ = true;  // 标记已收到状态

    // 根据工业机器人状态更新规划状态
    // 只有在轨迹执行回调未设置状态时才更新
    if (planning_status_ != "planning" && planning_status_ != "executing")
    {
        if (msg->in_motion.val == industrial_msgs::TriState::TRUE)
        {
            planning_status_ = "executing";  // 运动中
        }
        else if (msg->in_error.val == industrial_msgs::TriState::TRUE)
        {
            planning_status_ = "error";  // 错误状态
        }
        else
        {
            planning_status_ = "idle";  // 空闲状态
        }
    }
}

/**
 * @brief 轨迹执行事件回调函数
 * @param msg 轨迹执行事件消息
 */
void RobotStatusPublisher::trajectoryExecutionCallback(const std_msgs::String::ConstPtr& msg)
{
    // 根据轨迹执行事件更新规划状态
    std::string event = msg->data;
    if (event == "stop" || event == "cancel")
    {
        planning_status_ = "idle";  // 停止或取消
    }
    else if (event == "planning")
    {
        planning_status_ = "planning";  // 规划中
    }
    else if (event == "executing" || event == "execute")
    {
        planning_status_ = "executing";  // 执行中
    }
    // 其他事件可以在这里处理
}

/**
 * @brief 计算正运动学，获取笛卡尔位置
 * @param joint_positions 关节位置（弧度）
 * @param cartesian_pose 输出的笛卡尔位姿
 * @return 成功返回true，失败返回false
 */
bool RobotStatusPublisher::getForwardKinematics(const std::vector<double>& joint_positions,
                                                 geometry_msgs::Pose& cartesian_pose)
{
    if (joint_positions.size() < 6)
    {
        ROS_WARN("Insufficient joint positions for FK calculation");
        return false;
    }

    // 准备服务请求
    aubo_msgs::GetFK srv;
    srv.request.joint.resize(6);
    for (size_t i = 0; i < 6 && i < joint_positions.size(); ++i)
    {
        srv.request.joint[i] = static_cast<float>(joint_positions[i]);  // 转换为float
    }

    // 调用正运动学服务
    if (fk_client_.call(srv))
    {
        if (srv.response.pos.size() >= 3 && srv.response.ori.size() >= 4)
        {
            // 提取位置（x, y, z）
            cartesian_pose.position.x = srv.response.pos[0];
            cartesian_pose.position.y = srv.response.pos[1];
            cartesian_pose.position.z = srv.response.pos[2];
            // 提取姿态四元数（w, x, y, z）
            cartesian_pose.orientation.w = srv.response.ori[0];
            cartesian_pose.orientation.x = srv.response.ori[1];
            cartesian_pose.orientation.y = srv.response.ori[2];
            cartesian_pose.orientation.z = srv.response.ori[3];
            return true;
        }
    }
    else
    {
        ROS_DEBUG("Failed to call FK service");
    }

    return false;
}

/**
 * @brief 获取当前规划状态
 * @return 规划状态字符串（idle/planning/executing/error）
 */
std::string RobotStatusPublisher::getPlanningStatus()
{
    return planning_status_;
}

/**
 * @brief 发布机器人状态消息
 * 收集所有状态信息并发布到 /robot_status 话题
 */
void RobotStatusPublisher::publishRobotStatus()
{
    // 检查是否已收到关节状态
    if (!joint_states_received_)
    {
        ROS_DEBUG("Joint states not received yet");
        return;
    }

    demo_interface::RobotStatus status_msg;

    // 设置消息头
    status_msg.header.stamp = ros::Time::now();  // 当前时间戳
    status_msg.header.frame_id = base_frame_;    // 基础坐标系

    // 获取关节位置
    std::vector<double> joint_positions_rad;  // 弧度
    std::vector<double> joint_positions_deg;   // 度

    // 从关节状态中查找关节位置
    // 假设关节名称：shoulder_joint, upperArm_joint, foreArm_joint, 
    // wrist1_joint, wrist2_joint, wrist3_joint
    std::vector<std::string> joint_names = {
        "shoulder_joint", "upperArm_joint", "foreArm_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint"
    };

    joint_positions_rad.resize(6, 0.0);
    joint_positions_deg.resize(6, 0.0);

    // 根据关节名称匹配并提取位置
    for (size_t i = 0; i < joint_names.size() && i < 6; ++i)
    {
        for (size_t j = 0; j < current_joint_states_.name.size(); ++j)
        {
            if (current_joint_states_.name[j] == joint_names[i])
            {
                if (j < current_joint_states_.position.size())
                {
                    joint_positions_rad[i] = current_joint_states_.position[j];  // 弧度
                    joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;  // 转换为度
                }
                break;
            }
        }
    }

    // 如果关节名称不匹配，尝试使用前6个位置
    if (joint_positions_rad[0] == 0.0 && joint_positions_rad[1] == 0.0 && 
        current_joint_states_.position.size() >= 6)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            joint_positions_rad[i] = current_joint_states_.position[i];
            joint_positions_deg[i] = joint_positions_rad[i] * 180.0 / M_PI;
        }
    }

    // 设置关节位置（弧度和度）
    // 将 vector 转换为 boost::array
    for (size_t i = 0; i < 6 && i < joint_positions_rad.size(); ++i)
    {
        status_msg.joint_position_rad[i] = joint_positions_rad[i];
    }
    for (size_t i = 0; i < 6 && i < joint_positions_deg.size(); ++i)
    {
        status_msg.joint_position_deg[i] = joint_positions_deg[i];
    }

    // ========================================================================
    // 从工业机器人状态消息中获取机器人状态
    // 状态值在 aubo_driver.cpp 的 timerCallback 函数中设置（第185-191行）
    // ========================================================================
    if (industrial_status_received_)
    {
        // --------------------------------------------------------------------
        // is_online (bool): 机器人是否在线/连接
        // 映射自 aubo_driver.cpp:187 的 drives_powered.val
        // 源值: rs.robot_diagnosis_info_.armPowerStatus (驱动器上电状态)
        // 含义: drives_powered = TRUE 表示驱动器已上电，机器人已连接并在线
        //       使能操作（powerOn）会调用 rootServiceRobotStartup() 设置此状态
        // --------------------------------------------------------------------
        status_msg.is_online = (current_industrial_status_.drives_powered.val == 
                                industrial_msgs::TriState::TRUE);

        // --------------------------------------------------------------------
        // enable (bool): 机器人是否使能/可以接受运动命令
        // 映射自 aubo_driver.cpp:188 的 motion_possible.val
        // 源值: !start_move_ (当 start_move_ = false 时，motion_possible = true)
        // 含义: motion_possible = TRUE 表示机器人未在运动，可以接受新的运动命令
        //       注意：当机器人正在运动时(start_move_=true)，motion_possible=false
        //       使能后，start_move_ 初始为 false，所以 motion_possible = true
        //       逻辑：motion_possible.val = (int)(!start_move_)
        // --------------------------------------------------------------------
        status_msg.enable = (current_industrial_status_.motion_possible.val == 
                            industrial_msgs::TriState::TRUE);

        // --------------------------------------------------------------------
        // in_motion (bool): 机器人是否正在运动
        // 映射自 aubo_driver.cpp:189 的 in_motion.val
        // 源值: start_move_ (机器人是否已开始运动)
        // 含义: in_motion = TRUE 表示机器人当前正在执行运动
        //       当机器人开始运动时，start_move_ = true，同时 motion_possible = false
        // --------------------------------------------------------------------
        status_msg.in_motion = (current_industrial_status_.in_motion.val == 
                               industrial_msgs::TriState::TRUE);
    }
    else
    {
        // 如果工业机器人状态不可用（aubo_driver 未运行或未连接），使用默认值
        status_msg.is_online = false;  // 默认离线（更安全的假设）
        status_msg.enable = false;     // 默认未使能（更安全的假设）
        status_msg.in_motion = false;  // 默认未运动
    }

    // 获取规划状态
    status_msg.planning_status = getPlanningStatus();

    // 使用正运动学服务获取笛卡尔位置
    if (getForwardKinematics(joint_positions_rad, status_msg.cartesian_position))
    {
        // 成功获取笛卡尔位置
    }
    else
    {
        // 如果正运动学计算失败，设置默认位姿
        status_msg.cartesian_position.position.x = 0.0;
        status_msg.cartesian_position.position.y = 0.0;
        status_msg.cartesian_position.position.z = 0.0;
        status_msg.cartesian_position.orientation.w = 1.0;  // 单位四元数
        status_msg.cartesian_position.orientation.x = 0.0;
        status_msg.cartesian_position.orientation.y = 0.0;
        status_msg.cartesian_position.orientation.z = 0.0;
    }

    // 发布消息
    robot_status_pub_.publish(status_msg);
}

/**
 * @brief 定时器回调函数
 * @param event 定时器事件
 */
void RobotStatusPublisher::statusTimerCallback(const ros::TimerEvent& event)
{
    publishRobotStatus();  // 定期发布机器人状态
}

/**
 * @brief 主循环，保持节点运行
 */
void RobotStatusPublisher::spin()
{
    ros::spin();  // ROS 自旋，处理回调
}

} // namespace demo_driver

