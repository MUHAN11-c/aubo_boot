/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_speed_factor_server.h"
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
SetSpeedFactorServer::SetSpeedFactorServer()
    : nh_()
    , private_nh_("~")
    , move_group_(nullptr)
    , planning_scene_interface_(nullptr)
    , planning_group_name_("manipulator_e5")
    , base_frame_("base_link")
{
    // 获取参数
    private_nh_.param("planning_group_name", planning_group_name_, std::string("manipulator_e5"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));

    // 初始化 MoveIt 接口（需要在单独的线程中，因为 MoveIt 需要异步spinner）
    ROS_INFO("Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
    
    try
    {
        move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name_);
        planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
        
        // 设置参考坐标系
        move_group_->setPoseReferenceFrame(base_frame_);
        
        ROS_INFO("MoveIt interfaces initialized successfully");
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to initialize MoveIt interfaces: %s", e.what());
        throw;
    }

    // 初始化服务服务器
    set_speed_factor_service_ = nh_.advertiseService("/set_speed_factor",
                                                     &SetSpeedFactorServer::setSpeedFactorCallback, this);

    ROS_INFO("SetSpeedFactorServer initialized, service '/set_speed_factor' is ready");
}

SetSpeedFactorServer::~SetSpeedFactorServer()
{
    if (move_group_)
    {
        delete move_group_;
    }
    if (planning_scene_interface_)
    {
        delete planning_scene_interface_;
    }
}

/**
 * @brief 服务回调函数
 * @param req 服务请求
 * @param res 服务响应
 * @return 成功返回true
 */
bool SetSpeedFactorServer::setSpeedFactorCallback(demo_interface::SetSpeedFactor::Request& req,
                                                  demo_interface::SetSpeedFactor::Response& res)
{
    ROS_INFO("Received set_speed_factor request: velocity_factor = %.2f", req.velocity_factor);

    // 验证输入参数
    if (req.velocity_factor < 0.0 || req.velocity_factor > 1.0)
    {
        res.success = false;
        res.message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    // 设置速度因子
    std::string message;
    bool success = setSpeedFactor(req.velocity_factor, message);

    res.success = success;
    res.message = message;

    if (success)
    {
        ROS_INFO("Set speed factor succeeded: %s", message.c_str());
    }
    else
    {
        ROS_WARN("Set speed factor failed: %s", message.c_str());
    }

    return true;
}

/**
 * @brief 设置速度因子
 * @param velocity_factor 速度缩放因子（0.0-1.0）
 * @param message 输出消息
 * @return 成功返回true
 */
bool SetSpeedFactorServer::setSpeedFactor(float velocity_factor, std::string& message)
{
    if (!move_group_)
    {
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 设置速度缩放因子
        // 注意：某些 MoveIt 版本可能没有 getMaxVelocityScalingFactor() 方法
        // 如果 setMaxVelocityScalingFactor() 没有抛出异常，则认为设置成功
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        
        message = "Successfully set velocity factor to " + std::to_string(velocity_factor);
        ROS_INFO("Velocity factor set to %.2f", velocity_factor);
        return true;
    }
    catch (const std::exception& e)
    {
        message = std::string("Exception occurred: ") + e.what();
        ROS_ERROR("%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 * 注意：不使用 ros::spin()，因为已经在 main 中启动了 AsyncSpinner
 * 使用 ros::waitForShutdown() 等待关闭信号
 */
void SetSpeedFactorServer::spin()
{
    ros::waitForShutdown();
}

} // namespace demo_driver

