/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/execute_trajectory_server.h"
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
ExecuteTrajectoryServer::ExecuteTrajectoryServer()
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
    execute_trajectory_service_ = nh_.advertiseService("/execute_trajectory",
                                                       &ExecuteTrajectoryServer::executeTrajectoryCallback, this);

    ROS_INFO("ExecuteTrajectoryServer initialized, service '/execute_trajectory' is ready");
}

ExecuteTrajectoryServer::~ExecuteTrajectoryServer()
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
bool ExecuteTrajectoryServer::executeTrajectoryCallback(demo_interface::ExecuteTrajectory::Request& req,
                                                        demo_interface::ExecuteTrajectory::Response& res)
{
    ROS_INFO("Received execute_trajectory request");
    ROS_INFO("Trajectory has %zu waypoints", req.trajectory.points.size());

    // 验证轨迹
    if (req.trajectory.points.empty())
    {
        res.success = false;
        res.error_code = -1;
        res.message = "Trajectory is empty";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    if (req.trajectory.joint_names.empty())
    {
        res.success = false;
        res.error_code = -2;
        res.message = "Trajectory joint names are empty";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    // 执行轨迹
    int32_t error_code = 0;
    std::string message;
    bool success = executeTrajectory(req.trajectory, error_code, message);

    res.success = success;
    res.error_code = error_code;
    res.message = message;

    if (success)
    {
        ROS_INFO("Trajectory execution succeeded: %s", message.c_str());
    }
    else
    {
        ROS_WARN("Trajectory execution failed: %s (error_code: %d)", message.c_str(), error_code);
    }

    return true;
}

/**
 * @brief 执行轨迹
 * @param trajectory 要执行的轨迹
 * @param error_code 输出错误代码
 * @param message 输出消息
 * @return 成功返回true
 */
bool ExecuteTrajectoryServer::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                                int32_t& error_code,
                                                std::string& message)
{
    if (!move_group_)
    {
        error_code = -100;
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 将 trajectory_msgs::JointTrajectory 转换为 MoveIt 的 RobotTrajectory
        moveit_msgs::RobotTrajectory robot_trajectory_msg;
        robot_trajectory_msg.joint_trajectory = trajectory;

        // 创建 MoveIt 轨迹对象
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = robot_trajectory_msg;

        // 验证轨迹点数量
        if (plan.trajectory_.joint_trajectory.points.empty())
        {
            error_code = -101;
            message = "Converted trajectory is empty";
            return false;
        }

        ROS_INFO("Executing trajectory with %zu waypoints", 
                 plan.trajectory_.joint_trajectory.points.size());

        // 执行轨迹
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(plan);

        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            return false;
        }

        // 成功
        error_code = 0;
        message = "Successfully executed trajectory with " + 
                  std::to_string(plan.trajectory_.joint_trajectory.points.size()) + " waypoints";
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = -200;
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
void ExecuteTrajectoryServer::spin()
{
    ros::waitForShutdown();
}

} // namespace demo_driver

