/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/plan_trajectory_server.h"
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/GetIK.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <chrono>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
PlanTrajectoryServer::PlanTrajectoryServer()
    : nh_()
    , private_nh_("~")
    , move_group_(nullptr)
    , planning_scene_interface_(nullptr)
    , planning_group_name_("manipulator_e5")
    , base_frame_("base_link")
    , end_effector_link_("")
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
        
        // 获取末端执行器链接名称
        end_effector_link_ = move_group_->getEndEffectorLink();
        ROS_INFO("End effector link: %s", end_effector_link_.c_str());
        
        // 设置规划时间
        move_group_->setPlanningTime(10.0);
        
        ROS_INFO("MoveIt interfaces initialized successfully");
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to initialize MoveIt interfaces: %s", e.what());
        throw;
    }

    // 初始化服务服务器
    plan_trajectory_service_ = nh_.advertiseService("/plan_trajectory",
                                                     &PlanTrajectoryServer::planTrajectoryCallback, this);

    ROS_INFO("PlanTrajectoryServer initialized, service '/plan_trajectory' is ready");
}

PlanTrajectoryServer::~PlanTrajectoryServer()
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
bool PlanTrajectoryServer::planTrajectoryCallback(demo_interface::PlanTrajectory::Request& req,
                                                  demo_interface::PlanTrajectory::Response& res)
{
    ROS_INFO("Received plan_trajectory request");
    ROS_INFO("Target pose: x=%.3f, y=%.3f, z=%.3f", 
             req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z);
    ROS_INFO("Use joints: %s", req.use_joints ? "true" : "false");

    // 执行规划
    float planning_time = 0.0;
    std::string message;
    bool success = planTrajectory(req.target_pose, req.use_joints,
                                   res.trajectory, planning_time, message);

    res.success = success;
    res.planning_time = planning_time;
    res.message = message;

    if (success)
    {
        ROS_INFO("Trajectory planning succeeded: %s (time: %.3f s)", message.c_str(), planning_time);
    }
    else
    {
        ROS_WARN("Trajectory planning failed: %s", message.c_str());
    }

    return true;
}

/**
 * @brief 执行轨迹规划
 * @param target_pose 目标位姿
 * @param use_joints 是否使用关节空间规划（true=关节空间，false=笛卡尔空间）
 * @param trajectory 输出的轨迹
 * @param planning_time 输出的规划时间
 * @param message 输出消息
 * @return 成功返回true
 */
bool PlanTrajectoryServer::planTrajectory(const geometry_msgs::Pose& target_pose,
                                         bool use_joints,
                                         trajectory_msgs::JointTrajectory& trajectory,
                                         float& planning_time,
                                         std::string& message)
{
    if (!move_group_)
    {
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 记录规划开始时间
        auto start_time = std::chrono::high_resolution_clock::now();

        if (use_joints)
        {
            // 关节空间规划：需要先将目标位姿转换为关节角度
            ROS_INFO("Planning in joint space");
            
            // 获取当前关节状态
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            std::vector<double> joint_group_positions;
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup(planning_group_name_);
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            // 使用 IK 服务计算目标关节角度
            ros::ServiceClient ik_client = nh_.serviceClient<aubo_msgs::GetIK>("/aubo_driver/get_ik");
            
            if (ik_client.waitForExistence(ros::Duration(1.0)))
            {
                aubo_msgs::GetIK srv;
                // 使用当前关节位置作为参考
                for (size_t i = 0; i < 6 && i < joint_group_positions.size(); ++i)
                {
                    srv.request.ref_joint.push_back(static_cast<float>(joint_group_positions[i]));
                }
                srv.request.pos.push_back(target_pose.position.x);
                srv.request.pos.push_back(target_pose.position.y);
                srv.request.pos.push_back(target_pose.position.z);
                srv.request.ori.push_back(target_pose.orientation.w);
                srv.request.ori.push_back(target_pose.orientation.x);
                srv.request.ori.push_back(target_pose.orientation.y);
                srv.request.ori.push_back(target_pose.orientation.z);

                if (ik_client.call(srv) && srv.response.joint.size() >= 6)
                {
                    // 使用 IK 计算得到的关节角度
                    std::vector<double> target_joints;
                    for (size_t i = 0; i < 6; ++i)
                    {
                        target_joints.push_back(srv.response.joint[i]);
                    }
                    move_group_->setJointValueTarget(target_joints);
                }
                else
                {
                    message = "Failed to compute IK for target pose";
                    return false;
                }
            }
            else
            {
                // 如果没有 IK 服务，使用 MoveIt 内置的 IK 求解器
                move_group_->setPoseTarget(target_pose);
                ROS_INFO("IK service not available, using MoveIt's built-in IK solver");
            }
        }
        else
        {
            // 笛卡尔空间规划：直接设置目标位姿
            ROS_INFO("Planning in Cartesian space");
            move_group_->setPoseTarget(target_pose);
        }

        // 执行规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_->plan(my_plan);

        // 计算规划时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        planning_time = duration.count() / 1000.0f;

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            return false;
        }

        // 将 MoveIt 轨迹转换为 trajectory_msgs::JointTrajectory
        if (my_plan.trajectory_.joint_trajectory.joint_names.empty())
        {
            message = "Planned trajectory is empty";
            return false;
        }

        // 复制轨迹信息
        trajectory = my_plan.trajectory_.joint_trajectory;

        // 成功
        message = "Successfully planned trajectory with " + 
                  std::to_string(trajectory.points.size()) + " waypoints";
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
void PlanTrajectoryServer::spin()
{
    ros::waitForShutdown();
}

} // namespace demo_driver

