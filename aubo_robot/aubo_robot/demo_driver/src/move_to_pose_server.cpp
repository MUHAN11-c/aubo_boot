/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/move_to_pose_server.h"
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/GetIK.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <map>
#include <string>
#include <cmath>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
MoveToPoseServer::MoveToPoseServer()
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
    // 注意：MoveIt 接口必须在 AsyncSpinner 启动后初始化
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
    move_to_pose_service_ = nh_.advertiseService("/move_to_pose",
                                                 &MoveToPoseServer::moveToPoseCallback, this);

    ROS_INFO("MoveToPoseServer initialized, service '/move_to_pose' is ready");
}

MoveToPoseServer::~MoveToPoseServer()
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
bool MoveToPoseServer::moveToPoseCallback(demo_interface::MoveToPose::Request& req,
                                         demo_interface::MoveToPose::Response& res)
{
    ROS_INFO("Received move_to_pose request");
    ROS_INFO("Target pose: x=%.3f, y=%.3f, z=%.3f", 
             req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z);
    ROS_INFO("Use joints: %s", req.use_joints ? "true" : "false");
    ROS_INFO("Velocity factor: %.2f, Acceleration factor: %.2f", 
             req.velocity_factor, req.acceleration_factor);

    // 验证输入参数
    if (req.velocity_factor < 0.0 || req.velocity_factor > 1.0)
    {
        res.success = false;
        res.error_code = static_cast<int32_t>(MoveToPoseErrorCode::INVALID_VELOCITY_FACTOR);
        res.message = "Invalid velocity_factor, must be between 0.0 and 1.0";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    if (req.acceleration_factor < 0.0 || req.acceleration_factor > 1.0)
    {
        res.success = false;
        res.error_code = static_cast<int32_t>(MoveToPoseErrorCode::INVALID_ACCELERATION_FACTOR);
        res.message = "Invalid acceleration_factor, must be between 0.0 and 1.0";
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    // 执行运动
    int32_t error_code = 0;
    std::string message;
    bool success = moveToPose(req.target_pose, req.use_joints, 
                              req.velocity_factor, req.acceleration_factor,
                              error_code, message);

    res.success = success;
    res.error_code = error_code;
    res.message = message;

    if (success)
    {
        ROS_INFO("Move to pose succeeded: %s", message.c_str());
    }
    else
    {
        ROS_WARN("Move to pose failed: %s (error_code: %d)", message.c_str(), error_code);
    }

    return true;
}

/**
 * @brief 执行移动到目标位姿
 * @param target_pose 目标位姿
 * @param use_joints 是否使用关节空间规划（true=关节空间，false=笛卡尔空间）
 * @param velocity_factor 速度缩放因子
 * @param acceleration_factor 加速度缩放因子
 * @param error_code 输出错误代码
 * @param message 输出消息
 * @return 成功返回true
 */
bool MoveToPoseServer::moveToPose(const geometry_msgs::Pose& target_pose,
                                  bool use_joints,
                                  float velocity_factor,
                                  float acceleration_factor,
                                  int32_t& error_code,
                                  std::string& message)
{
    if (!move_group_)
    {
        error_code = static_cast<int32_t>(MoveToPoseErrorCode::MOVEIT_NOT_INITIALIZED);
        message = "MoveIt interface not initialized";
        return false;
    }

    try
    {
        // 如果速度和加速度因子为 0 或未设置，使用较低的默认值（降低速度）
        // 默认值 0.1 表示最大速度的 10%，适合安全测试和精细操作
        if (velocity_factor <= 0.0)
        {
            velocity_factor = 0.1;  // 默认速度因子 10%（较慢，更安全）
            ROS_INFO("Velocity factor is 0 or invalid, using default value 0.1 (10%%)");
        }
        if (acceleration_factor <= 0.0)
        {
            acceleration_factor = 0.1;  // 默认加速度因子 10%（启动和停止更平滑）
            ROS_INFO("Acceleration factor is 0 or invalid, using default value 0.1 (10%%)");
        }
        
        // 设置速度和加速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

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
            // 注意：这里简化处理，实际应该调用 IK 服务
            // 或者使用 MoveIt 的 IK 求解器
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
                    error_code = static_cast<int32_t>(MoveToPoseErrorCode::IK_COMPUTATION_FAILED);
                    message = "Failed to compute IK for target pose";
                    return false;
                }
            }
            else
            {
                // 如果没有 IK 服务，使用 MoveIt 内置的 IK 求解器
                // 设置目标位姿，MoveIt 会自动计算 IK 并转换为关节空间
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

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(plan_result.val);
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            return false;
        }

        ROS_INFO("Planning succeeded, checking trajectory for overspeed");

        // 检查轨迹是否超速
        if (!checkTrajectoryOverspeed(my_plan, error_code, message))
        {
            ROS_WARN("Trajectory overspeed detected, execution aborted: %s", message.c_str());
            return false;
        }

        ROS_INFO("Trajectory speed check passed, executing trajectory");

        // 执行运动
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(my_plan);

        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            return false;
        }

        // 成功
        error_code = static_cast<int32_t>(MoveToPoseErrorCode::SUCCESS);
        message = "Successfully moved to target pose";
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = static_cast<int32_t>(MoveToPoseErrorCode::EXCEPTION_OCCURRED);
        message = std::string("Exception occurred: ") + e.what();
        ROS_ERROR("%s", message.c_str());
        return false;
    }
}

/**
 * @brief 检查轨迹是否超速
 * 使用与 aubo_driver 相同的逻辑：速度 = |Δjoint| / 0.005
 * 
 * @param plan MoveIt 规划结果
 * @param error_code 输出错误代码
 * @param message 输出错误消息
 * @return 如果轨迹超速返回 false，否则返回 true
 */
bool MoveToPoseServer::checkTrajectoryOverspeed(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                                int32_t& error_code,
                                                std::string& message)
{
    // 定义关节最大速度限制（单位：rad/s），与 joint_limits.yaml 保持一致
    std::map<std::string, double> max_velocities;
    max_velocities["foreArm_joint"] = 3.14;
    max_velocities["shoulder_joint"] = 3.14;
    max_velocities["upperArm_joint"] = 3.14;
    max_velocities["wrist1_joint"] = 2.6;
    max_velocities["wrist2_joint"] = 2.6;
    max_velocities["wrist3_joint"] = 2.6;

    // 固定时间间隔（秒），与 aubo_driver 保持一致
    const double TIME_INTERVAL = 0.005;

    // 检查轨迹是否有效
    if (plan.trajectory_.joint_trajectory.points.empty())
    {
        error_code = static_cast<int32_t>(MoveToPoseErrorCode::TRAJECTORY_EMPTY);
        message = "Trajectory is empty";
        return false;
    }

    if (plan.trajectory_.joint_trajectory.joint_names.empty())
    {
        error_code = static_cast<int32_t>(MoveToPoseErrorCode::TRAJECTORY_JOINT_NAMES_EMPTY);
        message = "Trajectory joint names are empty";
        return false;
    }

    const auto& trajectory = plan.trajectory_.joint_trajectory;
    const auto& joint_names = trajectory.joint_names;
    const auto& points = trajectory.points;

    // 如果只有一个点，不需要检查速度
    if (points.size() < 2)
    {
        ROS_INFO("Trajectory has only one point, skipping overspeed check");
        return true;
    }

    // 遍历相邻点，检查速度
    for (size_t i = 1; i < points.size(); ++i)
    {
        const auto& prev_point = points[i - 1];
        const auto& curr_point = points[i];

        // 检查点的关节数量是否匹配
        if (prev_point.positions.size() != joint_names.size() || 
            curr_point.positions.size() != joint_names.size())
        {
            error_code = static_cast<int32_t>(MoveToPoseErrorCode::TRAJECTORY_POINT_SIZE_MISMATCH);
            message = "Trajectory point size mismatch";
            return false;
        }

        // 检查每个关节的速度
        for (size_t j = 0; j < joint_names.size(); ++j)
        {
            const std::string& joint_name = joint_names[j];
            
            // 计算关节角度变化
            double joint_delta = fabs(curr_point.positions[j] - prev_point.positions[j]);
            
            // 使用与 aubo_driver 相同的逻辑计算速度：速度 = |Δjoint| / 0.005
            double calculated_velocity = joint_delta / TIME_INTERVAL;

            // 查找该关节的最大速度限制
            auto it = max_velocities.find(joint_name);
            if (it != max_velocities.end())
            {
                double max_velocity = it->second;
                
                // 如果计算的速度超过限制
                if (calculated_velocity > max_velocity)
                {
                    error_code = static_cast<int32_t>(MoveToPoseErrorCode::TRAJECTORY_OVERSPEED);
                    message = "Joint '" + joint_name + "' overspeed detected: " +
                              "calculated velocity = " + std::to_string(calculated_velocity) + " rad/s, " +
                              "max velocity = " + std::to_string(max_velocity) + " rad/s, " +
                              "joint delta = " + std::to_string(joint_delta) + " rad, " +
                              "at trajectory point " + std::to_string(i);
                    
                    ROS_ERROR("Overspeed check failed: %s", message.c_str());
                    ROS_ERROR("  Joint %zu (%s): prev=%.6f, curr=%.6f, delta=%.6f, vel=%.6f, max=%.6f",
                              j, joint_name.c_str(),
                              prev_point.positions[j], curr_point.positions[j],
                              joint_delta, calculated_velocity, max_velocity);
                    
                    return false;
                }
            }
            else
            {
                // 如果关节名称不在限制列表中，记录警告但继续检查
                ROS_WARN("Joint '%s' not found in velocity limits map, skipping check", joint_name.c_str());
            }
        }
    }

    ROS_INFO("Trajectory overspeed check passed: %zu points checked", points.size());
    return true;
}

/**
 * @brief 主循环，保持节点运行
 * 注意：不使用 ros::spin()，因为已经在 main 中启动了 AsyncSpinner
 * 使用 ros::waitForShutdown() 等待关闭信号
 */
void MoveToPoseServer::spin()
{
    ros::waitForShutdown();
}

} // namespace demo_driver

