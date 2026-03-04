/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/set_robot_pose_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <aubo_msgs/srv/get_ik.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <thread>
#include <chrono>
#include <rclcpp/parameter_client.hpp>
#include <cstdlib>
#include <string>
#include <cmath>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace demo_driver
{

SetRobotPoseServer::SetRobotPoseServer(const rclcpp::NodeOptions& options)
    : Node("set_robot_pose_server_node", options)
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    set_robot_pose_service_ = this->create_service<demo_interface::srv::SetRobotPose>(
        "/set_robot_pose",
        std::bind(&SetRobotPoseServer::setRobotPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool SetRobotPoseServer::wait_for_robot_description(int timeout_seconds)
{
    static bool warned_once = false;
    if (!this->has_parameter("robot_description") || !this->has_parameter("robot_description_semantic")) {
        if (!warned_once) {
            RCLCPP_WARN(this->get_logger(), "robot_description or robot_description_semantic parameter not found");
            warned_once = true;
        }
    }
    
    auto start_time = std::chrono::steady_clock::now();
    int check_count = 0;
    
    while (rclcpp::ok())
    {
        try
        {
            auto test_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            
            std::string test_frame = test_move_group->getPlanningFrame();
            
            if (!test_frame.empty())
            {
                return true;
            }
        }
        catch (const std::exception& e)
        {
            check_count++;
            if (check_count % 10 == 0)
            {
                auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                RCLCPP_INFO(this->get_logger(), "Still waiting... (elapsed %ld seconds, error: %s)",
                           elapsed_seconds, e.what());
            }
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting (%d seconds)", timeout_seconds);
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

bool SetRobotPoseServer::initialize(int max_retries, int retry_delay_seconds)
{
    if (!wait_for_robot_description(30))
    {
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    for (int attempt = 1; attempt <= max_retries; ++attempt)
    {
        try
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), planning_group_name_);
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            move_group_->setPoseReferenceFrame(base_frame_);
            end_effector_link_ = move_group_->getEndEffectorLink();
            move_group_->setPlanningTime(10.0);
            return true;
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
                std::this_thread::sleep_for(std::chrono::seconds(retry_delay_seconds));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interfaces after %d attempts", max_retries);
                return false;
            }
        }
    }
    
    return false;
}

SetRobotPoseServer::~SetRobotPoseServer()
{
}

void SetRobotPoseServer::setRobotPoseCallback(
    const std::shared_ptr<demo_interface::srv::SetRobotPose::Request> req,
    std::shared_ptr<demo_interface::srv::SetRobotPose::Response> res)
{
    // 验证输入参数
    if (req->target_pose.size() != 6)
    {
        res->success = false;
        res->error_code = -1;
        res->message = "target_pose must have exactly 6 elements [x, y, z, roll, pitch, yaw]";
        RCLCPP_WARN(this->get_logger(), "SetRobotPose response: %s", res->message.c_str());
        return;
    }

    if (req->velocity < 0.0 || req->velocity > 100.0)
    {
        res->success = false;
        res->error_code = -2;
        res->message = "velocity must be between 0.0 and 100.0";
        RCLCPP_WARN(this->get_logger(), "SetRobotPose response: %s", res->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SetRobotPose request: pose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f), use_joints=%s, is_radian=%s, velocity=%.2f", 
               req->target_pose[0], req->target_pose[1], req->target_pose[2],
               req->target_pose[3], req->target_pose[4], req->target_pose[5],
               req->use_joints ? "true" : "false",
               req->is_radian ? "true" : "false",
               req->velocity);

    int32_t error_code = 0;
    std::string message;
    bool success = setRobotPose(req->target_pose, req->use_joints, req->is_radian, 
                               req->velocity, error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;
    
    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "SetRobotPose response: success (velocity=%.2f%%)",
                   req->velocity);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "SetRobotPose response: failed (error_code: %d, velocity=%.2f%%, message: %s)", 
                   error_code, req->velocity, message.c_str());
    }
}

bool SetRobotPoseServer::setRobotPose(const std::array<double, 6>& target_pose, 
                                     bool use_joints,
                                     bool is_radian,
                                     float velocity,
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
        // 提取位置和姿态
        double x = target_pose[0];
        double y = target_pose[1];
        double z = target_pose[2];
        double roll = target_pose[3];
        double pitch = target_pose[4];
        double yaw = target_pose[5];

        // 如果角度单位是度，转换为弧度
        if (!is_radian)
        {
            roll = roll * M_PI / 180.0;
            pitch = pitch * M_PI / 180.0;
            yaw = yaw * M_PI / 180.0;
            RCLCPP_INFO(this->get_logger(), "Converted angles from degrees to radians: roll=%.6f, pitch=%.6f, yaw=%.6f",
                       roll, pitch, yaw);
        }

        // 将速度百分比 (0-100) 转换为速度因子 (0.0-1.0)
        float velocity_factor = velocity / 100.0f;
        float acceleration_factor = velocity_factor;  // 使用相同的值作为加速度因子

        // 限制在有效范围内
        if (velocity_factor < 0.0f) velocity_factor = 0.0f;
        if (velocity_factor > 1.0f) velocity_factor = 1.0f;

        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        move_group_->setMaxAccelerationScalingFactor(acceleration_factor);
        
        RCLCPP_INFO(this->get_logger(), "SetRobotPose: Setting velocity_factor=%.2f, acceleration_factor=%.2f (from velocity=%.2f%%)", 
                   velocity_factor, acceleration_factor, velocity);

        // 将欧拉角 (roll-pitch-yaw) 转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        // 创建目标位姿消息
        geometry_msgs::msg::Pose target_pose_msg;
        target_pose_msg.position.x = x;
        target_pose_msg.position.y = y;
        target_pose_msg.position.z = z;
        target_pose_msg.orientation.x = q.x();
        target_pose_msg.orientation.y = q.y();
        target_pose_msg.orientation.z = q.z();
        target_pose_msg.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "SetRobotPose: Target pose - position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                   x, y, z, q.x(), q.y(), q.z(), q.w());

        if (use_joints)
        {
            // 使用关节空间规划
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            
            moveit::core::RobotStatePtr robot_state;
            if (!current_state)
            {
                const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
                if (!robot_model)
                {
                    error_code = -100;
                    message = "Failed to get robot model from MoveIt";
                    return false;
                }
                robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
                robot_state->setToDefaultValues();
            }
            else
            {
                robot_state = current_state;
            }
            
            std::vector<double> joint_group_positions;
            const moveit::core::JointModelGroup* joint_model_group = 
                robot_state->getJointModelGroup(planning_group_name_);
            
            if (!joint_model_group)
            {
                error_code = -101;
                message = "Failed to get joint model group from MoveIt";
                return false;
            }
            
            robot_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            // 尝试使用 IK 服务计算关节角度
            auto ik_client = this->create_client<aubo_msgs::srv::GetIK>("/aubo_driver/get_ik");
            
            if (ik_client->wait_for_service(std::chrono::seconds(1)))
            {
                auto request = std::make_shared<aubo_msgs::srv::GetIK::Request>();
                for (size_t i = 0; i < 6 && i < joint_group_positions.size(); ++i)
                {
                    request->ref_joint[i] = static_cast<float>(joint_group_positions[i]);
                }
                for (size_t i = joint_group_positions.size(); i < 6; ++i)
                {
                    request->ref_joint[i] = 0.0f;
                }
                request->pos[0] = target_pose_msg.position.x;
                request->pos[1] = target_pose_msg.position.y;
                request->pos[2] = target_pose_msg.position.z;
                request->ori[0] = target_pose_msg.orientation.w;
                request->ori[1] = target_pose_msg.orientation.x;
                request->ori[2] = target_pose_msg.orientation.y;
                request->ori[3] = target_pose_msg.orientation.z;

                auto result = ik_client->async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == 
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = result.get();
                    if (response->joint.size() >= 6)
                    {
                        std::vector<double> target_joints;
                        for (size_t i = 0; i < 6; ++i)
                        {
                            target_joints.push_back(response->joint[i]);
                        }
                        move_group_->setJointValueTarget(target_joints);
                        RCLCPP_INFO(this->get_logger(), "SetRobotPose: Using IK service for joint space planning");
                    }
                    else
                    {
                        error_code = -101;
                        message = "Failed to compute IK for target pose";
                        return false;
                    }
                }
                else
                {
                    error_code = -101;
                    message = "Failed to call IK service";
                    return false;
                }
            }
            else
            {
                // IK 服务不可用，使用 MoveIt 的 pose target
                move_group_->setPoseTarget(target_pose_msg);
                RCLCPP_INFO(this->get_logger(), "SetRobotPose: IK service not available, using MoveIt pose target");
            }
        }
        else
        {
            // 使用笛卡尔空间规划
            move_group_->setPoseTarget(target_pose_msg);
            RCLCPP_INFO(this->get_logger(), "SetRobotPose: Using Cartesian space planning");
        }

        // 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_->plan(my_plan);

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(plan_result.val);
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "SetRobotPose: Executing trajectory with %zu waypoints", 
                   my_plan.trajectory_.joint_trajectory.points.size());
        
        moveit::planning_interface::MoveItErrorCode execute_result = move_group_->execute(my_plan);

        if (execute_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_code = static_cast<int32_t>(execute_result.val);
            message = "Execution failed with error code: " + std::to_string(execute_result.val);
            return false;
        }

        error_code = 0;
        message = "Successfully moved to target pose";
        RCLCPP_INFO(this->get_logger(), "SetRobotPose: Execution completed");
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = -200;
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

void SetRobotPoseServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::SetRobotPoseServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("set_robot_pose_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("set_robot_pose_server_node"), 
                    "Exception in set_robot_pose_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

