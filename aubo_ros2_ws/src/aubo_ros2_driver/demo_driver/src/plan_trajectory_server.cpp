/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/plan_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aubo_msgs/srv/get_ik.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <string>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/exceptions.hpp>

namespace demo_driver
{



/**
 * @brief 构造函数，初始化 MoveIt 接口和服务服务器
 */
PlanTrajectoryServer::PlanTrajectoryServer(const rclcpp::NodeOptions& options)
    : Node("plan_trajectory_server_node", options)
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    // 获取参数
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);

    // 初始化服务服务器
    plan_trajectory_service_ = this->create_service<demo_interface::srv::PlanTrajectory>(
        "/plan_trajectory",
        std::bind(&PlanTrajectoryServer::planTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool PlanTrajectoryServer::wait_for_robot_description(int timeout_seconds)
{
    static bool info_logged = false;
    static bool warned_once = false;
    
    if (!info_logged) {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot_description parameter and MoveIt2 services...");
        info_logged = true;
    }
    
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
                static bool ready_logged = false;
                if (!ready_logged) {
                    RCLCPP_INFO(this->get_logger(), "Robot description ready, planning frame: %s", test_frame.c_str());
                    ready_logged = true;
                }
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

bool PlanTrajectoryServer::initialize(int max_retries, int retry_delay_seconds)
{
    static bool init_logged = false;
    if (!init_logged) {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces for planning group: %s", planning_group_name_.c_str());
        init_logged = true;
    }
    
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
            
            static bool success_logged = false;
            if (!success_logged) {
                RCLCPP_INFO(this->get_logger(), "PlanTrajectoryServer initialized, end effector: %s, service '/plan_trajectory' ready", 
                           end_effector_link_.c_str());
                success_logged = true;
            }
            return true;
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
                static int last_attempt_logged = 0;
                if (attempt != last_attempt_logged) {
                    RCLCPP_WARN(this->get_logger(), "Initialization attempt %d/%d failed: %s, retrying...", 
                               attempt, max_retries, e.what());
                    last_attempt_logged = attempt;
                }
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

PlanTrajectoryServer::~PlanTrajectoryServer()
{
}

/**
 * @brief 服务回调函数
 */
void PlanTrajectoryServer::planTrajectoryCallback(
    const std::shared_ptr<demo_interface::srv::PlanTrajectory::Request> req,
    std::shared_ptr<demo_interface::srv::PlanTrajectory::Response> res)
{
    static bool first_request = true;
    if (first_request) {
        RCLCPP_INFO(this->get_logger(), "PlanTrajectory service ready");
        first_request = false;
    }
    
    RCLCPP_INFO(this->get_logger(), "PlanTrajectory request: pose=(%.3f, %.3f, %.3f), use_joints=%s",
               req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z,
               req->use_joints ? "true" : "false");
    
    if (!move_group_)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized");
        res->success = false;
        res->planning_time = 0.0;
        res->message = "MoveIt interface not initialized";
        RCLCPP_WARN(this->get_logger(), "PlanTrajectory response: %s", res->message.c_str());
        return;
    }

    float planning_time = 0.0;
    std::string message;
    bool success = planTrajectory(req->target_pose, req->use_joints,
                                   res->trajectory, planning_time, message);

    res->success = success;
    res->planning_time = planning_time;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "PlanTrajectory response: success, %zu waypoints, planning_time=%.3f s",
                   res->trajectory.points.size(), planning_time);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "PlanTrajectory response: failed (message: %s)", message.c_str());
    }
}

/**
 * @brief 执行轨迹规划
 */
bool PlanTrajectoryServer::planTrajectory(const geometry_msgs::msg::Pose& target_pose,
                                         bool use_joints,
                                         trajectory_msgs::msg::JointTrajectory& trajectory,
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
        double velocity_scaling = 0.1;
        double acceleration_scaling = 0.1;
        
        if (this->has_parameter("moveit_velocity_scaling_factor"))
        {
            velocity_scaling = this->get_parameter("moveit_velocity_scaling_factor").as_double();
            move_group_->setMaxVelocityScalingFactor(velocity_scaling);
        }
        else
        {
            velocity_scaling = 0.1;
        }
        
        try
        {
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "move_group");
            if (param_client->wait_for_service(std::chrono::milliseconds(200)))
            {
                try
                {
                    auto params = param_client->get_parameters({"robot_description_planning.default_acceleration_scaling_factor"});
                    if (!params.empty())
                    {
                        acceleration_scaling = params[0].as_double();
                    }
                }
                catch (...)
                {
                }
            }
        }
        catch (...)
        {
        }
        
        RCLCPP_INFO(this->get_logger(), "PlanTrajectory: vel=%.2f, acc=%.2f",
                   velocity_scaling, acceleration_scaling);
        
        // 记录规划开始时间
        auto start_time = std::chrono::high_resolution_clock::now();

        if (use_joints)
        {
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            
            moveit::core::RobotStatePtr robot_state;
            if (!current_state)
            {
                static bool warned_once = false;
                if (!warned_once) {
                    RCLCPP_WARN(this->get_logger(), "getCurrentState() returned null, using default state from robot model");
                    warned_once = true;
                }
                const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
                if (!robot_model)
                {
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
                message = "Failed to get joint model group from MoveIt";
                return false;
            }
            
            robot_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

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
                request->pos[0] = target_pose.position.x;
                request->pos[1] = target_pose.position.y;
                request->pos[2] = target_pose.position.z;
                request->ori[0] = target_pose.orientation.w;
                request->ori[1] = target_pose.orientation.x;
                request->ori[2] = target_pose.orientation.y;
                request->ori[3] = target_pose.orientation.z;

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
                    }
                    else
                    {
                        message = "Failed to compute IK for target pose";
                        return false;
                    }
                }
                else
                {
                    message = "Failed to call IK service";
                    return false;
                }
            }
            else
            {
                static bool ik_warned = false;
                if (!ik_warned) {
                    RCLCPP_INFO(this->get_logger(), "IK service not available, using MoveIt's built-in IK solver");
                    ik_warned = true;
                }
                move_group_->setPoseTarget(target_pose);
            }
        }
        else
        {
            move_group_->setPoseTarget(target_pose);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode plan_result = move_group_->plan(my_plan);

        // 计算规划时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        planning_time = duration.count() / 1000.0f;

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            message = "Planning failed with error code: " + std::to_string(plan_result.val);
            RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
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
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void PlanTrajectoryServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

/**
 * @brief 主函数
 * 初始化 ROS 节点并启动规划轨迹服务服务器
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    rclcpp::init(argc, argv);
    
    // 创建执行器（MoveIt 需要多线程执行器）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        // 创建规划轨迹服务服务器对象，使用 automatically_declare_parameters_from_overrides 自动声明参数
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::PlanTrajectoryServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("plan_trajectory_server_node"), 
                        "Failed to initialize MoveIt interfaces");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        // 捕获异常并输出错误信息
        RCLCPP_ERROR(rclcpp::get_logger("plan_trajectory_server_node"), 
                    "Exception in plan_trajectory_server_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}