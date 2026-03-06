/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#include "demo_driver/move_to_pose_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <aubo_msgs/srv/get_ik.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>
#include <atomic>
#include <cmath>
#include <string>
#include <chrono>

namespace demo_driver
{

MoveToPoseServer::MoveToPoseServer(const rclcpp::NodeOptions& options)
    : Node("move_to_pose_server_node", options)
    , planning_group_name_("manipulator")
    , base_frame_("base_link")
    , end_effector_link_("")
{
    this->declare_parameter("planning_group_name", std::string("manipulator"));
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("robot_status_topic", std::string("/demo_robot_status"));
    this->get_parameter("planning_group_name", planning_group_name_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("robot_status_topic", robot_status_topic_);

    move_to_pose_service_ = this->create_service<demo_interface::srv::MoveToPose>(
        "/move_to_pose",
        std::bind(&MoveToPoseServer::moveToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 订阅机器人状态话题（demo_robot_status_ros2），获取当前关节角与笛卡尔位姿
    robot_status_sub_ = this->create_subscription<demo_interface::msg::RobotStatus>(
        robot_status_topic_, rclcpp::QoS(10), std::bind(&MoveToPoseServer::robotStatusCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "MoveToPoseServer: 已订阅机器人状态话题 '%s'", robot_status_topic_.c_str());
}

void MoveToPoseServer::robotStatusCallback(const demo_interface::msg::RobotStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(robot_status_mutex_);
    current_cartesian_pose_ = msg->cartesian_position;
    current_joint_positions_.clear();
    for (size_t i = 0; i < 6u && i < msg->joint_position_rad.size(); ++i)
        current_joint_positions_.push_back(msg->joint_position_rad[i]);
    has_robot_status_ = true;
}

bool MoveToPoseServer::wait_for_robot_description(int timeout_seconds)
{
    static bool warned_once = false;
    if (!this->has_parameter("robot_description") || !this->has_parameter("robot_description_semantic")) {
        if (!warned_once) {
            RCLCPP_WARN(this->get_logger(), "未找到参数 robot_description 或 robot_description_semantic");
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
                RCLCPP_INFO(this->get_logger(), "仍在等待... (已等待 %ld 秒, 错误: %s)",
                           elapsed_seconds, e.what());
            }
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds)
        {
            RCLCPP_ERROR(this->get_logger(), "等待超时 (%d 秒)", timeout_seconds);
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return false;
}

bool MoveToPoseServer::initialize(int max_retries, int retry_delay_seconds)
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
            std::string planning_frame = move_group_->getPlanningFrame();
            RCLCPP_INFO(this->get_logger(), "[MoveToPose] 初始化 规划组=%s | 参考系=%s | 末端=%s",
                        planning_group_name_.c_str(), planning_frame.c_str(), end_effector_link_.c_str());
            move_group_->setPlanningTime(20.0);
            move_group_->setNumPlanningAttempts(10);
            move_group_->setGoalPositionTolerance(0.001);
            move_group_->setGoalOrientationTolerance(0.001);
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
                RCLCPP_ERROR(this->get_logger(), "MoveIt 接口初始化失败，已尝试 %d 次", max_retries);
                return false;
            }
        }
    }
    
    return false;
}

MoveToPoseServer::~MoveToPoseServer()
{
}

void MoveToPoseServer::moveToPoseCallback(
    const std::shared_ptr<demo_interface::srv::MoveToPose::Request> req,
    std::shared_ptr<demo_interface::srv::MoveToPose::Response> res)
{
    if (move_group_)
    {
        std::string planning_frame = move_group_->getPlanningFrame();
        std::string ee = move_group_->getEndEffectorLink();
        RCLCPP_INFO(this->get_logger(), "[MoveToPose] 规划组=%s | 参考系=%s | 末端=%s",
                    planning_group_name_.c_str(), planning_frame.c_str(), ee.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "[MoveToPose] 目标 位姿=(%.3f, %.3f, %.3f) use_joints=%s vel=%.2f acc=%.2f",
                req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z,
                req->use_joints ? "true" : "false", req->velocity_factor, req->acceleration_factor);

    int32_t error_code = 0;
    std::string message;
    bool success = false;
    {
        std::lock_guard<std::mutex> lock(move_to_pose_mutex_);
        auto now = std::chrono::steady_clock::now();
        double dx = req->target_pose.position.x - last_target_x_;
        double dy = req->target_pose.position.y - last_target_y_;
        double dz = req->target_pose.position.z - last_target_z_;
        bool position_match = (std::abs(dx) <= kDedupPositionTol && std::abs(dy) <= kDedupPositionTol && std::abs(dz) <= kDedupPositionTol);
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_finish_time_).count();
        if (position_match && last_result_success_ && elapsed < kDedupTimeWindowSec)
        {
            res->success = true;
            res->error_code = 0;
            res->message = "duplicate request, returning prior success";
            RCLCPP_INFO(this->get_logger(), "[MoveToPose] 去重 — 与 %.1fs 内同目标，直接返回成功", elapsed);
            return;
        }
        if (req->velocity_factor < 0.f || req->velocity_factor > 1.f || req->acceleration_factor < 0.f || req->acceleration_factor > 1.f)
        {
            res->success = false;
            res->error_code = -1;
            res->message = "velocity_factor 与 acceleration_factor 须在 0.0～1.0 之间";
            RCLCPP_WARN(this->get_logger(), "[MoveToPose] 响应 失败: %s", res->message.c_str());
            return;
        }
        success = moveToPose(req->target_pose, req->use_joints,
                             req->velocity_factor, req->acceleration_factor,
                             error_code, message);
        last_target_x_ = req->target_pose.position.x;
        last_target_y_ = req->target_pose.position.y;
        last_target_z_ = req->target_pose.position.z;
        last_result_success_ = success;
        last_finish_time_ = std::chrono::steady_clock::now();
        res->success = success;
        res->error_code = error_code;
        res->message = message;
    }
    if (success)
        RCLCPP_INFO(this->get_logger(), "[MoveToPose] 响应 成功");
    else
        RCLCPP_WARN(this->get_logger(), "[MoveToPose] 响应 失败 error_code=%d | %s", error_code, message.c_str());
}

namespace
{
bool isTrajectoryNoOp(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto& pts = plan.trajectory_.joint_trajectory.points;
    if (pts.size() <= 1u) return true;
    const auto& a = pts.front().positions;
    const auto& b = pts.back().positions;
    if (a.size() != b.size()) return false;
    const double tol_rad = 1e-5;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > tol_rad) return false;
    return true;
}

struct ExecutingGuard
{
    std::atomic<bool>* flag_{nullptr};
    explicit ExecutingGuard(std::atomic<bool>& flag) : flag_(&flag) {}
    ~ExecutingGuard() { if (flag_) flag_->store(false); }
    ExecutingGuard(const ExecutingGuard&) = delete;
    ExecutingGuard& operator=(const ExecutingGuard&) = delete;
};

}  // namespace

bool MoveToPoseServer::moveToPose(const geometry_msgs::msg::Pose& target_pose,
                                  bool use_joints,
                                  float velocity_factor,
                                  float acceleration_factor,
                                  int32_t& error_code,
                                  std::string& message)
{
    if (!move_group_)
    {
        error_code = -100;
        message = "MoveIt 接口未初始化";
        return false;
    }

    static std::atomic<bool> is_executing(false);
    if (is_executing.exchange(true))
    {
        error_code = -5;
        message = "已有轨迹在执行，拒绝重复请求";
        RCLCPP_WARN(this->get_logger(), "[MoveToPose] %s", message.c_str());
        is_executing.store(false);
        return false;
    }

    try
    {
        ExecutingGuard guard(is_executing);
        move_group_->setMaxVelocityScalingFactor(velocity_factor);
        move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

        // 设置目标：use_joints 时尝试 IK，否则或失败时用位姿目标
        bool use_joint_target = false;
        if (use_joints)
        {
            moveit::core::RobotStatePtr state = move_group_->getCurrentState();
            if (!state) {
                state = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
                state->setToDefaultValues();
            }
            if (!state->getRobotModel()) { error_code = -100; message = "获取机器人模型失败"; return false; }
            const moveit::core::JointModelGroup* jmg = state->getJointModelGroup(planning_group_name_);
            if (!jmg) { error_code = -101; message = "获取关节模型组失败"; return false; }
            std::vector<double> ref_joints;
            state->copyJointGroupPositions(jmg, ref_joints);
            if (!ik_client_) ik_client_ = this->create_client<aubo_msgs::srv::GetIK>("/aubo_driver/get_ik");
            if (ik_client_->wait_for_service(std::chrono::seconds(1)))
            {
                auto req = std::make_shared<aubo_msgs::srv::GetIK::Request>();
                for (size_t i = 0; i < 6u; ++i) req->ref_joint[i] = (i < ref_joints.size()) ? static_cast<float>(ref_joints[i]) : 0.f;
                req->pos[0] = target_pose.position.x; req->pos[1] = target_pose.position.y; req->pos[2] = target_pose.position.z;
                req->ori[0] = target_pose.orientation.w; req->ori[1] = target_pose.orientation.x;
                req->ori[2] = target_pose.orientation.y; req->ori[3] = target_pose.orientation.z;
                auto fut = ik_client_->async_send_request(req);
                if (rclcpp::spin_until_future_complete(this->shared_from_this(), fut, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto res = fut.get();
                    if (res->joint.size() >= 6u)
                    {
                        std::vector<double> j(6);
                        for (size_t i = 0; i < 6u; ++i) j[i] = res->joint[i];
                        move_group_->setJointValueTarget(j);
                        use_joint_target = true;
                    }
                    else { error_code = -101; message = "逆解失败"; return false; }
                }
            }
        }
        if (!use_joint_target)
            move_group_->setPoseTarget(target_pose, end_effector_link_);

        // 多次规划，优先采用时长 < 5s 的轨迹，>5s 拒绝执行
        const double max_duration = 5.0;
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
        double best_duration = 1e9;
        bool has_plan = false;

        for (int attempt = 1; attempt <= 10; ++attempt)
        {
            moveit::planning_interface::MoveGroupInterface::Plan cand;
            if (move_group_->plan(cand) != moveit::core::MoveItErrorCode::SUCCESS)
            {
                if (attempt == 1) { error_code = -2; message = "关节规划失败"; return false; }
                continue;
            }
            double dur = 0.0;
            if (!cand.trajectory_.joint_trajectory.points.empty())
                dur = rclcpp::Duration(cand.trajectory_.joint_trajectory.points.back().time_from_start).seconds();
            if (dur < best_duration) { best_duration = dur; exec_plan = cand; has_plan = true; }
            if (dur < max_duration) break;
        }
        if (!has_plan) { error_code = -2; message = "多次规划均失败"; return false; }
        if (best_duration > max_duration) { error_code = -3; message = "轨迹时长 > 5s，拒绝执行"; return false; }
        if (isTrajectoryNoOp(exec_plan)) { error_code = 0; message = "无运动，已到达"; return true; }

        auto exec_res = move_group_->execute(exec_plan);
        if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
        {
            error_code = exec_res.val;
            message = "执行失败，错误码：" + std::to_string(exec_res.val);
            return false;
        }

        error_code = 0;
        message = "关节运动完成";
        return true;
    }
    catch (const std::exception& e)
    {
        error_code = -200;
        message = "异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "[MoveToPose] %s", message.c_str());
        is_executing.store(false);
        return false;
    }
}

void MoveToPoseServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // 多线程执行器：回调在单独线程中处理，与 spin 主线程分离
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try
    {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::MoveToPoseServer>(node_options);
        
        if (!server->initialize(10, 2))
        {
            RCLCPP_ERROR(rclcpp::get_logger("move_to_pose_server_node"),
                        "MoveIt 接口初始化失败");
            rclcpp::shutdown();
            return 1;
        }
        
        executor.add_node(server);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("move_to_pose_server_node"),
                    "move_to_pose_server_node 异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}