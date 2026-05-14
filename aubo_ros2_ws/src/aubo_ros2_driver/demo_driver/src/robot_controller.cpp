/*
 * RobotController — 组合模式封装 MoveIt 运动 + Aubo IO。
 * 消除 gripper_swap / execute_grasp / publish_grasps 三处的重复代码。
 * IO 语义统一: setGripper(pin, open=true) 打开夹爪。
 */
#include "demo_driver/robot_controller.h"
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose.h>
#include <chrono>
#include <thread>

namespace demo_driver
{

RobotController::RobotController(rclcpp::Node* owner, const std::string& planning_group)
    : node_(owner)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        owner->shared_from_this(), planning_group);
    move_group_->allowReplanning(true);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    eef_link_ = move_group_->getEndEffectorLink();

    io_client_ = owner->create_client<ivg_interfaces::srv::SetRobotIO>("/set_robot_io");
}

// =====================================================================
// 运动
// =====================================================================

bool RobotController::moveToHome(float vel, float acc)
{
    if (!move_group_) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);
    move_group_->setNamedTarget("camera_pose");
    return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveToJoints(const std::array<double, 6>& joints, float vel, float acc)
{
    if (!move_group_) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);
    move_group_->setJointValueTarget(std::vector<double>(joints.begin(), joints.end()));
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotController::moveCartesianZ(double offset_m, float vel, float acc)
{
    return moveCartesianPath({{'z', offset_m}}, vel, acc);
}

bool RobotController::moveCartesianPath(const std::vector<CartesianSegment>& segments,
                                        float vel, float acc)
{
    if (!move_group_ || segments.empty()) return false;
    setVelocityScaling(vel);
    setAccelerationScaling(acc);

    auto start = currentPoseInternal();
    std::vector<geometry_msgs::msg::Pose> waypoints{start};

    for (const auto& seg : segments) {
        auto next = waypoints.back();
        switch (seg.axis) {
            case 'x': next.position.x += seg.offset; break;
            case 'y': next.position.y += seg.offset; break;
            case 'z': next.position.z += seg.offset; break;
        }
        if (next.position.z < z_min_limit_) next.position.z = z_min_limit_;
        waypoints.push_back(next);
    }
    waypoints.erase(waypoints.begin());  // 去掉起点

    for (int attempt = 0; attempt < max_retries_; ++attempt) {
        moveit_msgs::msg::RobotTrajectory traj;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, traj);
        if (fraction < 0.99) {
            if (attempt < max_retries_ - 1)
                std::this_thread::sleep_for(std::chrono::duration<double>(retry_wait_sec_));
            continue;
        }
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return false;
}

// =====================================================================
// IO
// =====================================================================

bool RobotController::setGripper(int pin, bool open)
{
    if (!io_client_->wait_for_service(std::chrono::seconds(3))) return false;
    auto req = std::make_shared<ivg_interfaces::srv::SetRobotIO::Request>();
    req->io_type  = "digital_output";
    req->io_index = pin;
    req->value    = open ? 1.0 : 0.0;
    auto future = io_client_->async_send_request(req);
    return future.wait_for(std::chrono::seconds(10)) == std::future_status::ready &&
           future.get()->success;
}

bool RobotController::setQuickSwap(int pin, bool lock)
{
    return setGripper(pin, lock);
}

// =====================================================================
// 查询
// =====================================================================

geometry_msgs::msg::Pose RobotController::getCurrentPose()
{
    return move_group_->getCurrentPose(eef_link_);
}

std::vector<double> RobotController::getCurrentJoints()
{
    std::vector<double> jv;
    move_group_->getCurrentState()->copyJointGroupPositions(
        move_group_->getCurrentState()->getRobotModel()->getJointModelGroup(
            move_group_->getName()), jv);
    return jv;
}

std::string RobotController::getEndEffectorLink() const
{
    return eef_link_;
}

// =====================================================================
// 配置
// =====================================================================

void RobotController::setVelocityScaling(float v) { move_group_->setMaxVelocityScalingFactor(v); }
void RobotController::setAccelerationScaling(float a) { move_group_->setMaxAccelerationScalingFactor(a); }

geometry_msgs::msg::Pose RobotController::currentPoseInternal()
{
    return move_group_->getCurrentPose(eef_link_);
}

}  // namespace demo_driver
