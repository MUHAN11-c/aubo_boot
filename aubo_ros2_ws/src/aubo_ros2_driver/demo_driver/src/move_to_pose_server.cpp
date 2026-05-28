/*
 * /move_to_pose — 委托 RobotController。
 *   use_joints=true  → moveToJoints (setJointValueTarget → plan → execute)
 *   use_joints=false → computeCartesianPath (直线约束) → execute 喵~
 */
#include "demo_driver/move_to_pose_server.h"
#include "demo_driver/robot_controller.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace demo_driver
{

MoveToPoseServer::MoveToPoseServer(const rclcpp::NodeOptions& options)
    : Node("move_to_pose_server_node", options)
{
    service_ = create_service<ivg_interfaces::srv::MoveToPose>(
        "/move_to_pose",
        [this](const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
               std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> res) {
          onMoveToPoseRequest(req, res);
        });
}

void MoveToPoseServer::init()
{
    robot_controller_ = std::make_unique<RobotController>(this, "manipulator");
    if (!robot_controller_->init()) {
        RCLCPP_ERROR(get_logger(), "RobotController init failed");
        return;
    }
    RCLCPP_INFO(get_logger(), "MoveToPoseServer ready (via RobotController)");
}

void MoveToPoseServer::onMoveToPoseRequest(
    const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
    std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> res)
{
    std::lock_guard<std::mutex> lock(service_mutex_);

    if (!robot_controller_) {
        res->success = false; res->error_code = -100; res->message = "not initialized"; return;
    }
    if (req->velocity_factor < 0.0f || req->velocity_factor > 1.0f ||
        req->acceleration_factor < 0.0f || req->acceleration_factor > 1.0f) {
        res->success = false; res->error_code = -1;
        res->message = "velocity/acceleration factor must be in [0,1]"; return;
    }

    bool ok;
    if (req->use_joints) {
        std::array<double, 6> joints;
        std::copy_n(req->target_joints.begin(), 6, joints.begin());
        ok = robot_controller_->moveToJoints(joints, req->velocity_factor, req->acceleration_factor);
    } else {
        // 笛卡尔直线: computeCartesianPath 保证 TCP 直线约束
        geometry_msgs::msg::Pose target;
        target.position    = req->target_pose.position;
        target.orientation = req->target_pose.orientation;

        auto mg = robot_controller_->moveGroup();
        if (!mg) {
            res->success = false; res->error_code = -100; res->message = "move_group not ready"; return;
        }

        robot_controller_->setVelocityScaling(req->velocity_factor);
        robot_controller_->setAccelerationScaling(req->acceleration_factor);

        auto start = robot_controller_->getCurrentPose();
        std::vector<geometry_msgs::msg::Pose> waypoints = {start, target};

        moveit_msgs::msg::RobotTrajectory traj;
        double fraction = mg->computeCartesianPath(waypoints, 0.015, 0.0, traj);
        if (fraction < 0.99) {
            res->success = false; res->error_code = -2;
            res->message = "cartesian path fraction=" + std::to_string(fraction); return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        ok = mg->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    res->success = ok; res->error_code = ok ? 0 : -2;
    res->message = ok ? "ok" : "move failed";
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::MoveToPoseServer>(opts);
    node->init();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
