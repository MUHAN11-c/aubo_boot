/*
 * /move_to_pose — 关节空间或笛卡尔空间移动到目标位姿。
 * MoveGroupInterface 在 init() 中创建喵~
 */
#include "demo_driver/move_to_pose_server.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace demo_driver
{

static constexpr double kZMinLimit = -0.05;  // Z 轴安全下限 (m)

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
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");
    move_group_->allowReplanning(true);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(get_logger(), "MoveToPoseServer ready");
}

void MoveToPoseServer::onMoveToPoseRequest(
    const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
    std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> res)
{
    std::lock_guard<std::mutex> lock(service_mutex_);

    if (!move_group_) {
        res->success = false; res->error_code = -100; res->message = "not initialized"; return;
    }
    if (req->velocity_factor < 0.0f || req->velocity_factor > 1.0f ||
        req->acceleration_factor < 0.0f || req->acceleration_factor > 1.0f) {
        res->success = false; res->error_code = -1;
        res->message = "velocity/acceleration factor must be in [0,1]"; return;
    }

    bool ok;
    if (req->use_joints) {
        ok = moveToJoints(req->target_joints.data(), req->velocity_factor, req->acceleration_factor);
    } else {
        const auto& p = req->target_pose.position;
        const auto& q = req->target_pose.orientation;
        ok = moveToPose(p.x, p.y, p.z, q.x, q.y, q.z, q.w, req->velocity_factor, req->acceleration_factor);
    }

    res->success = ok; res->error_code = ok ? 0 : -2;
    res->message = ok ? "ok" : "move failed";
}

bool MoveToPoseServer::moveToJoints(const double* joints, float vel, float acc)
{
    if (!move_group_) return false;
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);
    move_group_->setJointValueTarget(std::vector<double>(joints, joints + 6));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveToPoseServer::moveToPose(double x, double y, double z,
                                  double qx, double qy, double qz, double qw,
                                  float vel, float acc)
{
    if (!move_group_) return false;
    if (z < kZMinLimit) z = kZMinLimit;  // 安全限位

    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);

    geometry_msgs::msg::Pose target;
    target.position.x = x; target.position.y = y; target.position.z = z;
    target.orientation.x = qx; target.orientation.y = qy;
    target.orientation.z = qz; target.orientation.w = qw;
    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return false;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
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
