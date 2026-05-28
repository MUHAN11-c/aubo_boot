/*
 * /execute_trajectory — 执行预规划轨迹。
 * MoveGroupInterface 在 init() 中创建喵~
 */
#include "demo_driver/execute_trajectory_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace demo_driver
{

ExecuteTrajectoryServer::ExecuteTrajectoryServer(const rclcpp::NodeOptions& options)
    : Node("execute_trajectory_server_node", options)
{
    if (!has_parameter("planning_group_name"))
        declare_parameter("planning_group_name", "manipulator");
    if (!has_parameter("base_frame"))
        declare_parameter("base_frame", "base_link");
    planning_group_name_ = get_parameter("planning_group_name").as_string();
    base_frame_          = get_parameter("base_frame").as_string();

    service_ = create_service<ivg_interfaces::srv::ExecuteTrajectory>(
        "/execute_trajectory",
        [this](const std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Request> req,
               std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Response> res) {
          executeTrajectoryCallback(req, res);
        });
}

void ExecuteTrajectoryServer::init()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
    move_group_->setPoseReferenceFrame(base_frame_);

    RCLCPP_INFO(get_logger(), "ExecuteTrajectoryServer ready");
}

void ExecuteTrajectoryServer::executeTrajectoryCallback(
    const std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ExecuteTrajectory::Response> res)
{
    if (req->trajectory.points.empty()) {
        res->success = false; res->error_code = -1; res->message = "empty trajectory"; return;
    }
    if (req->trajectory.joint_names.empty()) {
        res->success = false; res->error_code = -2; res->message = "empty joint_names"; return;
    }

    int32_t ec = 0; std::string msg;
    res->success = executeTrajectory(req->trajectory, ec, msg);
    res->error_code = ec;
    res->message = msg;
}

bool ExecuteTrajectoryServer::executeTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    int32_t& error_code, std::string& message)
{
    if (!move_group_) { error_code = -100; message = "not initialized"; return false; }

    try {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = trajectory;

        // 读取速度/加速度因子 (由 launch 文件注入的本地参数)
        if (has_parameter("moveit_velocity_scaling_factor"))
            move_group_->setMaxVelocityScalingFactor(
                get_parameter("moveit_velocity_scaling_factor").as_double());

        moveit::core::MoveItErrorCode result = move_group_->execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            error_code = static_cast<int32_t>(result.val);
            message = "execute failed, error_code=" + std::to_string(result.val);
            return false;
        }

        error_code = 0;
        message = "ok";
        return true;
    } catch (const std::exception& e) {
        error_code = -200;
        message = e.what();
        return false;
    }
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::ExecuteTrajectoryServer>(opts);
    node->init();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
