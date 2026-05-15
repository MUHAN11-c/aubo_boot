/*
 * /set_speed_factor — 设置 MoveIt 速度缩放因子, 同步到关联节点。
 * 构造函数内创建 MoveGroupInterface + AsyncParametersClient, 无需外部 initialize。
 */
#include "demo_driver/set_speed_factor_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace demo_driver
{

SetSpeedFactorServer::SetSpeedFactorServer(const rclcpp::NodeOptions& options)
    : Node("set_speed_factor_server_node", options)
{
    declare_parameter("planning_group_name", "manipulator");
    declare_parameter("base_frame", "base_link");
    planning_group_name_ = get_parameter("planning_group_name").as_string();
    base_frame_          = get_parameter("base_frame").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);
    move_group_->setPoseReferenceFrame(base_frame_);

    service_ = create_service<ivg_interfaces::srv::SetSpeedFactor>(
        "/set_speed_factor",
        [this](const std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Request> req,
               std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Response> res) {
          setSpeedFactorCallback(req, res);
        });

    // Reentrant 组, 避免 set_parameters 阻塞死锁
    param_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    plan_traj_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        shared_from_this(), "plan_trajectory_server");
    exec_traj_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        shared_from_this(), "execute_trajectory_server");

    RCLCPP_INFO(get_logger(), "SetSpeedFactorServer ready");
}

void SetSpeedFactorServer::setSpeedFactorCallback(
    const std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetSpeedFactor::Response> res)
{
    if (req->velocity_factor < 0.0f || req->velocity_factor > 1.0f) {
        res->success = false;
        res->message = "velocity_factor must be in [0.0, 1.0]";
        return;
    }

    std::string msg;
    res->success = setSpeedFactor(req->velocity_factor, msg);
    res->message = msg;
}

bool SetSpeedFactorServer::setSpeedFactor(float v, std::string& msg)
{
    if (!move_group_) { msg = "MoveIt not initialized"; return false; }

    try {
        move_group_->setMaxVelocityScalingFactor(v);

        std::vector<rclcpp::Parameter> params{rclcpp::Parameter("moveit_velocity_scaling_factor", v)};
        if (plan_traj_client_->wait_for_service(std::chrono::seconds(1)))
            plan_traj_client_->set_parameters(params,
                [](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>){});
        if (exec_traj_client_->wait_for_service(std::chrono::seconds(1)))
            exec_traj_client_->set_parameters(params,
                [](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>){});

        msg = "velocity_factor=" + std::to_string(v);
        return true;
    } catch (const std::exception& e) {
        msg = e.what();
        return false;
    }
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::SetSpeedFactorServer>(opts);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
