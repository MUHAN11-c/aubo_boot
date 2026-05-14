/*
 * /set_robot_enable → 代理到仪表盘 /aubo/startup + /aubo/shutdown
 * 客户端独立回调组, 避免 future.wait_for 死锁。
 */
#include "demo_driver/set_robot_enable_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>
#include <string>

namespace demo_driver
{

SetRobotEnableServer::SetRobotEnableServer(const rclcpp::NodeOptions& options)
    : Node("set_robot_enable_server_node", options)
{
    // 创建指向仪表盘的客户端（独立回调组, 避免与服务默认组互斥导致 future.wait_for 死锁）
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    startup_client_  = this->create_client<std_srvs::srv::Trigger>(
        "/aubo/startup", rmw_qos_profile_services_default, client_cb_group_);
    shutdown_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/aubo/shutdown", rmw_qos_profile_services_default, client_cb_group_);

    RCLCPP_INFO(this->get_logger(),
        "SetRobotEnableServer creating service '/set_robot_enable'...");

    set_robot_enable_service_ = this->create_service<ivg_interfaces::srv::SetRobotEnable>(
        "/set_robot_enable",
        std::bind(&SetRobotEnableServer::setRobotEnableCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "SetRobotEnableServer ready — proxies to /aubo/startup and /aubo/shutdown");
}

SetRobotEnableServer::~SetRobotEnableServer()
{
}

void SetRobotEnableServer::setRobotEnableCallback(
    const std::shared_ptr<ivg_interfaces::srv::SetRobotEnable::Request> req,
    std::shared_ptr<ivg_interfaces::srv::SetRobotEnable::Response> res)
{
    auto client = req->enable ? startup_client_ : shutdown_client_;
    const std::string svc_name = req->enable ? "/aubo/startup" : "/aubo/shutdown";

    RCLCPP_INFO(this->get_logger(),
        "Request: enable=%s → calling %s", req->enable ? "true" : "false", svc_name.c_str());

    // 等待仪表盘服务就绪（最多 3s）
    if (!client->wait_for_service(std::chrono::seconds(3))) {
        res->success    = false;
        res->error_code = -1;
        res->message    = svc_name + " service not available — is the dashboard node running?";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future  = client->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        auto trigger_res = future.get();
        res->success    = trigger_res->success;
        res->error_code = trigger_res->success ? 0 : -1;
        res->message    = trigger_res->message;

        RCLCPP_INFO(this->get_logger(),
            "Response from %s: success=%s, message='%s'",
            svc_name.c_str(),
            trigger_res->success ? "true" : "false",
            trigger_res->message.c_str());
    } else {
        res->success    = false;
        res->error_code = -2;
        res->message    = svc_name + " timeout (10s) — SDK call may be blocked";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
    }
}

void SetRobotEnableServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

}  // namespace demo_driver

// ===================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::SetRobotEnableServer>(node_options);
        executor.add_node(server);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("set_robot_enable_server_node"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
