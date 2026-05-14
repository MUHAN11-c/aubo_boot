/*
 * /read_robot_io — 订阅 /robot_io_status 缓存 + 按索引查询。
 * io_state_mutex_ 保护缓存, io_state_received_ atomic 避免数据竞争。
 */
#include "demo_driver/read_robot_io_server.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <algorithm>
#include <cctype>
#include <string>

namespace demo_driver
{

ReadRobotIOServer::ReadRobotIOServer(const rclcpp::NodeOptions& options)
    : Node("read_robot_io_server_node", options)
{
    this->declare_parameter("robot_io_status_topic", std::string("/robot_io_status"));
    this->get_parameter("robot_io_status_topic", robot_io_status_topic_);

    RCLCPP_INFO(this->get_logger(),
        "ReadRobotIOServer subscribing to '%s'...", robot_io_status_topic_.c_str());

    robot_io_status_sub_ = this->create_subscription<ivg_interfaces::msg::RobotIOStatus>(
        robot_io_status_topic_,
        rclcpp::QoS(10),
        std::bind(&ReadRobotIOServer::robotIOStatusCallback, this, std::placeholders::_1));

    read_robot_io_service_ = this->create_service<ivg_interfaces::srv::ReadRobotIO>(
        "/read_robot_io",
        std::bind(&ReadRobotIOServer::readRobotIOCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "ReadRobotIOServer ready — service '/read_robot_io', topic '%s'",
        robot_io_status_topic_.c_str());
}

ReadRobotIOServer::~ReadRobotIOServer()
{
}

// ===================================================================
// 订阅者回调 — SDK 内部线程或 ROS executor 线程
// ===================================================================
void ReadRobotIOServer::robotIOStatusCallback(
    const ivg_interfaces::msg::RobotIOStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(io_state_mutex_);
    current_io_state_ = *msg;
    io_state_received_.store(true, std::memory_order_release);
}

// ===================================================================
// 服务回调 — io_type 对大小写不敏感
// ===================================================================
void ReadRobotIOServer::readRobotIOCallback(
    const std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ReadRobotIO::Response> res)
{
    res->success = false;
    res->value   = 0.0;
    res->message = "";

    // 检查是否已收到 IO 状态 (atomic, 不加锁)
    if (!io_state_received_.load(std::memory_order_acquire)) {
        res->message = "IO status not yet received — /robot_io_status topic may not be published";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 归一化 io_type
    std::string io_type = req->io_type;
    std::transform(io_type.begin(), io_type.end(), io_type.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    const int idx = req->io_index;

    std::lock_guard<std::mutex> lock(io_state_mutex_);

    // ---- 控制箱数字量 ----
    if (io_type == "digital_input" || io_type == "digital") {
        if (idx >= 0 && static_cast<size_t>(idx) < current_io_state_.digital_inputs.size()) {
            res->value   = current_io_state_.digital_inputs[idx] ? 1.0 : 0.0;
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "digital_input index " + std::to_string(idx) + " out of range";
        }
    }
    else if (io_type == "digital_output") {
        if (idx >= 0 && static_cast<size_t>(idx) < current_io_state_.digital_outputs.size()) {
            res->value   = current_io_state_.digital_outputs[idx] ? 1.0 : 0.0;
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "digital_output index " + std::to_string(idx) + " out of range";
        }
    }
    // ---- 控制箱模拟量 ----
    else if (io_type == "analog_input" || io_type == "analog") {
        if (idx >= 0 && static_cast<size_t>(idx) < current_io_state_.analog_inputs.size()) {
            res->value   = current_io_state_.analog_inputs[idx];
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "analog_input index " + std::to_string(idx) + " out of range";
        }
    }
    else if (io_type == "analog_output") {
        if (idx >= 0 && static_cast<size_t>(idx) < current_io_state_.analog_outputs.size()) {
            res->value   = current_io_state_.analog_outputs[idx];
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "analog_output index " + std::to_string(idx) + " out of range";
        }
    }
    // ---- 工具端 ----
    else if (io_type == "tool_digital_input") {
        const auto& t = current_io_state_.tool_io_status;
        if (idx >= 0 && static_cast<size_t>(idx) < t.digital_inputs.size()) {
            res->value   = t.digital_inputs[idx] ? 1.0 : 0.0;
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "tool_digital_input index " + std::to_string(idx) + " out of range";
        }
    }
    else if (io_type == "tool_digital_output") {
        const auto& t = current_io_state_.tool_io_status;
        if (idx >= 0 && static_cast<size_t>(idx) < t.digital_outputs.size()) {
            res->value   = t.digital_outputs[idx] ? 1.0 : 0.0;
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "tool_digital_output index " + std::to_string(idx) + " out of range";
        }
    }
    else if (io_type == "tool_analog_input") {
        const auto& t = current_io_state_.tool_io_status;
        if (idx >= 0 && static_cast<size_t>(idx) < t.analog_inputs.size()) {
            res->value   = t.analog_inputs[idx];
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "tool_analog_input index " + std::to_string(idx) + " out of range";
        }
    }
    else if (io_type == "tool_analog_output") {
        const auto& t = current_io_state_.tool_io_status;
        if (idx >= 0 && static_cast<size_t>(idx) < t.analog_outputs.size()) {
            res->value   = t.analog_outputs[idx];
            res->success = true;
            res->message = "ok";
        } else {
            res->message = "tool_analog_output index " + std::to_string(idx) + " out of range";
        }
    }
    else {
        res->message = "Unknown io_type '" + req->io_type +
            "'. Supported: digital_input, digital_output, analog_input, analog_output, "
            "tool_digital_input, tool_digital_output, tool_analog_input, tool_analog_output";
    }

    if (!res->success) {
        RCLCPP_WARN(this->get_logger(), "ReadRobotIO failed: %s", res->message.c_str());
    }
}

void ReadRobotIOServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

}  // namespace demo_driver

// ===================================================================
// main — MultiThreadedExecutor(2) 支持订阅者和服务并发处理
// ===================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    try {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto server = std::make_shared<demo_driver::ReadRobotIOServer>(node_options);
        executor.add_node(server);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("read_robot_io_server_node"),
                     "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
