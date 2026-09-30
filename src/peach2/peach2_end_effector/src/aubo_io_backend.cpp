#include "peach2_end_effector/aubo_io_backend.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace peach2_end_effector
{

AuboIoBackend::AuboIoBackend(
  NodeInterfaces node, const Config & config, rclcpp::CallbackGroup::SharedPtr group)
: config_(config),
  logger_(node.get_node_logging_interface()->get_logger().get_child("aubo_io"))
{
  if (config_.cmd_pin == config_.feedback_pin) {
    throw std::invalid_argument(
            "tool feedback_pin must differ from cmd_pin (" + std::to_string(config_.cmd_pin) +
            "): the command output cannot confirm itself");
  }
  if (config_.cmd_pin < 0 || config_.feedback_pin < 0) {
    throw std::invalid_argument("tool IO pins must be >= 0");
  }
  client_ = rclcpp::create_client<aubo_msgs::srv::SetIO>(
    node.get_node_base_interface(), node.get_node_graph_interface(),
    node.get_node_services_interface(), "/aubo_io_controller/set_io",
    rclcpp::ServicesQoS(), group);

  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  auto topics = node.get_node_topics_interface();
  sub_ = rclcpp::create_subscription<aubo_msgs::msg::IOState>(
    topics, "/aubo_io_controller/io_states", rclcpp::QoS(5).best_effort(),
    [this](const aubo_msgs::msg::IOState & msg) {on_io_state(msg);}, options);
}

bool AuboIoBackend::set_output(int pin, bool level)
{
  if (pin != config_.cmd_pin) {
    RCLCPP_ERROR(logger_, "refusing SetIO on pin %d (cmd_pin is %d)", pin, config_.cmd_pin);
    return false;
  }
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(logger_, "/aubo_io_controller/set_io not available");
    return false;
  }
  auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  request->fun = static_cast<int8_t>(config_.fun);
  request->pin = static_cast<int8_t>(pin);
  request->state = level ? 1.0F : 0.0F;
  auto future = client_->async_send_request(request);
  if (future.wait_for(config_.service_timeout) != std::future_status::ready) {
    client_->remove_pending_request(future.request_id);
    RCLCPP_ERROR(logger_, "SetIO fun=%d pin=%d timed out", config_.fun, pin);
    return false;
  }
  const bool ok = future.get()->success;
  if (!ok) {
    RCLCPP_ERROR(logger_, "SetIO fun=%d pin=%d rejected by controller", config_.fun, pin);
  }
  return ok;
}

std::optional<bool> AuboIoBackend::feedback(int pin)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pin < 0 || static_cast<size_t>(pin) >= tool_inputs_.size()) {
    return std::nullopt;
  }
  if (std::chrono::steady_clock::now() - received_ > config_.feedback_stale) {
    return std::nullopt;
  }
  return tool_inputs_[static_cast<size_t>(pin)];
}

void AuboIoBackend::on_io_state(const aubo_msgs::msg::IOState & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tool_inputs_.assign(msg.tool_io_states.size(), std::nullopt);
  for (const auto & d : msg.tool_io_states) {
    if (d.pin < tool_inputs_.size() && d.flag) {
      tool_inputs_[d.pin] = d.state;
    }
  }
  received_ = std::chrono::steady_clock::now();
}

}  // namespace peach2_end_effector
