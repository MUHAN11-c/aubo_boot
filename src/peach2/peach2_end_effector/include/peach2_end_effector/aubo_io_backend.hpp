#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <vector>

#include "aubo_msgs/msg/io_state.hpp"
#include "aubo_msgs/srv/set_io.hpp"
#include "peach2_end_effector/io_backend.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"

namespace peach2_end_effector
{

/// Tool IO through aubo_io_controller (driver stack, read-only for us):
///   write: service `/aubo_io_controller/set_io` (aubo_msgs/SetIO, fun from the tool profile)
///   read:  topic `/aubo_io_controller/io_states` (aubo_msgs/IOState.tool_io_states[pin])
///
/// Tool DO and tool DI are distinct channels on the AUBO tool flange; the command pin must
/// never be used as its own feedback (old stack read DO pin0 back as "blade closed", P0-3),
/// so the constructor rejects cmd_pin == feedback_pin.
///
/// set_output() blocks on the service response with a bounded wait. Call it only from a
/// worker thread; the client lives in `group`, which must be spun by another executor thread.
class AuboIoBackend : public IoBackend
{
public:
  using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeGraphInterface,
    rclcpp::node_interfaces::NodeServicesInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface>;

  struct Config
  {
    int fun{3};                   ///< SetIO fun; tool profile io.fun
    int cmd_pin{0};
    int feedback_pin{1};
    std::chrono::milliseconds service_timeout{1000};
    std::chrono::milliseconds feedback_stale{500};
  };

  AuboIoBackend(
    NodeInterfaces node, const Config & config,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  bool set_output(int pin, bool level) override;
  std::optional<bool> feedback(int pin) override;
  /// TODO(M0): no actuator current channel wired on the current tool; always nullopt.
  std::optional<double> current() override {return std::nullopt;}

private:
  void on_io_state(const aubo_msgs::msg::IOState & msg);

  Config config_;
  rclcpp::Logger logger_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr client_;
  rclcpp::Subscription<aubo_msgs::msg::IOState>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<std::optional<bool>> tool_inputs_;
  std::chrono::steady_clock::time_point received_{};
};

}  // namespace peach2_end_effector
