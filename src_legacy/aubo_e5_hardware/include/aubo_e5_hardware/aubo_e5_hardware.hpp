#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "AuboRobotMetaType.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

class ServiceInterface;

namespace aubo_e5_hardware
{
class AuboE5Hardware final : public hardware_interface::SystemInterface
{
public:
  ~AuboE5Hardware() override;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  static void joint_state_callback(
    const aubo_robot_namespace::JointStatus * joint_status, int size, void * user_data);
  void update_joint_state(const aubo_robot_namespace::JointStatus * joint_status, int size);
  bool connect();
  void disconnect();
  void health_loop();

  static constexpr size_t kJointCount = 6;
  std::unique_ptr<ServiceInterface> service_;
  std::array<double, kJointCount> positions_{};
  std::array<double, kJointCount> commands_{};
  std::array<double, kJointCount> velocities_{};
  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  std::mutex sdk_call_mutex_;
  std::mutex health_mutex_;
  std::condition_variable health_cv_;
  std::thread health_thread_;
  std::atomic<bool> health_stop_{false};
  std::atomic<bool> reconnecting_{false};
  std::atomic<bool> health_failed_{false};
  std::chrono::steady_clock::time_point last_state_time_{};
  std::string server_host_{"127.0.0.1"};
  int server_port_{8899};
  double state_timeout_seconds_{3.0};
  bool enable_real_hardware_{false};
  bool allow_motion_commands_{false};
  bool connected_{false};
  bool received_state_{false};
};
}  // namespace aubo_e5_hardware
