#include "aubo_e5_hardware/aubo_e5_hardware.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <functional>

#include "AuboRobotMetaType.h"
#include "rclcpp/rclcpp.hpp"
#include "serviceinterface.h"

namespace aubo_e5_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

AuboE5Hardware::~AuboE5Hardware()
{
  disconnect();
}

CallbackReturn AuboE5Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJointCount) {
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != HW_IF_POSITION ||
        joint.state_interfaces.empty()) {
      return CallbackReturn::ERROR;
    }
  }
  auto value = [&](const char * name, const std::string & fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };
  server_host_ = value("server_host", server_host_);
  server_port_ = std::stoi(value("server_port", std::to_string(server_port_)));
  state_timeout_seconds_ = std::stod(value("state_timeout_seconds", "3.0"));
  enable_real_hardware_ = value("enable_real_hardware", "false") == "true";
  allow_motion_commands_ = value("allow_motion_commands", "false") == "true";
  if (state_timeout_seconds_ <= 0.0) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AuboE5Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 2);
  for (size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AuboE5Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION, &commands_[i]);
  }
  return interfaces;
}

CallbackReturn AuboE5Hardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!enable_real_hardware_) {
    return CallbackReturn::SUCCESS;
  }
  return connect() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn AuboE5Hardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!enable_real_hardware_) return CallbackReturn::SUCCESS;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!connected_ || !received_state_) {
    return CallbackReturn::ERROR;
  }
  commands_ = positions_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AuboE5Hardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (enable_real_hardware_) disconnect();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AuboE5Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!enable_real_hardware_) {
    positions_ = commands_;
    return hardware_interface::return_type::OK;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_state_time_).count();
  if (!allow_motion_commands_) {
    if (health_failed_.load()) {
      return hardware_interface::return_type::ERROR;
    }
    if (reconnecting_.load() || !received_state_ || age > state_timeout_seconds_) {
      velocities_.fill(0.0);
    }
    return hardware_interface::return_type::OK;
  }
  return received_state_ && age <= state_timeout_seconds_ ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type AuboE5Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!enable_real_hardware_) return hardware_interface::return_type::OK;
  if (!allow_motion_commands_) return hardware_interface::return_type::OK;
  if (!connected_ || !service_) return hardware_interface::return_type::ERROR;
  double target[kJointCount];
  { std::lock_guard<std::mutex> lock(state_mutex_); std::copy(commands_.begin(), commands_.end(), target); }
  std::lock_guard<std::mutex> sdk_lock(sdk_call_mutex_);
  return service_->robotServiceFollowModeJointMove(target) == 0 ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool AuboE5Hardware::connect()
{
  health_failed_.store(false);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_state_ = false;
    velocities_.fill(0.0);
  }
  service_ = std::make_unique<ServiceInterface>();
  if (service_->robotServiceLogin(server_host_.c_str(), server_port_, "aubo", "123456") != 0) {
    service_.reset(); return false;
  }
  if (service_->robotServiceRobotHandShake(true) != 0) {
    disconnect();
    return false;
  }
  std::array<aubo_robot_namespace::JointStatus, kJointCount> initial_state{};
  int initial_state_result = -1;
  constexpr int kInitialStateAttempts = 5;
  for (int attempt = 1; attempt <= kInitialStateAttempts; ++attempt) {
    {
      std::lock_guard<std::mutex> sdk_lock(sdk_call_mutex_);
      initial_state_result = service_->robotServiceGetRobotJointStatus(
        initial_state.data(), static_cast<int>(initial_state.size()));
    }
    if (initial_state_result == 0) {
      break;
    }
    RCLCPP_WARN(
      get_logger(), "Initial AUBO joint-state read failed (ret=%d), attempt %d/%d",
      initial_state_result, attempt, kInitialStateAttempts);
    if (attempt < kInitialStateAttempts) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  if (initial_state_result != 0) {
    disconnect();
    return false;
  }
  update_joint_state(initial_state.data(), static_cast<int>(initial_state.size()));
  connected_ = true;
  reconnecting_.store(false);
  health_stop_.store(false);
  health_thread_ = std::thread(&AuboE5Hardware::health_loop, this);
  return true;
}

void AuboE5Hardware::disconnect()
{
  health_stop_.store(true);
  health_cv_.notify_all();
  if (health_thread_.joinable()) {
    health_thread_.join();
  }
  if (service_) {
    std::lock_guard<std::mutex> sdk_lock(sdk_call_mutex_);
    service_->robotServiceLogout();
  }
  service_.reset();
  connected_ = false;
  reconnecting_.store(false);
  std::lock_guard<std::mutex> lock(state_mutex_);
  received_state_ = false;
}

void AuboE5Hardware::health_loop()
{
  std::unique_lock<std::mutex> health_lock(health_mutex_);
  size_t poll_count = 0;
  while (!health_stop_.load()) {
    health_lock.unlock();
    std::array<aubo_robot_namespace::JointStatus, kJointCount> state{};
    int result = -1;
    {
      std::lock_guard<std::mutex> sdk_lock(sdk_call_mutex_);
      if (service_) {
        ++poll_count;
        result = poll_count % 50 == 0 ? service_->robotServiceRobotHandShake(true) : 0;
        if (result == 0) {
          result = service_->robotServiceGetRobotJointStatus(
            state.data(), static_cast<int>(state.size()));
        }
      }
    }
    if (result != 0) {
      bool recovered = false;
      if (!allow_motion_commands_ && !health_stop_.load()) {
        reconnecting_.store(true);
        RCLCPP_WARN(
          get_logger(),
          "AUBO SDK state/heartbeat failed; starting read-only reconnect attempts");
        constexpr int kReconnectAttempts = 5;
        for (int attempt = 1; attempt <= kReconnectAttempts && !health_stop_.load(); ++attempt) {
          {
            std::lock_guard<std::mutex> sdk_lock(sdk_call_mutex_);
            if (service_) {
              service_->robotServiceLogout();
            }
            service_ = std::make_unique<ServiceInterface>();
            result = service_->robotServiceLogin(
              server_host_.c_str(), server_port_, "aubo", "123456");
            if (result == 0) {
              result = service_->robotServiceRobotHandShake(true);
            }
            if (result == 0) {
              result = service_->robotServiceGetRobotJointStatus(
                state.data(), static_cast<int>(state.size()));
            }
            recovered = result == 0;
          }
          if (recovered) {
            RCLCPP_INFO(get_logger(), "AUBO SDK read-only connection recovered");
            poll_count = 0;
            break;
          }
          RCLCPP_WARN(
            get_logger(), "AUBO SDK read-only reconnect attempt %d/%d failed",
            attempt, kReconnectAttempts);
          if (attempt < kReconnectAttempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
          }
        }
      }
      if (health_stop_.load()) {
        break;
      }
      if (!recovered) {
        RCLCPP_ERROR(
          get_logger(),
          "AUBO SDK connection could not be recovered; hardware state will be deactivated");
        health_failed_.store(true);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        received_state_ = false;
        reconnecting_.store(false);
        break;
      }
    }
    update_joint_state(state.data(), static_cast<int>(state.size()));
    reconnecting_.store(false);
    health_lock.lock();
    if (health_cv_.wait_for(
        health_lock, std::chrono::milliseconds(100), [this]() {return health_stop_.load();}))
    {
      break;
    }
  }
}

void AuboE5Hardware::joint_state_callback(
  const aubo_robot_namespace::JointStatus * state, int size, void * user_data)
{
  if (user_data) static_cast<AuboE5Hardware *>(user_data)->update_joint_state(state, size);
}

void AuboE5Hardware::update_joint_state(
  const aubo_robot_namespace::JointStatus * state, int size)
{
  if (!state || size < static_cast<int>(kJointCount)) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const double dt = received_state_ ?
    std::chrono::duration<double>(now - last_state_time_).count() : 0.0;
  for (size_t i = 0; i < kJointCount; ++i) {
    const double new_position = state[i].jointPosJ;
    velocities_[i] = dt > 0.0001 ? (new_position - positions_[i]) / dt : 0.0;
    positions_[i] = new_position;
  }
  last_state_time_ = now;
  received_state_ = true;
  state_cv_.notify_all();
}
}  // namespace aubo_e5_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(aubo_e5_hardware::AuboE5Hardware, hardware_interface::SystemInterface)
