// Copyright 2024, Universal Robots A/S (original passthrough_trajectory_controller)
// Copyright 2026, aubo_e5_ros2_ws authors (AUBO E5 rewrite)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * AuboPassthroughTrajectoryController 实现 —— 与 UR 原版的差异详见头文件
 * （remapJointNames、blendToFirstPoint、goal_hold 延迟 result、抢占）。
 * 行为蓝本：aubo_boot 的 joint_trajectory_controller。
 */
//----------------------------------------------------------------------

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <sstream>
#include <tuple>
#include <unordered_map>

#include <rclcpp/logging.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "aubo_e5_controllers/aubo_passthrough_trajectory_controller.hpp"

namespace aubo_e5_controllers
{

// 非实时上下文向实时 box 写值：try_set 失败时带重试（实时线程持锁时稍等再试）
template<class RTBox, class ValueType>
bool set_rt_box_from_non_rt(RTBox & rt_box, const ValueType & value)
{
  int tries = 0;
  while (!rt_box.try_set(value)) {
    if (tries > 9) {
      RCLCPP_ERROR(rclcpp::get_logger("AuboPassthroughTrajectoryController"),
                   "Failed to set value in realtime box.");
      return false;
    }
    RCLCPP_WARN(rclcpp::get_logger("AuboPassthroughTrajectoryController"),
                "Waiting for value to be set in realtime box, retrying...");
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    tries++;
  }
  return true;
}

double duration_to_double(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec + (duration.nanosec / 1000000000.0);
}

builtin_interfaces::msg::Duration double_to_duration(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(seconds);
  duration.nanosec = static_cast<uint32_t>((seconds - duration.sec) * 1e9);
  return duration;
}

// 非实时上下文从实时 box 读当前活动 goal（同样带重试）
std::optional<AuboPassthroughTrajectoryController::RealtimeGoalHandlePtr>
AuboPassthroughTrajectoryController::get_rt_goal_from_non_rt()
{
  RealtimeGoalHandlePtr active_goal = nullptr;
  int tries = 0;
  while (!rt_active_goal_.try_get([&active_goal](const RealtimeGoalHandlePtr & goal) {
      active_goal = goal;
      }))
  {
    if (tries > 9) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get active goal from realtime box.");
      return std::nullopt;
    }
    RCLCPP_WARN(get_node()->get_logger(), "Waiting for active goal to be read, retrying...");
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    tries++;
  }
  return active_goal;
}

// 非实时上下文向实时 box 写当前活动 goal
bool AuboPassthroughTrajectoryController::set_rt_goal_from_non_rt(RealtimeGoalHandlePtr & rt_goal)
{
  int tries = 0;
  while (!rt_active_goal_.try_set([&rt_goal](RealtimeGoalHandlePtr & goal) {goal = rt_goal;})) {
    if (tries > 9) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set active goal in realtime box.");
      return false;
    }
    RCLCPP_WARN(get_node()->get_logger(), "Waiting for active goal to be set, retrying...");
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    tries++;
  }
  return true;
}

controller_interface::CallbackReturn AuboPassthroughTrajectoryController::on_init()
{
  param_listener_ =
    std::make_shared<aubo_passthrough_trajectory_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  current_index_ = 0;
  joint_names_ = params_.joints;
  number_of_joints_ = joint_names_.size();
  state_interface_types_ = params_.state_interfaces;
  clock_ = get_node()->get_clock();
  return controller_interface::CallbackReturn::SUCCESS;
}

// 不在实时循环中执行
controller_interface::CallbackReturn
AuboPassthroughTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  if (number_of_joints_ == 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Parameter 'joints' is empty, cannot configure controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // N6：tf_prefix 防御 —— 当前 aubo_description 的 xacro 未提供 prefix
  // 机制，导出的接口名固定无前缀；非空 tf_prefix 会导致接口找不到、
  // 激活失败。这里提前 WARN 指明根因（不阻断，交由接口认领失败兜底）。
  if (!params_.tf_prefix.empty()) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "tf_prefix is '%s', but the AUBO E5 URDF has no prefix mechanism: "
        "non-empty prefix will make interface lookup fail at activation.",
        params_.tf_prefix.c_str());
  }

  start_action_server();
  trajectory_active_ = false;

  joint_state_interface_names_.clear();
  joint_state_interface_names_.reserve(number_of_joints_ * state_interface_types_.size());
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      joint_state_interface_names_.emplace_back(joint_name + "/" + interface_type);
    }
  }

  return ControllerInterface::on_configure(previous_state);
}

void AuboPassthroughTrajectoryController::start_action_server(void)
{
  send_trajectory_action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
      std::bind(&AuboPassthroughTrajectoryController::goal_received_callback, this,
      std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&AuboPassthroughTrajectoryController::goal_cancelled_callback, this,
      std::placeholders::_1),
      std::bind(&AuboPassthroughTrajectoryController::goal_accepted_callback, this,
      std::placeholders::_1));
  return;
}

controller_interface::InterfaceConfiguration AuboPassthroughTrajectoryController::
state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::copy(joint_state_interface_names_.cbegin(), joint_state_interface_names_.cend(),
      std::back_inserter(conf.names));

  conf.names.push_back(params_.speed_scaling_interface_name);

  // 反馈增强：机器人侧真实目标（追加在列表尾部，不得插入中间 ——
  // on_activate 按名查找，顺序本身不参与下标语义，但保持尾部追加惯例）。
  const std::string tf_prefix = params_.tf_prefix;
  for (size_t i = 0; i < number_of_joints_; ++i) {
    conf.names.emplace_back(tf_prefix + "aubo_io/tag_pos_" + std::to_string(i));
  }
  for (size_t i = 0; i < number_of_joints_; ++i) {
    conf.names.emplace_back(tf_prefix + "aubo_io/tag_vel_" + std::to_string(i));
  }

  return conf;
}

controller_interface::InterfaceConfiguration AuboPassthroughTrajectoryController::
command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // 顺序有讲究：update() 按 i*3 + {0,1,2} 的下标访问各 setpoint 接口。
  for (size_t i = 0; i < number_of_joints_; ++i) {
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_positions_" +
        std::to_string(i));
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_velocities_" +
        std::to_string(i));
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_accelerations_" +
        std::to_string(i));
  }

  config.names.push_back(tf_prefix + "trajectory_passthrough/abort");
  config.names.emplace_back(tf_prefix + "trajectory_passthrough/transfer_state");
  config.names.emplace_back(tf_prefix + "trajectory_passthrough/time_from_start");
  config.names.emplace_back(tf_prefix + "trajectory_passthrough/trajectory_size");

  return config;
}

controller_interface::CallbackReturn
AuboPassthroughTrajectoryController::on_activate(const rclcpp_lifecycle::State & state)
{
  // 清空 vector，防止重新激活时残留旧数据
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  tag_pos_state_interface_.clear();
  tag_vel_state_interface_.clear();

  for (auto & interface_name : joint_state_interface_names_) {
    auto interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](auto & interface) {return  interface.get_name() == interface_name;});
    if (interface_it != state_interfaces_.end()) {
      if (interface_it->get_interface_name() == "position") {
        joint_position_state_interface_.emplace_back(*interface_it);
      } else if (interface_it->get_interface_name() == "velocity") {
        joint_velocity_state_interface_.emplace_back(*interface_it);
      }
    }
  }

  if (joint_position_state_interface_.size() != number_of_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Did not find position state interfaces for all %zu joints.",
                 number_of_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto & interface) {
        return  interface.get_name() == params_.speed_scaling_interface_name;
  });
  if (it != state_interfaces_.end()) {
    scaling_state_interface_ = *it;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Did not find speed scaling interface in state interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 机器人侧真实目标接口（硬依赖：sim/real 硬件插件必定导出；找不到说明
  // 硬件插件与控制器版本不匹配，直接激活失败比静默失真反馈更安全）。
  const std::string tf_prefix = params_.tf_prefix;
  for (size_t i = 0; i < number_of_joints_; ++i) {
    const std::string pos_name = tf_prefix + "aubo_io/tag_pos_" + std::to_string(i);
    const std::string vel_name = tf_prefix + "aubo_io/tag_vel_" + std::to_string(i);
    auto pos_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](auto & interface) {return interface.get_name() == pos_name;});
    auto vel_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](auto & interface) {return interface.get_name() == vel_name;});
    if (pos_it == state_interfaces_.end() || vel_it == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Did not find '%s'/'%s' in state interfaces (hardware plugin must export them).",
                   pos_name.c_str(), vel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    tag_pos_state_interface_.emplace_back(*pos_it);
    tag_vel_state_interface_.emplace_back(*vel_it);
  }

  const std::pair<const char *,
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
  command_ifs[] = {
    {"trajectory_passthrough/abort", &abort_command_interface_},
    {"trajectory_passthrough/trajectory_size", &trajectory_size_command_interface_},
    {"trajectory_passthrough/transfer_state", &transfer_command_interface_},
    {"trajectory_passthrough/time_from_start", &time_from_start_command_interface_},
  };
  for (auto & [short_name, target] : command_ifs) {
    const std::string interface_name = tf_prefix + short_name;
    auto cmd_it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
        [&](auto & interface) {return  interface.get_name() == interface_name;});
    if (cmd_it != command_interfaces_.end()) {
      *target = *cmd_it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.",
          interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn
AuboPassthroughTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!abort_command_interface_->get().set_value(1.0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not write to abort command interface.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (trajectory_active_) {
    RealtimeGoalHandlePtr active_goal = nullptr;
    bool success = rt_active_goal_.try_get([&active_goal](const RealtimeGoalHandlePtr & goal) {
          active_goal = goal;
          });
    if (!success) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Could not read active goal from realtime buffer, deactivation of "
                                             "controller failed.");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (active_goal) {
      std::shared_ptr<FollowJTrajAction::Result> result =
        std::make_shared<FollowJTrajAction::Result>();
      result->set__error_string(
          "Aborting current goal, since the controller is being deactivated.");
      active_goal->setAborted(result);
    }
    success = rt_active_goal_.try_set([](RealtimeGoalHandlePtr & goal) {
          goal = RealtimeGoalHandlePtr();
          });
    if (!success) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Failed to set active goal, deactivation of controller failed.");
      return controller_interface::CallbackReturn::ERROR;
    }
    end_goal();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AuboPassthroughTrajectoryController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  RealtimeGoalHandlePtr active_goal = nullptr;
  const bool read_success =
    rt_active_goal_.try_get([&active_goal](const RealtimeGoalHandlePtr & goal) {
        active_goal = goal;
      });
  if (!read_success) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Could not read active goal from realtime buffer, skipping cycle.");
    return controller_interface::return_type::OK;
  }

  const auto current_transfer_state =
    transfer_command_interface_->get().get_optional().value_or(TRANSFER_STATE_IDLE);

  bool write_success = true;
  if (active_goal && trajectory_active_) {
    if (current_transfer_state != TRANSFER_STATE_IDLE) {
      // 检查轨迹是否被硬件中止（例如安全事件）。该判断只在我们的传输已经
      // 开始（current_index_ > 0）后才有意义；在此之前 abort 可能是我们自己
      // 在抢占上一条传输时写入的。
      if (abort_command_interface_->get().get_optional().value_or(0.0) == 1.0 &&
        current_index_ > 0)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory aborted by hardware, aborting action.");
        std::shared_ptr<FollowJTrajAction::Result> result =
          std::make_shared<FollowJTrajAction::Result>();
        result->set__error_string("Trajectory aborted by hardware.");
        active_goal->setAborted(result);
        end_goal();
        return controller_interface::return_type::OK;
      }
    }

    if (current_index_ == 0) {
      if (current_transfer_state == TRANSFER_STATE_IDLE) {
        // 开始传输一条新轨迹（已重排、必要时前面补了融合段）。
        size_t traj_size = 0;
        if (!rt_active_traj_.try_get(
            [&traj_size](const trajectory_msgs::msg::JointTrajectory & traj) {
              traj_size = traj.points.size();
            }))
        {
          RCLCPP_ERROR(get_node()->get_logger(),
              "Could not read active trajectory, skipping cycle.");
          return controller_interface::return_type::OK;
        }
        goal_hold_count_ = 0;
        last_goal_check_time_ = time;
        goal_start_time_ = time;
        write_success &= abort_command_interface_->get().set_value(0.0);
        write_success &=
          transfer_command_interface_->get().set_value(TRANSFER_STATE_NEW_TRAJECTORY);
        write_success &=
          trajectory_size_command_interface_->get().set_value(static_cast<double>(traj_size));
        transfer_requested_ = true;
      } else if (!transfer_requested_) {
        // transfer_requested_ 标志修复的 bug：本 else 是“抢占锁存”分支——我们
        // 抢占了上一条传输（或硬件还没排空上一条传输的余量），于是持续锁存
        // abort=1.0 直到硬件回到 IDLE。但若硬件回 1.0（WAITING_FOR_POINT）是
        // 在应答我们自己发起的传输，这个 else 会误触发：abort=1 让硬件丢掉首
        // 点、状态机卡死形成死锁。因此加 !transfer_requested_ 守卫，绝不为自
        // 己发起的传输锁存 abort。UR 原版用单一组合条件天然规避（ur_controllers/
        // src/passthrough_trajectory_controller.cpp:359 附近：
        // `if (current_index_ == 0 && current_transfer_state == TRANSFER_STATE_IDLE)`，
        // 硬件非 IDLE 时什么都不写）。
        write_success &= abort_command_interface_->get().set_value(1.0);
      }
    }

    // 硬件读完上一个点后，才向命令接口写入新的点（每周期一点的交替协议）。
    if (current_transfer_state == TRANSFER_STATE_WAITING_FOR_POINT) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      size_t traj_size = 0;
      if (!rt_active_traj_.try_get([&](const trajectory_msgs::msg::JointTrajectory & traj) {
          traj_size = traj.points.size();
          if (current_index_ < traj_size) {
            point = traj.points[current_index_];
          }
          }))
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not read active trajectory, skipping cycle.");
        return controller_interface::return_type::OK;
      }

      if (current_index_ < traj_size) {
        // 写入该点的 time_from_start 参数。
        write_success &=
          time_from_start_command_interface_->get().set_value(duration_to_double(
            point.time_from_start));

        // 轨迹点在接受时已重排到权威关节顺序，速度/加速度缺省已补 0，因此
        // 这里可以无条件整组写入。命令接口的下标顺序为 i*3 + {0,1,2}
        // （位置/速度/加速度），由 command_interface_configuration() 固定。
        for (size_t i = 0; i < number_of_joints_; i++) {
          write_success &= command_interfaces_[i * 3].set_value(point.positions[i]);
          write_success &= command_interfaces_[i * 3 + 1].set_value(point.velocities[i]);
          write_success &= command_interfaces_[i * 3 + 2].set_value(point.accelerations[i]);
        }
        // 通知硬件接口：这个点已写好，可以读取。
        write_success &= transfer_command_interface_->get().set_value(TRANSFER_STATE_TRANSFERRING);
        current_index_++;
        // 检查是否所有点都已写入硬件接口。
      } else if (current_index_ == traj_size) {
        write_success &= transfer_command_interface_->get().set_value(TRANSFER_STATE_TRANSFER_DONE);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Hardware waiting for trajectory point while none is present!");
      }

      // 硬件正在执行轨迹。
    } else if (current_transfer_state == TRANSFER_STATE_IN_MOTION) {
      if (effective_goal_time_ > 0.0 &&
        (time - goal_start_time_).seconds() > max_trajectory_time_ + effective_goal_time_)
      {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock_, 1000,
                             "Trajectory should be finished by now. You may want to cancel this "
                             "goal, if it is not.");
      }

      // 硬件上报 DONE：goal_hold 延迟 result（aubo_boot 语义）。
      // 每隔 goal_check_ms 检查一次：所有关节满足 |实际 - 终点| < 位置容差
      // 且 |速度| < 速度容差，连续 goal_hold_frames 次通过才回成功。
    } else if (current_transfer_state == TRANSFER_STATE_DONE) {
      const double check_period_s = static_cast<double>(params_.goal_check_ms) / 1000.0;
      if ((time - last_goal_check_time_).seconds() >= check_period_s) {
        last_goal_check_time_ = time;

        trajectory_msgs::msg::JointTrajectoryPoint final_point;
        std::vector<control_msgs::msg::JointTolerance> tolerances;
        bool traj_ok =
          rt_active_traj_.try_get(
          [&final_point](const trajectory_msgs::msg::JointTrajectory & traj) {
            if (!traj.points.empty()) {
              final_point = traj.points.back();
            }
        });
        bool tol_ok = goal_tolerance_.try_get(
          [&tolerances](const std::vector<control_msgs::msg::JointTolerance> & tol) {
            tolerances = tol;
        });
        if (!traj_ok || !tol_ok) {
          RCLCPP_ERROR(get_node()->get_logger(),
              "Could not read goal data for goal-hold check, skipping cycle.");
          return controller_interface::return_type::OK;
        }

        if (withinGoalHold(final_point, tolerances)) {
          if (++goal_hold_count_ >= params_.goal_hold_frames) {
            auto result = active_goal->preallocated_result_;
            result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
            result->error_string = "Trajectory executed successfully (goal-hold confirmed in " +
              std::to_string((time - goal_start_time_).seconds()) + " s).";
            active_goal->setSucceeded(result);
            end_goal();
            RCLCPP_INFO(get_node()->get_logger(), "%s", result->error_string.c_str());
          }
        } else {
          goal_hold_count_ = 0;
        }
      }

      // goal_time 超时 -> aborted（延迟 result 语义：不提前成功、超时才失败）。
      if (trajectory_active_ && effective_goal_time_ > 0.0 &&
        (time - goal_start_time_).seconds() > max_trajectory_time_ + effective_goal_time_)
      {
        auto result = active_goal->preallocated_result_;
        result->error_code = FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Goal not reached within goal_time. Missed goal time by " +
          std::to_string((time - goal_start_time_).seconds() - max_trajectory_time_ -
                                              effective_goal_time_) +
          " seconds.";
        write_success &= abort_command_interface_->get().set_value(1.0);
        active_goal->setAborted(result);
        end_goal();
        RCLCPP_ERROR(get_node()->get_logger(), "%s", result->error_string.c_str());
      }
    }

    // 反馈：actual 取关节状态接口的实际位置/速度。
    // desired 语义变化（反馈增强）：取机器人控制器当前正在执行的目标
    // （aubo_io/tag_pos_*、tag_vel_*，来自 SDK 推送帧），而不是"正在透传
    // 的 setpoint"。原因：RIB 板载缓冲最深约 2 秒，"正在透传的点"比"机器
    // 人正在执行的点"相位超前可达 ~2s，用它做 desired 会让反馈严重失真；
    // tag_* 与执行进度同相。tag 接口取不到值时回退到旧的透传点行为。
    if (trajectory_active_) {
      auto & feedback = active_goal->preallocated_feedback_;
      if (feedback) {
        feedback->header.stamp = time;
        for (size_t i = 0; i < number_of_joints_; ++i) {
          feedback->actual.positions[i] =
            joint_position_state_interface_[i].get().get_optional().value_or(0.0);
          if (i < feedback->actual.velocities.size() &&
            i < joint_velocity_state_interface_.size())
          {
            feedback->actual.velocities[i] =
              joint_velocity_state_interface_[i].get().get_optional().value_or(0.0);
          }
        }
        // 先尝试机器人侧目标；任一接口读不到值（暂未推送）则整体回退。
        bool tag_ok = tag_pos_state_interface_.size() == number_of_joints_ &&
          tag_vel_state_interface_.size() == number_of_joints_;
        if (tag_ok) {
          // error = desired - actual（位置、速度都填；此前 error.velocities
          // 预分配了但从不填充、恒为 0）。
          for (size_t i = 0; i < number_of_joints_; ++i) {
            const auto p = tag_pos_state_interface_[i].get().get_optional();
            const auto v = tag_vel_state_interface_[i].get().get_optional();
            if (!p.has_value() || !v.has_value()) {
              tag_ok = false;
              break;
            }
            feedback->desired.positions[i] = p.value();
            feedback->desired.velocities[i] = v.value();
            feedback->error.positions[i] = p.value() - feedback->actual.positions[i];
            feedback->error.velocities[i] = v.value() - feedback->actual.velocities[i];
          }
        }
        if (!tag_ok) {
          rt_active_traj_.try_get([&](const trajectory_msgs::msg::JointTrajectory & traj) {
              if (!traj.points.empty()) {
                const auto & desired = traj.points[std::min(current_index_.load(),
                  traj.points.size() - 1)];
                feedback->desired.positions = desired.positions;
                feedback->desired.velocities = desired.velocities;
                for (size_t i = 0; i < number_of_joints_; ++i) {
                  feedback->error.positions[i] = desired.positions[i] -
                  feedback->actual.positions[i];
                  if (i < desired.velocities.size()) {
                    feedback->error.velocities[i] = desired.velocities[i] -
                    feedback->actual.velocities[i];
                  }
                }
              }
          });
        }
        active_goal->setFeedback(feedback);
      }
    }
  } else if (current_transfer_state != TRANSFER_STATE_IDLE &&  // NOLINT(readability/braces)
    current_transfer_state != TRANSFER_STATE_DONE)
  {
    // 没有活动 goal，但状态机不在 IDLE：说明 goal 已被取消，锁存 abort 让
    // 硬件清空队列回到 IDLE。
    write_success &= abort_command_interface_->get().set_value(1.0);

  } else if (current_transfer_state == TRANSFER_STATE_DONE) {
    // 硬件告知轨迹执行完毕，这里复位状态机。
    write_success &= transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE);
    write_success &= abort_command_interface_->get().set_value(0.0);
  }
  if (!write_success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not write to a command interfaces.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse AuboPassthroughTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory.");
  // 前提：控制器处于活动状态
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Can't accept new trajectories. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Can't accept new trajectory. Trajectory has no points.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // 校验轨迹各部分是否合法。注意：正在执行的轨迹不会拒绝新 goal——
  // 新 goal 直接抢占它（aubo_boot handleAccepted 语义）。
  if (!check_goal_joints(goal) || !check_goal_positions(goal) || !check_goal_velocities(goal) ||
    !check_goal_accelerations(goal) || !check_goal_tolerances(goal))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool AuboPassthroughTrajectoryController::check_goal_joints(
  const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const
{
  // aubo_boot remapJointNames 语义：含未知关节或缺关节的 goal 拒绝；
  // 权威关节集合中的每个关节必须恰好出现一次。
  const auto & names = goal->trajectory.joint_names;
  if (names.size() != number_of_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Can't accept new trajectory. Goal has %zu joints, this controller controls %zu "
                 "joints.",
                 names.size(), number_of_joints_);
    return false;
  }
  for (const auto & name : names) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. Unknown joint '%s'.",
          name.c_str());
      return false;
    }
    if (std::count(names.begin(), names.end(), name) > 1) {
      RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. Duplicate joint '%s'.",
          name.c_str());
      return false;
    }
  }
  return true;
}

bool AuboPassthroughTrajectoryController::check_goal_tolerances(
  const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const
{
  auto & tolerances = goal->goal_tolerance;

  if (!tolerances.empty()) {
    for (auto & tol : tolerances) {
      auto found_it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
      if (found_it == joint_names_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Tolerance for joint '%s' given. This joint is not known to this controller.",
            tol.name.c_str());
        return false;
      }
    }
    if (tolerances.size() != number_of_joints_) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Tolerances for %lu joints given. This controller knows %lu joints.",
                   tolerances.size(), number_of_joints_);
      return false;
    }
  }
  return true;
}

bool AuboPassthroughTrajectoryController::check_goal_positions(
  const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].positions.size() != number_of_joints_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Can't accept new trajectory. All trajectory points must have positions for "
                   "all joints of the robot (%zu joint positions per point). Point nr %d has: "
                   "%zu positions.",
                   number_of_joints_, i + 1, goal->trajectory.points[i].positions.size());
      return false;
    }
  }
  return true;
}

bool AuboPassthroughTrajectoryController::check_goal_velocities(
  const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].velocities.size() != number_of_joints_ &&
      goal->trajectory.points[i].velocities.size() != 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Can't accept new trajectory. All trajectory points must either not have "
                   "velocities or have them for all joints of the robot (%zu joint velocities "
                   "per point). Point nr %d has: %zu velocities.",
                   number_of_joints_, i + 1, goal->trajectory.points[i].velocities.size());
      return false;
    }
    if (goal->trajectory.points[i].velocities.size() !=
      goal->trajectory.points[0].velocities.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Can't accept new trajectory. All trajectory points must consistently have "
                   "velocities for all joints of the robot (%zu joint velocities per point). "
                   "Point nr %d has: %zu velocities.",
                   number_of_joints_, i, goal->trajectory.points[i].velocities.size());
      return false;
    }
  }
  return true;
}

bool AuboPassthroughTrajectoryController::check_goal_accelerations(
  const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].accelerations.size() != 0 &&
      goal->trajectory.points[i].accelerations.size() != number_of_joints_)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Can't accept new trajectory. All trajectory points must either not have "
                   "accelerations or have them for all joints of the robot (%zu joint "
                   "accelerations per point). Point nr %d has: %zu accelerations.",
                   number_of_joints_, i, goal->trajectory.points[i].accelerations.size());
      return false;
    }
    if (goal->trajectory.points[i].accelerations.size() !=
      goal->trajectory.points[0].accelerations.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Can't accept new trajectory. All trajectory points must consistently have "
                   "accelerations for all joints of the robot (%zu joint accelerations per "
                   "point). Point nr %d has: %zu accelerations.",
                   number_of_joints_, i, goal->trajectory.points[i].accelerations.size());
      return false;
    }
  }
  return true;
}

rclcpp_action::CancelResponse AuboPassthroughTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // 检查取消请求是否对应当前活动的 goal（若有）
  auto goal = get_rt_goal_from_non_rt();
  if (!goal.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get goal handle.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  RealtimeGoalHandlePtr active_goal = goal.value();

  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory requested.");

    // 取消/抢占原语（aubo_boot）：请求硬件中止；硬件清空队列并丢弃 RIB 余量
    // （由 IO 异步线程发 RobotMoveStop），然后回到 IDLE。
    if (abort_command_interface_.has_value()) {
      std::ignore = abort_command_interface_->get().set_value(1.0);
    }

    auto new_goal = RealtimeGoalHandlePtr();
    if (!set_rt_goal_from_non_rt(new_goal)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset active goal, cancel request failed.");
      return rclcpp_action::CancelResponse::REJECT;
    }

    // 把当前 goal 标记为已取消
    auto result = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(result);

    trajectory_active_ = false;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// action goal 已被接受，为新轨迹初始化各项状态。
void AuboPassthroughTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Accepted new trajectory with " << goal->trajectory.points.size() <<
      " points.");

  // 抢占（aubo_boot handleAccepted）：中止正在运行的传输；等硬件回到
  // IDLE 后由 update() 启动新传输。
  if (trajectory_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Preempting currently active trajectory.");
    if (abort_command_interface_.has_value()) {
      std::ignore = abort_command_interface_->get().set_value(1.0);
    }
    auto old_goal = get_rt_goal_from_non_rt();
    if (old_goal.has_value() && old_goal.value()) {
      auto preempted_result = std::make_shared<FollowJTrajAction::Result>();
      preempted_result->set__error_string("Preempted by a new goal.");
      old_goal.value()->setAborted(preempted_result);
    }
  }
  current_index_ = 0;

  // 按关节名重排到权威顺序（aubo_boot remapJointNames）。
  auto traj = remapJointNames(goal->trajectory);

  // 首轨迹点与当前位置偏差超过阈值时，补一段从当前位置到首点的融合段
  // （aubo_boot blendToFirstPoint）。
  if (joint_position_state_interface_.size() == number_of_joints_ && !traj.points.empty()) {
    std::vector<double> current_joints(number_of_joints_, 0.0);
    for (size_t i = 0; i < number_of_joints_; ++i) {
      current_joints[i] = joint_position_state_interface_[i].get().get_optional().value_or(0.0);
    }
    auto blend_points = blendToFirstPoint(current_joints, traj.points.front());
    if (!blend_points.empty()) {
      const double blend_time = static_cast<double>(params_.blend_steps) * 0.005;
      for (auto & pt : traj.points) {
        pt.time_from_start = double_to_duration(duration_to_double(pt.time_from_start) +
            blend_time);
      }
      traj.points.insert(traj.points.begin(), std::make_move_iterator(blend_points.begin()),
                         std::make_move_iterator(blend_points.end()));
      RCLCPP_INFO(get_node()->get_logger(),
          "Prepended %zu blend points (%.0f ms) to the trajectory.",
                  blend_points.size(), blend_time * 1000.0);
    }
  }

  max_trajectory_time_ = duration_to_double(traj.points.back().time_from_start);
  const double goal_time_tol = duration_to_double(goal->goal_time_tolerance);
  effective_goal_time_ = goal_time_tol > 0.0 ? goal_time_tol : params_.goal_time;

  if (!set_rt_box_from_non_rt(rt_active_traj_, traj)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set active trajectory in realtime box.");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = "Failed to set active trajectory.";
    goal_handle->abort(result);
    return;
  }

  // 把 goal 容差按内部关节顺序排序（goal 已通过校验）
  std::vector<control_msgs::msg::JointTolerance> goal_tolerances;
  if (!goal->goal_tolerance.empty()) {
    std::stringstream ss;
    ss << "Using goal tolerances\n";
    for (const auto & joint_name : joint_names_) {
      auto found_it = std::find_if(goal->goal_tolerance.begin(), goal->goal_tolerance.end(),
          [&joint_name](auto & tol) {return tol.name == joint_name;});
      if (found_it != goal->goal_tolerance.end()) {
        goal_tolerances.push_back(*found_it);
        ss << joint_name << " -- position: " << found_it->position << ", velocity: " <<
          found_it->velocity
           << ", acceleration: " << found_it->acceleration << std::endl;
      }
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), ss.str());
  }

  if (!set_rt_box_from_non_rt(goal_tolerance_, goal_tolerances)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set goal tolerances in realtime box.");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = "Failed to set goal tolerances.";
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
      "Effective goal time: " << effective_goal_time_ << " sec");

  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  // 预分配 feedback，update() 里只填数值、不做分配。
  auto & feedback = rt_goal->preallocated_feedback_;
  feedback->joint_names = joint_names_;
  feedback->actual.positions.assign(number_of_joints_, 0.0);
  feedback->actual.velocities.assign(number_of_joints_, 0.0);
  feedback->desired.positions.assign(number_of_joints_, 0.0);
  feedback->desired.velocities.assign(number_of_joints_, 0.0);
  feedback->error.positions.assign(number_of_joints_, 0.0);
  feedback->error.velocities.assign(number_of_joints_, 0.0);

  if (!set_rt_goal_from_non_rt(rt_goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set active goal, aborting goal.");
    auto result = std::make_shared<FollowJTrajAction::Result>();
    result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
    result->error_string = "Failed to set active goal.";
    goal_handle->abort(result);
    return;
  }

  // action 的后续处理放到定时器回调里做，避免在实时线程中执行。先把已有
  // 定时器（若有）复位删除，再创建新的。
  rt_goal->execute();
  goal_handle_timer_.reset();
  goal_handle_timer_ =
    get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime,
      rt_goal));
  trajectory_active_ = true;
  return;
}

// 实时线程调用（goal_hold 检查，aubo_boot withinGoalConstraints + 速度检查
// + 连续帧确认；goal 自带容差可按关节覆盖默认值）。
bool AuboPassthroughTrajectoryController::withinGoalHold(
  const trajectory_msgs::msg::JointTrajectoryPoint & final_point,
  const std::vector<control_msgs::msg::JointTolerance> & tolerances)
{
  if (final_point.positions.size() < number_of_joints_) {
    return false;
  }
  for (size_t i = 0; i < number_of_joints_; ++i) {
    double pos_tol = params_.goal_tolerance_rad;
    double vel_tol = params_.goal_vel_tolerance;
    if (tolerances.size() == number_of_joints_) {
      // 字段为 0.0 表示“未指定” -> 保留控制器默认值。
      if (tolerances[i].position > 0.0) {
        pos_tol = tolerances[i].position;
      }
      if (tolerances[i].velocity > 0.0) {
        vel_tol = tolerances[i].velocity;
      }
    }

    const auto joint_pos = joint_position_state_interface_[i].get().get_optional();
    if (!joint_pos.has_value()) {
      return false;
    }
    if (std::abs(joint_pos.value() - final_point.positions[i]) > pos_tol) {
      return false;
    }

    if (i < joint_velocity_state_interface_.size()) {
      const auto joint_vel = joint_velocity_state_interface_[i].get().get_optional();
      if (!joint_vel.has_value()) {
        return false;
      }
      if (std::abs(joint_vel.value()) > vel_tol) {
        return false;
      }
    }
  }
  return true;
}

void AuboPassthroughTrajectoryController::end_goal()
{
  trajectory_active_ = false;
  transfer_requested_ = false;
  if (!transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not write to transfer command interface.");
  }
}

// aubo_boot remapJointNames：按关节名重排到权威顺序；缺失的速度/加速度补 0。
trajectory_msgs::msg::JointTrajectory
AuboPassthroughTrajectoryController::remapJointNames(
  const trajectory_msgs::msg::JointTrajectory & traj) const
{
  trajectory_msgs::msg::JointTrajectory out = traj;
  out.joint_names = joint_names_;

  std::unordered_map<std::string, size_t> name_to_index;
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    name_to_index[traj.joint_names[i]] = i;
  }

  for (auto & point : out.points) {
    const auto src_positions = point.positions;
    const auto src_velocities = point.velocities;
    const auto src_accelerations = point.accelerations;
    point.positions.assign(number_of_joints_, 0.0);
    point.velocities.assign(number_of_joints_, 0.0);
    point.accelerations.assign(number_of_joints_, 0.0);
    for (size_t j = 0; j < number_of_joints_; ++j) {
      const auto it = name_to_index.find(joint_names_[j]);
      if (it == name_to_index.end()) {
        continue;  // 不可能发生：goal 已校验包含全部关节
      }
      const size_t src = it->second;
      if (src < src_positions.size()) {
        point.positions[j] = src_positions[src];
      }
      if (src < src_velocities.size()) {
        point.velocities[j] = src_velocities[src];
      }
      if (src < src_accelerations.size()) {
        point.accelerations[j] = src_accelerations[src];
      }
    }
  }
  return out;
}

// aubo_boot blendToFirstPoint：blend_steps 个 5ms 的 smoothstep
// （s = 3t^2 - 2t^3，端点速度为零、加速度非零，数学上为 C1）融合点，从
// 当前位置过渡到轨迹首点。默认阈值 0.01 rad（blend_threshold_rad），默认
// 30 步 = 150ms（blend_steps），均为 aubo_boot 实测默认值。
std::vector<trajectory_msgs::msg::JointTrajectoryPoint> AuboPassthroughTrajectoryController::
blendToFirstPoint(
  const std::vector<double> & current_joints,
  const trajectory_msgs::msg::JointTrajectoryPoint & first_point) const
{
  const int steps = params_.blend_steps;
  if (steps <= 0 || first_point.positions.size() < number_of_joints_) {
    return {};
  }

  double max_deviation = 0.0;
  for (size_t i = 0; i < number_of_joints_; ++i) {
    max_deviation = std::max(max_deviation, std::abs(first_point.positions[i] - current_joints[i]));
  }
  if (max_deviation <= params_.blend_threshold_rad) {
    return {};
  }

  constexpr double kPointDt = 0.005;  // 每个融合步 5ms（aubo_boot 按 200Hz）
  const double blend_duration = steps * kPointDt;

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> blend_points;
  blend_points.reserve(steps);
  for (int k = 1; k <= steps; ++k) {
    const double t = static_cast<double>(k) / steps;
    const double s = t * t * (3.0 - 2.0 * t);                // smoothstep
    const double sd = (6.0 * t - 6.0 * t * t) / blend_duration;   // ds/dtime
    const double sdd = (6.0 - 12.0 * t) / (blend_duration * blend_duration);  // d2s/dtime2

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(number_of_joints_);
    point.velocities.resize(number_of_joints_);
    point.accelerations.resize(number_of_joints_);
    for (size_t i = 0; i < number_of_joints_; ++i) {
      const double delta = first_point.positions[i] - current_joints[i];
      point.positions[i] = current_joints[i] + s * delta;
      point.velocities[i] = sd * delta;
      point.accelerations[i] = sdd * delta;
    }
    point.time_from_start = double_to_duration(k * kPointDt);
    blend_points.push_back(std::move(point));
  }
  return blend_points;
}

}  // namespace aubo_e5_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aubo_e5_controllers::AuboPassthroughTrajectoryController,
                       controller_interface::ControllerInterface)
