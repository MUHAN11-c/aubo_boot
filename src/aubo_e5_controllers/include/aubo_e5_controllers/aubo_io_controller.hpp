// Copyright (c) 2021 PickNik LLC (original gpio_controller)
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
 * AuboIOController —— AUBO E5 驱动的 GPIO 控制器插件，本地重写自 UR 的
 * ur_controllers::GPIOController（参考：reference/
 * Universal_Robots_ROS2_Driver/ur_controllers/src/gpio_controller.cpp）。
 * 从 aubo_io GPIO 状态接口发布板载/工具 IO 状态（aubo_msgs/IOState）、
 * 机器人状态（aubo_msgs/RobotStatus）、RIB 状态
 * （std_msgs/Int32MultiArray）和关节级详细状态（aubo_msgs/JointStatus，
 * ~/joint_status：电流/温度/机器人侧目标/跟随误差/错误码），并提供
 * ~/set_io 服务（aubo_msgs/SetIO），
 * 通过 set_io_async_success 命令接口确认执行结果（UR
 * check_io_successfull_retries 模式）。UR 专有功能（speed slider、payload、
 * FTS、resend program、tool voltage）不移植。
 */
//----------------------------------------------------------------------

#ifndef AUBO_E5_CONTROLLERS__AUBO_IO_CONTROLLER_HPP_
#define AUBO_E5_CONTROLLERS__AUBO_IO_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "aubo_msgs/msg/io_state.hpp"
#include "aubo_msgs/msg/joint_status.hpp"
#include "aubo_msgs/msg/robot_status.hpp"
#include "aubo_msgs/srv/set_io.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "aubo_e5_controllers/aubo_io_controller_parameters.hpp"

namespace aubo_e5_controllers
{
// 命令接口偏移（顺序由 command_interface_configuration 固定）：
//   do_0..15, ao_0..3, tool_do_0..1, tool_ao_0..1, set_io_async_success
enum CommandInterfaces
{
  DIGITAL_OUTPUTS_CMD = 0u,       // aubo_io/do_0 .. do_15
  ANALOG_OUTPUTS_CMD = 16,        // aubo_io/ao_0 .. ao_3
  TOOL_DIGITAL_OUTPUTS_CMD = 20,  // aubo_io/tool_do_0 .. tool_do_1
  TOOL_ANALOG_OUTPUTS_CMD = 22,   // aubo_io/tool_ao_0 .. tool_ao_1
  SET_IO_ASYNC_SUCCESS = 24,      // aubo_io/set_io_async_success
};

// 状态接口偏移（顺序由 state_interface_configuration 固定）：
//   di_0..15, ai_0..3, tool_di_0..1, tool_ai_0..1,
//   estop, protective_stop, power_on, collision, in_motion, rib_level,
//   joint_error_0..5,
//   send_queue_points, send_rate_pps,          （反馈增强：发送流水线指标）
//   tag_pos_0..5, tag_vel_0..5,                （反馈增强：机器人侧真实目标）
//   joint_current_0..5, joint_temp_0..5,       （反馈增强：电流/温度）
//   <joint>/position 0..5, <joint>/velocity 0..5  （following_error 的 actual）
//   event_type, event_code, health             （事件/健康上报）
// 注意：新增接口只允许追加到列表尾部（偏移与注册顺序手工对齐，插入中间
// 会让后面的偏移整体错位）。
enum StateInterfaces
{
  DIGITAL_INPUTS = 0u,      // aubo_io/di_0 .. di_15
  ANALOG_INPUTS = 16,       // aubo_io/ai_0 .. ai_3
  TOOL_DIGITAL_INPUTS = 20,  // aubo_io/tool_di_0 .. tool_di_1
  TOOL_ANALOG_INPUTS = 22,   // aubo_io/tool_ai_0 .. tool_ai_1
  ESTOP = 24,
  PROTECTIVE_STOP = 25,
  POWER_ON = 26,
  COLLISION = 27,
  IN_MOTION = 28,
  RIB_LEVEL = 29,
  JOINT_ERRORS = 30,  // aubo_io/joint_error_0 .. joint_error_5
  SEND_QUEUE_POINTS = 36,  // aubo_io/send_queue_points（重采样点队列深度）
  SEND_RATE_PPS = 37,      // aubo_io/send_rate_pps（瞬时吞吐 pts/s）
  TAG_POS = 38,            // aubo_io/tag_pos_0 .. tag_pos_5
  TAG_VEL = 44,            // aubo_io/tag_vel_0 .. tag_vel_5
  JOINT_CURRENT = 50,      // aubo_io/joint_current_0 .. joint_current_5
  JOINT_TEMP = 56,         // aubo_io/joint_temp_0 .. joint_temp_5
  JOINT_POSITIONS = 62,    // <joint>/position（params.joints 顺序）
  JOINT_VELOCITIES = 68,   // <joint>/velocity（params.joints 顺序）
  EVENT_TYPE = 74,         // aubo_io/event_type（最近一次 SDK 事件类型）
  EVENT_CODE = 75,         // aubo_io/event_code（最近一次 SDK 事件码值）
  HEALTH = 76,             // aubo_io/health（health_ 原值：0=OK 1=ESTOP 2=FAULT）
};

class AuboIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  // 控制器未激活时拒绝服务请求。
  // 控制器处于活动状态、请求可继续处理时返回 true。
  template <typename ResponseT>
  bool ensureActive(const ResponseT& resp);

  bool setIO(aubo_msgs::srv::SetIO::Request::SharedPtr req, aubo_msgs::srv::SetIO::Response::SharedPtr resp);

  void publishIO(const rclcpp::Time& time);

  void publishRobotStatus();

  void publishRibStatus();

  // 反馈增强：发布关节级详细状态（电流/温度/机器人侧目标/跟随误差/错误码）。
  void publishJointStatus();

  // 事件/健康上报：/diagnostics（1Hz 节流 + health 变化立即发）与
  // ~/events（event_type 变化时发一条 "type=<n> code=<n>"，transient_local
  // 持久，后启动的订阅者也能补到最后一条事件）。
  void publishDiagnostics(const rclcpp::Time& time);
  void publishEvents();

  /**
   * @brief 等待 async-success 命令接口离开 NaN 等待态（硬件应答 1.0 = 成功
   * / -1.0 = 失败），或达到参数 check_io_successfull_retries 的重试上限
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);

  rclcpp::Service<aubo_msgs::srv::SetIO>::SharedPtr set_io_srv_;

  // 在 ros2_control 实时循环中发布，因此全部用非阻塞 try_publish 包裹，防止控制器超时（overrun）
  std::shared_ptr<realtime_tools::RealtimePublisher<aubo_msgs::msg::IOState>> io_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<aubo_msgs::msg::RobotStatus>> robot_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>> rib_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<aubo_msgs::msg::JointStatus>> joint_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> diag_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> events_pub_;

  aubo_msgs::msg::IOState io_msg_;
  aubo_msgs::msg::RobotStatus robot_status_msg_;
  std_msgs::msg::Int32MultiArray rib_status_msg_;
  aubo_msgs::msg::JointStatus joint_status_msg_;
  diagnostic_msgs::msg::DiagnosticArray diag_msg_;
  std_msgs::msg::String event_msg_;

  // /diagnostics 节流（1Hz）与变化检测；哨兵 -1 表示"尚未发布过"，
  // 保证激活后第一条诊断立即发出。last_event_type_ 的 -1 同时与硬件
  // event_type 的"无事件"哨兵（N9，初值 -1）协调：无事件时不发伪消息。
  rclcpp::Time last_diag_stamp_{0, 0, RCL_ROS_TIME};
  double last_health_ = -1.0;
  double last_event_type_ = -1.0;

  // N7：~/io_states stamp（string，语义为纳秒字符串）的预分配 buffer。
  // update() 里 snprintf 写入 + assign 复用 std::string 容量（on_activate
  // 里 reserve），避免 std::to_string 在 RT 路径每周期的堆分配。
  // 24 字节足够放下 int64 纳秒值（含符号最多 20 位 + '\0'）。
  char io_stamp_buf_[24] = {'\0'};

  // aubo_io_controller 的 ROS 参数
  std::shared_ptr<aubo_io_controller::ParamListener> param_listener_;
  aubo_io_controller::Params params_;

  // 串行化并发的 ~/set_io 调用，保护共享的 async-success 标志。
  std::mutex set_io_mutex_;
};
}  // namespace aubo_e5_controllers

#endif  // AUBO_E5_CONTROLLERS__AUBO_IO_CONTROLLER_HPP_
