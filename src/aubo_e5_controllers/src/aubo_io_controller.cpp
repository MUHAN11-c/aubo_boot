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
 * AuboIOController 实现 —— 适用范围及与 UR 原版的差异详见头文件。
 */
//----------------------------------------------------------------------

#include "aubo_e5_controllers/aubo_io_controller.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>

namespace aubo_e5_controllers
{
namespace
{
// NaN 命令槽：写入命令接口表示“等待硬件应答”的哨兵值（UR asyncThread 模式）。
// set_io 先把 async-success 清成该值，硬件的 IO 异步线程消费后回写
// 1.0（成功）/ -1.0（失败）。
constexpr double kAsyncWaiting = std::numeric_limits<double>::quiet_NaN();

// /diagnostics 各 status 在 diag_msg_.status 里的固定下标（on_activate
// 预分配 5 项、名字只设一次，update() 里只填数值/级别）。
constexpr size_t kDiagHardwareHealth = 0;
constexpr size_t kDiagSafetyIo = 1;
constexpr size_t kDiagRibStream = 2;
constexpr size_t kDiagJointErrors = 3;
constexpr size_t kDiagLastEvent = 4;
constexpr size_t kDiagCount = 5;

// 简化事件名解码表（数值对应 SDK AuboRobotMetaType.h 的 RobotEventType）。
// 设计取舍：状态接口只能传数值，完整解码表（约 60 项）只放硬件 ioLoop
// 一处，避免两个包重复维护；这里只覆盖现场最常见的十几个事件，未覆盖的
// 显示数值，完整事件文本见 controller_manager 日志里的 [AuboEvent] 行。
const char * eventNameOrNull(int type)
{
  switch (type) {
    case 0: return "armCanbusError(CAN总线错误)";
    case 3: return "jointError(关节错误)";
    case 6: return "softEmergency(软急停)";
    case 7: return "exitSoftEmergency(退出软急停)";
    case 8: return "collision(碰撞)";
    case 11: return "powerChanged(电源状态改变)";
    case 14: return "encoderError(编码器错误)";
    case 17: return "currentAlarm(电流异常)";
    case 26: return "robotControllerError(控制错误)";
    case 27: return "socketDisconnected(socket断开)";
    case 28: return "robotControlException(控制异常)";
    case 35: return "RobotHandShakeSucc(握手成功)";
    case 1300: return "MoveEnterStopState(运动进入stop)";
    default: return nullptr;
  }
}

// 读状态接口，取不到值时按 0.0 处理
double read_or_zero(const hardware_interface::LoanedStateInterface & interface)
{
  return interface.get_optional().value_or(0.0);
}
}  // namespace

template<typename ResponseT>
bool AuboIOController::ensureActive(const ResponseT & resp)
{
  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new requests. Controller is not running.");
    resp->success = false;
    return false;
  }
  return true;
}

controller_interface::CallbackReturn AuboIOController::on_init()
{
  try {
    // 创建参数监听器并读取参数
    param_listener_ = std::make_shared<aubo_io_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AuboIOController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // 顺序由 CommandInterfaces 枚举固定。
  for (size_t i = 0; i < 16; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/do_" + std::to_string(i));
  }
  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/ao_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tool_do_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tool_ao_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "aubo_io/set_io_async_success");

  return config;
}

controller_interface::InterfaceConfiguration AuboIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // 顺序由 StateInterfaces 枚举固定。
  for (size_t i = 0; i < 16; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/di_" + std::to_string(i));
  }
  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/ai_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tool_di_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tool_ai_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "aubo_io/estop");
  config.names.emplace_back(tf_prefix + "aubo_io/protective_stop");
  config.names.emplace_back(tf_prefix + "aubo_io/power_on");
  config.names.emplace_back(tf_prefix + "aubo_io/collision");
  config.names.emplace_back(tf_prefix + "aubo_io/in_motion");
  config.names.emplace_back(tf_prefix + "aubo_io/rib_level");
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/joint_error_" + std::to_string(i));
  }
  // 反馈增强（全部追加在尾部，与 StateInterfaces 枚举偏移手工对齐）：
  // 发送流水线指标 -> 机器人侧目标 -> 电流/温度 -> 关节 position/velocity
  // （following_error 的 actual；关节名不带 tf_prefix，与 URDF 关节命名一致）。
  config.names.emplace_back(tf_prefix + "aubo_io/send_queue_points");
  config.names.emplace_back(tf_prefix + "aubo_io/send_rate_pps");
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tag_pos_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/tag_vel_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/joint_current_" + std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "aubo_io/joint_temp_" + std::to_string(i));
  }
  for (const auto & joint : params_.joints) {
    config.names.emplace_back(joint + "/position");
  }
  for (const auto & joint : params_.joints) {
    config.names.emplace_back(joint + "/velocity");
  }
  // 事件/健康上报（尾部追加，与 StateInterfaces 枚举偏移对齐）。
  config.names.emplace_back(tf_prefix + "aubo_io/event_type");
  config.names.emplace_back(tf_prefix + "aubo_io/event_code");
  config.names.emplace_back(tf_prefix + "aubo_io/health");

  return config;
}

controller_interface::return_type AuboIOController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  publishIO(time);
  publishRobotStatus();
  publishRibStatus();
  publishJointStatus();
  publishDiagnostics(time);
  publishEvents();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
AuboIOController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 从监听器重新取参数，防止运行期被更新过
  params_ = param_listener_->get_params();

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

  try {
    // 在实时 update 循环之外注册发布者
    io_pub_ = std::make_shared<realtime_tools::RealtimePublisher<aubo_msgs::msg::IOState>>(
        get_node()->create_publisher<aubo_msgs::msg::IOState>("~/io_states",
        rclcpp::SystemDefaultsQoS()));

    robot_status_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<aubo_msgs::msg::RobotStatus>>(
        get_node()->create_publisher<aubo_msgs::msg::RobotStatus>("~/robot_status",
        rclcpp::SystemDefaultsQoS()));

    rib_status_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>>(
        get_node()->create_publisher<std_msgs::msg::Int32MultiArray>("~/rib_status",
        rclcpp::SystemDefaultsQoS()));

    joint_status_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<aubo_msgs::msg::JointStatus>>(
        get_node()->create_publisher<aubo_msgs::msg::JointStatus>("~/joint_status",
        rclcpp::SystemDefaultsQoS()));

    // 事件/健康上报：/diagnostics 用全局话题名（aggregator 约定）；
    // ~/events 用 transient_local 持久 QoS，后启动的订阅者也能补到最后
    // 一条事件（事件是低频关键信息，丢一条比乱序更不能接受）。
    diag_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
        get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::SystemDefaultsQoS()));

    events_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(
        get_node()->create_publisher<std_msgs::msg::String>(
            "~/events", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable()));

    // 在实时 update 循环之外注册服务；控制器未激活时请求会被拒绝
    set_io_srv_ = get_node()->create_service<aubo_msgs::srv::SetIO>(
        "~/set_io",
        std::bind(&AuboIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void AuboIOController::publishIO(const rclcpp::Time & time)
{
  // N7：stamp 语义保持"纳秒字符串"（IOState.msg 的 string stamp 是 ROS1
  // 遗留，不动消息定义）；写入 on_activate 预分配的 buffer 后 assign 复用
  // std::string 已有容量，避免 std::to_string 在 RT 路径每周期的堆分配。
  const int stamp_len = std::snprintf(
      io_stamp_buf_, sizeof(io_stamp_buf_), "%lld",
      static_cast<long long>(time.nanoseconds()));  // NOLINT(runtime/int)
  io_msg_.stamp.assign(io_stamp_buf_, static_cast<size_t>(stamp_len > 0 ? stamp_len : 0));

  for (size_t i = 0; i < 16; ++i) {
    io_msg_.digital_in_states[i].pin = i;
    io_msg_.digital_in_states[i].flag = true;
    io_msg_.digital_in_states[i].state = read_or_zero(state_interfaces_[DIGITAL_INPUTS + i]) != 0.0;
  }

  // aubo_io 契约中没有 DO 状态接口：回显命令值（尚未命令过、即 NaN 时
  // 报 flag=false / state=false）。
  for (size_t i = 0; i < 16; ++i) {
    const double cmd = command_interfaces_[DIGITAL_OUTPUTS_CMD +
        i].get_optional().value_or(kAsyncWaiting);
    io_msg_.digital_out_states[i].pin = i;
    io_msg_.digital_out_states[i].flag = !std::isnan(cmd);
    io_msg_.digital_out_states[i].state = !std::isnan(cmd) && cmd != 0.0;
  }

  for (size_t i = 0; i < 4; ++i) {
    io_msg_.analog_in_states[i].pin = i;
    io_msg_.analog_in_states[i].state =
      static_cast<float>(read_or_zero(state_interfaces_[ANALOG_INPUTS + i]));
  }

  for (size_t i = 0; i < 4; ++i) {
    const double cmd = command_interfaces_[ANALOG_OUTPUTS_CMD + i].get_optional().value_or(0.0);
    io_msg_.analog_out_states[i].pin = i;
    io_msg_.analog_out_states[i].state = static_cast<float>(std::isnan(cmd) ? 0.0 : cmd);
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.tool_io_states[i].pin = i;
    io_msg_.tool_io_states[i].flag = true;
    io_msg_.tool_io_states[i].state = read_or_zero(state_interfaces_[TOOL_DIGITAL_INPUTS + i]) !=
      0.0;
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.tool_ai_states[i].pin = i;
    io_msg_.tool_ai_states[i].state =
      static_cast<float>(read_or_zero(state_interfaces_[TOOL_ANALOG_INPUTS + i]));
  }

  // 安全输入：pin 0 = 急停（estop），pin 1 = 防护停止（protective stop）。
  io_msg_.safety_in_states[0].pin = 0;
  io_msg_.safety_in_states[0].flag = true;
  io_msg_.safety_in_states[0].state = read_or_zero(state_interfaces_[ESTOP]) != 0.0;
  io_msg_.safety_in_states[1].pin = 1;
  io_msg_.safety_in_states[1].flag = true;
  io_msg_.safety_in_states[1].state = read_or_zero(state_interfaces_[PROTECTIVE_STOP]) != 0.0;
  // aubo_io 契约中没有安全输出：safety_out_states 保持为空。

  io_pub_->try_publish(io_msg_);
}

void AuboIOController::publishRobotStatus()
{
  // 简化的 industrial_msgs/RobotStatus 语义，字段映射自 aubo_io 状态接口
  // （estop / protective_stop / power_on / collision / in_motion /
  // joint_error_*）。
  const bool estop = read_or_zero(state_interfaces_[ESTOP]) != 0.0;
  const bool protective_stop = read_or_zero(state_interfaces_[PROTECTIVE_STOP]) != 0.0;
  const bool power_on = read_or_zero(state_interfaces_[POWER_ON]) != 0.0;
  const bool collision = read_or_zero(state_interfaces_[COLLISION]) != 0.0;
  const bool in_motion = read_or_zero(state_interfaces_[IN_MOTION]) != 0.0;

  int32_t error_code = 0;
  bool joint_error = false;
  for (size_t i = 0; i < 6; ++i) {
    const double err = read_or_zero(state_interfaces_[JOINT_ERRORS + i]);
    if (err != 0.0) {
      error_code = static_cast<int32_t>(err);
      joint_error = true;
      break;
    }
  }

  // mode: -1 = 未知（未上电），1 = 近似手动（已上电但急停中），
  // 2 = 自动（已上电、无急停）。做了简化：这套接口上旧版 SDK 不区分
  // 示教/回放（teach/play）模式。
  robot_status_msg_.mode = !power_on ? -1 : (estop ? 1 : 2);
  robot_status_msg_.e_stopped = estop ? 1 : 0;
  robot_status_msg_.drives_powered = power_on ? 1 : 0;
  robot_status_msg_.motion_possible = (power_on && !estop && !protective_stop &&
    !in_motion) ? 1 : 0;
  robot_status_msg_.in_motion = in_motion ? 1 : 0;
  robot_status_msg_.in_error = (protective_stop || collision || joint_error) ? 1 : 0;
  robot_status_msg_.error_code = error_code;

  robot_status_pub_->try_publish(robot_status_msg_);
}

void AuboIOController::publishRibStatus()
{
  // [rib_level, 发送队列长度, 瞬时吞吐]。后两项来自反馈增强新增的
  // aubo_io/send_queue_points、send_rate_pps 状态接口（此前固定发 0 占位）。
  rib_status_msg_.data[0] = static_cast<int32_t>(read_or_zero(state_interfaces_[RIB_LEVEL]));
  rib_status_msg_.data[1] =
    static_cast<int32_t>(read_or_zero(state_interfaces_[SEND_QUEUE_POINTS]));
  rib_status_msg_.data[2] = static_cast<int32_t>(read_or_zero(state_interfaces_[SEND_RATE_PPS]));

  rib_status_pub_->try_publish(rib_status_msg_);
}

void AuboIOController::publishJointStatus()
{
  // 关节级详细状态（反馈增强）：current/temp/tag_pos/tag_vel 直接透传新
  // 状态接口；following_error = |tag_pos - 实际位置|，actual 取关节
  // position 状态接口（velocity 接口一并认领，供后续扩展速度误差）。
  for (size_t i = 0; i < 6; ++i) {
    const double tag_pos = read_or_zero(state_interfaces_[TAG_POS + i]);
    const double actual_pos = read_or_zero(state_interfaces_[JOINT_POSITIONS + i]);
    joint_status_msg_.current[i] = read_or_zero(state_interfaces_[JOINT_CURRENT + i]);
    joint_status_msg_.temperature[i] = read_or_zero(state_interfaces_[JOINT_TEMP + i]);
    joint_status_msg_.tag_pos[i] = tag_pos;
    joint_status_msg_.tag_vel[i] = read_or_zero(state_interfaces_[TAG_VEL + i]);
    joint_status_msg_.following_error[i] = std::abs(tag_pos - actual_pos);
    joint_status_msg_.error_code[i] =
      static_cast<uint16_t>(read_or_zero(state_interfaces_[JOINT_ERRORS + i]));
  }

  joint_status_pub_->try_publish(joint_status_msg_);
}

void AuboIOController::publishDiagnostics(const rclcpp::Time & time)
{
  // 本函数内的字符串拼接保留：1Hz 节流（health 变化才立即发），非 RT 热点。
  const double health = read_or_zero(state_interfaces_[HEALTH]);
  // 1Hz 节流；health 变化（含激活后首次，last_health_ 哨兵 -1）立即发，
  // 故障不等下一个整秒。
  const bool health_changed = health != last_health_;
  if (!health_changed && (time - last_diag_stamp_).seconds() < 1.0) {
    return;
  }
  last_diag_stamp_ = time;
  last_health_ = health;

  diag_msg_.header.stamp = time;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  // hardware_health：health 原值（0=OK 1=ESTOP 2=FAULT，枚举定义见硬件
  // aubo_e5_hardware.hpp 的 Health）。
  auto & hw = diag_msg_.status[kDiagHardwareHealth];
  const int health_int = static_cast<int>(health);
  hw.level = health_int == 0 ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
  hw.message = health_int == 0 ? "OK" : "health=" + std::to_string(health_int);
  hw.values[0].value = std::to_string(health_int);

  // safety_io：estop/protective_stop 任一触发 -> ERROR（安全停），
  // collision -> WARN（碰撞标志来自诊断信息，未必伴随停机）。
  const bool estop = read_or_zero(state_interfaces_[ESTOP]) != 0.0;
  const bool protective = read_or_zero(state_interfaces_[PROTECTIVE_STOP]) != 0.0;
  const bool collision = read_or_zero(state_interfaces_[COLLISION]) != 0.0;
  auto & sf = diag_msg_.status[kDiagSafetyIo];
  sf.values[0].value = estop ? "1" : "0";
  sf.values[1].value = protective ? "1" : "0";
  sf.values[2].value = collision ? "1" : "0";
  sf.message = "";
  if (estop) {sf.message += "estop ";}
  if (protective) {sf.message += "protective_stop ";}
  if (collision) {sf.message += "collision ";}
  if (sf.message.empty()) {
    sf.level = DiagnosticStatus::OK;
    sf.message = "OK";
  } else {
    sf.message = "triggered: " + sf.message;
    sf.level = (estop || protective) ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN;
  }

  // rib_stream：发送流水线指标。WARN 判据（简单规则，别过度设计）：
  // 队列深度接近硬件 send_queue_ 容量 16384，或轨迹在跑但吞吐异常低。
  const double rib = read_or_zero(state_interfaces_[RIB_LEVEL]);
  const double queue_pts = read_or_zero(state_interfaces_[SEND_QUEUE_POINTS]);
  const double rate = read_or_zero(state_interfaces_[SEND_RATE_PPS]);
  const bool in_motion = read_or_zero(state_interfaces_[IN_MOTION]) != 0.0;
  auto & rs = diag_msg_.status[kDiagRibStream];
  rs.values[0].value = std::to_string(static_cast<int>(rib));
  rs.values[1].value = std::to_string(static_cast<int>(queue_pts));
  rs.values[2].value = std::to_string(static_cast<int>(rate));
  if (queue_pts > 15000.0) {
    rs.level = DiagnosticStatus::WARN;
    rs.message = "send queue near full";
  } else if (in_motion && rate < 50.0) {
    rs.level = DiagnosticStatus::WARN;
    rs.message = "in motion but send rate abnormally low";
  } else {
    rs.level = DiagnosticStatus::OK;
    rs.message = "OK";
  }

  // joint_errors：任一关节错误码非零 -> ERROR，message 列出关节号 + 错误码
  // （十进制 + 十六进制，十六进制方便对照 SDK/驱动器手册的位域定义）。
  auto & je = diag_msg_.status[kDiagJointErrors];
  je.level = DiagnosticStatus::OK;
  je.message = "OK";
  char hex_buf[16];
  for (size_t i = 0; i < 6; ++i) {
    const int err = static_cast<int>(read_or_zero(state_interfaces_[JOINT_ERRORS + i]));
    je.values[i].value = std::to_string(err);
    if (err != 0) {
      std::snprintf(hex_buf, sizeof(hex_buf), "0x%04X", err);
      if (je.level == DiagnosticStatus::OK) {
        je.level = DiagnosticStatus::ERROR;
        je.message = "";
      } else {
        je.message += " ";
      }
      je.message += "joint" + std::to_string(i) + "=" + std::to_string(err) + "(" + hex_buf + ")";
    }
  }

  // last_event：最近一次 SDK 事件。类型名走本文件的简化解码表（完整表在
  // 硬件 ioLoop；完整事件文本见 controller_manager 日志 [AuboEvent] 行）。
  // level 恒 OK：事件本身的严重性由 hardware_health/safety_io 反映，这里
  // 只做"最后一次发生了什么"的记录。
  // N9：event_type < 0 是"无事件"哨兵（硬件初值 -1），显示 none —— 此前
  // 哨兵为 0，与 armCanbusError 同值，激活后无事件时被误显示成 CAN 错误。
  const int ev_type = static_cast<int>(read_or_zero(state_interfaces_[EVENT_TYPE]));
  const int ev_code = static_cast<int>(read_or_zero(state_interfaces_[EVENT_CODE]));
  auto & le = diag_msg_.status[kDiagLastEvent];
  le.level = DiagnosticStatus::OK;
  if (ev_type < 0) {
    le.message = "none";
  } else {
    const char * ev_name = eventNameOrNull(ev_type);
    le.message = "type=" + std::to_string(ev_type);
    if (ev_name != nullptr) {
      le.message += "(" + std::string(ev_name) + ")";
    }
    le.message += " code=" + std::to_string(ev_code) + " (full text in controller_manager log)";
  }
  le.values[0].value = std::to_string(ev_type);
  le.values[1].value = std::to_string(ev_code);

  diag_pub_->try_publish(diag_msg_);
}

void AuboIOController::publishEvents()
{
  const double type = read_or_zero(state_interfaces_[EVENT_TYPE]);
  // N9：event_type < 0 是"无事件"哨兵（硬件初值 -1，sim 恒 -1）——不发
  // events 话题（此前哨兵 0 与 armCanbusError 同值，激活即误发一条伪事
  // 件）。变化检测哨兵 last_event_type_ 初值同为 -1，与硬件初值协调：
  // 无事件时两者相等、且被 < 0 提前拦截，激活后不会发伪消息。
  if (type < 0.0 || type == last_event_type_) {
    return;
  }
  last_event_type_ = type;
  // 只发 type/code 数值（状态接口传不了字符串，完整文本见硬件日志）。
  // 字符串拼接保留：事件驱动、频率极低，非 RT 热点。
  // 同一 update 周期内连续多个事件只能看到最后一个 —— 状态接口语义如此，
  // 完整事件序列以 controller_manager 日志的 [AuboEvent] 行为准。
  event_msg_.data = "type=" + std::to_string(static_cast<int>(type)) +
    " code=" +
    std::to_string(static_cast<int>(read_or_zero(state_interfaces_[EVENT_CODE])));
  events_pub_->try_publish(event_msg_);
}

controller_interface::CallbackReturn AuboIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // （重新）调整消息数组大小；消息对象跨周期复用。
  io_msg_.digital_in_states.resize(16);
  io_msg_.digital_out_states.resize(16);
  io_msg_.analog_in_states.resize(4);
  io_msg_.analog_out_states.resize(4);
  io_msg_.tool_io_states.resize(2);
  io_msg_.tool_ai_states.resize(2);
  io_msg_.safety_in_states.resize(2);
  io_msg_.safety_out_states.resize(0);
  rib_status_msg_.data.resize(3, 0);
  // joint_status 数组预分配为 6 关节，update() 里只填数值、不做分配。
  joint_status_msg_.current.resize(6, 0.0);
  joint_status_msg_.temperature.resize(6, 0.0);
  joint_status_msg_.tag_pos.resize(6, 0.0);
  joint_status_msg_.tag_vel.resize(6, 0.0);
  joint_status_msg_.following_error.resize(6, 0.0);
  joint_status_msg_.error_code.resize(6, 0);

  // /diagnostics 消息预分配（5 个 status 的名字/key/hardware_id 只设一次，
  // update() 里只填数值/级别/消息）。N9：name 带层级前缀（诊断工具按
  // 层级展示），hardware_id 统一标 "aubo_e5"。
  constexpr const char * kDiagPrefix = "aubo_e5/io_controller/";
  diag_msg_.status.resize(kDiagCount);
  for (auto & status : diag_msg_.status) {
    status.hardware_id = "aubo_e5";
  }
  diag_msg_.status[kDiagHardwareHealth].name = std::string(kDiagPrefix) + "hardware_health";
  diag_msg_.status[kDiagHardwareHealth].values.resize(1);
  diag_msg_.status[kDiagHardwareHealth].values[0].key = "health";
  diag_msg_.status[kDiagSafetyIo].name = std::string(kDiagPrefix) + "safety_io";
  diag_msg_.status[kDiagSafetyIo].values.resize(3);
  diag_msg_.status[kDiagSafetyIo].values[0].key = "estop";
  diag_msg_.status[kDiagSafetyIo].values[1].key = "protective_stop";
  diag_msg_.status[kDiagSafetyIo].values[2].key = "collision";
  diag_msg_.status[kDiagRibStream].name = std::string(kDiagPrefix) + "rib_stream";
  diag_msg_.status[kDiagRibStream].values.resize(3);
  diag_msg_.status[kDiagRibStream].values[0].key = "rib_level";
  diag_msg_.status[kDiagRibStream].values[1].key = "send_queue_points";
  diag_msg_.status[kDiagRibStream].values[2].key = "send_rate_pps";
  diag_msg_.status[kDiagJointErrors].name = std::string(kDiagPrefix) + "joint_errors";
  diag_msg_.status[kDiagJointErrors].values.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    diag_msg_.status[kDiagJointErrors].values[i].key = "joint_" + std::to_string(i);
  }
  diag_msg_.status[kDiagLastEvent].name = std::string(kDiagPrefix) + "last_event";
  diag_msg_.status[kDiagLastEvent].values.resize(2);
  diag_msg_.status[kDiagLastEvent].values[0].key = "event_type";
  diag_msg_.status[kDiagLastEvent].values[1].key = "event_code";
  // N7：stamp 字符串预分配（reserve 后 update() 内 assign 复用已有容量，
  // 不再触发每周期堆分配）。
  io_msg_.stamp.reserve(sizeof(io_stamp_buf_));
  io_stamp_buf_[0] = '\0';
  // 哨兵复位：重新激活后第一条诊断/事件立即发出。
  last_diag_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_health_ = -1.0;
  last_event_type_ = -1.0;

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AuboIOController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 无需清理。发布者和服务在 on_configure 中创建、on_cleanup 中释放。
  // on_activate/on_deactivate 在实时 update 循环内执行，因此把 DDS 管理
  // 挪到循环外，避免拖住 controller_manager 的更新周期。
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AuboIOController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    io_pub_.reset();
    robot_status_pub_.reset();
    rib_status_pub_.reset();
    joint_status_pub_.reset();
    diag_pub_.reset();
    events_pub_.reset();
    set_io_srv_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool AuboIOController::setIO(
  aubo_msgs::srv::SetIO::Request::SharedPtr req,
  aubo_msgs::srv::SetIO::Response::SharedPtr resp)
{
  if (!ensureActive(resp)) {
    return false;
  }

  // 与其他 set_io 调用互斥：async-success 标志是共享的。
  std::lock_guard<std::mutex> lock(set_io_mutex_);

  size_t command_index = 0;
  switch (req->fun) {
    case aubo_msgs::srv::SetIO::Request::FUN_SET_ROBOT_BOARD_USER_DO:
      if (req->pin < 0 || req->pin > 15) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid digital output pin %d (valid: 0..15).",
          req->pin);
        resp->success = false;
        return false;
      }
      command_index = DIGITAL_OUTPUTS_CMD + req->pin;
      break;
    case aubo_msgs::srv::SetIO::Request::FUN_SET_ROBOT_BOARD_USER_AO:
      if (req->pin < 0 || req->pin > 3) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid analog output pin %d (valid: 0..3).",
          req->pin);
        resp->success = false;
        return false;
      }
      command_index = ANALOG_OUTPUTS_CMD + req->pin;
      break;
    case aubo_msgs::srv::SetIO::Request::FUN_SET_TOOL_DIGITAL_IO:
      if (req->pin < 0 || req->pin > 1) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid tool digital output pin %d (valid: 0..1).",
          req->pin);
        resp->success = false;
        return false;
      }
      command_index = TOOL_DIGITAL_OUTPUTS_CMD + req->pin;
      break;
    case aubo_msgs::srv::SetIO::Request::FUN_SET_ROBOT_TOOL_AO:
      if (req->pin < 0 || req->pin > 1) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid tool analog output pin %d (valid: 0..1).",
          req->pin);
        resp->success = false;
        return false;
      }
      command_index = TOOL_ANALOG_OUTPUTS_CMD + req->pin;
      break;
    default:
      // FUN_SET_TOOL_POWER_TYPE 在 aubo_io 接口契约中没有对应项，有意不支持。
      RCLCPP_ERROR(get_node()->get_logger(), "Unsupported SetIO function %d.", req->fun);
      resp->success = false;
      return false;
  }

  // 发命令前先把 async-success 标志清成 NaN（= 等待应答）；硬件的 IO 异步
  // 线程应答 1.0（成功）/ -1.0（失败）——UR asyncThread 握手模式。
  std::ignore = command_interfaces_[SET_IO_ASYNC_SUCCESS].set_value(kAsyncWaiting);
  std::ignore = command_interfaces_[command_index].set_value(static_cast<double>(req->state));

  RCLCPP_INFO(get_node()->get_logger(), "Setting IO (fun %d, pin %d) to state '%f'.", req->fun,
      req->pin, req->state);

  if (!waitForAsyncCommand(
      [&]() {
        return command_interfaces_[SET_IO_ASYNC_SUCCESS].get_optional().value_or(kAsyncWaiting);
      }))
  {
    RCLCPP_WARN(get_node()->get_logger(),
        "Could not verify that io was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success =
    command_interfaces_[SET_IO_ASYNC_SUCCESS].get_optional().value_or(kAsyncWaiting) == 1.0;
  return resp->success;
}

bool AuboIOController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = params_.check_io_successfull_retries;
  int retries = 0;
  while (std::isnan(get_value())) {
    // 轮询间隔沿用 UR gpio_controller 的写法；硬件 IO 异步线程周期为
    // 20ms，50ms 一次重试至少覆盖两个周期。
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries) {
      return false;
    }
  }
  return true;
}

}  // namespace aubo_e5_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aubo_e5_controllers::AuboIOController,
  controller_interface::ControllerInterface)
