// Copyright 2026, aubo_e5_ros2_ws authors
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

// ============================================================================
// aubo_e5_hardware.cpp —— AUBO E5 机械臂 ros2_control SystemInterface 插件
// （旧版 SDK 1.3.1，TCP2CAN 位置流 + RIB 流量控制）。
//
// 职责：把控制器下发的 passthrough 轨迹点重采样成 5ms 点流，经 TCP2CAN
// 写入控制柜接口板（RIB）；同时回读关节状态 / IO / 安全状态到状态接口。
//
// 行为基线（蓝本逐条移植出处；aubo_boot = /home/wjz/桌面/aubo_boot 实测驱动，
// ROS1 = /home/wjz/aubo_robot/aubo_robot/aubo_driver）：
//   - sendLoop RIB 流量控制（降速续发档位 + EMA 补偿 + 自适应睡眠）
//     aubo_boot joint_trajectory_controller.cpp:125-180
//   - 五次插值系数 quinticInterpolate
//     aubo_boot joint_trajectory_controller.cpp:183-202
//   - 双连接登录（各重试 5 次）
//     aubo_boot aubo_hardware_interface.cpp:38-58
//   - TCP2CAN 进入（失败 Leave+Enter 重试一次）
//     aubo_boot aubo_hardware_interface.cpp:79-110
//   - 安全 IO 判定：DI0/8 急停、DI1/9 防护停（digitalIn[30]）
//     aubo_boot aubo_hardware_interface.cpp:250-261
//   - 电机 RPM -> 关节 rad/s 换算（减速比 121/101）
//     aubo_boot aubo_callback_monitor.cpp:29-32
//   - startup 参数（动力学全零/碰撞等级6/readPose=true/静态碰撞=true/
//     boardMaxAcc=1000/阻塞等待）
//     aubo_boot aubo_dashboard_node.cpp:245-251
//   - tryPopWaypoint 速度守卫（去重 + 超速拆分）
//     ROS1 aubo_driver.cpp:844-924
//   - publishWaypointToRobot 续发批量 ceil((400-rib)/6)
//     ROS1 aubo_driver.cpp:926-978（第 959 行）
//   - setIO 映射（UserDO 按名称 U_DO_XX（S4）、UserAO、工具 IO）
//     ROS1 aubo_driver.cpp:1016-1047
// UR 驱动仅作为 Jazzy ros2_control 代码风格参照（旧式接口导出、异步 IO 线程）。
//
// 线程模型（哪个函数在哪个线程跑）：
//   - read() / write()        RT 线程（controller_manager 控制循环）：
//                             只做内存读写，禁止任何 SDK 调用与阻塞。
//   - sendLoop()              发送线程：独占 conn_control_（查 RIB +
//                             SetRobotPosData2Canbus），周期 send_period_ms
//                             （默认 4ms）。
//   - ioLoop()                IO 异步线程：独占 conn_status_（IO 读写、安全
//                             IO 轮询、RobotMoveStop 停止原语），周期 20ms。
//   - jointStatusCallback() / robotEventCallback()
//                             SDK 内部推送线程：只写缓存 / 原子量，绝不回调
//                             SDK（防重入死锁，详见回调处注释）。
//   单实例不并发约束：SDK 的 ServiceInterface 实例非线程安全，同一实例被
//   两个线程并发调用会损坏内部状态，因此 conn_control_ 只由发送线程触碰、
//   conn_status_ 只由 IO 线程触碰（on_configure/on_activate 中 conn_status_
//   的调用均发生在线程启动之前，不构成并发）。
// ============================================================================

#include "aubo_e5_hardware/aubo_e5_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace aubo_e5_hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<double> parseDoubleList(const std::string & csv)
{
  std::vector<double> out;
  std::string::size_type start = 0;
  while (start <= csv.size()) {
    const auto comma = csv.find(',', start);
    const std::string token = csv.substr(start, comma == std::string::npos ? comma : comma - start);
    if (!token.empty()) {
      out.push_back(std::stod(token));
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return out;
}

bool parseBool(const std::string & value) {return value == "true" || value == "1";}

// 提取 IO 名称末尾的数字（"U_DI_12" -> 12），蓝本同款辅助函数。
int parseIoPin(const std::string & name, int fallback)
{
  std::string digits;
  for (const char ch : name) {
    if (std::isdigit(static_cast<unsigned char>(ch))) {
      digits.push_back(ch);
    }
  }
  return digits.empty() ? fallback : std::atoi(digits.c_str());
}
}  // namespace

// 权威关节顺序（方案 A.2）<=> SDK 数组下标 0..5。
const std::array<std::string, AuboE5Hardware::kNumJoints> AuboE5Hardware::kJointNames = {
  "shoulder_joint", "upperArm_joint", "foreArm_joint",
  "wrist1_joint", "wrist2_joint", "wrist3_joint"};

// 速度换算（方案 A.6）：jointSpeedMoto 是电机侧 RPM，转关节 rad/s 需乘
// (2*pi/60)/减速比；E5 前 3 个大关节减速比 121，后 3 个小关节 101。
// 蓝本：aubo_boot aubo_callback_monitor.cpp:29-32。
const std::array<double, AuboE5Hardware::kNumJoints> AuboE5Hardware::kV2R = {
  (2.0 * kPi / 60.0) / 121.0, (2.0 * kPi / 60.0) / 121.0, (2.0 * kPi / 60.0) / 121.0,
  (2.0 * kPi / 60.0) / 101.0, (2.0 * kPi / 60.0) / 101.0, (2.0 * kPi / 60.0) / 101.0};

// 析构兜底：正常路径 on_deactivate 已 join，这里防插件被直接销毁时泄漏线程。
AuboE5Hardware::~AuboE5Hardware()
{
  send_running_ = false;
  io_running_ = false;
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

// ============================================================================
// on_init：校验关节/接口布局，读取 B 表参数，预分配（不做任何网络通信）
// ============================================================================
hardware_interface::CallbackReturn AuboE5Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validateInterfaceLayout()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parseParams()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 命令槽初始化为"无请求"（NaN 哨兵，IO 线程据此跳过未更新的槽位）。
  io_do_cmd_.fill(kNoNewCmd);
  io_ao_cmd_.fill(kNoNewCmd);
  io_tool_do_cmd_.fill(kNoNewCmd);
  io_tool_ao_cmd_.fill(kNoNewCmd);

  RCLCPP_INFO(
    rclcpp::get_logger("AuboE5Hardware"),
    "on_init OK (%s:%d, rib_target=%d, slowdown=%d/%d, batch=%d..%d, speed_guard=%s)",
    params_.robot_ip.c_str(), params_.server_port, params_.rib_target, params_.rib_slowdown_1,
    params_.rib_slowdown_2, params_.batch_min, params_.batch_max,
    params_.speed_guard_enabled ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 校验 URDF 声明的关节/GPIO 接口与本插件导出的接口一一对应；
// 布局不符直接报错，避免运行期才发现接口缺失。
bool AuboE5Hardware::validateInterfaceLayout() const
{
  if (info_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AuboE5Hardware"), "expected %zu joints, got %zu", kNumJoints,
      info_.joints.size());
    return false;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.name != kJointNames[i]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AuboE5Hardware"), "joint %zu must be '%s', got '%s'", i,
        kJointNames[i].c_str(), joint.name.c_str());
      return false;
    }
    const auto has_if = [](const auto & ifs, const std::string & name) {
        return std::any_of(ifs.begin(), ifs.end(), [&](const auto & obj) {
                   return obj.name == name;
          });
      };
    if (!has_if(joint.command_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_if(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_if(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("AuboE5Hardware"),
          "joint '%s' missing position command/state or velocity state",
        joint.name.c_str());
      return false;
    }
  }

  const auto find_gpio =
    [this](const std::string & name) -> const hardware_interface::ComponentInfo * {
      for (const auto & gpio : info_.gpios) {
        if (gpio.name == name) {
          return &gpio;
        }
      }
      return nullptr;
    };
  const auto has_if = [](const auto & ifs, const std::string & name) {
      return std::any_of(ifs.begin(), ifs.end(), [&](const auto & obj) {return obj.name == name;});
    };

  const auto * traj = find_gpio("trajectory_passthrough");
  if (traj == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "gpio 'trajectory_passthrough' missing");
    return false;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    for (const auto * prefix : {"setpoint_positions_", "setpoint_velocities_",
        "setpoint_accelerations_"})
    {
      if (!has_if(traj->command_interfaces, std::string(prefix) + std::to_string(i))) {
        RCLCPP_ERROR(
          rclcpp::get_logger("AuboE5Hardware"), "trajectory_passthrough missing %s%zu", prefix, i);
        return false;
      }
    }
  }
  for (const auto * name : {"transfer_state", "time_from_start", "abort", "trajectory_size"}) {
    if (!has_if(traj->command_interfaces, name)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AuboE5Hardware"), "trajectory_passthrough missing '%s'", name);
      return false;
    }
  }

  const auto * scaling = find_gpio("speed_scaling");
  if (scaling == nullptr || !has_if(scaling->state_interfaces, "speed_scaling_factor")) {
    RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"),
        "gpio 'speed_scaling/speed_scaling_factor' missing");
    return false;
  }

  const auto * io = find_gpio("aubo_io");
  if (io == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "gpio 'aubo_io' missing");
    return false;
  }
  for (std::size_t i = 0; i < kNumBoardDIO; ++i) {
    if (!has_if(io->command_interfaces, "do_" + std::to_string(i)) ||
      !has_if(io->state_interfaces, "di_" + std::to_string(i)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing do/di_%zu", i);
      return false;
    }
  }
  for (std::size_t i = 0; i < kNumBoardAIO; ++i) {
    if (!has_if(io->command_interfaces, "ao_" + std::to_string(i)) ||
      !has_if(io->state_interfaces, "ai_" + std::to_string(i)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing ao/ai_%zu", i);
      return false;
    }
  }
  for (std::size_t i = 0; i < kNumToolIO; ++i) {
    if (!has_if(io->command_interfaces, "tool_do_" + std::to_string(i)) ||
      !has_if(io->command_interfaces, "tool_ao_" + std::to_string(i)) ||
      !has_if(io->state_interfaces, "tool_di_" + std::to_string(i)) ||
      !has_if(io->state_interfaces, "tool_ai_" + std::to_string(i)))
    {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing tool io %zu", i);
      return false;
    }
  }
  for (const auto * name :
    {"estop", "protective_stop", "power_on", "collision", "in_motion", "rib_level"})
  {
    if (!has_if(io->state_interfaces, name)) {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing state '%s'", name);
      return false;
    }
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!has_if(io->state_interfaces, "joint_error_" + std::to_string(i))) {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing joint_error_%zu", i);
      return false;
    }
  }
  // 反馈增强接口（SDK JointStatus 全量 + 发送流水线指标），sim/real 共用同
  // 一 GPIO 声明，故两种插件都要导出这些名字。
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    for (const auto * prefix : {"tag_pos_", "tag_vel_", "joint_current_", "joint_temp_"}) {
      if (!has_if(io->state_interfaces, std::string(prefix) + std::to_string(i))) {
        RCLCPP_ERROR(
          rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing state '%s%zu'", prefix, i);
        return false;
      }
    }
  }
  for (const auto * name : {"send_queue_points", "send_rate_pps", "event_type", "event_code",
      "health"})
  {
    if (!has_if(io->state_interfaces, name)) {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing state '%s'", name);
      return false;
    }
  }
  if (!has_if(io->command_interfaces, "set_io_async_success")) {
    RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "aubo_io missing set_io_async_success");
    return false;
  }
  return true;
}

// 解析 URDF hardware 参数（方案 B 表；默认值见 aubo_e5.ros2_control.xacro，
// 与蓝本 aubo_boot 实测值一致：rib_target=400、slowdown=300/350、
// batch=2..8、ema=0.1/(10,14,20)ms、stop_retry=20ms）。
bool AuboE5Hardware::parseParams()
{
  const auto required = [this](const char * name) -> std::string {
      const auto it = info_.hardware_parameters.find(name);
      if (it == info_.hardware_parameters.end()) {
        throw std::runtime_error(std::string("missing hardware parameter '") + name + "'");
      }
      return it->second;
    };
  const auto optional = [this](const char * name, const std::string & fallback) -> std::string {
      const auto it = info_.hardware_parameters.find(name);
      return it == info_.hardware_parameters.end() ? fallback : it->second;
    };

  try {
    params_.robot_ip = required("robot_ip");
    params_.server_port = std::stoi(required("server_port"));
    params_.sdk_username = required("sdk_username");
    params_.sdk_password = required("sdk_password");
    params_.send_period_ms = std::stoi(required("send_period_ms"));
    params_.rib_target = std::stoi(required("rib_target"));
    params_.rib_slowdown_1 = std::stoi(required("rib_slowdown_1"));
    params_.rib_slowdown_2 = std::stoi(required("rib_slowdown_2"));
    params_.batch_min = std::stoi(required("batch_min"));
    params_.batch_max = std::stoi(required("batch_max"));
    params_.ema_alpha = std::stod(required("ema_alpha"));
    params_.stop_retry_ms = std::stoi(required("stop_retry_ms"));
    params_.prefill_points = std::stoi(required("prefill_points"));
    params_.force_start_delay_ms = std::stoi(required("force_start_delay_ms"));
    params_.speed_guard_enabled = parseBool(required("speed_guard_enabled"));
    params_.point_spacing_s = std::stod(required("point_spacing_s"));
    params_.same_point_eps = std::stod(required("same_point_eps"));
    params_.dedup_threshold = std::stod(required("dedup_threshold"));
    params_.state_timeout_ms = std::stoi(required("state_timeout_ms"));
    params_.auto_power_on = parseBool(required("auto_power_on"));

    const auto boost = parseDoubleList(required("ema_boost_ms"));
    if (boost.size() != params_.ema_boost_ms.size()) {
      throw std::runtime_error("ema_boost_ms must have exactly 3 entries");
    }
    std::copy(boost.begin(), boost.end(), params_.ema_boost_ms.begin());

    const auto max_vel = parseDoubleList(required("max_joint_velocity"));
    const auto max_acc = parseDoubleList(required("max_joint_acceleration"));
    if (max_vel.size() != kNumJoints || max_acc.size() != kNumJoints) {
      throw std::runtime_error("max_joint_velocity/max_joint_acceleration must have 6 entries");
    }
    std::copy_n(max_vel.begin(), kNumJoints, params_.max_joint_velocity.begin());
    std::copy_n(max_acc.begin(), kNumJoints, params_.max_joint_acceleration.begin());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("AuboE5Hardware"), "parameter parse error: %s", e.what());
    return false;
  }
  return true;
}

// ============================================================================
// 接口导出（旧式写法，直接绑定成员变量；UR Jazzy 同款模式）。硬件侧把
// passthrough 状态机的反馈回写进 command 接口，控制器据此读状态。
// ============================================================================
std::vector<hardware_interface::StateInterface> AuboE5Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(2 * kNumJoints + 73);

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
  }

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "speed_scaling", "speed_scaling_factor", &speed_scaling_factor_state_));

  for (std::size_t i = 0; i < kNumBoardDIO; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "di_" + std::to_string(i), &io_di_state_[i]));
  }
  for (std::size_t i = 0; i < kNumBoardAIO; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "ai_" + std::to_string(i), &io_ai_state_[i]));
  }
  for (std::size_t i = 0; i < kNumToolIO; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "tool_di_" + std::to_string(i), &io_tool_di_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "tool_ai_" + std::to_string(i), &io_tool_ai_state_[i]));
  }
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "estop", &io_estop_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "protective_stop", &io_protective_stop_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "power_on", &io_power_on_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "collision", &io_collision_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "in_motion", &io_in_motion_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "rib_level", &io_rib_level_state_));
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "joint_error_" + std::to_string(i), &io_joint_error_state_[i]));
  }
  // 反馈增强：SDK JointStatus 全量字段 + 发送流水线指标。顺序与 xacro 的
  // aubo_io GPIO 声明保持一致（新增一律追加在末尾，消费侧按名认领）。
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "tag_pos_" + std::to_string(i), &io_tag_pos_state_[i]));
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "tag_vel_" + std::to_string(i), &io_tag_vel_state_[i]));
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "joint_current_" + std::to_string(i), &io_joint_current_state_[i]));
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "aubo_io", "joint_temp_" + std::to_string(i), &io_joint_temp_state_[i]));
  }
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "send_queue_points",
      &io_send_queue_points_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "send_rate_pps", &io_send_rate_pps_state_));
  // 事件/健康上报（SDK RobotEventInfo 最近一次事件 + health_ 原值）。
  // 追加在末尾（xacro/sim 同步），消费侧按名认领。
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "event_type", &io_event_type_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "event_code", &io_event_code_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("aubo_io", "health", &io_health_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AuboE5Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kNumJoints + 50);

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "trajectory_passthrough", "setpoint_positions_" + std::to_string(i),
      &traj_setpoint_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "trajectory_passthrough", "setpoint_velocities_" + std::to_string(i),
      &traj_setpoint_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "trajectory_passthrough", "setpoint_accelerations_" + std::to_string(i),
      &traj_setpoint_accelerations_[i]));
  }
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "trajectory_passthrough", "transfer_state", &traj_transfer_state_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "trajectory_passthrough", "time_from_start", &traj_time_from_start_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("trajectory_passthrough", "abort", &traj_abort_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "trajectory_passthrough", "trajectory_size", &traj_trajectory_size_));

  for (std::size_t i = 0; i < kNumBoardDIO; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "aubo_io", "do_" + std::to_string(i), &io_do_cmd_[i]));
  }
  for (std::size_t i = 0; i < kNumBoardAIO; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "aubo_io", "ao_" + std::to_string(i), &io_ao_cmd_[i]));
  }
  for (std::size_t i = 0; i < kNumToolIO; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "aubo_io", "tool_do_" + std::to_string(i), &io_tool_do_cmd_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "aubo_io", "tool_ao_" + std::to_string(i), &io_tool_ao_cmd_[i]));
  }
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "aubo_io", "set_io_async_success", &io_set_async_success_));
  return command_interfaces;
}

// ============================================================================
// 生命周期
// ============================================================================
hardware_interface::CallbackReturn AuboE5Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto log = rclcpp::get_logger("AuboE5Hardware");

  // 防 terminate 双保险之二：error 恢复时 CM 可能不经 on_cleanup 直接再次
  // on_configure；已登录就先走完整 logout 序列（注销回调 + 双登出），
  // 避免对同一 ServiceInterface 实例重复 login（SDK 未定义重登录行为）。
  if (logged_in_) {
    RCLCPP_WARN(log, "on_configure with active login, logging out first");
    conn_status_.robotServiceSetRealTimeJointStatusPush(false);
    conn_status_.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
    conn_status_.robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
    conn_status_.robotServiceLogout();
    conn_control_.robotServiceLogout();
    logged_in_ = false;
  }

  // 双连接登录，各重试 5 次（方案 A.5；
  // 蓝本 aubo_boot aubo_hardware_interface.cpp:38-58）。
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;
  int count = 0;
  do {
    ++count;
    ret = conn_control_.robotServiceLogin(
      params_.robot_ip.c_str(), params_.server_port, params_.sdk_username.c_str(),
      params_.sdk_password.c_str());
  } while (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < kLoginRetries);
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    RCLCPP_ERROR(log, "conn_control_ login failed after %d retries (ret=%d)", kLoginRetries, ret);
    return hardware_interface::CallbackReturn::ERROR;
  }

  count = 0;
  do {
    ++count;
    ret = conn_status_.robotServiceLogin(
      params_.robot_ip.c_str(), params_.server_port, params_.sdk_username.c_str(),
      params_.sdk_password.c_str());
  } while (ret != aubo_robot_namespace::InterfaceCallSuccCode && count < kLoginRetries);
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    RCLCPP_ERROR(log, "conn_status_ login failed after %d retries (ret=%d)", kLoginRetries, ret);
    conn_control_.robotServiceLogout();
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 两条连接都登录成功才置位（上面的失败路径已各自登出，不会留下半登录态）。
  logged_in_ = true;

  bool real_exist = false;
  if (conn_status_.robotServiceGetIsRealRobotExist(real_exist) ==
    aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_INFO(log, real_exist ? "Real robot detected" : "No real robot (simulation server)");
  } else {
    RCLCPP_WARN(log, "GetIsRealRobotExist failed");
  }

  // 工作模式校验（V5）：非真实模式只告警不阻断 —— TCP2CAN 在仿真模式下
  // 不驱动真机，但仿真链路本身是合法调试路径，阻断会连调试也堵死。
  {
    aubo_robot_namespace::RobotWorkMode work_mode;
    if (conn_status_.robotServiceGetRobotWorkMode(work_mode) ==
      aubo_robot_namespace::InterfaceCallSuccCode)
    {
      if (work_mode != aubo_robot_namespace::RobotModeReal) {
        RCLCPP_WARN(
          log, "robot work mode is not REAL (mode=%d): TCP2CAN will not drive a real robot",
          static_cast<int>(work_mode));
      }
    } else {
      RCLCPP_WARN(log, "robotServiceGetRobotWorkMode failed");
    }
  }

  // 在 conn_status_ 上注册推送回调（蓝本顺序：先开推送开关，再注册回调）。
  conn_status_.robotServiceSetRealTimeJointStatusPush(true);
  conn_status_.robotServiceRegisterRealTimeJointStatusCallback(
    &AuboE5Hardware::jointStatusCallback, this);
  conn_status_.robotServiceRegisterRobotEventInfoCallback(
    &AuboE5Hardware::robotEventCallback, this);

  // 等待第一帧关节状态推送（<= 5s，启动流程第 3 步）。
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(kFirstFrameTimeoutSec);
  bool got_frame = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto snap = state_box_.try_get();
    if (snap.has_value() && snap->valid) {
      got_frame = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!got_frame) {
    RCLCPP_ERROR(log, "no joint status push within %.1f s", kFirstFrameTimeoutSec);
    conn_status_.robotServiceSetRealTimeJointStatusPush(false);
    conn_status_.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
    conn_status_.robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
    conn_status_.robotServiceLogout();
    conn_control_.robotServiceLogout();
    logged_in_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }

  health_.store(kHealthOk);
  RCLCPP_INFO(log, "on_configure OK (dual login, push active)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboE5Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto log = rclcpp::get_logger("AuboE5Hardware");

  // 可选自动上电（方案 A.8；默认不启用，改由 dashboard 服务触发）。
  // 参数基线（aubo_boot aubo_dashboard_node.cpp:245-251）：动力学参数全零
  // （无工具）、碰撞等级 6、readPose=true、staticCollisionDetect=true、
  // boardMaxAcc=1000、IsBlock=true（阻塞等待启动完成）。
  if (params_.auto_power_on) {
    aubo_robot_namespace::ToolDynamicsParam tool_dynamics;
    std::memset(&tool_dynamics, 0, sizeof(tool_dynamics));
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    const int ret = conn_status_.rootServiceRobotStartup(
      tool_dynamics, kStartupCollisionClass, true, true, kStartupBoardMaxAcc, result, true);
    // ret 只是调用通道的返回码，启动结果在出参 result（ROBOT_SERVICE_STATE，
    // AuboRobotMetaType.h:329-340）里；成功值是 ROBOT_SERVICE_WORKING，
    // 两个都要查 —— 只查 ret 会把"调用通了但上电失败"误判为成功（S5）。
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode ||
      result != aubo_robot_namespace::ROBOT_SERVICE_WORKING)
    {
      RCLCPP_ERROR(
        log, "rootServiceRobotStartup failed (ret=%d, result=%d)", ret, static_cast<int>(result));
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(log, "auto_power_on: robot startup done");
  }

  // 在 conn_control_ 上进入 TCP2CAN（蓝本：失败则 Leave + Enter 重试一次，
  // aubo_boot aubo_hardware_interface.cpp:79-110）。进入后示教器失去运动
  // 控制权，接口板 RIB 由本插件独占喂点。
  int ret = conn_control_.robotServiceEnterTcp2CanbusMode();
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    conn_control_.robotServiceLeaveTcp2CanbusMode();
    ret = conn_control_.robotServiceEnterTcp2CanbusMode();
  }
  if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    RCLCPP_ERROR(log, "EnterTcp2CanbusMode failed (ret=%d)", ret);
    return hardware_interface::CallbackReturn::ERROR;
  }
  tcp2can_active_ = true;

  // 命令 = 当前实际位置（激活瞬间不跳变），速度守卫滤波器同样对齐实际值。
  const auto snap = state_box_.try_get();
  if (snap.has_value() && snap->valid) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      hw_position_commands_[i] = snap->pos[i];
      guard_joint_filter_[i] = snap->pos[i];
    }
  }

  // 复位轨迹流水线与健康状态。代际 generation_+1 让发送线程丢弃一切旧
  // 轨迹残留；clear_motion_=true 请发送线程清空其内部队列（SPSC 队列只能
  // 由消费端清空，详见成员声明处注释）。
  generation_.fetch_add(1);
  clear_motion_.store(true);
  stop_motion_requested_.store(false);
  health_.store(kHealthOk);
  emergency_stopped_.store(false);
  protective_stopped_.store(false);
  traj_transfer_state_ = kTransferIdle;
  traj_abort_ = 0.0;
  abort_latched_ = false;
  drain_wait_active_ = false;
  trajectory_active_ = false;
  points_received_ = 0;
  expected_points_ = 0.0;
  // N8：推送超时原因标志一并复位（一次性日志闸门是 ioLoop 局部变量，
  // 线程重启即复位，无需在此处理）。
  push_stale_fault_.store(false);

  // 防 std::terminate 双保险之一：std::thread 在 joinable 状态下被赋值会
  // terminate。正常路径 on_deactivate/on_error 已 join；这里兜住 deactivate
  // 被跳过的异常序列（如上次 activate 中途失败后 CM 直接再次 activate）。
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  send_running_.store(true);
  send_thread_ = std::thread(&AuboE5Hardware::sendLoop, this);
  io_running_.store(true);
  io_thread_ = std::thread(&AuboE5Hardware::ioLoop, this);

  RCLCPP_INFO(log, "on_activate OK (TCP2CAN active, send + IO threads running)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboE5Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto log = rclcpp::get_logger("AuboE5Hardware");

  // 停线程（含板载停止原语）+ 退 TCP2CAN，与 teardown 公共前段；
  // logout 留给 on_cleanup（deactivate 后连接保持，可直接再 activate）。
  stopThreadsAndLeaveTcp2Can();

  // 两个线程都已 join，此刻没有生产者/消费者，从本线程直接清空队列是安全的。
  Setpoint sp;
  while (setpoint_queue_.try_dequeue(sp)) {
  }
  PlanningState ps;
  while (send_queue_.try_dequeue(ps)) {
  }
  traj_transfer_state_ = kTransferIdle;
  drain_wait_active_ = false;
  trajectory_active_ = false;

  RCLCPP_INFO(log, "on_deactivate OK (left TCP2CAN, teach pendant back in control)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboE5Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (logged_in_) {
    conn_status_.robotServiceSetRealTimeJointStatusPush(false);
    conn_status_.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
    conn_status_.robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
    conn_status_.robotServiceLogout();
    conn_control_.robotServiceLogout();
    logged_in_ = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboE5Hardware"), "on_cleanup OK (dual logout)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboE5Hardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 任意状态直接 shutdown（不再经过 deactivate/cleanup）时也要完整拆卸。
  teardown();
  RCLCPP_INFO(rclcpp::get_logger("AuboE5Hardware"), "on_shutdown OK (teardown done)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboE5Hardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // read()/write() 报 ERROR 或生命周期回调抛异常后 CM 调 on_error：
  // 走与正常路径同一份 teardown，保证错误路径不泄漏线程/连接/板载点。

  // ---- 故障原因汇总（先于 teardown，趁现场还在）----
  // 背景：ioLoop 的 health 边沿/事件日志在故障窗口内可能正阻塞于
  // conn_status_ 的同步 SDK 调用，返回后直接进 teardown 而错过上报，
  // 导致 read() ERROR"静默"。on_error 是 CM 错误路径必经点，在这里兜底：
  // 1) 先取现场快照（dropped_events_ 会被 drainEventQueue 清零，必须先读）；
  // 2) drain 一次事件队列 —— 积压的 ERROR 级事件（如 socketDisconnected）
  //    在此补打日志（moodycamel 队列多消费者安全，与 ioLoop 竞争无妨，
  //    事件只会被其中一侧取走并记录）；
  // 3) 再打一行汇总：read() 记录的分支原因、health、快照年龄、最后事件。
  const auto log = rclcpp::get_logger("AuboE5Hardware");
  const int reason = read_error_reason_.load();
  const int64_t age_ms = read_error_snapshot_age_ms_.load();
  const int health = health_.load();
  const int last_event = static_cast<int>(event_type_.load());
  const int last_event_code = static_cast<int>(event_code_.load());
  const uint64_t dropped_events = dropped_events_.load();
  const uint64_t box_misses = read_box_misses_.load();
  drainEventQueue();
  const char * reason_text = "unknown";
  switch (reason) {
    case 0:
      reason_text = "no read() error recorded (error came from write() or lifecycle callback)";
      break;
    case 1:
      reason_text = "no valid joint status snapshot (push never arrived after activate)";
      break;
    case 2:
      reason_text = "joint status push stale (SDK push link stalled/disconnected)";
      break;
    case 3:
      reason_text = "health FAULT latched by event/send path (see [AuboEvent]/sendLoop logs)";
      break;
    default:
      break;
  }
  const char * event_name = (last_event >= 0) ? eventToString(last_event) : nullptr;
  RCLCPP_ERROR(
    log,
    "on_error summary: read_error_reason=%d (%s), health=%d, snapshot_age_ms=%lld, "
    "read_box_misses=%llu, last_event=%s(%d) code=%d, dropped_events=%llu",
    reason, reason_text, health, static_cast<long long>(age_ms),  // NOLINT(runtime/int)
    static_cast<unsigned long long>(box_misses),  // NOLINT(runtime/int)
    (last_event >= 0) ? (event_name ? event_name : "unknown") : "none", last_event,
    last_event_code, static_cast<unsigned long long>(dropped_events));  // NOLINT(runtime/int)

  teardown();
  RCLCPP_INFO(log, "on_error OK (teardown done)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// teardown：完整拆卸（on_error/on_shutdown 共用，全程幂等）
// ============================================================================
void AuboE5Hardware::teardown()
{
  stopThreadsAndLeaveTcp2Can();

  // 注销推送回调（先关推送开关）+ 双连接 logout。
  if (logged_in_) {
    conn_status_.robotServiceSetRealTimeJointStatusPush(false);
    conn_status_.robotServiceRegisterRealTimeJointStatusCallback(nullptr, nullptr);
    conn_status_.robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
    conn_status_.robotServiceLogout();
    conn_control_.robotServiceLogout();
    logged_in_ = false;
  }

  // 两个线程都已 join，此刻没有生产者/消费者，从本线程直接清空队列是安全的。
  Setpoint sp;
  while (setpoint_queue_.try_dequeue(sp)) {
  }
  PlanningState ps;
  while (send_queue_.try_dequeue(ps)) {
  }

  // health 复位（N3）：保证 error -> shutdown 之后重新 configure -> activate
  // 的链路上健康初值干净（不做运行时自动恢复的理由见 pollSafetyIo 注释）。
  health_.store(kHealthOk);
}

// teardown 与 on_deactivate 的公共前段。顺序关键：停止原语（RobotMoveStop /
// 降级 robotMoveFastStop）活在 ioLoop 里，必须在 join io 线程之前先请求
// 停止并等其完成 —— 先停 io 线程就没人执行停止，板载 RIB 残留点会继续走。
void AuboE5Hardware::stopThreadsAndLeaveTcp2Can()
{
  const auto log = rclcpp::get_logger("AuboE5Hardware");

  // 1) 停发送线程（joinable 防御：未启动或已 join 时直接跳过）。
  send_running_.store(false);
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  // 2) 请求板载停止并等 ioLoop 执行完（io_running_ 此时必须仍为 true，
  //    否则 ioLoop 已死、没人执行停止；io 线程未运行则跳过 —— 没有执行者，
  //    请求了也无人消费）。超时放弃并 WARN：卡住 teardown 会连带卡死 CM。
  if (tcp2can_active_ && io_running_.load()) {
    stop_motion_requested_.store(true);
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(kStopAckTimeoutMs);
    while (stop_motion_requested_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kStopAckPollMs));
    }
    if (stop_motion_requested_.load()) {
      RCLCPP_WARN(log, "stop-motion ack timeout (%d ms), giving up", kStopAckTimeoutMs);
    }
  }

  // 3) 停 IO 线程。
  io_running_.store(false);
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  // 4) 退出 TCP2CAN：把运动控制权交还示教器。
  if (tcp2can_active_) {
    conn_control_.robotServiceLeaveTcp2CanbusMode();
    tcp2can_active_ = false;
  }
}

// ============================================================================
// read()（RT 线程）：推送快照 -> 状态接口 + 新鲜度/健康门控
// ============================================================================
hardware_interface::return_type AuboE5Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 事件/健康上报先行拷贝：即使后面因快照失效或 FAULT 提前返回 ERROR，
  // health 状态接口也已反映最新值（诊断消费侧需要看到故障码本身）。
  // event_type 的"无事件"哨兵为 -1（N9，0 与 armCanbusError 枚举同值）。
  io_event_type_state_ = event_type_.load();
  io_event_code_state_ = event_code_.load();
  io_health_state_ = static_cast<double>(health_.load());

  // RealtimeThreadSafeBox::try_get() 是 best-effort：内部 try_lock，恰好
  // 撞上 SDK 推送线程 try_set 持锁（拷贝快照的窗口）就返回 nullopt。
  // 这是瞬时调度事件而非数据缺失 —— 此前把它当致命错误，运行中一次撞锁
  // 即触发 hardware ERROR -> CM 停用全部控制器（2026-07-29 真机事故：
  // 正常执行轨迹后空闲 220s，单次撞锁全线停机，read_error_reason=1 的
  // "push never arrived" 文案具有误导性）。因此取数失败时回退到 RT 线程
  // 本地缓存的上一帧良好快照；真正的"推送断流"由下方 stamp 超时检查
  // （state_timeout_ms，xacro 默认 200ms）兜底，语义不变。与蓝本 UR 对
  // try_get 失败重试/沿用旧值的处理一致。
  const auto fresh = state_box_.try_get();
  if (fresh.has_value() && fresh->valid) {
    last_snap_ = *fresh;
    have_last_snap_ = true;
  } else {
    // 撞锁，或首帧到来前（box 初值 valid=false）：计数供 on_error 汇总观测。
    read_box_misses_.fetch_add(1, std::memory_order_relaxed);
  }
  if (!have_last_snap_) {
    // 进程启动以来从未拿到有效帧（正常会被 on_configure 的首帧等待拦住；
    // 走到这里说明 configure 之后推送链路又出了状况）。
    read_error_reason_.store(1);
    read_error_snapshot_age_ms_.store(-1);
    return hardware_interface::return_type::ERROR;
  }

  // 推送超时（默认 200ms）说明 SDK 推送链路断了，置 FAULT 并报 ERROR，
  // 让 controller_manager 走错误恢复，而不是拿着旧数据继续发点。
  // N8：RT 线程不打日志 —— 只标记原因（push_stale_fault_ +
  // read_error_reason_/快照年龄）+ 置 health，日志由 ioLoop 的 health
  // 边沿与 on_error() 双路上报（ioLoop 阻塞时 on_error 兜底，不会静默）。
  // 注意：即使本周期回退用了缓存帧，stamp 仍是该帧的真实到达时刻，
  // 所以"缓存硬顶"超过 state_timeout_ms 一样会触发 stale FAULT。
  const auto age = std::chrono::steady_clock::now() - last_snap_.stamp;
  if (age > std::chrono::milliseconds(params_.state_timeout_ms)) {
    push_stale_fault_.store(true);
    health_.store(kHealthFault);
    read_error_reason_.store(2);
    read_error_snapshot_age_ms_.store(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count());
    return hardware_interface::return_type::ERROR;
  }

  // N8：同上，read() 只留返回码与原因；日志在 ioLoop 边沿 / on_error()。
  if (health_.load() != kHealthOk) {
    read_error_reason_.store(3);
    read_error_snapshot_age_ms_.store(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count());
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    hw_position_states_[i] = last_snap_.pos[i];
    hw_velocity_states_[i] = last_snap_.vel_moto[i] * kV2R[i];
    io_joint_error_state_[i] = static_cast<double>(last_snap_.err[i]);
    // 反馈增强：目标位置/速度、电流（SDK 原始单位）、温度。
    // tag_vel 与关节速度同一条换算路径（电机 RPM * kV2R -> 关节 rad/s）。
    io_tag_pos_state_[i] = last_snap_.tag_pos[i];
    io_tag_vel_state_[i] = last_snap_.tag_vel_moto[i] * kV2R[i];
    io_joint_current_state_[i] = last_snap_.current_i[i];
    io_joint_temp_state_[i] = last_snap_.temp[i];
  }

  io_estop_state_ = emergency_stopped_.load() ? 1.0 : 0.0;
  io_protective_stop_state_ = protective_stopped_.load() ? 1.0 : 0.0;
  io_power_on_state_ = power_on_.load() ? 1.0 : 0.0;
  io_collision_state_ = collision_.load() ? 1.0 : 0.0;
  io_rib_level_state_ = static_cast<double>(rib_level_.load());
  io_in_motion_state_ = rib_level_.load() > 0 ? 1.0 : 0.0;
  speed_scaling_factor_state_ = 1.0;

  // 发送流水线可观测性：队列深度是近似值（moodycamel SPSC 的 size_approx
  // 本就允许跨线程读，精度足够做监控）；速率由发送线程每秒写入原子量。
  io_send_queue_points_state_ = static_cast<double>(send_queue_.size_approx());
  io_send_rate_pps_state_ = send_rate_pps_.load();

  // 通过全部门控：清错误原因，供 on_error() 区分"本次错误"与历史残留。
  read_error_reason_.store(0);

  return hardware_interface::return_type::OK;
}

// ============================================================================
// write()（RT 线程，纯内存操作）：trajectory_passthrough 传递状态机。
// 状态语义（UR passthrough 契约，本地重写实现；控制器写、硬件读写并回写）：
//   0 IDLE            空闲；硬件在 取消/完成/异常 后回写到此。
//   1 ACCEPTED        硬件已受理，控制器可发下一点；硬件收点/接新轨迹后回写。
//   2 TRANSFER_POINT  控制器写入一个轨迹点并置此值请求传递；硬件收点后回写 1，
//                     队列满则保持 2 让控制器重发同一点。
//   3 TRANSFER_DONE   控制器宣告全部点发完；硬件转 4 并等待队列排空。
//   4 IN_MOTION       执行中（点已收齐，RIB/发送队列仍在消费）。
//   5 DONE            硬件回写：全部点已传递+重采样+发送队列排空（执行完毕）。
//   6 NEW_TRAJECTORY  控制器发起新轨迹（trajectory_size 为总点数）；
//                     硬件清旧轨迹后回写 1。
// 取消：控制器把 abort 写 1.0，硬件清队列、停发、丢弃板载 RIB 点并回写 0。
// ============================================================================
hardware_interface::return_type AuboE5Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (health_.load() != kHealthOk) {
    drain_wait_active_ = false;
    trajectory_active_ = false;
    traj_transfer_state_ = kTransferIdle;
    return hardware_interface::return_type::OK;
  }

  // 取消/抢占：清两条队列、停止发送、板载 RIB 点经 RobotMoveStop 丢弃
  // （由 IO 异步线程在 conn_status_ 上发出）。
  if (traj_abort_ == 1.0 && !abort_latched_) {
    abort_latched_ = true;
    generation_.fetch_add(1);
    clear_motion_.store(true);
    stop_motion_requested_.store(true);
    drain_wait_active_ = false;
    trajectory_active_ = false;
    traj_transfer_state_ = kTransferIdle;
    return hardware_interface::return_type::OK;
  }

  if (traj_transfer_state_ == kTransferNewTrajectory) {
    generation_.fetch_add(1);
    clear_motion_.store(true);
    stop_motion_requested_.store(false);
    abort_latched_ = false;
    traj_abort_ = 0.0;
    points_received_ = 0;
    expected_points_ = traj_trajectory_size_;
    drain_wait_active_ = false;
    trajectory_active_ = true;
    traj_transfer_state_ = kTransferAccepted;
    return hardware_interface::return_type::OK;
  }

  if (traj_transfer_state_ == kTransferPoint) {
    Setpoint sp;
    sp.pos = traj_setpoint_positions_;
    sp.vel = traj_setpoint_velocities_;
    sp.acc = traj_setpoint_accelerations_;
    sp.time_from_start = traj_time_from_start_;
    sp.generation = generation_.load();
    if (setpoint_queue_.try_enqueue(sp)) {
      ++points_received_;
      traj_transfer_state_ = kTransferAccepted;
    }
    // 队列满：状态保持 2，控制器会重发同一点。
    return hardware_interface::return_type::OK;
  }

  if (traj_transfer_state_ == kTransferDoneCmd) {
    traj_transfer_state_ = kTransferInMotion;
    drain_wait_active_ = true;
    return hardware_interface::return_type::OK;
  }

  // DONE(5) 的判定 = 点已全部收齐 + 已全部重采样 + 发送队列已排空。
  // goal_hold（方案 A.9）与 goal_time 的判定在控制器侧（它持有 action
  // 结果的所有权）；硬件只负责汇报"执行链路已排空"。
  if (drain_wait_active_) {
    const bool all_received = expected_points_ <= 0.0 || points_received_ >= expected_points_;
    const bool all_resampled = setpoints_resampled_.load() >= points_received_;
    const bool queue_empty = send_queue_.size_approx() == 0;
    if (all_received && all_resampled && queue_empty) {
      drain_wait_active_ = false;
      trajectory_active_ = false;
      traj_transfer_state_ = kTransferDone;
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// 发送线程（aubo_boot sendLoop 移植，joint_trajectory_controller.cpp:125-180，
// 默认 4ms 周期）：重采样 + RIB 流量控制
// ============================================================================
void AuboE5Hardware::sendLoop()
{
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  using std::chrono::milliseconds;
  using std::chrono::steady_clock;
  const auto log = rclcpp::get_logger("AuboE5Hardware.sendLoop");

  // EMA 补偿档位的批量下限由 batch_min/batch_max 推导，使蓝本默认值
  // （2/8 -> 4/6/8，对应 ema>10/14/20ms 三档）不带魔数地复现。
  const int boost_b1 = std::min(2 * params_.batch_min, params_.batch_max);
  const int boost_b2 = std::min(3 * params_.batch_min, params_.batch_max);
  const int boost_b3 = params_.batch_max;

  int rib = 0;
  int rib_fail_count = 0;
  auto last_diag = steady_clock::now() - milliseconds(kIdleDiagIntervalMs);
  double ema_ms = 0.0;
  uint64_t sent_ok = 0;
  uint64_t send_fail = 0;
  auto last_metrics = steady_clock::now();
  uint64_t sent_since_metrics = 0;
  // 丢点重发（S7）：SetRobotPosData2Canbus 失败时把该批存进 pending_batch，
  // 下周期优先重发，成功后才从 send_queue_ 取新批（保证 TCP2CAN 点序不乱）。
  std::vector<aubo_robot_namespace::wayPoint_S> pending_batch;
  // 连续发送失败计数（成功清零）：重发是恢复手段，闩锁是失效边界 ——
  // 连败达到阈值说明发送链路已断，锁 FAULT 走 CM 错误恢复而非无限重发。
  int fail_streak = 0;

  // 重采样状态（段拼接 + 全局 5ms 栅格；效果等价于 aubo_boot 的整段轨迹
  // 预计算扫描，只是改为随收随采的流水线形式）。
  std::optional<Setpoint> last_sp;
  double next_grid_t = 0.0;
  bool grid_based = false;
  uint64_t seen_generation = generation_.load();

  // 可选预填充闸门（ROS1 遗留行为；默认 0/0 即关闭）。
  bool prefill_open = params_.prefill_points <= 0;
  bool gate_armed = false;
  steady_clock::time_point gate_arm_time{};

  // 速度守卫状态（ROS1 tryPopWaypoint，aubo_driver.cpp:844-924）。
  std::array<double, kNumJoints> guard_last_vel{};

  RCLCPP_INFO(log, "sendLoop started");

  while (send_running_.load()) {
    // ---- 清队请求（取消 / 急停 / 新轨迹 / 重新激活）----
    if (clear_motion_.exchange(false)) {
      PlanningState ps;
      while (send_queue_.try_dequeue(ps)) {
      }
      last_sp.reset();
      grid_based = false;
      setpoints_resampled_.store(0);
      prefill_open = params_.prefill_points <= 0;
      gate_armed = false;
      pending_batch.clear();  // 清场同时丢弃待重发批（属于已撤销的运动）
      continue;
    }
    if (generation_.load() != seen_generation) {
      seen_generation = generation_.load();
      last_sp.reset();
      grid_based = false;
    }

    // ---- 把每条完整的段重采样成 5ms PlanningState ----
    Setpoint sp;
    while (setpoint_queue_.try_dequeue(sp)) {
      if (sp.generation != seen_generation) {
        continue;  // 已取消/被抢占轨迹的残留点，丢弃
      }
      if (!grid_based) {
        grid_based = true;
        next_grid_t = sp.time_from_start;  // 一般为 0（混合段起点）
        last_sp = sp;
        setpoints_resampled_.fetch_add(1);
        continue;
      }
      const double seg_t0 = last_sp->time_from_start;
      const double seg_t1 = sp.time_from_start;
      const double T = seg_t1 - seg_t0;
      if (T > params_.dedup_threshold) {
        while (next_grid_t <= seg_t1 + params_.dedup_threshold) {
          const double ts = std::min(std::max(next_grid_t - seg_t0, 0.0), T);
          PlanningState ps;
          quinticInterpolate(*last_sp, sp, ts, T, ps);
          if (!send_queue_.try_enqueue(ps)) {
            RCLCPP_ERROR_THROTTLE(
              log, *get_clock(), 1000, "send queue full, dropping 5 ms point");
            break;
          }
          next_grid_t += kResamplePeriodSec;
        }
      }
      last_sp = sp;
      setpoints_resampled_.fetch_add(1);
    }

    // ---- 查 RIB 板载缓冲水位（与发送同一条连接；忙时每周期都查）----
    const std::size_t avail = send_queue_.size_approx();
    const auto now = steady_clock::now();
    const int diag_interval_ms = avail > 0 ? 0 : kIdleDiagIntervalMs;
    if (rib <= 0 || duration_cast<milliseconds>(now - last_diag).count() >= diag_interval_ms) {
      int r = 0;
      if (readRibLevel(r)) {
        rib = r;
        rib_level_.store(r);
        rib_fail_count = 0;
      } else {
        // 查询失败保守起见沿用上次水位，并累计失败次数。
        if (++rib_fail_count >= kRibFailFaultThreshold) {
          health_.store(kHealthFault);
          RCLCPP_ERROR(log, "RIB query failed %d times in a row -> FAULT", rib_fail_count);
          // S10：链路健康辅助定位 —— 连败闩锁时顺带查一次 MAC 通信状态
          // （serviceinterface.h:913，服务查询接口，必须走连接；conn_control_
          // 由本线程独占，线程归属自洽）。只多打一条 WARN，不改变闩锁逻辑。
          bool mac_ok = false;
          if (conn_control_.robotServiceGetMacCommunicationStatus(mac_ok) ==
            aubo_robot_namespace::InterfaceCallSuccCode)
          {
            RCLCPP_WARN(log, "MAC communication status: %s", mac_ok ? "OK" : "BROKEN");
          } else {
            RCLCPP_WARN(log, "MAC communication status query failed (link likely down)");
          }
        }
      }
      last_diag = now;
    }

    // ---- 可选预填充闸门（ROS1：先攒够 N 点或超时强制启动）----
    if (!prefill_open) {
      if (avail > 0) {
        if (!gate_armed) {
          gate_armed = true;
          gate_arm_time = now;
        }
        bool open = static_cast<int>(avail) >= params_.prefill_points;
        if (!open && params_.force_start_delay_ms > 0 &&
          duration_cast<milliseconds>(now - gate_arm_time).count() >= params_.force_start_delay_ms)
        {
          open = true;
        }
        prefill_open = open;
      }
      if (!prefill_open) {
        std::this_thread::sleep_for(milliseconds(params_.send_period_ms));
        continue;
      }
    }

    // ---- 按 RIB 水位定本批续发点数（方案 A.7：只降速、不停发）----
    // 三档（默认值见 xacro：rib_target=400、slowdown_1=300、slowdown_2=350、
    // batch=2..8；蓝本 aubo_boot joint_trajectory_controller.cpp:143-155 +
    // ROS1 aubo_driver.cpp:959 ceil((400-rib)/6)）：
    //   rib < 300          全速续发：clamp(ceil((400-rib)/6), 2, 8)，
    //                      发送耗时 EMA 超 10/14/20ms 时批量下限抬到 4/6/8；
    //   300 <= rib < 350   缓冲偏满，每批最多 2 点；
    //   rib >= 350         接近板载上限，每批只续 1 点。
    int need = 0;
    if (rib < params_.rib_slowdown_1) {
      need = std::clamp(
        static_cast<int>(std::ceil((params_.rib_target - rib) / 6.0)), params_.batch_min,
        params_.batch_max);
      if (ema_ms > params_.ema_boost_ms[0]) {
        need = std::max(need, boost_b1);
      }
      if (ema_ms > params_.ema_boost_ms[1]) {
        need = std::max(need, boost_b2);
      }
      if (ema_ms > params_.ema_boost_ms[2]) {
        need = std::max(need, boost_b3);
      }
      need = std::min(need, params_.batch_max);
    } else if (rib < params_.rib_slowdown_2) {
      need = kSlowdownBatch1;
    } else {
      need = kSlowdownBatch2;
    }

    // ---- 发送一批（健康门控：FAULT 闩锁后自然停发，等 CM 错误恢复）----
    if (health_.load() == kHealthOk) {
      std::vector<aubo_robot_namespace::wayPoint_S> batch;
      if (!pending_batch.empty()) {
        // 优先重发上一周期失败的批，成功后才取新批（点序保证）。
        batch = pending_batch;
      } else {
        const std::size_t n = std::min(avail, static_cast<std::size_t>(need));
        if (n > 0) {
          if (params_.speed_guard_enabled) {
            batch = popGuardedBatch(n);
          } else {
            batch.reserve(n);
            PlanningState ps;
            for (std::size_t k = 0; k < n && send_queue_.try_dequeue(ps); ++k) {
              aubo_robot_namespace::wayPoint_S wp{};
              std::copy_n(ps.joint_pos.begin(), kNumJoints, wp.jointpos);
              batch.push_back(wp);
            }
          }
        }
      }

      if (!batch.empty()) {
        const auto t0 = steady_clock::now();
        const int ret = conn_control_.robotServiceSetRobotPosData2Canbus(batch);
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
          const double ms = duration_cast<microseconds>(steady_clock::now() - t0).count() / 1000.0;
          ema_ms = (ema_ms <=
            0.0) ? ms : ((1.0 - params_.ema_alpha) * ema_ms + params_.ema_alpha * ms);
          ++sent_ok;
          sent_since_metrics += batch.size();
          pending_batch.clear();
          fail_streak = 0;
        } else {
          ++send_fail;
          ++fail_streak;  // 重发也算一次失败计数
          pending_batch = std::move(batch);  // 保存待下周期重发
          if (send_fail <= 3 || send_fail % 50 == 0) {
            // S10：WARN 追加错误码解码文本辅助定位。getErrDescByCode
            // （serviceinterface.h:793）是非静态成员、位于 SDK 工具函数区
            // （坐标/姿态转换等纯本地函数之间，头文件未见网络语义），但
            // 非静态意味着必须经由某个连接实例调用 —— 用 conn_control_
            // （本线程独占），无论其内部是否触网，线程归属都自洽。
            RCLCPP_WARN(log, "SetRobotPosData2Canbus failed #%llu (ret=%d: %s)",
                        static_cast<unsigned long long>(send_fail), ret,  // NOLINT(runtime/int)
                        conn_control_.getErrDescByCode(
                          static_cast<aubo_robot_namespace::RobotErrorCode>(ret)).c_str());
          }
          if (fail_streak >= kRibFailFaultThreshold) {
            health_.store(kHealthFault);
            RCLCPP_ERROR(log, "SetRobotPosData2Canbus failed %d times in a row -> FAULT",
                         fail_streak);
          }
        }
      }
    }

    // ---- 指标（验收可观测性：rib / 队列 / 吞吐）----
    if (duration_cast<milliseconds>(now - last_metrics).count() >= 1000) {
      const double secs = duration_cast<microseconds>(now - last_metrics).count() / 1e6;
      const double rate = sent_since_metrics / secs;
      // 顺手暴露给状态接口 aubo_io/send_rate_pps（io 控制器 rib_status 的
      // data[2] 以及 ~/joint_status 都从这里取）。
      send_rate_pps_.store(rate);
      // 例行指标静默（DEBUG 可查）：1Hz INFO 刷屏会淹没真问题；仅异常时 INFO：
      // 发送失败增长、RIB 饿死（队列非空但接口板不消费）即属异常。
      // 队列满溢丢弃由上方独立 ERROR 覆盖，此处不重复。
      static unsigned long long last_logged_fail = 0;  // NOLINT(runtime/int)
      const bool anomalous =
        send_fail != last_logged_fail ||
        (rib <= 0 && send_queue_.size_approx() > 0);
      if (anomalous) {
        RCLCPP_INFO(
          log, "rib=%d queue=%zu rate=%.0f pts/s ema=%.1fms ok=%llu fail=%llu", rib,
          send_queue_.size_approx(), rate, ema_ms,
          static_cast<unsigned long long>(sent_ok),  // NOLINT(runtime/int)
          static_cast<unsigned long long>(send_fail));  // NOLINT(runtime/int)
        last_logged_fail = send_fail;
      } else {
        RCLCPP_DEBUG(
          log, "rib=%d queue=%zu rate=%.0f pts/s ema=%.1fms ok=%llu fail=%llu", rib,
          send_queue_.size_approx(), rate, ema_ms,
          static_cast<unsigned long long>(sent_ok),  // NOLINT(runtime/int)
          static_cast<unsigned long long>(send_fail));  // NOLINT(runtime/int)
      }
      sent_since_metrics = 0;
      last_metrics = now;
    }

    // ---- 自适应睡眠（蓝本）：队列深或 RIB 低时快转（1ms），否则按周期 ----
    if (send_queue_.size_approx() > kBusyQueueWatermark) {
      std::this_thread::sleep_for(milliseconds(kFastSleepMs));
    } else if (rib < kLowRibWatermark && send_queue_.size_approx() > 0) {
      std::this_thread::sleep_for(milliseconds(kFastSleepMs));
    } else {
      std::this_thread::sleep_for(milliseconds(params_.send_period_ms));
    }
  }
  RCLCPP_INFO(
    log, "sendLoop exit (ok=%llu fail=%llu)",
    static_cast<unsigned long long>(sent_ok),  // NOLINT(runtime/int)
    static_cast<unsigned long long>(send_fail));  // NOLINT(runtime/int)
}

// 查 RIB 板载缓冲水位（macTargetPosDataSize，单位见 rib_level 说明）。
// 在 conn_control_ 上查询 —— 与发送同一条连接，由发送线程独占调用。
bool AuboE5Hardware::readRibLevel(int & rib)
{
  aubo_robot_namespace::RobotDiagnosis diag;
  if (conn_control_.robotServiceGetRobotDiagnosisInfo(diag) ==
    aubo_robot_namespace::InterfaceCallSuccCode)
  {
    rib = diag.macTargetPosDataSize;
    return true;
  }
  return false;
}

// ROS1 tryPopWaypoint 移植（aubo_driver.cpp:844-924）：同点去重 +
// 超速点拆分。仅在 speed_guard_enabled=true 时使用（默认 false）。
std::vector<aubo_robot_namespace::wayPoint_S> AuboE5Hardware::popGuardedBatch(std::size_t count)
{
  static std::array<double, kNumJoints> last_vel{};  // 构造上只被发送线程触碰
  std::vector<aubo_robot_namespace::wayPoint_S> out;
  PlanningState ps;

  for (std::size_t k = 0; k < count && send_queue_.try_dequeue(ps); ++k) {
    // 与上一发送点逐关节比较（阈值 same_point_eps，默认 0.00015，
    // ROS1 aubo_driver.cpp:857），6 关节全同则丢弃该点（0x3F 掩码）。
    unsigned same_point = 0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (std::fabs(ps.joint_pos[i] - guard_joint_filter_[i]) < params_.same_point_eps) {
        same_point |= 1u << i;
      }
    }
    if (same_point == 0x3Fu) {
      continue;  // 完全重复的点，丢弃
    }

    // 目标速度 = 位置差 / 点距（默认 5ms），任一关节超限则判定超速。
    std::array<double, kNumJoints> target_vel{};
    bool over_speed = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      target_vel[i] =
        std::fabs(ps.joint_pos[i] - guard_joint_filter_[i]) / params_.point_spacing_s;
      if (target_vel[i] > params_.max_joint_velocity[i]) {
        over_speed = true;
      }
    }

    if (over_speed) {
      // 超速点按 ROS1 策略等距拆分：n_equalpart = ceil(max_vel / MaxVelc[0])，
      // 先补发 n_equalpart-1 个中间点，最后发原目标点（aubo_driver.cpp:875-894）。
      auto sorted = target_vel;
      std::sort(sorted.begin(), sorted.end());
      const int n_equalpart =
        static_cast<int>(std::ceil(sorted[kNumJoints - 1] / params_.max_joint_velocity[0]));
      RCLCPP_WARN(
        rclcpp::get_logger("AuboE5Hardware.sendLoop"),
        "overspeed point (max %.3f rad/s), splitting into %d parts", sorted[kNumJoints - 1],
        n_equalpart);
      auto interp = guard_joint_filter_;
      for (int e = 0; e < n_equalpart - 1; ++e) {
        for (std::size_t j = 0; j < kNumJoints; ++j) {
          interp[j] += (ps.joint_pos[j] - guard_joint_filter_[j]) / n_equalpart;
        }
        aubo_robot_namespace::wayPoint_S wp{};
        std::copy_n(interp.begin(), kNumJoints, wp.jointpos);
        out.push_back(wp);
      }
    }

    aubo_robot_namespace::wayPoint_S wp{};
    std::copy_n(ps.joint_pos.begin(), kNumJoints, wp.jointpos);
    out.push_back(wp);
    last_vel = target_vel;
    guard_joint_filter_ = ps.joint_pos;
  }
  (void)last_vel;  // 为与 ROS1 对齐而保留（ROS1 的加速度检查只打日志不拦截）
  return out;
}

// 五次插值，系数与 aubo_boot quinticInterpolate 逐字一致（方案 A.1；
// joint_trajectory_controller.cpp:183-202）：给定段首尾位置/速度/加速度，
// 求段内时刻 t 的位置/速度/加速度。
void AuboE5Hardware::quinticInterpolate(
  const Setpoint & last, const Setpoint & curr, double t, double T, PlanningState & out) const
{
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double a1 = last.vel[i];
    const double a2 = 0.5 * last.acc[i];
    const double h = curr.pos[i] - last.pos[i];
    const double a3 =
      0.5 / T3 * (20.0 * h - (8.0 * curr.vel[i] + 12.0 * last.vel[i]) * T -
      (3.0 * last.acc[i] - curr.acc[i]) * T2);
    const double a4 =
      0.5 / T4 * (-30.0 * h + (14.0 * curr.vel[i] + 16.0 * last.vel[i]) * T +
      (3.0 * last.acc[i] - 2.0 * curr.acc[i]) * T2);
    const double a5 =
      0.5 / T5 * (12.0 * h - 6.0 * (curr.vel[i] + last.vel[i]) * T +
      (curr.acc[i] - last.acc[i]) * T2);
    out.joint_pos[i] = last.pos[i] + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    out.joint_vel[i] = a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4;
    out.joint_acc[i] = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3;
  }
}

// ============================================================================
// IO 异步线程（UR asyncThread 模式；conn_status_ 的唯一使用者，20ms 周期）
// ============================================================================
void AuboE5Hardware::ioLoop()
{
  const auto log = rclcpp::get_logger("AuboE5Hardware.ioLoop");
  unsigned tick = 0;
  bool stop_active = false;
  auto stop_begin = std::chrono::steady_clock::now();
  auto last_stop_retry = std::chrono::steady_clock::now();
  // N8：health 边沿一次性日志闸门（原 read() 里 read_error_reported_ 的
  // 等价迁移：RT 路径不打日志，由本线程观察 health_ 跳变统一上报）。
  bool health_error_reported = false;

  RCLCPP_INFO(log, "ioLoop started");
  while (io_running_.load()) {
    // 0) drain SDK 事件队列：解码打日志 + 刷新 event_type_/event_code_
    //    （robotEventCallback 只入队，解码集中在 ioLoop 单线程输出）。
    drainEventQueue();

    // 0b) N8：health 边沿上报。read() 的推送超时分支只置 health/原因标志，
    //     日志在这里打 —— 离开 OK 时打一次（闸门闩住），回到 OK 复位闸门
    //     并清残留原因标志（恢复后允许再次报告，同原 read() 复位语义）。
    const int health_now = health_.load();
    if (health_now != kHealthOk) {
      if (!health_error_reported) {
        health_error_reported = true;
        if (push_stale_fault_.exchange(false)) {
          RCLCPP_ERROR(log, "joint status push stale (> %d ms) -> FAULT",
                       params_.state_timeout_ms);
        } else {
          RCLCPP_ERROR(log, "health not OK (%d) -> read reports ERROR", health_now);
        }
      }
    } else {
      health_error_reported = false;
      push_stale_fault_.store(false);
    }

    // 1) 处理 aubo_io 命令接口上的 IO 写请求。
    handleIoCommands();

    // 2) 安全 IO 轮询 ~10Hz（方案 A.4）。
    if (tick % kSafetyIoDivisor == 0) {
      pollSafetyIo();
    }

    // 3) 低频板载/工具 IO 状态 + 诊断信息。
    if (tick % kIoStateDivisor == 0) {
      pollIoStates();
    }

    // 4) 停止原语：RobotMoveStop 按 stop_retry_ms（默认 20ms）间隔重试，
    //    持续 1000ms 仍失败则降级 robotMoveFastStop（丢弃板载 RIB 点，
    //    保证取消/急停后板上不再有点继续执行）。
    const auto now = std::chrono::steady_clock::now();
    if (stop_motion_requested_.load()) {
      if (!stop_active) {
        stop_active = true;
        stop_begin = now;
        last_stop_retry = now - std::chrono::milliseconds(params_.stop_retry_ms);
      }
      if (now - last_stop_retry >= std::chrono::milliseconds(params_.stop_retry_ms)) {
        last_stop_retry = now;
        const int ret = conn_status_.rootServiceRobotMoveControl(
          aubo_robot_namespace::RobotMoveStop);
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
          stop_motion_requested_.store(false);
          stop_active = false;
          RCLCPP_INFO(log, "RobotMoveStop done (on-board RIB points dropped)");
        } else if (now - stop_begin >= std::chrono::milliseconds(kStopRetryTimeoutMs)) {
          RCLCPP_WARN(log, "RobotMoveStop keeps failing (ret=%d), fallback robotMoveFastStop", ret);
          conn_status_.robotMoveFastStop();
          stop_motion_requested_.store(false);
          stop_active = false;
        }
      }
    } else {
      stop_active = false;
    }

    ++tick;
    std::this_thread::sleep_for(std::chrono::milliseconds(kIoLoopPeriodMs));
  }
  RCLCPP_INFO(log, "ioLoop exit");
}

// ROS1 setIO 映射（方案 A.3；aubo_driver.cpp:1016-1047）：
//   板载 DO -> RobotBoardUserDO（S4：按名称 "U_DO_XX" 写，见下），
//   板载 AO -> RobotBoardUserAO，工具 AO -> RobotToolAO，
//   工具 DO -> SetToolDigitalIOType + SetToolDOStatus。
// 写后读回校验；结果经 set_io_async_success 反馈（1.0 成功 / -1.0 失败）。
void AuboE5Hardware::handleIoCommands()
{
  namespace ns = aubo_robot_namespace;

  for (std::size_t i = 0; i < kNumBoardDIO; ++i) {
    const double v = io_do_cmd_[i];
    if (std::isnan(v)) {
      continue;
    }
    // S4：DO 按名称写（SDK 按名称重载存在：serviceinterface.h:1042 写 /
    // 1061 读），规避基地址歧义 —— vendored 头文件 robotiomatetype.h 定义
    // UserDO 基地址 0x20=32（ROS1 setIO 用 pin+32，aubo_driver.cpp:1016），
    // 而 vendor 文档 C++.md:5528 示例用 40，两处不一致；真机验证项 V2 出
    // 结果前以名称方式免疫。SDK 名称表无 08/09（U_DO_00..07/U_DO_10..17，
    // robotiomatetype.h:72-87），槽位 i>=8 映射到引脚 i+2，与 pollIoStates
    // 的 S2 槽位对齐同口径。
    char io_name[12];
    std::snprintf(io_name, sizeof(io_name), "U_DO_%02d",
                  static_cast<int>(i) < 8 ? static_cast<int>(i) : static_cast<int>(i) + 2);
    bool ok = conn_status_.robotServiceSetBoardIOStatus(ns::RobotBoardUserDO, io_name, v) ==
      ns::InterfaceCallSuccCode;
    if (ok) {
      double readback = 0.0;
      ok = conn_status_.robotServiceGetBoardIOStatus(ns::RobotBoardUserDO, io_name, readback) ==
        ns::InterfaceCallSuccCode && readback == v;
    }
    io_set_async_success_ = ok ? 1.0 : -1.0;
    io_do_cmd_[i] = kNoNewCmd;
  }

  for (std::size_t i = 0; i < kNumBoardAIO; ++i) {
    const double v = io_ao_cmd_[i];
    if (std::isnan(v)) {
      continue;
    }
    bool ok = conn_status_.robotServiceSetBoardIOStatus(ns::RobotBoardUserAO, static_cast<int>(i),
        v) ==
      ns::InterfaceCallSuccCode;
    if (ok) {
      double readback = 0.0;
      ok = conn_status_.robotServiceGetBoardIOStatus(
        ns::RobotBoardUserAO, static_cast<int>(i), readback) == ns::InterfaceCallSuccCode &&
        std::fabs(readback - v) < 1e-6;
    }
    io_set_async_success_ = ok ? 1.0 : -1.0;
    io_ao_cmd_[i] = kNoNewCmd;
  }

  for (std::size_t i = 0; i < kNumToolIO; ++i) {
    const double v = io_tool_do_cmd_[i];
    if (!std::isnan(v)) {
      const auto addr = static_cast<ns::ToolDigitalIOAddr>(i);
      bool ok = false;
      if (v < 0.0) {
        // ROS1 fun=3 state=-1：配置为输入。
        ok = conn_status_.robotServiceSetToolDigitalIOType(addr, ns::IO_IN) ==
          ns::InterfaceCallSuccCode;
      } else {
        ok = conn_status_.robotServiceSetToolDigitalIOType(addr, ns::IO_OUT) ==
          ns::InterfaceCallSuccCode;
        if (ok) {
          ok = conn_status_.robotServiceSetToolDOStatus(
            addr, v > 0.5 ? ns::IO_STATUS_VALID : ns::IO_STATUS_INVALID) ==
            ns::InterfaceCallSuccCode;
        }
      }
      io_set_async_success_ = ok ? 1.0 : -1.0;
      io_tool_do_cmd_[i] = kNoNewCmd;
    }

    const double a = io_tool_ao_cmd_[i];
    if (!std::isnan(a)) {
      const bool ok = conn_status_.robotServiceSetBoardIOStatus(
        ns::RobotToolAO, static_cast<int>(i), a) == ns::InterfaceCallSuccCode;
      io_set_async_success_ = ok ? 1.0 : -1.0;
      io_tool_ao_cmd_[i] = kNoNewCmd;
    }
  }
}

// 安全 IO 判定（方案 A.4；蓝本 aubo_boot aubo_hardware_interface.cpp:250-261）：
// 控制器 DI[0]==0 或 DI[8]==0 -> 急停；DI[1]==0 或 DI[9]==0 -> 防护停。
// （安全回路为低电平有效，digitalIn[30] 为蓝本的控制器 DI 数组宽度。）
void AuboE5Hardware::pollSafetyIo()
{
  namespace ns = aubo_robot_namespace;
  std::vector<ns::RobotIoDesc> di_vec;
  const std::vector<ns::RobotIoType> io_type{ns::RobotBoardControllerDI};
  if (conn_status_.robotServiceGetBoardIOStatus(io_type, di_vec) != ns::InterfaceCallSuccCode) {
    return;
  }

  std::array<double, kControllerDiCount> digital_in{};
  for (const auto & desc : di_vec) {
    if (desc.ioAddr >= 0 && static_cast<std::size_t>(desc.ioAddr) < kControllerDiCount) {
      digital_in[static_cast<std::size_t>(desc.ioAddr)] = desc.ioValue;
    }
  }

  const bool estop =
    digital_in[kEstopPinA] == 0.0 || digital_in[kEstopPinB] == 0.0;
  const bool protective =
    digital_in[kProtectivePinA] == 0.0 || digital_in[kProtectivePinB] == 0.0;
  emergency_stopped_.store(estop);
  protective_stopped_.store(protective);

  if (estop || protective) {
    // 安全停：只停发点、清队列；机械臂本体由硬件安全回路接管，软件不再
    // 下发任何运动指令（方案 A.4）。
    generation_.fetch_add(1);
    clear_motion_.store(true);
    // N3：健康一旦离开 kHealthOk 就不在硬件层做运行时自动恢复 —— read()
    // 报 ERROR 后 CM 已把组件置 UNCONFIGURED 并走 error/shutdown 链路，
    // 硬件层自己把 health 改回 OK 没有任何消费者，反而会把故障静默吞掉。
    // 恢复统一走 teardown() 末尾的 health 复位 + 重新 configure/activate。
    if (health_.load() == kHealthOk) {
      health_.store(kHealthEstop);
      RCLCPP_ERROR(
        rclcpp::get_logger("AuboE5Hardware.ioLoop"), "%s detected -> queues cleared, health ESTOP",
        estop ? "Emergency stop" : "Protective stop");
    }
  }
}

// 低频轮询：板载/工具 IO 状态 + 上电/碰撞诊断位。
void AuboE5Hardware::pollIoStates()
{
  namespace ns = aubo_robot_namespace;
  std::vector<ns::RobotIoDesc> sv;

  // 用户 DI（蓝本行为：返回数组从下标 6 开始才是用户引脚）。
  {
    const std::vector<ns::RobotIoType> t{ns::RobotBoardUserDI};
    if (conn_status_.robotServiceGetBoardIOStatus(t, sv) == ns::InterfaceCallSuccCode) {
      for (std::size_t i = 6; i < sv.size(); ++i) {
        const int pin = parseIoPin(sv[i].ioName, static_cast<int>(i - 6));
        // S2 槽位对齐：SDK 命名 U_DI_00..07 / U_DI_10..17（无 08/09），而
        // 状态接口槽位是连续 0..15；pin>=10 减 2 使槽 8..15 对应物理 10..17，
        // 与 DO 侧 handleIoCommands 的名称映射（U_DO_00..07/U_DO_10..17，S4）
        // 同口径（修复前 di_8/9 恒 0、do_8 与 di_10 错位）。
        const int slot = pin < 10 ? pin : pin - 2;
        if (slot >= 0 && static_cast<std::size_t>(slot) < kNumBoardDIO) {
          io_di_state_[static_cast<std::size_t>(slot)] = sv[i].ioValue != 0.0 ? 1.0 : 0.0;
        }
      }
    }
  }

  // 用户 AI。
  sv.clear();
  {
    const std::vector<ns::RobotIoType> t{ns::RobotBoardUserAI};
    if (conn_status_.robotServiceGetBoardIOStatus(t, sv) == ns::InterfaceCallSuccCode) {
      for (const auto & desc : sv) {
        if (desc.ioAddr >= 0 && static_cast<std::size_t>(desc.ioAddr) < kNumBoardAIO) {
          io_ai_state_[static_cast<std::size_t>(desc.ioAddr)] = desc.ioValue;
        }
      }
    }
  }

  // 工具 DI / AI。
  sv.clear();
  if (conn_status_.robotServiceGetAllToolDigitalIOStatus(sv) == ns::InterfaceCallSuccCode) {
    for (const auto & desc : sv) {
      if (desc.ioAddr >= 0 && static_cast<std::size_t>(desc.ioAddr) < kNumToolIO) {
        io_tool_di_state_[static_cast<std::size_t>(desc.ioAddr)] = desc.ioValue != 0.0 ? 1.0 : 0.0;
      }
    }
  }
  sv.clear();
  if (conn_status_.robotServiceGetAllToolAIStatus(sv) == ns::InterfaceCallSuccCode) {
    for (const auto & desc : sv) {
      if (desc.ioAddr >= 0 && static_cast<std::size_t>(desc.ioAddr) < kNumToolIO) {
        io_tool_ai_state_[static_cast<std::size_t>(desc.ioAddr)] = desc.ioValue;
      }
    }
  }

  // 上电 / 碰撞标志位取自诊断信息（状态连接）。
  ns::RobotDiagnosis diag;
  if (conn_status_.robotServiceGetRobotDiagnosisInfo(diag) == ns::InterfaceCallSuccCode) {
    power_on_.store(diag.armPowerStatus);
    collision_.store(diag.robotCollision);
  }
}

// ============================================================================
// SDK 回调（跑在 SDK 内部推送线程）。回调纪律：只写缓存/原子量 ——
// 不调 SDK、不打日志、不做堆分配。原因：回调由 SDK 接收线程在内核数据
// 路径上触发，回调里再调 SDK 会重入其内部锁导致死锁；阻塞或分配会拖慢
// 推送线程造成帧堆积/丢失。状态经 RealtimeThreadSafeBox 传递给 RT read()。
// ============================================================================
void AuboE5Hardware::jointStatusCallback(
  const aubo_robot_namespace::JointStatus * status, int size, void * arg)
{
  auto * self = static_cast<AuboE5Hardware *>(arg);
  if (status == nullptr || self == nullptr) {
    return;
  }
  JointStateSnapshot snap;
  const int n = std::min(size, static_cast<int>(kNumJoints));
  for (int i = 0; i < n; ++i) {
    snap.pos[static_cast<std::size_t>(i)] = static_cast<double>(status[i].jointPosJ);
    snap.vel_moto[static_cast<std::size_t>(i)] = static_cast<double>(status[i].jointSpeedMoto);
    snap.tag_pos[static_cast<std::size_t>(i)] = static_cast<double>(status[i].jointTagPosJ);
    snap.tag_vel_moto[static_cast<std::size_t>(i)] =
      static_cast<double>(status[i].jointTagSpeedMoto);
    snap.current_i[static_cast<std::size_t>(i)] = static_cast<double>(status[i].jointCurrentI);
    snap.temp[static_cast<std::size_t>(i)] = static_cast<double>(status[i].jointCurTemp);
    snap.err[static_cast<std::size_t>(i)] = static_cast<int>(status[i].jointErrorNum);
  }
  snap.stamp = std::chrono::steady_clock::now();
  snap.valid = true;
  self->state_box_.try_set(snap);  // 写冲突时丢帧（新帧很快会再来）
}

void AuboE5Hardware::robotEventCallback(
  const aubo_robot_namespace::RobotEventInfo * info, void * arg)
{
  auto * self = static_cast<AuboE5Hardware *>(arg);
  if (info == nullptr || self == nullptr) {
    return;
  }
  if (info->eventType == aubo_robot_namespace::RobotEvent_socketDisconnected) {
    self->health_.store(kHealthFault);
  }
  // 所有事件入队，由 ioLoop 统一解码打日志（回调里绝不打日志）。
  // 纪律权衡：事件回调与 200Hz 关节状态回调不同 —— 事件频率极低（错误/
  // 状态变化才触发），拷贝 eventContent 字符串的一次堆分配可接受；换来
  // 的纪律收益是回调里依然零日志、零 SDK 调用，且日志集中在 ioLoop 一个
  // 线程输出（时序不交错）。队列满时丢弃并计数，ioLoop 打 WARN 兜底。
  EventItem item;
  item.type = static_cast<int>(info->eventType);
  item.code = info->eventCode;
  item.content = info->eventContent;
  if (!self->event_queue_.try_enqueue(std::move(item))) {
    self->dropped_events_.fetch_add(1);
  }
}

// RobotEventType 全量解码表（AuboRobotMetaType.h:895-1018，含枚举自带中文
// 注释的转写）。完整表只放硬件这一处；IO 控制器的诊断里用简化表（状态
// 接口传不了字符串，避免两个包重复维护大表）。
const char * AuboE5Hardware::eventToString(int type)
{
  namespace ns = aubo_robot_namespace;
  // 用户自定义事件区间（RobotEvent_User..RobotEvent_MaxUser = 9000..9999）。
  if (type >= ns::RobotEvent_User && type <= ns::RobotEvent_MaxUser) {
    return "RobotEvent_User(用户自定义事件)";
  }
  switch (type) {
    case ns::RobotEvent_armCanbusError: return "RobotEvent_armCanbusError(机械臂CAN总线错误)";
    case ns::RobotEvent_remoteHalt: return "RobotEvent_remoteHalt(远程关机)";
    case ns::RobotEvent_remoteEmergencyStop: return "RobotEvent_remoteEmergencyStop(远程急停)";
    case ns::RobotEvent_jointError: return "RobotEvent_jointError(关节错误)";
    case ns::RobotEvent_forceControl: return "RobotEvent_forceControl(力控制)";
    case ns::RobotEvent_exitForceControl: return "RobotEvent_exitForceControl(退出力控制)";
    case ns::RobotEvent_softEmergency: return "RobotEvent_softEmergency(软急停)";
    case ns::RobotEvent_exitSoftEmergency: return "RobotEvent_exitSoftEmergency(退出软急停)";
    case ns::RobotEvent_collision: return "RobotEvent_collision(碰撞)";
    case ns::RobotEvent_collisionStatusChanged: return
        "RobotEvent_collisionStatusChanged(碰撞状态改变)";
    case ns::RobotEvent_tcpParametersSucc: return
        "RobotEvent_tcpParametersSucc(工具动力学参数设置成功)";
    case ns::RobotEvent_powerChanged: return "RobotEvent_powerChanged(电源开关状态改变)";
    case ns::RobotEvent_ArmPowerOff: return "RobotEvent_ArmPowerOff(机械臂电源关闭)";
    case ns::RobotEvent_mountingPoseChanged: return "RobotEvent_mountingPoseChanged(安装位置改变)";
    case ns::RobotEvent_encoderError: return "RobotEvent_encoderError(编码器错误)";
    case ns::RobotEvent_encoderLinesError: return "RobotEvent_encoderLinesError(编码器线数不一致)";
    case ns::RobotEvent_singularityOverspeed: return "RobotEvent_singularityOverspeed(奇异点超速)";
    case ns::RobotEvent_currentAlarm: return "RobotEvent_currentAlarm(电流异常)";
    case ns::RobotEvent_toolioError: return "RobotEvent_toolioError(工具端错误)";
    case ns::RobotEvent_robotStartupPhase: return "RobotEvent_robotStartupPhase(启动阶段)";
    case ns::RobotEvent_robotStartupDoneResult: return
        "RobotEvent_robotStartupDoneResult(启动完成结果)";
    case ns::RobotEvent_robotShutdownDone: return "RobotEvent_robotShutdownDone(关机结果)";
    case ns::RobotEvent_atTrackTargetPos: return "RobotEvent_atTrackTargetPos(轨迹运动到位)";
    case ns::RobotSetPowerOnDone: return "RobotSetPowerOnDone(设置电源状态完成)";
    case ns::RobotReleaseBrakeDone: return "RobotReleaseBrakeDone(刹车释放完成)";
    case ns::RobotEvent_robotControllerStateChaned: return
        "RobotEvent_robotControllerStateChaned(控制状态改变)";
    case ns::RobotEvent_robotControllerError: return
        "RobotEvent_robotControllerError(控制错误/算法规划问题)";
    case ns::RobotEvent_socketDisconnected: return "RobotEvent_socketDisconnected(socket断开连接)";
    case ns::RobotEvent_robotControlException: return "RobotEvent_robotControlException(控制异常)";
    case ns::RobotEvent_trackPlayInterrupte: return "RobotEvent_trackPlayInterrupte(轨迹回放中断)";
    case ns::RobotEvent_staticCollisionStatusChanged: return
        "RobotEvent_staticCollisionStatusChanged(静态碰撞状态改变)";
    case ns::RobotEvent_MountingPoseWarning: return "RobotEvent_MountingPoseWarning(安装位置警告)";
    case ns::RobotEvent_MacDataInterruptWarning: return
        "RobotEvent_MacDataInterruptWarning(Mac数据中断警告)";
    case ns::RobotEvent_ToolIoError: return "RobotEvent_ToolIoError(工具IO错误)";
    case ns::RobotEvent_InterfacBoardSafeIoEvent: return
        "RobotEvent_InterfacBoardSafeIoEvent(接口板安全IO事件)";
    case ns::RobotEvent_RobotHandShakeSucc: return "RobotEvent_RobotHandShakeSucc(握手成功)";
    case ns::RobotEvent_RobotHandShakeFailed: return "RobotEvent_RobotHandShakeFailed(握手失败)";
    case ns::RobotEvent_RobotErrorInfoNotify: return
        "RobotEvent_RobotErrorInfoNotify(错误信息通知)";
    case ns::RobotEvent_InterfacBoardDIChanged: return
        "RobotEvent_InterfacBoardDIChanged(接口板DI改变)";
    case ns::RobotEvent_InterfacBoardDOChanged: return
        "RobotEvent_InterfacBoardDOChanged(接口板DO改变)";
    case ns::RobotEvent_InterfacBoardAIChanged: return
        "RobotEvent_InterfacBoardAIChanged(接口板AI改变)";
    case ns::RobotEvent_InterfacBoardAOChanged: return
        "RobotEvent_InterfacBoardAOChanged(接口板AO改变)";
    case ns::RobotEvent_UpdateJoint6Rot360Flag: return
        "RobotEvent_UpdateJoint6Rot360Flag(关节6旋转360标志更新)";
    case ns::RobotEvent_RobotMoveControlDone: return
        "RobotEvent_RobotMoveControlDone(运动控制完成)";
    case ns::RobotEvent_RobotMoveControlStopDone: return
        "RobotEvent_RobotMoveControlStopDone(运动停止完成)";
    case ns::RobotEvent_RobotMoveControlPauseDone: return
        "RobotEvent_RobotMoveControlPauseDone(运动暂停完成)";
    case ns::RobotEvent_RobotMoveControlContinueDone: return
        "RobotEvent_RobotMoveControlContinueDone(运动继续完成)";
    case ns::RobotEvent_RobotSwitchToOnlineMaster: return
        "RobotEvent_RobotSwitchToOnlineMaster(切换在线主模式)";
    case ns::RobotEvent_RobotSwitchToOnlineSlave: return
        "RobotEvent_RobotSwitchToOnlineSlave(切换在线从模式)";
    case ns::RobotEvent_ConveyorTrackRobotStartup: return
        "RobotEvent_ConveyorTrackRobotStartup(传送带跟踪启动)";
    case ns::RobotEvent_ConveyorTrackRobotCatchup: return
        "RobotEvent_ConveyorTrackRobotCatchup(传送带跟踪追上)";
    case ns::RobotEvent_exceptEvent: return "RobotEvent_exceptEvent(异常事件)";
    case ns::RobotEventInvalid: return "RobotEventInvalid(无效事件)";
    case ns::RobotEventMoveJConfigError: return "RobotEventMoveJConfigError(moveJ属性配置错误)";
    case ns::RobotEventMoveLConfigError: return "RobotEventMoveLConfigError(moveL属性配置错误)";
    case ns::RobotEventMovePConfigError: return "RobotEventMovePConfigError(moveP属性配置错误)";
    case ns::RobotEventInvailConfigError: return "RobotEventInvailConfigError(无效运动属性配置)";
    case ns::RobotEventWaitRobotStopped: return "RobotEventWaitRobotStopped(等待机器人停止)";
    case ns::RobotEventJointOutRange: return "RobotEventJointOutRange(超出关节运动范围)";
    case ns::RobotEventFirstWaypointSetError: return
        "RobotEventFirstWaypointSetError(MODEP第一个路点设置错误)";
    case ns::RobotEventConveyorTrackConfigError: return
        "RobotEventConveyorTrackConfigError(传送带跟踪配置错误)";
    case ns::RobotEventConveyorTrackTrajectoryTypeError: return
        "RobotEventConveyorTrackTrajectoryTypeError(传送带轨迹类型错误)";
    case ns::RobotEventRelativeTransformIKFailed: return
        "RobotEventRelativeTransformIKFailed(相对坐标变换逆解失败)";
    case ns::RobotEventTeachModeCollision: return "RobotEventTeachModeCollision(示教模式碰撞)";
    case ns::RobotEventextErnalToolConfigError: return
        "RobotEventextErnalToolConfigError(外部工具/手持工件配置错误)";
    case ns::RobotEventTrajectoryAbnormal: return "RobotEventTrajectoryAbnormal(轨迹异常)";
    case ns::RobotEventOnlineTrajectoryPlanError: return
        "RobotEventOnlineTrajectoryPlanError(在线轨迹规划失败)";
    case ns::RobotEventOnlineTrajectoryTypeIIError: return
        "RobotEventOnlineTrajectoryTypeIIError(二型在线轨迹规划失败)";
    case ns::RobotEventIKFailed: return "RobotEventIKFailed(逆解失败)";
    case ns::RobotEventAbnormalLimitProtect: return
        "RobotEventAbnormalLimitProtect(动力学限制保护)";
    case ns::RobotEventConveyorTrackingFailed: return
        "RobotEventConveyorTrackingFailed(传送带跟踪失败)";
    case ns::RobotEventConveyorOutWorkingRange: return
        "RobotEventConveyorOutWorkingRange(超出传送带工作范围)";
    case ns::RobotEventTrajectoryJointOutOfRange: return
        "RobotEventTrajectoryJointOutOfRange(轨迹关节超出范围)";
    case ns::RobotEventTrajectoryJointOverspeed: return
        "RobotEventTrajectoryJointOverspeed(轨迹关节超速)";
    case ns::RobotEventOfflineTrajectoryPlanFailed: return
        "RobotEventOfflineTrajectoryPlanFailed(离线轨迹规划失败)";
    case ns::RobotEventControllerIKFailed: return
        "RobotEventControllerIKFailed(控制器异常/逆解失败)";
    case ns::RobotEventControllerStatusException: return
        "RobotEventControllerStatusException(控制器异常/状态异常)";
    case ns::RobotEventMoveEnterStopState: return "RobotEventMoveEnterStopState(运动进入stop阶段)";
    case ns::robot_event_unknown: return "robot_event_unknown(未知事件)";
    default: return nullptr;  // 未覆盖的数值由调用方按 "unknown(<n>)" 打印
  }
}

// 事件严重级别映射（0=DEBUG 1=INFO 2=WARN 3=ERROR）：
//   ERROR  明确错误/故障类（含控制器错误 1000-1300 区间、硬件异常
//          2001-2999 区间、Mac 数据中断）；
//   WARN   状态变化类（碰撞/电源/安装位姿/控制状态变化等）；
//   DEBUG  接口板 DI/DO/AI/AO 变化（可能高频，不打 INFO 防刷屏）；
//   INFO   生命周期/流程通知（启动/关机/握手/到位/力控进出等）。
int AuboE5Hardware::eventLogLevel(int type)
{
  namespace ns = aubo_robot_namespace;
  // 控制器异常事件区间（RobotEventInvalid=1000 起到 MoveEnterStopState=1300，
  // AuboRobotMetaType.h:967-1002 注释：这些事件会引起机械臂运动错误返回）。
  if (type >= ns::RobotEventInvalid && type <= ns::RobotEventMoveEnterStopState) {
    return 3;
  }
  // 2xxx 硬件异常事件区间：vendored 1.3.1 头文件没有 2xxx 枚举定义
  // （RobotEventType 止于 1300 与 9000..9999 用户区），但固件会上报该区间的
  // 硬件异常，按数值区间覆盖为 ERROR（S1）。
  if (type >= 2001 && type <= 2999) {
    return 3;
  }
  switch (type) {
    case ns::RobotEvent_armCanbusError:
    case ns::RobotEvent_remoteEmergencyStop:
    case ns::RobotEvent_jointError:
    case ns::RobotEvent_softEmergency:
    case ns::RobotEvent_collision:
    case ns::RobotEvent_encoderError:
    case ns::RobotEvent_encoderLinesError:
    case ns::RobotEvent_singularityOverspeed:
    case ns::RobotEvent_currentAlarm:
    case ns::RobotEvent_toolioError:
    case ns::RobotEvent_ToolIoError:
    case ns::RobotEvent_robotControllerError:
    case ns::RobotEvent_socketDisconnected:
    case ns::RobotEvent_robotControlException:
    case ns::RobotEvent_InterfacBoardSafeIoEvent:
    case ns::RobotEvent_RobotHandShakeFailed:
    case ns::RobotEvent_RobotErrorInfoNotify:
    case ns::RobotEvent_exceptEvent:
    case ns::RobotEvent_MacDataInterruptWarning:  // Mac 数据中断 = 板载链路故障，升 ERROR（S1）
      return 3;
    case ns::RobotEvent_remoteHalt:
    case ns::RobotEvent_exitSoftEmergency:
    case ns::RobotEvent_collisionStatusChanged:
    case ns::RobotEvent_powerChanged:
    case ns::RobotEvent_ArmPowerOff:
    case ns::RobotEvent_mountingPoseChanged:
    case ns::RobotEvent_trackPlayInterrupte:
    case ns::RobotEvent_staticCollisionStatusChanged:
    case ns::RobotEvent_MountingPoseWarning:
    case ns::RobotEvent_robotControllerStateChaned:
      return 2;
    case ns::RobotEvent_InterfacBoardDIChanged:
    case ns::RobotEvent_InterfacBoardDOChanged:
    case ns::RobotEvent_InterfacBoardAIChanged:
    case ns::RobotEvent_InterfacBoardAOChanged:
      return 0;
    default:
      return 1;
  }
}

// ioLoop 每周期 drain 事件队列：按严重级别打日志并刷新 event_type_/
// event_code_ 原子量（read() 据此更新 aubo_io/event_type、event_code）。
void AuboE5Hardware::drainEventQueue()
{
  const auto log = rclcpp::get_logger("AuboE5Hardware.ioLoop");
  EventItem item;
  while (event_queue_.try_dequeue(item)) {
    const int level = eventLogLevel(item.type);
    const char * name = eventToString(item.type);
    if (name == nullptr) {
      // 未覆盖数值：不猜名字，原样打印数值。
      RCLCPP_WARN(log, "[AuboEvent] type=unknown(%d) code=%d content=%s", item.type, item.code,
                  item.content.c_str());
    } else {
      switch (level) {
        case 3:
          RCLCPP_ERROR(log, "[AuboEvent] type=%s(%d) code=%d content=%s", name, item.type,
                       item.code, item.content.c_str());
          break;
        case 2:
          RCLCPP_WARN(log, "[AuboEvent] type=%s(%d) code=%d content=%s", name, item.type,
                      item.code, item.content.c_str());
          break;
        case 0:
          RCLCPP_DEBUG(log, "[AuboEvent] type=%s(%d) code=%d content=%s", name, item.type,
                       item.code, item.content.c_str());
          break;
        default:
          RCLCPP_INFO(log, "[AuboEvent] type=%s(%d) code=%d content=%s", name, item.type,
                      item.code, item.content.c_str());
          break;
      }
    }
    // ERROR 级事件 = 明确故障：除日志外执行清场三件套 + 锁 FAULT（S1）。
    // 处置链路：health FAULT -> read() 返回 ERROR -> CM 停控制器 ->
    // 控制器 deactivate 时 abort goal（action 所有权在控制器侧，硬件不碰）。
    // socketDisconnected 原本就在 robotEventCallback 里锁了 FAULT，这里重复
    // 置位是原子幂等，无害，不必特判。
    if (level == 3) {
      generation_.fetch_add(1);
      clear_motion_.store(true);
      stop_motion_requested_.store(true);
      health_.store(kHealthFault);
    }
    event_type_.store(static_cast<double>(item.type));
    event_code_.store(static_cast<double>(item.code));
  }
  // 队列溢出兜底：只 WARN 一次并清零（溢出本身说明事件突发超出 64 条缓冲，
  // 具体丢了哪些无从得知，但丢事件这件事必须可见）。
  const uint64_t dropped = dropped_events_.exchange(0);
  if (dropped > 0) {
    RCLCPP_WARN(log, "[AuboEvent] %lu event(s) dropped (queue full)", dropped);
  }
}

}  // namespace aubo_e5_hardware

PLUGINLIB_EXPORT_CLASS(aubo_e5_hardware::AuboE5Hardware, hardware_interface::SystemInterface)
