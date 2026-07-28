// ============================================================================
// aubo_e5_sim_hardware.cpp —— AUBO E5 passthrough 流水线的控制柜模拟器插件。
// 定位：UR 驱动生态里 ursim 的等价物 —— 没有真机时，让
// AuboPassthroughTrajectoryController（及 MoveIt）跑通完整闭环。
//
// 仿真行为（与真机插件 AuboE5Hardware 契约一致）：
//   - 相同的 joint + gpio 接口（trajectory_passthrough / speed_scaling / aubo_io）
//   - transfer 状态机 0/1/2/3/4/5/6，含硬件回写（语义同真机插件 write()）
//   - 收到的轨迹点按五次插值重采样为 5ms 点流（系数与 aubo_boot
//     quinticInterpolate 逐字一致，joint_trajectory_controller.cpp:183-202）
//   - 虚拟接口板：每个控制周期恰好消费 1 个点（200Hz == 5ms/点，与真板
//     消费节奏一致）；rib_level = 板载队列点数 * 6 —— 模拟真板 RIB
//     （macTargetPosDataSize）按"关节分量数"计数，1 点 = 6 个分量，与
//     ROS1 publishWaypointToRobot 中 ceil((400-rib)/6) 的换算口径一致
//     （aubo_driver.cpp:959）
//   - 关节位置跟随点流；板载队列排空后回写 DONE(5)
//   - abort 丢弃已排队的点（相当于真机的 RobotMoveStop 停止原语）
// 线程模型：单线程 —— 只有 controller_manager 的 RT 线程调 read()/write()，
// 没有像真机插件那样的发送/IO 异步线程（虚拟板在 write() 内同步消费）。
// ============================================================================
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aubo_e5_hardware
{

class AuboE5SimHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (info_.joints.size() != kNumJoints) {
      RCLCPP_ERROR(rclcpp::get_logger("AuboE5SimHardware"), "expected 6 joints");
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> out;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      out.emplace_back(info_.joints[i].name, "position", &pos_[i]);
      out.emplace_back(info_.joints[i].name, "velocity", &vel_[i]);
    }
    out.emplace_back("speed_scaling", "speed_scaling_factor", &speed_scaling_);
    // 模拟恒安全态：除 power_on=1（已上电）外其余状态全 0。
    // 反馈增强接口（tag_pos/tag_vel/joint_current/joint_temp/send_queue_points/
    // send_rate_pps）与事件/健康接口（event_type/event_code/health）与真机
    // 插件同名导出，具体数值在 read()/write() 里按虚拟板状态填（模拟值，
    // 详见 read() 注释）。
    for (const auto & gpio : {"di_0", "di_1", "di_2", "di_3", "di_4", "di_5", "di_6", "di_7",
                              "di_8", "di_9", "di_10", "di_11", "di_12", "di_13", "di_14",
                              "di_15", "ai_0", "ai_1", "ai_2", "ai_3", "tool_di_0",
                              "tool_di_1", "tool_ai_0", "tool_ai_1", "estop",
                              "protective_stop", "power_on", "collision", "in_motion",
                              "rib_level", "joint_error_0", "joint_error_1", "joint_error_2",
                              "joint_error_3", "joint_error_4", "joint_error_5",
                              "tag_pos_0", "tag_pos_1", "tag_pos_2", "tag_pos_3", "tag_pos_4",
                              "tag_pos_5", "tag_vel_0", "tag_vel_1", "tag_vel_2", "tag_vel_3",
                              "tag_vel_4", "tag_vel_5", "joint_current_0", "joint_current_1",
                              "joint_current_2", "joint_current_3", "joint_current_4",
                              "joint_current_5", "joint_temp_0", "joint_temp_1", "joint_temp_2",
                              "joint_temp_3", "joint_temp_4", "joint_temp_5",
                              "send_queue_points", "send_rate_pps",
                              // 事件/健康上报（与真机插件同名）：event_code 恒 0、
                              // health 恒 0 = kHealthOk（模拟恒健康）；
                              // event_type 恒 -1 = "无事件"哨兵（N9：模拟器
                              // 没有 SDK 事件源，0 会被 io 控制器误解码成
                              // armCanbusError 显示）。
                              "event_type", "event_code", "health"}) {
      aubo_io_states_[gpio] = (std::string(gpio) == "power_on") ? 1.0 :
                              ((std::string(gpio) == "event_type") ? -1.0 : 0.0);
      out.emplace_back("aubo_io", gpio, &aubo_io_states_[gpio]);
    }
    return out;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> out;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      out.emplace_back(info_.joints[i].name, "position", &cmd_[i]);
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      out.emplace_back("trajectory_passthrough", "setpoint_positions_" + std::to_string(i),
                       &sp_pos_[i]);
      out.emplace_back("trajectory_passthrough", "setpoint_velocities_" + std::to_string(i),
                       &sp_vel_[i]);
      out.emplace_back("trajectory_passthrough", "setpoint_accelerations_" + std::to_string(i),
                       &sp_acc_[i]);
    }
    out.emplace_back("trajectory_passthrough", "transfer_state", &transfer_state_);
    out.emplace_back("trajectory_passthrough", "time_from_start", &sp_time_);
    out.emplace_back("trajectory_passthrough", "abort", &abort_);
    out.emplace_back("trajectory_passthrough", "trajectory_size", &traj_size_);
    // IO 命令槽以 NaN 表示"无请求"（同真机插件的哨兵约定）。
    for (const auto & gpio : {"do_0", "do_1", "do_2", "do_3", "do_4", "do_5", "do_6", "do_7",
                              "do_8", "do_9", "do_10", "do_11", "do_12", "do_13", "do_14",
                              "do_15", "ao_0", "ao_1", "ao_2", "ao_3", "tool_do_0",
                              "tool_do_1", "tool_ao_0", "tool_ao_1", "set_io_async_success"}) {
      aubo_io_cmds_[gpio] = std::numeric_limits<double>::quiet_NaN();
      out.emplace_back("aubo_io", gpio, &aubo_io_cmds_[gpio]);
    }
    return out;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 命令 = 当前位置，激活瞬间不跳变（同真机插件）。
    for (std::size_t i = 0; i < kNumJoints; ++i) cmd_[i] = pos_[i];
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    // 速度由位置差分估算；period 异常时按 5ms 点距兜底。
    const double dt = period.seconds() > 0.0 ? period.seconds() : 0.005;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      vel_[i] = (pos_[i] - prev_pos_[i]) / dt;
      prev_pos_[i] = pos_[i];
    }
    // rib_level：模拟真板 RIB 口径，板载队列点数 * 6（1 点 = 6 关节分量）。
    aubo_io_states_["rib_level"] = static_cast<double>(board_queue_.size()) * 6.0;
    aubo_io_states_["in_motion"] = board_queue_.empty() ? 0.0 : 1.0;
    // 反馈增强（全部为模拟值，与真机插件同名同语义）：
    // tag_pos = 虚拟板最新消费点的位置（write() 里更新到 tag_pos_consumed_），
    // tag_vel = 相邻消费点差分 / 5ms；电流/温度恒 0（虚拟板没有热模型）；
    // send_queue_points = 板载队列长度；send_rate_pps 恒 200（每周期恰好
    // 消费 1 个 5ms 点 == 200 pts/s，与文件头仿真契约一致）。
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      aubo_io_states_["tag_pos_" + std::to_string(i)] = tag_pos_consumed_[i];
      aubo_io_states_["tag_vel_" + std::to_string(i)] = tag_vel_consumed_[i];
      // joint_current_* / joint_temp_* 保持导出时的 0。
    }
    aubo_io_states_["send_queue_points"] = static_cast<double>(board_queue_.size());
    aubo_io_states_["send_rate_pps"] = 200.0;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("AuboE5SimHardware"), sim_clock_, 500,
                         "write: ts=%.1f abort=%.1f size=%.1f recv=%lu sp=%zu board=%zu",
                         transfer_state_, abort_, traj_size_, points_received_,
                         setpoints_.size(), board_queue_.size());
    // ---- transfer 状态机（语义与真机插件一致，UR passthrough 契约）----
    if (abort_ == 1.0 && transfer_state_ != kIdle) {
      // 模拟 RobotMoveStop：丢弃板载点，回到 IDLE(0)。
      setpoints_.clear();
      board_queue_.clear();
      have_prev_setpoint_ = false;
      have_consumed_point_ = false;  // 差分基准失效，下一点 tag_vel 重新从 0 起
      transfer_state_ = kIdle;
    } else if (transfer_state_ == kNewTrajectory) {  // 6 新轨迹：清旧轨迹 -> 1
      setpoints_.clear();
      board_queue_.clear();
      have_prev_setpoint_ = false;
      have_consumed_point_ = false;
      points_received_ = 0;
      transfer_state_ = kAccepted;
    } else if (transfer_state_ == kTransferring) {   // 2 收一个点 -> 1
      Setpoint sp{sp_pos_, sp_vel_, sp_acc_, sp_time_};
      setpoints_.push_back(sp);
      ++points_received_;
      transfer_state_ = kAccepted;
    } else if (transfer_state_ == kTransferDoneCmd) {  // 3 点发完 -> 4 执行中
      transfer_state_ = kInMotion;
    }

    // ---- 把新收齐的段重采样成 5ms 板载点 ----
    while (setpoints_.size() >= 2 || (transfer_state_ == kInMotion && !setpoints_.empty())) {
      if (!have_prev_setpoint_) {
        prev_setpoint_ = setpoints_.front();
        setpoints_.pop_front();
        have_prev_setpoint_ = true;
        if (setpoints_.empty() && transfer_state_ != kInMotion) break;
        if (setpoints_.empty()) { emit_point(prev_setpoint_); break; }
      }
      const Setpoint curr = setpoints_.front();
      setpoints_.pop_front();
      const double dt = curr.time_from_start - prev_setpoint_.time_from_start;
      const int steps = std::max(1, static_cast<int>(std::lround(dt / 0.005)));
      for (int s = 1; s <= steps; ++s) {
        const double t = dt * static_cast<double>(s) / static_cast<double>(steps);
        emit_point(quintic(prev_setpoint_, curr, dt, t));
      }
      prev_setpoint_ = curr;
    }

    // ---- 虚拟接口板：每周期恰好消费一个 5ms 点 ----
    if (!board_queue_.empty()) {
      // 记录最新消费点为 tag_pos，并按相邻消费点差分估算 tag_vel（5ms 点距）。
      const auto & consumed = board_queue_.front();
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        tag_vel_consumed_[i] = have_consumed_point_ ? (consumed[i] - tag_pos_consumed_[i]) / 0.005 : 0.0;
        tag_pos_consumed_[i] = consumed[i];
      }
      have_consumed_point_ = true;
      pos_ = consumed;
      board_queue_.pop_front();
      for (std::size_t i = 0; i < kNumJoints; ++i) cmd_[i] = pos_[i];
    } else if (transfer_state_ == kInMotion &&
               points_received_ >= static_cast<uint64_t>(traj_size_) && setpoints_.empty()) {
      // 点已收齐、重采样完、板载队列排空 -> 回写 DONE(5)。
      transfer_state_ = kDone;
    }
    return hardware_interface::return_type::OK;
  }

private:
  struct Setpoint
  {
    std::array<double, 6> pos{}, vel{}, acc{};
    double time_from_start{0.0};
  };

  static Setpoint quintic(const Setpoint & l, const Setpoint & c, double T, double t)
  {
    // 蓝本五次插值系数（aubo_boot quinticInterpolate，183-202），逐字一致。
    Setpoint out;
    out.time_from_start = l.time_from_start + t;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    for (int i = 0; i < 6; ++i) {
      const double a1 = l.vel[i], a2 = 0.5 * l.acc[i];
      const double h = c.pos[i] - l.pos[i];
      const double a3 = 0.5 / T3 * (20 * h - (8 * c.vel[i] + 12 * l.vel[i]) * T -
                                    (3 * l.acc[i] - c.acc[i]) * T2);
      const double a4 = 0.5 / T4 * (-30 * h + (14 * c.vel[i] + 16 * l.vel[i]) * T +
                                    (3 * l.acc[i] - 2 * c.acc[i]) * T2);
      const double a5 = 0.5 / T5 * (12 * h - 6 * (c.vel[i] + l.vel[i]) * T +
                                    (c.acc[i] - l.acc[i]) * T2);
      out.pos[i] = l.pos[i] + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
      out.vel[i] = a1 + 2 * a2 * t + 3 * a3 * t2 + 4 * a4 * t3 + 5 * a5 * t4;
      out.acc[i] = 2 * a2 + 6 * a3 * t + 12 * a4 * t2 + 20 * a5 * t3;
    }
    return out;
  }

  void emit_point(const Setpoint & sp) { board_queue_.push_back(sp.pos); }

  static constexpr std::size_t kNumJoints = 6;
  // transfer 状态机取值（0..6，语义同真机插件 write() 注释块）。
  static constexpr double kIdle = 0.0, kAccepted = 1.0, kTransferring = 2.0;
  static constexpr double kTransferDoneCmd = 3.0, kInMotion = 4.0, kDone = 5.0;
  static constexpr double kNewTrajectory = 6.0;

  std::array<double, kNumJoints> pos_{}, vel_{}, prev_pos_{}, cmd_{};
  std::array<double, kNumJoints> sp_pos_{}, sp_vel_{}, sp_acc_{};
  double sp_time_{0.0}, transfer_state_{kIdle}, abort_{0.0}, traj_size_{0.0};
  double speed_scaling_{1.0};
  std::map<std::string, double> aubo_io_states_, aubo_io_cmds_;

  std::deque<Setpoint> setpoints_;                    // 已收未重采样的轨迹点
  std::deque<std::array<double, kNumJoints>> board_queue_;  // 虚拟板载 5ms 点队列
  // 虚拟板最新消费点（-> tag_pos_*）及其差分速度（-> tag_vel_*），模拟值。
  std::array<double, kNumJoints> tag_pos_consumed_{}, tag_vel_consumed_{};
  bool have_consumed_point_{false};
  Setpoint prev_setpoint_{};
  bool have_prev_setpoint_{false};
  uint64_t points_received_{0};
  rclcpp::Clock sim_clock_{RCL_STEADY_TIME};
};

}  // namespace aubo_e5_hardware

PLUGINLIB_EXPORT_CLASS(aubo_e5_hardware::AuboE5SimHardware, hardware_interface::SystemInterface)
