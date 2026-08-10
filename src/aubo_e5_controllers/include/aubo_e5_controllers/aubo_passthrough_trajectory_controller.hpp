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
 * AuboPassthroughTrajectoryController —— AUBO E5 驱动的轨迹直通控制器插件，
 * 本地重写自 UR 的 ur_controllers::PassthroughTrajectoryController
 * （参考：github.com/UniversalRobots/Universal_Robots_ROS2_Driver
 * （jazzy 分支）ur_controllers/src/passthrough_trajectory_controller.cpp）。核心思路与 UR 一致：通过
 * trajectory_passthrough/ 前缀下的一组命令接口把轨迹点逐个周期透传给
 * 硬件插件，硬件自己插补执行。
 *
 * 与 UR 原版相比的差异（行为蓝本：aubo_boot 实测驱动的
 * joint_trajectory_controller）：
 *  - remapJointNames：接受 goal 时按关节名重排到权威顺序（params.joints）；
 *    含未知关节或缺关节的 goal 直接拒绝。
 *  - blendToFirstPoint：首轨迹点与当前关节位置偏差超过 blend_threshold_rad
 *    时，在前面补一段 smoothstep（3t²-2t³，C1）融合段（blend_steps x 5ms）。
 *  - goal_hold 延迟 result：硬件上报 DONE 后不立即成功，等实际关节状态
 *    连续 goal_hold_frames 次（周期 goal_check_ms）满足容差
 *    （位置 < goal_tolerance_rad 且 |速度| < goal_vel_tolerance）才回成功；
 *    goal 自带容差可覆盖默认值；goal_time 超时则 aborted。
 *  - 抢占（preemption）：新 goal 直接中止正在传输的轨迹（abort=1.0），
 *    等硬件回到 IDLE 后再开始新传输。
 */
//----------------------------------------------------------------------

#ifndef AUBO_E5_CONTROLLERS__AUBO_PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
#define AUBO_E5_CONTROLLERS__AUBO_PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_

#include <stdint.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "aubo_e5_controllers/aubo_passthrough_trajectory_controller_parameters.hpp"

namespace aubo_e5_controllers
{

/*
 * 与硬件插件之间的传输状态（transfer_state）契约（沿用 UR passthrough 命名）。
 * 协议为“每周期一个点”的交替握手：控制器写 setpoint 后置 2.0，硬件消费完
 * 回答 1.0，控制器再写下一点，直到全部点传完。
 * 各值的读写方约定：
 * 0.0 IDLE：                无轨迹，控制器空闲 / 可接受新轨迹。
 *                           写方：控制器；读方：双方。
 * 6.0 NEW_TRAJECTORY：      控制器接受了新轨迹（写方：控制器）；硬件收到后
 *                           清空自身缓冲区并回答 1.0（写方：硬件）。
 * 1.0 WAITING_FOR_POINT：   硬件已就绪，等待下一个 setpoint（写方：硬件；
 *                           读方：控制器；= 硬件侧常量 kTransferAccepted，
 *                           同一状态两个名字）。
 * 2.0 TRANSFERRING：        控制器写好了 1 个 setpoint（写方：控制器）；硬件
 *                           取走后回答 1.0，如此交替直至所有点传完。
 * 3.0 TRANSFER_DONE：       全部 setpoint 已移交（写方：控制器）；硬件随后切
 *                           到 4.0。
 * 4.0 IN_MOTION：           硬件正在按队列执行轨迹（写方：硬件）。
 * 5.0 DONE：                硬件队列已排空（写方：硬件）；控制器据此做
 *                           goal_hold 检查并回 action result，之后复位为 0.0。
 */
const double TRANSFER_STATE_IDLE = 0.0;
const double TRANSFER_STATE_WAITING_FOR_POINT = 1.0;
const double TRANSFER_STATE_TRANSFERRING = 2.0;
const double TRANSFER_STATE_TRANSFER_DONE = 3.0;
const double TRANSFER_STATE_IN_MOTION = 4.0;
const double TRANSFER_STATE_DONE = 5.0;
const double TRANSFER_STATE_NEW_TRAJECTORY = 6.0;

class AuboPassthroughTrajectoryController : public controller_interface::ControllerInterface
{
public:
  AuboPassthroughTrajectoryController() = default;
  ~AuboPassthroughTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeThreadSafeBox<RealtimeGoalHandlePtr>;

  RealtimeGoalHandleBuffer rt_active_goal_;  ///< 当前活动的 action goal（若有）
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;

  /// 当前正在透传的轨迹：已重排到权威关节顺序，且（需要时）前面补了融合段。
  /// 在接受 goal 时写入。
  realtime_tools::RealtimeThreadSafeBox<trajectory_msgs::msg::JointTrajectory> rt_active_traj_{
    trajectory_msgs::msg::JointTrajectory()
  };
  realtime_tools::RealtimeThreadSafeBox<std::vector<control_msgs::msg::JointTolerance>>
  goal_tolerance_{
    std::vector<control_msgs::msg::JointTolerance>()
  };

  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(std::chrono::milliseconds(50));

  void start_action_server();

  void end_goal();

  // --- goal 校验（非实时，在 goal_received_callback 中调用） ---
  bool check_goal_joints(const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;
  bool check_goal_positions(const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;
  bool check_goal_velocities(const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;
  bool check_goal_accelerations(const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;
  bool check_goal_values_and_timing(
    const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;
  bool check_goal_tolerances(const std::shared_ptr<const FollowJTrajAction::Goal> & goal) const;

  /// 按关节名把轨迹点重排到权威顺序（蓝本：aubo_boot remapJointNames，
  /// joint_trajectory_controller.cpp:73,223）。缺失的速度/加速度补 0
  /// （aubo_boot 对缺省导数量一律补 0）。
  trajectory_msgs::msg::JointTrajectory remapJointNames(
    const trajectory_msgs::msg::JointTrajectory & traj) const;

  /// 从当前关节位置到轨迹首点的 smoothstep 融合段（s = 3t^2 - 2t^3，C1）。
  /// 不需要融合时（最大偏差 <= blend_threshold_rad）返回空 vector。
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>
  blendToFirstPoint(
    const std::vector<double> & current_joints,
    const trajectory_msgs::msg::JointTrajectoryPoint & first_point) const;

  /// goal_hold 检查（实时线程）：逐关节要求 |实际 - 终点| < 位置容差
  /// 且 |速度| < 速度容差。
  bool withinGoalHold(
    const trajectory_msgs::msg::JointTrajectoryPoint & final_point,
    const std::vector<control_msgs::msg::JointTolerance> & tolerances);

  std::optional<RealtimeGoalHandlePtr> get_rt_goal_from_non_rt();
  bool set_rt_goal_from_non_rt(RealtimeGoalHandlePtr & rt_goal);

  rclcpp_action::Server<FollowJTrajAction>::SharedPtr send_trajectory_action_server_;

  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJTrajAction::Goal> goal);

  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  std::shared_ptr<aubo_passthrough_trajectory_controller::ParamListener> param_listener_;
  aubo_passthrough_trajectory_controller::Params params_;

  std::vector<std::string> joint_names_;  ///< 权威关节顺序（params.joints）
  std::vector<std::string> state_interface_types_;
  size_t number_of_joints_{0};

  std::vector<std::string> joint_state_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  joint_velocity_state_interface_;
  // 机器人侧真实目标（SDK jointTagPosJ / jointTagSpeedMoto 经硬件插件换算），
  // 用于 action feedback 的 desired/error。硬依赖：我们的 sim/real 硬件插件
  // 一定导出 aubo_io/tag_pos_*、tag_vel_*，on_activate 找不到直接报错，
  // 不做"可选接口"处理（避免静默退回失真的透传点反馈）。
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  tag_pos_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  tag_vel_state_interface_;

  std::atomic<size_t> current_index_{0};
  std::atomic<bool> trajectory_active_{false};
  // 本标志修复一个抢占死锁 bug：update() 里“current_index_ == 0 且硬件非 IDLE”
  // 的 else 分支会锁存 abort=1.0，本意是抢占/清空上一条传输；但如果硬件正
  // 在应答我们自己发起的传输（回 WAITING_FOR_POINT=1），该 else 会误触发，
  // abort=1 让硬件丢掉首点、状态机卡住造成死锁。因此仅当本次传输不是由我们
  // 发起（transfer_requested_ == false）时才允许锁存 abort。UR 原版用单一组
  // 合条件规避此问题（ur_controllers/src/passthrough_trajectory_controller.cpp:359
  // 附近：`if (current_index_ == 0 && current_transfer_state == TRANSFER_STATE_IDLE)`），
  // 本实现拆分条件后用本标志区分“硬件在忙我们的传输”与“硬件在忙上一条传输”
  // （只有后者需要 abort 锁存）。
  std::atomic<bool> transfer_requested_{false};

  // goal_hold / goal_time 记账（仅实时线程使用）
  int goal_hold_count_{0};
  rclcpp::Time last_goal_check_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_start_time_{0, 0, RCL_ROS_TIME};
  double max_trajectory_time_{0.0};     ///< 最后一个点（融合段平移后）的 time_from_start [s]
  double effective_goal_time_{0.0};     ///< goal.goal_time_tolerance（>0 时）
                                        ///< 否则 params.goal_time [s]

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  scaling_state_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  abort_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  trajectory_size_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  transfer_command_interface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  time_from_start_command_interface_;

  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace aubo_e5_controllers
#endif  // AUBO_E5_CONTROLLERS__AUBO_PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
