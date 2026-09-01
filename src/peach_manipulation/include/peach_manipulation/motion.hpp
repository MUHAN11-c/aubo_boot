// 功能：MoveIt 运动接口（唯一实现，节点直接构造）。TF 查询 + 规划/执行；
// 真实下发前必须过注入的安全门回调（TRANSIT 级底座：Active ∧ robotReady，
// 见 execution_authority.hpp；CONTACT/TOOL 级在阶段函数与 GraspTask 门加查）。
#ifndef PEACH_MANIPULATION__MOTION_HPP_
#define PEACH_MANIPULATION__MOTION_HPP_

#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>

#include <functional>
#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_manipulation
{

// MoveIt 运动接口的运行配置（默认值以 config/peach_manipulation.yaml 为权威源）。
struct MoveItMotionConfig
{
  std::string base_frame;
  std::string tip_frame;  // 规划/IK 末端连杆（MoveIt 组 tip_link，当前为 tcp）
  std::string camera_frame;
  std::string tool_frame;
  std::string pilz_pipeline;
  std::string fallback_pipeline;
  // 自由空间转移速度档（观察视点、拍照位姿往返）。
  double transit_velocity_scaling{0.10};
  double transit_acceleration_scaling{0.10};
  // 观察/相机转移轨迹护栏（默认与 config/peach_manipulation.yaml 对齐）。
  double transit_max_duration_s{0.0};
  double transit_max_total_joint_travel_rad{6.0};
  double transit_max_single_joint_travel_rad{2.5};
  double observe_planning_time_s{1.0};
  int observe_planning_attempts{1};
  double observe_max_duration_s{0.0};
  double observe_max_total_joint_travel_rad{2.5};
  double observe_max_single_joint_travel_rad{1.5};
  double photo_planning_time_s{3.0};
  double default_planning_time_s{1.5};
  int default_planning_attempts{1};
  // 工具姿态保持门（度）：PTP 回退段加 OrientationConstraint，tip 姿态相对
  // 目标姿态的偏差不得超过该值（与对轴门同源，config 的 mtc_approach_max_align_deg）。
  // LIN/CIRC 直线插值天然保姿态；Pilz PTP 关节空间插值忽略约束（无害），
  // 约束对 OMPL 采样段生效。
  double orientation_gate_deg{20.0};
};

// MoveIt 运动接口：tip/camera 位姿规划执行、TF 查询、拍照位往返。
// 生命周期：节点持有（unique_ptr<MoveItMotionInterface>）；本对象不拥有
//   MoveGroup/TF Buffer，仅引用节点持有的实例，析构先于它们发生由节点保证。
// 线程安全：与节点既有同步模型一致（周期运行独占调用），内部不新增线程。
// 安全门经回调注入（I5 不得旁路）：所有 execute 路径下发前必须复核；
// safety_block_hook 在执行被安全门拦下时由节点投影状态（可为空）。
class MoveItMotionInterface
{
public:
  MoveItMotionInterface(
    moveit::planning_interface::MoveGroupInterface * move_group,
    tf2_ros::Buffer * tf_buffer,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    MoveItMotionConfig config,
    std::function<bool(std::string &)> safety_gate,
    std::function<void(const std::string &)> safety_block_hook);

  // 查询 target<-source 的最新变换；失败返回 nullopt（实现记录原因日志）。
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source);

  // 以 tip 连杆目标位姿规划（execute=true 时并执行）。planner_id 为实现认识
  // 的规划器标识（PTP/LIN）；allow_fallback 允许 LIN 失败回退 Pilz PTP。
  // 失败返回 false，不抛异常；execute=true 时下发前复核安全门。
  bool planOrMoveTip(
    const Eigen::Isometry3d & tip_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback);

  // 以相机目标位姿规划/执行（内部经 TF 换算到 tip）。
  bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback);

  // 移动到 SRDF 命名状态（拍照位姿）：先点对点管线规划，失败回退自由空间；
  // execute=false 时仅规划。真实下发前必须复核硬件安全门（I5）。
  // message 始终写入面向操作员的结果描述（成功/失败原因）。
  bool goToPhotoPose(
    const std::string & named_target, bool execute, std::string & message);

private:
  moveit::planning_interface::MoveGroupInterface * move_group_;
  tf2_ros::Buffer * tf_buffer_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  MoveItMotionConfig config_;
  std::function<bool(std::string &)> safety_gate_;
  std::function<void(const std::string &)> safety_block_hook_;
};

}  // namespace peach_manipulation

#endif  // PEACH_MANIPULATION__MOTION_HPP_
