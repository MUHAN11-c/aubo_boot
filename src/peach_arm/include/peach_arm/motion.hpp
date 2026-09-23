// 功能：MoveIt 运动接口（唯一实现，节点直接构造）。TF 查询 + 规划/执行；
// 真实下发前必须过注入的安全门回调（TRANSIT 级底座：Active ∧ robotReady，
// 见 cycle_support.hpp；CONTACT/TOOL 级在阶段函数与 GraspTask 门加查）。
#ifndef PEACH_MANIPULATION__MOTION_HPP_
#define PEACH_MANIPULATION__MOTION_HPP_

#include <Eigen/Geometry>
#include <atomic>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_ros/buffer.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace moveit::core
{
class RobotState;
}  // namespace moveit::core

namespace moveit::planning_interface
{
class MoveGroupInterface;
}  // namespace moveit::planning_interface

namespace peach_arm
{

class RetireBucket;  // execution_guard.hpp（src/ 私有头；指针成员避免安装头依赖）

// MoveIt 运动接口的运行配置（默认值以 config/peach_arm.yaml 为权威源）。
struct MoveItMotionConfig
{
  std::string base_frame;   ///< 模型基座系（规划参考系）。
  std::string tip_frame;    ///< 规划/IK 末端连杆（MoveIt 组 tip_link，当前为 tcp）。
  std::string camera_frame; ///< 相机光学系（planOrMoveCamera 的目标系）。
  std::string tool_frame;   ///< 工具系（tip←tool 变换查询用）。
  std::string pilz_pipeline;    ///< 主规划管线（PTP/LIN 工业插值）。
  std::string fallback_pipeline;  ///< Pilz 失败后的兜底管线（OMPL）。
  // 自由空间转移速度档（观察视点、拍照位姿往返）。
  double transit_velocity_scaling{0.10};        ///< 转移段速度缩放（真机初始验证不得高于 0.1）。
  double transit_acceleration_scaling{0.10};    ///< 转移段加速度缩放（同上档位约束）。
  // 观察/相机转移护栏。结构默认值仅兜底；现行值以部署 yaml
  // config/peach_arm.yaml 为单一事实源，节点构造时整体覆盖。
  double transit_max_duration_s{0.0};           ///< goToPhotoPose 新规划时长上限 [s]；0=不按时长拒发。
  double transit_max_total_joint_travel_rad{6.0};   ///< goToPhotoPose 新规划累计行程上限 [rad]（原路返程不受限）。
  double transit_max_single_joint_travel_rad{2.5};  ///< goToPhotoPose 新规划单轴行程上限 [rad]。
  double observe_planning_time_s{1.0};          ///< 观察短 LIN 单次规划时间上限 [s]。
  int observe_planning_attempts{1};             ///< 观察短 LIN 规划尝试次数。
  double observe_max_duration_s{0.0};           ///< 观察转移轨迹时长上限 [s]；0=不查。
  double observe_max_total_joint_travel_rad{2.5};  ///< 观察转移累计行程上限 [rad]。
  double observe_max_single_joint_travel_rad{1.5}; ///< 观察转移单轴行程上限 [rad]。
  double photo_planning_time_s{3.0};            ///< 回拍照位 OMPL 回退规划时间上限 [s]。
  double photo_ptp_planning_time_s{0.5};        ///< 回拍照位 Pilz PTP 预算 [s]（确定性插值不必用满）。
  double default_planning_time_s{1.5};          ///< 其余规划默认时间上限 [s]。
  int default_planning_attempts{1};             ///< 其余规划默认尝试次数。
  // goToPhotoPose 成功出口复核：当前关节须在命名状态且静止。
  double photo_pose_joint_tolerance_rad{0.05};    ///< 出口每轴 |Δq| 容差 [rad]（execute=false 仍核）。
  double photo_pose_max_joint_vel_rad_s{0.05};    ///< 出口任一轴 |qdot| 上限 [rad/s]（未静止即 mismatch）。
  // 执行有界等待：move_group TEM 异常（2026-09-23 travel_max 的 stop 事件
  // 风暴）会让同步 execute 永久阻塞且不理取消，一次即永久卡死动作通道。
  double execute_timeout_s{90.0};  ///< 单条轨迹执行有界等待 [s]；超时 stop() 并判失败。
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
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    tf2_ros::Buffer * tf_buffer,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock,
    MoveItMotionConfig config,
    std::function<bool(std::string &)> safety_gate,
    std::function<void(const std::string &)> safety_block_hook,
    std::function<bool()> cancel_probe = {});
  ~MoveItMotionInterface();  // 弃等线程 dispose（完结的 join，滞留的 detach）

  // 停止 move_group 当前轨迹执行（MGI::stop 发 move_group 级停止事件，
  // 与哪个接口实例发起执行无关；供 MTC 等不暴露 stop 的路径共用）。
  void stopExecution();

  // 有界执行在途（含弃等滞留）时为 false；节点空闲改参重建 motion_ 前必须
  // 查它，避免销毁正被 MoveTo/周期线程使用的实例（审查 P1-1）。
  bool executionIdle() const;

  // 查询 target<-source 的最新变换；失败返回 nullopt（实现记录原因日志）。
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target, const std::string & source);

  // 以 tip 连杆目标位姿规划（execute=true 时并执行）。planner_id 为实现认识
  // 的规划器标识（PTP/LIN）。观察短移 allow_fallback=false：LIN 失败换候选，
  // 不改 PTP。失败返回 false，不抛异常；execute=true 时下发前复核安全门。
  bool planOrMoveTip(
    const Eigen::Isometry3d & tip_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback);

  // 以相机目标位姿规划/执行（内部经 TF 换算到 tip）。
  bool planOrMoveCamera(
    const Eigen::Isometry3d & camera_pose, const std::string & planner_id,
    bool execute, const std::string & label, bool allow_fallback);

  // 移动到 SRDF 命名状态（拍照位姿）：有「拍照位→预抓取」接近轨迹且当前
  // 关节在其终点时，原路返程（不新规划 PTP、不过 transit 6 rad——返程行程
  // 等于已过门的接近）。否则先 Pilz PTP，失败回退 OMPL。execute=false 时
  // 仅规划，仍核当前关节。真实下发前必须复核硬件安全门。成功出口 atNamedTarget。
  bool goToPhotoPose(
    const std::string & named_target, bool execute, std::string & message);

  // 记录刚执行成功、且从拍照位出发的接近轨迹，供下次回拍照位原路返程。
  void rememberPhotoApproach(trajectory_msgs::msg::JointTrajectory trajectory);
  void clearPhotoApproach();

private:
  bool atNamedTarget(
    const std::string & named_target, std::string & message);
  bool jointsMatchTrajectoryPoint(
    const moveit::core::RobotState & state,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const;
  bool namedTargetMatchesPoint(
    const std::string & named_target,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions) const;
  bool reverseLastApproachToPhoto(
    const std::string & named_target, std::string & message);
  // 有界执行：见 src/execution_guard.hpp（弃等线程移交 retiring_，闭包只
  // 持共享 MGI 与 Plan 拷贝，绝不引用 this）。
  moveit::core::MoveItErrorCode boundedExecute(
    moveit::planning_interface::MoveGroupInterface::Plan & plan);
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_shared_;
  moveit::planning_interface::MoveGroupInterface * move_group_;
  tf2_ros::Buffer * tf_buffer_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  MoveItMotionConfig config_;
  std::function<bool(std::string &)> safety_gate_;
  std::function<void(const std::string &)> safety_block_hook_;
  std::function<bool()> cancel_probe_;
  std::unique_ptr<RetireBucket> retiring_;  // 弃等线程桶（execution_guard.hpp，src/ 私有头）
  std::atomic_int executing_{0};  // 有界执行在途计数（0=可安全重建 motion_）
  std::optional<trajectory_msgs::msg::JointTrajectory> last_photo_approach_;
};

}  // namespace peach_arm

#endif  // PEACH_MANIPULATION__MOTION_HPP_
