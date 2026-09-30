// RobotController — 组合模式封装 MoveIt 运动 + AUBO IO。
// 源自 aubo_boot demo_driver/robot_controller（2026-09-30 移植）：
//  - IO 从 ivg SetRobotIO 改为 aubo_msgs/SetIO（/aubo_io_controller/set_io，
//    板载用户 DO，FUN_SET_ROBOT_BOARD_USER_DO）
//  - 增加 io_simulated_ 仿真旁路（mock 无 IO 控制器时 setGripper 直接成功）
//  - 规划组默认 manipulator_e5（本仓 SRDF 组名）
#ifndef IVG_DEMO_SERVICES__ROBOT_CONTROLLER_HPP_
#define IVG_DEMO_SERVICES__ROBOT_CONTROLLER_HPP_

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ivg_demo_services/motion_utils.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace ivg_demo_services
{

/**
 * 运动薄层：封装 MoveGroupInterface 常用运动 + 夹爪/快换 IO。
 * Worker 节点持有成员并调用，两阶段初始化（构造后必须 init()）。
 * IO 语义统一：setGripper(pin, open=true) 打开夹爪。
 */
class RobotController
{
public:
  explicit RobotController(
    rclcpp::Node * owner,
    const std::string & planning_group = "manipulator_e5");

  /// 两阶段初始化：构造后必须调用 init() 才能使用运动/IO 功能
  bool init();

  // ── 运动 ──
  bool moveToHome(float vel = 0.5f, float acc = 0.5f);
  bool moveToJoints(const std::array<double, 6> & joints, float vel, float acc);
  bool moveToPose(const geometry_msgs::msg::Pose & target, float vel, float acc);
  bool moveToPosition(double x, double y, double z, float vel, float acc);
  bool moveCartesianZ(double offset_m, float vel, float acc);
  bool moveCartesianPath(const std::vector<CartesianSegment> & segments, float vel, float acc);
  bool moveCartesianStraight(const geometry_msgs::msg::Pose & target, float vel, float acc);
  /// 批量笛卡尔路径规划+执行（fraction >= 0.95 才执行）
  bool executeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, float vel, float acc);

  // ── IO（板载用户 DO，经 /aubo_io_controller/set_io）──
  bool setGripper(int pin, bool open);   // open=true 打开
  bool setQuickSwap(int pin, bool lock); // lock=true 锁紧

  // ── 查询 ──
  geometry_msgs::msg::Pose getCurrentPose();
  std::vector<double> getCurrentJoints();
  geometry_msgs::msg::Pose jointsToPose(const std::array<double, 6> & joints);
  std::string getEndEffectorLink() const;
  void setEndEffectorLink(const std::string & link);  // ── 配置 ──
  void setVelocityScaling(float v);
  void setAccelerationScaling(float a);
  void setEefStep(double step) {eef_step_ = step;}
  void setZMinLimit(double limit) {z_min_limit_ = limit;}
  void setCartRetryParams(int retries, double wait_sec)
  {
    max_retries_ = retries;
    retry_wait_sec_ = wait_sec;
  }
  /// 仿真旁路：true 时 setGripper 直接成功（mock 无 IO 控制器）
  void setIoSimulated(bool simulated) {io_simulated_ = simulated;}
  void setHomeTarget(const std::string & target) {home_target_ = target;}
  /// 规划管线与规划器（本仓 moveit 默认 pilz，Pilz 必须显式 planner_id PTP/LIN/CIRC）
  void setPlanningPipeline(const std::string & pipeline, const std::string & planner_id)
  {
    planning_pipeline_ = pipeline;
    planner_id_ = planner_id;
  }

  moveit::planning_interface::MoveGroupInterface & moveGroup() {return *move_group_;}

private:
  /// plan/move 前统一套用规划管线与规划器（pilz 需显式 planner_id）
  void applyPlannerSelection();
  geometry_msgs::msg::Pose currentPoseInternal();

  rclcpp::Node * node_;
  std::string planning_group_;
  std::string home_target_{"camera_pose"};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr io_client_;
  std::string eef_link_;
  double eef_step_{0.01};
  double z_min_limit_{0.05};
  int max_retries_{3};
  double retry_wait_sec_{0.5};
  bool io_simulated_{false};
  std::string planning_pipeline_{"pilz_industrial_motion_planner"};
  std::string planner_id_{"PTP"};
};

}  // namespace ivg_demo_services

#endif  // IVG_DEMO_SERVICES__ROBOT_CONTROLLER_HPP_
