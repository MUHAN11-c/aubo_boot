/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_
#define DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <demo_interface/srv/move_to_pose.hpp>
#include <demo_interface/srv/move_to_joints.hpp>
#include <demo_interface/srv/set_robot_io.hpp>

namespace demo_driver
{

/**
 * @brief 三夹爪快换 Demo 节点
 *
 * 与 aubo_moveit_config/config/aubo_i5.srdf 配合使用（规划组 manipulator，末端 tool_tcp）。
 * 通过 /move_to_pose、/move_to_joints、/demo_driver/set_io 与 MoveGroup 笛卡尔路径，
 * 依次对三个夹爪执行：到位 -> 沿 Z 下降 -> 延时 -> 沿 Z 上升 -> 回 home。
 * 需先启动 move_to_pose_server_node、set_robot_io_server_node（可选）。
 */
class MoveitArclineDemo : public rclcpp::Node
{
public:
  explicit MoveitArclineDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveitArclineDemo() override = default;

  /**
   * 工厂函数：构造节点并立即创建 MoveGroupInterface。
   * 因 C++ 构造函数执行时对象尚未被 shared_ptr 管理，无法在构造函数内使用 shared_from_this()，
   * 故在 create() 中构造完成后立即调用 initMoveGroup()，实现“构造阶段”完成 MoveGroup 创建。
   */
  static std::shared_ptr<MoveitArclineDemo> create(const rclcpp::NodeOptions& options);

  /** 等待所需服务就绪，超时返回 false */
  bool waitForServices(std::chrono::seconds timeout);

  /** 执行完整 demo 流程，任一步失败返回 false */
  bool run();

private:
  /** 在对象已被 shared_ptr 持有后调用，创建 MoveGroupInterface（仅由 create() 调用） */
  void initMoveGroup();
  /** 调用 /move_to_joints */
  bool moveToJoints(const std::array<double, 6>& joint_positions_rad,
                   float velocity_factor = 0.5f,
                   float acceleration_factor = 0.5f);

  /** 调用 /move_to_pose */
  bool moveToPose(double x, double y, double z,
                  double qx, double qy, double qz, double qw,
                  bool use_joints = false,
                  float velocity_factor = 0.5f,
                  float acceleration_factor = 0.5f);

  /** 移动到 home（/move_to_joints + 预设 home 关节角） */
  bool moveToHome();

  /** 移动到圆弧起点 (0.4, 0, 0.45)（/move_to_pose） */
  bool moveToArcStart();

  /** 沿当前末端位姿 Z 轴做笛卡尔直线移动，z_offset 为正抬升、为负下降，单位 m */
  bool runArcPath(double z_offset = 0.2);

  /** 沿指定轴（'x'/'y'/'z'）做笛卡尔直线移动，offset 为位移量（米），正负表示方向 */
  bool runArcPath(char axis, double offset);

  /**
   * 设置数字输出（调用 /demo_driver/set_io，io_type=digital_output）
   * @param io_index 数字输出引脚索引（>=0）
   * @param high true=高电平(1)，false=低电平(0)
   * @return 成功返回 true
   */
  bool setDigitalOutput(int32_t io_index, bool high);

  rclcpp::Client<demo_interface::srv::MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp::Client<demo_interface::srv::MoveToJoints>::SharedPtr move_to_joints_client_;
  rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr set_robot_io_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  static const std::array<double, 6> kHomeJointsRad1;
  static const std::string kMoveToPoseService;
  static const std::string kMoveToJointsService;
  static const std::string kSetRobotIOService;
  static constexpr int kServiceWaitSeconds = 10;
  static constexpr int kCallTimeoutSeconds = 60; 
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEIT_ARCLINE_DEMO_H_
