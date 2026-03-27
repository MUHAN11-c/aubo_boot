/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_MOVEIT_GRIPPER_IO_BASE_H_
#define DEMO_DRIVER_MOVEIT_GRIPPER_IO_BASE_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <demo_interface/srv/set_robot_io.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace demo_driver
{

/** 笛卡尔路径单段：沿 axis 轴移动 offset 米 */
struct CartesianSegment
{
  char axis;    /**< 轴 'x'/'y'/'z' */
  double offset;/**< 偏移量 (m) */
};

/**
 * @brief 夹爪 IO 控制基类
 *
 * 提供 MoveIt2 关节、位姿、笛卡尔路径运动及 Aubo 夹爪 IO 控制（开关夹爪）。
 */
class MoveitGripperIoBase : public rclcpp::Node
{
public:
  explicit MoveitGripperIoBase(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MoveitGripperIoBase() override = default;

  /** 创建节点并初始化 MoveGroup，需 shared_ptr 时使用 */
  static std::shared_ptr<MoveitGripperIoBase> create(const rclcpp::NodeOptions& options);

  /** 等待依赖服务就绪，超时返回 false */
  bool waitForServices(std::chrono::seconds timeout);
  /** 默认流程，子类可重写 */
  virtual bool run();

protected:
  /** 初始化 MoveGroupInterface */
  void initMoveGroup();

  /** 关节空间运动：6 轴关节角 (rad)，速度/加速度因子 0~1 */
  bool moveToJoints(const std::array<double, 6>& joint_positions_rad, float velocity_factor = 0.5f,
                    float acceleration_factor = 0.5f);
  /** 位姿空间运动：位置 (x,y,z)、四元数 (qx,qy,qz,qw) */
  bool moveToPose(double x, double y, double z, double qx, double qy, double qz, double qw, bool use_joints = false,
                  float velocity_factor = 0.5f, float acceleration_factor = 0.5f);

  /** 移动到命名目标 camera_pose，velocity_factor/acceleration_factor 控制速度与加速度 [0~1] */
  bool moveToHome(float velocity_factor = 0.5f, float acceleration_factor = 0.5f);
  /** 移动到 arc 起点位姿 */
  bool moveToArcStart();
  /**
   * 沿 Z 轴笛卡尔直线移动 z_offset 米。
   * @param acceleration_factor 加速度缩放 [0~1]；<0 时与 velocity_factor 相同（与 publish_grasps_client_worker 一致）
   */
  bool runArcPath(double z_offset = 0.2, float velocity_factor = 0.5f, float acceleration_factor = -1.f);
  /** 沿指定轴笛卡尔直线移动（多段路点一次 computeCartesianPath + 一次 execute） */
  bool runArcPath(char axis, double offset, float velocity_factor = 0.5f, float acceleration_factor = -1.f);
  /** 多段笛卡尔路径一条轨迹一次执行 */
  bool runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor = 0.5f,
                          float acceleration_factor = -1.f);
  /** 设置夹爪 IO：io_index 为 pin 号，high 为电平（开关夹爪/锁紧等） */
  bool setGripperIo(int32_t io_index, bool high);

  /** 获取当前关节状态并打印为 moveToJoints 可用格式，供继承类调用 */
  bool logCurrentState();

protected:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  static constexpr double kZMinLimit = 0.2;  /**< Z 轴安全下限 (m)，末端 z 坐标不能低于此值 */
  static const int32_t kQuickSwapIoIndex;    /**< 快换 IO 默认 pin：7 */
  static const int32_t kGripperIoIndex;      /**< 夹爪 IO 默认 pin：6 */

private:
  rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr aubo_set_io_client_;

  static const std::array<double, 6> kHomeJointsRad1;  /**< 回零关节角 (rad) */
  static const std::string kAuboSetIOService;          /**< Aubo SetIO 服务名 */
  static constexpr int kCallTimeoutSeconds = 60;       /**< 服务调用超时 (s) */
};

}  // namespace demo_driver

#endif  // DEMO_DRIVER_MOVEIT_GRIPPER_IO_BASE_H_
