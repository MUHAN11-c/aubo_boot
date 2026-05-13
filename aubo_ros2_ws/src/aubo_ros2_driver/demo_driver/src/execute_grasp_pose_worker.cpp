/**
 * @file execute_grasp_pose_worker.cpp
 * @brief 执行抓取位姿 Worker（服务驱动模式）：支持视觉估计和参数常量两种抓取模式
 */

#include "demo_driver/execute_grasp_pose_worker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <thread>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>

// 引用 interface 包的服务定义
#include <ivg_interfaces/srv/estimate_pose.hpp>

#define CHECK(expr)                                                                                                       \
  do                                                                                                                      \
  {                                                                                                                       \
    if (!(expr))                                                                                                          \
      return false;                                                                                                       \
  } while (0)

namespace demo_driver
{

/** 供 SIGINT 处理访问，在 main 中设置 */
static ExecuteGraspPoseWorker* g_worker_for_signal = nullptr;

static void sigintHandler(int)
{
  if (g_worker_for_signal)
    rclcpp::shutdown();
}

static constexpr int kArcPathMaxRetries = 3;
static constexpr double kArcPathRetryDelaySec = 0.5;
static constexpr double kCartesianEefStep = 0.015;
static constexpr double kCartesianJumpThreshold = 0.0;
static constexpr double kExecuteGraspZMinLimit = 0.19;
static constexpr int kVisionEstimateMaxRetries = 3;
static constexpr double kVisionEstimateRetryDelaySec = 1.0;

// ============================================================================
// 临时调试开关（切回真实机械臂时只需改这里）
// ----------------------------------------------------------------------------
// 1) kUseTemporaryFixedGraspPose = true:
//    忽略视觉服务返回的 grab_position，使用 kTemporary* 常量。
//    切回真实流程：改为 false（将使用 response->grab_position[0]）。
//
// 2) kSkipTemporaryGripperIo = true:
//    仿真临时模式：跳过步骤 4/7 的 setGripperIo（不调用 /aubo_driver/set_io）。
//    适用场景：仿真环境未提供 /aubo_driver/set_io，避免流程因 IO 失败中断。
//    切回真实流程：改为 false（恢复真实夹爪 IO 控制）。
// ============================================================================
static constexpr bool kUseTemporaryFixedGraspPose = false;
static constexpr bool kSkipTemporaryGripperIo = false;

static constexpr double kTemporaryGraspX = 0.396;
static constexpr double kTemporaryGraspY = -0.128;
static constexpr double kTemporaryGraspZ = 0.192;
static constexpr double kTemporaryQx = -0.598711;
static constexpr double kTemporaryQy = -0.800965;
static constexpr double kTemporaryQz = -0.000000;
static constexpr double kTemporaryQw = -0.000000;

static double normalizeYawToShortEquivalent(double yaw_rad)
{
  double a = yaw_rad;
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  if (a > (M_PI / 2.0))
    a -= M_PI;
  else if (a < (-M_PI / 2.0))
    a += M_PI;
  return a;
}

// ============================================================================
// 构造与初始化
// ============================================================================
//
// 参数说明（可通过 ros2 run 或 launch 覆盖；统一前缀 egp_，与 main 中 automatically_declare_parameters_from_overrides 配合）：
//
// | 参数                       | 类型     | 默认           | 含义                                               |
// |----------------------------|----------|----------------|----------------------------------------------------|
// | egp_grasp_position         | double[3]| 见代码         | 抓取位置 (x, y, z) in base_link                    |
// | egp_grasp_orientation      | double[4]| 见代码         | 抓取姿态四元数 (qx,qy,qz,qw)                        |
// | egp_object_id              | string   | default        | 循环抓取默认工件 ID（可选）                         |
// | egp_grasp_z_offset         | double   | 0.01           | gripper_tip→end_effector 沿 z 轴补偿 (m)            |
// | egp_height_above           | double   | 0.1            | 抓取点上方安全高度 (m)                              |
// | egp_joint_velocity_scaling | double   | 0.7            | 速度缩放 [0~1]：moveToHome / 抓取接近 / 抬起 / 放置笛卡尔 共用 |
// | egp_joint_acceleration_scaling| double| 0.3            | 加速度缩放 [0~1]，同上共用                          |
// | egp_gripper_io_index       | int      | 7              | Aubo 夹爪 IO pin 号 (true=打开, 参见 ivg_utils.io.GRIPPER_OPEN) |
// | egp_lift_offset            | double   | 0.2            | 抓取后沿 Z 轴抬起高度 (m)                            |
// | egp_place_offset_y         | double   | -0.2           | 安全位后笛卡尔 y 偏移 (m)                           |
// | egp_place_offset_z         | double   | -0.15          | 安全位后笛卡尔 z 偏移 (m)                           |
// | egp_joint_cartesian_switch_delay_sec | double | 0.2 | 关节↔笛卡尔切换衔接延时 (s)，每处 j↔c 边界各一次；0 关闭 |
// | egp_cartesian_max_points   | int      | 40             | 抓取接近笛卡尔轨迹点数上限                           |
//
// ============================================================================

ExecuteGraspPoseWorker::ExecuteGraspPoseWorker(const rclcpp::NodeOptions& options)
  : MoveitGripperIoBase(options)
{
  // launch 已通过 automatically_declare_parameters_from_overrides 注入 egp_* 时不再 declare，避免重复声明
  if (!has_parameter("egp_grasp_position"))
    declare_parameter("egp_grasp_position", std::vector<double>({ 0.41176897287368774, 0.18364354968070984, 0.2553410828113556 }));
  if (!has_parameter("egp_grasp_orientation"))
    declare_parameter("egp_grasp_orientation", std::vector<double>({ 0.706803, 0.707410, 0.000126, -0.000515 }));
  if (!has_parameter("egp_object_id"))
    declare_parameter("egp_object_id", std::string("default"));
  if (!has_parameter("egp_grasp_z_offset"))
    declare_parameter("egp_grasp_z_offset", 0.01);
  if (!has_parameter("egp_height_above"))
    declare_parameter("egp_height_above", 0.1);
  if (!has_parameter("egp_joint_velocity_scaling"))
    declare_parameter("egp_joint_velocity_scaling", 0.7);
  if (!has_parameter("egp_joint_acceleration_scaling"))
    declare_parameter("egp_joint_acceleration_scaling", 0.3);
  if (!has_parameter("egp_gripper_io_index"))
    declare_parameter("egp_gripper_io_index", kGripperIoIndex);
  if (!has_parameter("egp_lift_offset"))
    declare_parameter("egp_lift_offset", 0.2);
  if (!has_parameter("egp_place_offset_y"))
    declare_parameter("egp_place_offset_y", -0.2);
  if (!has_parameter("egp_place_offset_z"))
    declare_parameter("egp_place_offset_z", -0.15);
  if (!has_parameter("egp_joint_cartesian_switch_delay_sec"))
    declare_parameter("egp_joint_cartesian_switch_delay_sec", 0.2);
  if (!has_parameter("egp_cartesian_max_points"))
    declare_parameter("egp_cartesian_max_points", 40);

  // 读取抓取姿态四元数
  auto quat_param = get_parameter("egp_grasp_orientation").as_double_array();
  double qx_orig = 0.706803;
  double qy_orig = 0.707410;
  double qz_orig = 0.000126;
  double qw_orig = -0.000515;
  if (quat_param.size() >= 4)
  {
    qx_orig = quat_param[0];
    qy_orig = quat_param[1];
    qz_orig = quat_param[2];
    qw_orig = quat_param[3];
  }
  
  // 从四元数中提取Z轴旋转分量
  Eigen::Quaterniond q_orig(qw_orig, qx_orig, qy_orig, qz_orig);
  q_orig.normalize();
  Eigen::Matrix3d rot_matrix = q_orig.toRotationMatrix();
  grasp_z_rotation_ = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
  grasp_orientation_.x = q_orig.x();
  grasp_orientation_.y = q_orig.y();
  grasp_orientation_.z = q_orig.z();
  grasp_orientation_.w = q_orig.w();

  grasp_z_offset_ = get_parameter("egp_grasp_z_offset").as_double();
  height_above_ = get_parameter("egp_height_above").as_double();
  joint_velocity_scaling_ = get_parameter("egp_joint_velocity_scaling").as_double();
  joint_acceleration_scaling_ = get_parameter("egp_joint_acceleration_scaling").as_double();
  gripper_io_index_ = get_parameter("egp_gripper_io_index").as_int();
  lift_offset_ = get_parameter("egp_lift_offset").as_double();
  place_offset_y_ = get_parameter("egp_place_offset_y").as_double();
  place_offset_z_ = get_parameter("egp_place_offset_z").as_double();
  joint_cartesian_switch_delay_sec_ = std::max(0.0, get_parameter("egp_joint_cartesian_switch_delay_sec").as_double());
  {
    int val = static_cast<int>(get_parameter("egp_cartesian_max_points").as_int());
    cartesian_max_points_ = (val < 1) ? 1 : val;
  }

  grasp_position_ = { 0.41176897287368774, 0.18364354968070984, 0.2553410828113556 };
  {
    auto p = get_parameter("egp_grasp_position").as_double_array();
    for (size_t i = 0; i < 3 && i < p.size(); ++i)
      grasp_position_[i] = p[i];
  }

  RCLCPP_INFO(get_logger(),
              "ExecuteGraspPoseWorker 初始化完成：抓取位置=[%.3f, %.3f, %.3f], Z轴旋转=%.4f rad (%.2f°)",
              grasp_position_[0], grasp_position_[1], grasp_position_[2], grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);

  // 创建服务（放入独立 callback group，避免阻塞 MoveIt current_state_monitor 默认组）
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 创建服务
  execute_single_grasp_service_ = create_service<ivg_interfaces::srv::ExecuteGraspPose>(
      "/execute_single_grasp",
      std::bind(&ExecuteGraspPoseWorker::handleExecuteSingleGrasp, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_cb_group_);

  loop_grasp_control_service_ = create_service<std_srvs::srv::SetBool>(
      "/loop_grasp_control",
      std::bind(&ExecuteGraspPoseWorker::handleLoopGraspControl, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_cb_group_);

  // 视觉 client 使用 Reentrant 回调组，避免服务回调内 wait_for 时与默认互斥组死锁；executor.add_node 会注册本组，勿重复 add_callback_group
  estimate_pose_client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  estimate_pose_client_ = create_client<ivg_interfaces::srv::EstimatePose>(
      "/estimate_pose", rmw_qos_profile_services_default, estimate_pose_client_cb_group_);

  RCLCPP_INFO(get_logger(), "服务已创建: /execute_single_grasp, /loop_grasp_control");
}

std::shared_ptr<ExecuteGraspPoseWorker> ExecuteGraspPoseWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<ExecuteGraspPoseWorker>(options);
  node->initMoveGroup();
  return node;
}

ExecuteGraspPoseWorker::~ExecuteGraspPoseWorker()
{
  // 请求优雅停止并在析构阶段等待线程退出，避免后台线程悬挂。
  stopLoopGrasp();
  if (loop_thread_.joinable())
  {
    loop_thread_.join();
  }
}

bool ExecuteGraspPoseWorker::sleepJointCartesianSwitchDelay(const char* where)
{
  if (joint_cartesian_switch_delay_sec_ <= 0.0)
    return true;
  RCLCPP_INFO(get_logger(), "%s: 关节↔笛卡尔切换延时 %.3f s", where, joint_cartesian_switch_delay_sec_);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(joint_cartesian_switch_delay_sec_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return rclcpp::ok();
}

// ============================================================================
// 变换与姿态计算
// ============================================================================

Eigen::Matrix4d ExecuteGraspPoseWorker::buildGraspToEndEffectorTransform()
{
  // 不再使用，保留接口兼容性
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  return T;
}

geometry_msgs::msg::Pose ExecuteGraspPoseWorker::applyTransformationToPose(
    const geometry_msgs::msg::Pose& pose, const Eigen::Matrix4d& transform_local)
{
  (void)transform_local;  // 不再使用复杂变换
  
  // 简化：只在抓取位姿上方偏移 grasp_z_offset
  geometry_msgs::msg::Pose out;
  out.position.x = pose.position.x;
  out.position.y = pose.position.y;
  out.position.z = pose.position.z + grasp_z_offset_;  // 在上方偏移
  out.orientation = pose.orientation;  // 姿态保持不变
  
  return out;
}

geometry_msgs::msg::Quaternion ExecuteGraspPoseWorker::quatSameHemisphere(
    const geometry_msgs::msg::Quaternion& q_ref, const geometry_msgs::msg::Quaternion& q)
{
  double dot = q_ref.x * q.x + q_ref.y * q.y + q_ref.z * q.z + q_ref.w * q.w;
  geometry_msgs::msg::Quaternion out;
  if (dot >= 0)
  {
    out = q;
  }
  else
  {
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;
    out.w = -q.w;
  }
  return out;
}

geometry_msgs::msg::Quaternion ExecuteGraspPoseWorker::createOrientationFromZRotation(double z_rotation_rad)
{
  Eigen::AngleAxisd rotation_z(z_rotation_rad, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q(rotation_z);

  geometry_msgs::msg::Quaternion quat;
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();
  return quat;
}

// ============================================================================
// 抓取接近
// ============================================================================

bool ExecuteGraspPoseWorker::runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above,
                                               float vel, float acc)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] MoveGroup 未初始化");
    return false;
  }

  const std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty())
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] 无法获取末端 link");
    return false;
  }

  double gx = pose_ee.position.x;
  double gy = pose_ee.position.y;
  double gz = pose_ee.position.z;
  double z_above = gz + height_above;

  if (gz < kExecuteGraspZMinLimit)
  {
    RCLCPP_WARN(get_logger(), "[runGraspApproach] Z 轴安全限位: 抓取点 z=%.3f < %.2f m，覆盖为 %.2f m 执行", gz,
                kExecuteGraspZMinLimit, kExecuteGraspZMinLimit);
    gz = kExecuteGraspZMinLimit;
    z_above = gz + height_above;
  }

  // 与 publish_grasps_client_worker::runGraspApproach 一致：每轮 preparePlanningState、刷新当前位姿、
  // 路点 当前→X→Y→抬升→姿态→下降，一条笛卡尔轨迹一次 execute。
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);

    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose(eef_link).pose;

    if (attempt == 1)
    {
      RCLCPP_INFO(get_logger(), "[runGraspApproach] 当前位姿 Position: [%.6f, %.6f, %.6f]",
                  current_pose.position.x, current_pose.position.y, current_pose.position.z);
      RCLCPP_INFO(get_logger(), "[runGraspApproach] 当前位姿 Orientation (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
                  current_pose.orientation.x, current_pose.orientation.y,
                  current_pose.orientation.z, current_pose.orientation.w);
    }

    const geometry_msgs::msg::Quaternion grasp_ori = pose_ee.orientation;
    const geometry_msgs::msg::Quaternion grasp_ori_short =
        quatSameHemisphere(current_pose.orientation, grasp_ori);

    Eigen::Quaterniond q_after_up(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y,
                                    current_pose.orientation.z);
    Eigen::Quaterniond q_goal(grasp_ori_short.w, grasp_ori_short.x, grasp_ori_short.y, grasp_ori_short.z);
    q_goal.normalize();
    q_after_up.normalize();

    Eigen::Quaterniond q_delta = q_after_up.conjugate() * q_goal;
    q_delta.normalize();
    Eigen::Matrix3d delta_rot = q_delta.toRotationMatrix();
    const double delta_yaw = std::atan2(delta_rot(1, 0), delta_rot(0, 0));
    const double delta_yaw_short = normalizeYawToShortEquivalent(delta_yaw);
    Eigen::Quaterniond q_delta_short(Eigen::AngleAxisd(delta_yaw_short, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_target = q_after_up * q_delta_short;
    q_target.normalize();

    if (attempt == 1)
    {
      RCLCPP_INFO(get_logger(), "[runGraspApproach] 路点顺序与 publish_grasps_client_worker 对齐（X→Y→抬升→旋转→下降）");
      RCLCPP_INFO(get_logger(), "  相对 Z 旋转: %.4f rad (%.2f°), 短角: %.4f rad (%.2f°)", delta_yaw,
                  delta_yaw * 180.0 / M_PI, delta_yaw_short, delta_yaw_short * 180.0 / M_PI);
    }

    geometry_msgs::msg::Pose p_x;
    p_x.position.x = gx;
    p_x.position.y = current_pose.position.y;
    p_x.position.z = current_pose.position.z;
    p_x.orientation = current_pose.orientation;

    geometry_msgs::msg::Pose p_y;
    p_y.position.x = gx;
    p_y.position.y = gy;
    p_y.position.z = current_pose.position.z;
    p_y.orientation = current_pose.orientation;

    geometry_msgs::msg::Pose p_up;
    p_up.position.x = gx;
    p_up.position.y = gy;
    p_up.position.z = z_above;
    p_up.orientation = current_pose.orientation;

    geometry_msgs::msg::Pose p_rot;
    p_rot.position.x = gx;
    p_rot.position.y = gy;
    p_rot.position.z = z_above;
    p_rot.orientation.x = q_target.x();
    p_rot.orientation.y = q_target.y();
    p_rot.orientation.z = q_target.z();
    p_rot.orientation.w = q_target.w();

    geometry_msgs::msg::Pose p_down;
    p_down.position.x = gx;
    p_down.position.y = gy;
    p_down.position.z = gz;
    p_down.orientation = p_rot.orientation;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);
    waypoints.push_back(p_x);
    waypoints.push_back(p_y);
    waypoints.push_back(p_up);
    waypoints.push_back(p_rot);
    waypoints.push_back(p_down);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold,
                                                              trajectory);
    const size_t num_points = trajectory.joint_trajectory.points.size();

    RCLCPP_INFO(get_logger(), "[runGraspApproach] 抓取接近笛卡尔: fraction=%.2f%%, 点数=%zu (尝试 %d/%d)",
                fraction * 100.0, num_points, attempt, kArcPathMaxRetries);

    if (fraction < 1.0)
    {
      const int segment_count = static_cast<int>(waypoints.size()) - 1;
      const int seg_idx = std::min(segment_count - 1, static_cast<int>(fraction * segment_count));
      RCLCPP_ERROR(get_logger(),
                   "[runGraspApproach] 笛卡尔未达 100%%, fraction=%.2f, 截断于第%d段；%.3f s 后重试", fraction,
                   seg_idx + 1, kArcPathRetryDelaySec);
      if (attempt < kArcPathMaxRetries)
        std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
      continue;
    }
    if (num_points > static_cast<size_t>(cartesian_max_points_))
    {
      RCLCPP_ERROR(get_logger(), "[runGraspApproach] 轨迹点数过多 (%zu > %d)，%.3f s 后重试", num_points,
                   cartesian_max_points_, kArcPathRetryDelaySec);
      if (attempt < kArcPathMaxRetries)
        std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto exec_ok = move_group_->execute(plan);
    if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "[runGraspApproach] 执行失败，错误码=%d；%.3f s 后重试", exec_ok.val,
                   kArcPathRetryDelaySec);
      if (attempt < kArcPathMaxRetries)
        std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
      continue;
    }

    RCLCPP_INFO(get_logger(), "[runGraspApproach] 抓取接近完成（单次 execute）");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[runGraspApproach] %d 次尝试后仍失败", kArcPathMaxRetries);
  return false;
}

// ============================================================================
// runOneCycle
// ============================================================================

bool ExecuteGraspPoseWorker::runOneCycle()
{
  if (!rclcpp::ok())
    return false;

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "┌────────────────────────────────────────────────────────────┐");
  RCLCPP_INFO(get_logger(), "│                   开始抓取周期 (runOneCycle)              │");
  RCLCPP_INFO(get_logger(), "└────────────────────────────────────────────────────────────┘");

  RCLCPP_INFO(get_logger(), "► 步骤 0/8: 回安全位");
  if (!moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 0 失败：回安全位失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 0 完成");

  if (!rclcpp::ok())
    return false;

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 1/8: 构建抓取位姿");
  geometry_msgs::msg::Pose grasp_pose;
  grasp_pose.position.x = grasp_position_[0];
  grasp_pose.position.y = grasp_position_[1];
  grasp_pose.position.z = grasp_position_[2];
  grasp_pose.orientation = grasp_orientation_;
  RCLCPP_INFO(get_logger(), "  抓取位姿 Position: [%.6f, %.6f, %.6f]",
              grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
  RCLCPP_INFO(get_logger(), "  抓取位姿 Orientation (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
              grasp_pose.orientation.x, grasp_pose.orientation.y, 
              grasp_pose.orientation.z, grasp_pose.orientation.w);
  RCLCPP_INFO(get_logger(), "  Z轴旋转: %.4f rad (%.2f°)", grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);
  RCLCPP_INFO(get_logger(), "✓ 步骤 1 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 2/8: gripper_tip 变换为 end_effector 目标");
  Eigen::Matrix4d T_local = buildGraspToEndEffectorTransform();
  geometry_msgs::msg::Pose pose_ee = applyTransformationToPose(grasp_pose, T_local);
  RCLCPP_INFO(get_logger(), "  目标位姿 Position: [%.6f, %.6f, %.6f]",
              pose_ee.position.x, pose_ee.position.y, pose_ee.position.z);
  RCLCPP_INFO(get_logger(), "  目标位姿 Orientation (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
              pose_ee.orientation.x, pose_ee.orientation.y,
              pose_ee.orientation.z, pose_ee.orientation.w);
  RCLCPP_INFO(get_logger(), "✓ 步骤 2 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 3/9: 抓取前开夹爪 (IO=%d, true=打开, 参见 ivg_utils.io.GRIPPER_OPEN)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过抓取前开夹爪 IO 设置（未调用 /aubo_driver/set_io）。"
                "如需真实机械臂夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
    RCLCPP_INFO(get_logger(), "✓ 步骤 3 完成（已跳过IO）");
  }
  else
  {
    if (!setGripperIo(gripper_io_index_, true))  // true=打开 (ivg_utils.io.GRIPPER_OPEN)
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 3 失败：抓取前开夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 3 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 4/9: 抓取接近 (多段笛卡尔路径)");
  CHECK(sleepJointCartesianSwitchDelay("步骤 4 前（关节→笛卡尔）"));
  if (!runGraspApproach(pose_ee, height_above_, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 4 失败：抓取接近失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 4 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 5/9: 闭夹爪 (IO=%d, 状态=false)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过闭夹爪 IO 设置（未调用 /aubo_driver/set_io）。"
                "如需真实机械臂夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
    RCLCPP_INFO(get_logger(), "✓ 步骤 5 完成（已跳过IO）");
  }
  else
  {
    if (!setGripperIo(gripper_io_index_, false))  // false=闭合 (ivg_utils.io.GRIPPER_CLOSE)
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 5 失败：闭夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 5 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 6/9: 抬起 (z=%.2f m)", lift_offset_);
  if (!runArcPath('z', lift_offset_, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 6 失败：抬起失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 6 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 7/9: 移动到放置位 (安全位 + y/x/z 偏移)");
  CHECK(sleepJointCartesianSwitchDelay("步骤 7 前（笛卡尔→关节）"));
  if (!moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：回安全位失败");
    return false;
  }
  CHECK(sleepJointCartesianSwitchDelay("步骤 7 放置笛卡尔前（关节→笛卡尔）"));
  const std::vector<CartesianSegment> place_segments = {
    { 'y', place_offset_y_ },
    { 'x', -0.2 },
    { 'z', place_offset_z_ },
  };
  RCLCPP_INFO(get_logger(), "  执行偏移：y=%.2f, x=-0.2, z=%.2f", place_offset_y_, place_offset_z_);
  if (!runArcPathSequence(place_segments, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：放置位多段笛卡尔失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 7 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 8/9: 开夹爪 (IO=%d, 状态=true)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过开夹爪 IO 设置（未调用 /aubo_driver/set_io）。"
                "如需真实机械臂夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
    RCLCPP_INFO(get_logger(), "✓ 步骤 8 完成（已跳过IO）");
  }
  else
  {
    if (!setGripperIo(gripper_io_index_, true))  // true=打开 (ivg_utils.io.GRIPPER_OPEN)
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 8 失败：开夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 8 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 9/9: 回安全位");
  CHECK(sleepJointCartesianSwitchDelay("步骤 9 前（笛卡尔→关节）"));
  if (!moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 9 失败：回安全位失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 9 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "┌────────────────────────────────────────────────────────────┐");
  RCLCPP_INFO(get_logger(), "│             ✓ 抓取周期成功完成 (9/9 步骤)                │");
  RCLCPP_INFO(get_logger(), "└────────────────────────────────────────────────────────────┘");
  RCLCPP_INFO(get_logger(), "");

  return true;
}

// ============================================================================
// 主循环
// ============================================================================

// ============================================================================
// 视觉估计与服务回调
// ============================================================================

bool ExecuteGraspPoseWorker::estimatePoseFromVision(const std::string& object_id)
{
  RCLCPP_INFO(get_logger(), "========================================");
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 开始调用视觉位姿估计服务");
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   object_id: %s", object_id.c_str());
  RCLCPP_INFO(get_logger(), "========================================");

  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 等待服务 /estimate_pose 可用...");
  if (!estimate_pose_client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(get_logger(), "[estimatePoseFromVision] ❌ 视觉估计服务 /estimate_pose 不可用（超时5秒）");
    RCLCPP_ERROR(get_logger(), "[estimatePoseFromVision] 请确保 visual_pose_estimation 节点正在运行");
    return false;
  }
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] ✓ 服务 /estimate_pose 已连接");

  auto request = std::make_shared<ivg_interfaces::srv::EstimatePose::Request>();
  request->object_id = object_id;
  // image/color 留空：visual_pose_estimation_python 会 SoftwareTrigger + 临时订阅取图（单线程内需 spin_once，见 ros2_communication._capture_images_on_trigger）
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 发送服务请求...");

  auto future = estimate_pose_client_->async_send_request(request);

  // 等待服务响应（最多30秒）
  // 注意：本节点已在 main 的 MultiThreadedExecutor 中 spin，禁止再调用 spin_until_future_complete
  //（会抛 std::runtime_error: Node has already been added to an executor）
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 等待服务响应（最多30秒）...");
  constexpr std::chrono::seconds k_estimate_pose_timeout{30};
  if (future.wait_for(k_estimate_pose_timeout) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[estimatePoseFromVision] ❌ 调用视觉估计服务超时（30秒）");
    return false;
  }

  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] ✓ 收到服务响应");
  auto response = future.get();
  
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 响应分析：");
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   success_num: %d", response->success_num);
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   grab_position 数量: %zu", response->grab_position.size());
  
  if (response->success_num < 1 || response->grab_position.empty())
  {
    RCLCPP_WARN(get_logger(), "[estimatePoseFromVision] ⚠ 视觉估计未检测到目标");
    RCLCPP_WARN(get_logger(), "[estimatePoseFromVision]   可能原因：");
    RCLCPP_WARN(get_logger(), "[estimatePoseFromVision]   1. 视野中没有目标工件");
    RCLCPP_WARN(get_logger(), "[estimatePoseFromVision]   2. 光照条件不佳");
    RCLCPP_WARN(get_logger(), "[estimatePoseFromVision]   3. 模板匹配失败");
    return false;
  }

  // 使用第一个检测到的抓取位姿
  const auto& grab_pos = response->grab_position[0];
  
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] 检测到目标，提取第1个抓取位姿：");
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   原始位置: (%.6f, %.6f, %.6f)",
              grab_pos.position.x, grab_pos.position.y, grab_pos.position.z);
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   原始四元数: (qx=%.6f, qy=%.6f, qz=%.6f, qw=%.6f)",
              grab_pos.orientation.x, grab_pos.orientation.y, grab_pos.orientation.z, grab_pos.orientation.w);
  
  if (kUseTemporaryFixedGraspPose)
  {
    // 临时方案（仿真环境）：使用固定抓取位姿常量。
    grasp_position_[0] = kTemporaryGraspX;
    grasp_position_[1] = kTemporaryGraspY;
    grasp_position_[2] = kTemporaryGraspZ;
    RCLCPP_WARN(get_logger(),
                "[estimatePoseFromVision] 当前为临时模式：已忽略视觉返回抓取位姿，使用固定抓取位姿常量。"
                "如需切回真实机械臂，请将 kUseTemporaryFixedGraspPose 改为 false。");
  }
  else
  {
    // 真实机械臂流程：直接使用视觉服务返回的抓取位姿。
    grasp_position_[0] = grab_pos.position.x;
    grasp_position_[1] = grab_pos.position.y;
    grasp_position_[2] = grab_pos.position.z;
  }

  const double qx = kUseTemporaryFixedGraspPose ? kTemporaryQx : grab_pos.orientation.x;
  const double qy = kUseTemporaryFixedGraspPose ? kTemporaryQy : grab_pos.orientation.y;
  const double qz = kUseTemporaryFixedGraspPose ? kTemporaryQz : grab_pos.orientation.z;
  const double qw = kUseTemporaryFixedGraspPose ? kTemporaryQw : grab_pos.orientation.w;

  // 从最终选定四元数中提取 Z 轴旋转
  Eigen::Quaterniond q_visual(qw, qx, qy, qz);
  q_visual.normalize();
  Eigen::Matrix3d rot_matrix = q_visual.toRotationMatrix();
  double old_z_rotation = grasp_z_rotation_;
  grasp_z_rotation_ = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));
  grasp_orientation_.x = q_visual.x();
  grasp_orientation_.y = q_visual.y();
  grasp_orientation_.z = q_visual.z();
  grasp_orientation_.w = q_visual.w();

  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision] ✓ 视觉估计成功，已更新抓取位姿：");
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   位置: [%.3f, %.3f, %.3f] m",
              grasp_position_[0], grasp_position_[1], grasp_position_[2]);
  RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   Z轴旋转: %.4f rad (%.2f°)",
              grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);
  if (old_z_rotation != grasp_z_rotation_)
  {
    RCLCPP_INFO(get_logger(), "[estimatePoseFromVision]   Z轴旋转变化: %.4f rad (%.2f°) -> %.4f rad (%.2f°)",
                old_z_rotation, old_z_rotation * 180.0 / M_PI,
                grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);
  }
  RCLCPP_INFO(get_logger(), "========================================");

  return true;
}

void ExecuteGraspPoseWorker::handleExecuteSingleGrasp(
    const std::shared_ptr<ivg_interfaces::srv::ExecuteGraspPose::Request> request,
    std::shared_ptr<ivg_interfaces::srv::ExecuteGraspPose::Response> response)
{
  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║        [单次抓取服务] 收到新的抓取请求                    ║");
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] 请求参数：");
  RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   object_id: %s", request->object_id.c_str());
  RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   use_visual_estimation: %s", 
              request->use_visual_estimation ? "true（使用视觉估计）" : "false（使用参数常量）");

  if (loop_grasp_running_.load() || cycle_in_progress_.load())
  {
    response->success = false;
    response->message = "循环抓取执行中，拒绝单次抓取请求（避免并发运动）";
    RCLCPP_WARN(get_logger(), "[handleExecuteSingleGrasp] ⚠ %s", response->message.c_str());
    RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
    return;
  }

  // 如果启用视觉估计，先调用视觉服务
  if (request->use_visual_estimation)
  {
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] 步骤 1/2: 调用视觉位姿估计服务...");
    bool vision_success = false;
    for (int attempt = 1; attempt <= kVisionEstimateMaxRetries; ++attempt)
    {
      RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   视觉估计尝试 %d/%d",
                  attempt, kVisionEstimateMaxRetries);
      if (estimatePoseFromVision(request->object_id))
      {
        vision_success = true;
        break;
      }

      if (attempt < kVisionEstimateMaxRetries)
      {
        RCLCPP_WARN(get_logger(),
                    "[handleExecuteSingleGrasp]   视觉估计失败，%.1f 秒后重试...",
                    kVisionEstimateRetryDelaySec);
        std::this_thread::sleep_for(std::chrono::duration<double>(kVisionEstimateRetryDelaySec));
      }
    }

    if (!vision_success)
    {
      response->success = false;
      response->message = "视觉位姿估计失败（已重试3次）";
      RCLCPP_ERROR(get_logger(), "[handleExecuteSingleGrasp] ❌ %s", response->message.c_str());
      RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
      return;
    }
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] ✓ 视觉估计完成");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] 使用参数常量抓取位姿：");
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   位置: [%.3f, %.3f, %.3f] m",
                grasp_position_[0], grasp_position_[1], grasp_position_[2]);
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   Z轴旋转: %.4f rad (%.2f°)",
                grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);
  }

  // 执行抓取周期
  RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] 步骤 %s: 开始执行抓取周期...", 
              request->use_visual_estimation ? "2/2" : "1/1");
  RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────────");
  
  bool success = runOneCycle();
  
  RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────────");

  // 填充响应
  response->success = success;
  response->message = success ? "单次抓取完成" : "单次抓取失败";
  response->final_position.x = grasp_position_[0];
  response->final_position.y = grasp_position_[1];
  response->final_position.z = grasp_position_[2];

  response->final_orientation = grasp_orientation_;

  if (success)
  {
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] ✓ %s", response->message.c_str());
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp] 最终抓取位姿：");
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   位置: (%.3f, %.3f, %.3f) m",
                response->final_position.x, response->final_position.y, response->final_position.z);
    RCLCPP_INFO(get_logger(), "[handleExecuteSingleGrasp]   姿态: (qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f)",
                response->final_orientation.x, response->final_orientation.y,
                response->final_orientation.z, response->final_orientation.w);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "[handleExecuteSingleGrasp] ❌ %s", response->message.c_str());
  }
  
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), "");
}

void ExecuteGraspPoseWorker::handleLoopGraspControl(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const std::string action = request->data ? "启动" : "停止";
  
  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║        [循环抓取控制] 收到%s请求                          ║", action.c_str());
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
  
  if (request->data)
  {
    // 启动循环抓取
    if (loop_grasp_running_)
    {
      response->success = false;
      response->message = "循环抓取已在运行中";
      RCLCPP_WARN(get_logger(), "[handleLoopGraspControl] ⚠ %s", response->message.c_str());
      RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
      RCLCPP_INFO(get_logger(), "");
      return;
    }

    RCLCPP_INFO(get_logger(), "[handleLoopGraspControl] 正在启动循环抓取线程...");
    startLoopGrasp();
    response->success = true;
    response->message = "循环抓取已启动";
    RCLCPP_INFO(get_logger(), "[handleLoopGraspControl] ✓ %s", response->message.c_str());
  }
  else
  {
    // 停止循环抓取
    if (!loop_grasp_running_)
    {
      response->success = false;
      response->message = "循环抓取未在运行";
      RCLCPP_WARN(get_logger(), "[handleLoopGraspControl] ⚠ %s", response->message.c_str());
      RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
      RCLCPP_INFO(get_logger(), "");
      return;
    }

    RCLCPP_INFO(get_logger(), "[handleLoopGraspControl] 已发送优雅停止请求（当前轮结束后停止）...");
    stopLoopGrasp();
    response->success = true;
    response->message = cycle_in_progress_.load() ? "已收到停止请求，将在当前轮结束后停止"
                                                  : "已收到停止请求，当前无活动周期，立即停止";
    RCLCPP_INFO(get_logger(), "[handleLoopGraspControl] ✓ %s", response->message.c_str());
  }
  
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), "");
}

void ExecuteGraspPoseWorker::startLoopGrasp()
{
  // 若上一次循环线程已退出但尚未 join，先回收线程资源再重启。
  if (loop_thread_.joinable())
  {
    loop_thread_.join();
  }

  stop_loop_flag_ = false;
  stop_after_cycle_ = false;
  loop_grasp_running_ = true;

  // 启动循环抓取线程
  loop_thread_ = std::thread(&ExecuteGraspPoseWorker::loopGraspThread, this);
}

void ExecuteGraspPoseWorker::stopLoopGrasp()
{
  if (!loop_grasp_running_)
    return;

  // 优雅停止：请求当前轮结束后退出，避免在服务回调内长时间阻塞。
  stop_after_cycle_ = true;
  stop_loop_flag_ = true;
}

void ExecuteGraspPoseWorker::loopGraspThread()
{
  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║           [循环抓取线程] 已启动                           ║");
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");

  // 获取 object_id（可以从参数中读取，默认为 "default"）
  std::string object_id = "default";
  if (has_parameter("egp_object_id"))
  {
    object_id = get_parameter("egp_object_id").as_string();
  }
  RCLCPP_INFO(get_logger(), "[loopGraspThread] 循环抓取配置：");
  RCLCPP_INFO(get_logger(), "[loopGraspThread]   object_id: %s", object_id.c_str());
  RCLCPP_INFO(get_logger(), "[loopGraspThread]   模式: 视觉估计 + 自动抓取");
  RCLCPP_INFO(get_logger(), "[loopGraspThread]   停止条件: 收到停止信号或ROS节点关闭");

  int cycle_count = 0;
  while (!stop_loop_flag_ && rclcpp::ok())
  {
    cycle_in_progress_ = true;
    cycle_count++;
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(get_logger(), "║           循环抓取 - 第 %3d 次循环                        ║", cycle_count);
    RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");

    // 1. 调用视觉估计
    RCLCPP_INFO(get_logger(), "[loopGraspThread] 步骤 1/2: 调用视觉估计...");
    if (!estimatePoseFromVision(object_id))
    {
      RCLCPP_WARN(get_logger(), "[loopGraspThread] ⚠ 视觉估计失败，等待2秒后重试...");
      cycle_in_progress_ = false;
      if (stop_after_cycle_)
        break;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    RCLCPP_INFO(get_logger(), "[loopGraspThread] ✓ 视觉估计成功");

    // 2. 执行抓取周期
    RCLCPP_INFO(get_logger(), "[loopGraspThread] 步骤 2/2: 执行抓取周期...");
    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────────");
    
    bool success = runOneCycle();
    
    RCLCPP_INFO(get_logger(), "────────────────────────────────────────────────────────────");
    
    if (!success)
    {
      RCLCPP_WARN(get_logger(), "[loopGraspThread] ⚠ 抓取周期失败，等待2秒后重试...");
      cycle_in_progress_ = false;
      if (stop_after_cycle_)
        break;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    cycle_in_progress_ = false;
    if (stop_after_cycle_)
      break;

    RCLCPP_INFO(get_logger(), "[loopGraspThread] ✓ 循环第 %d 次完成，等待1秒后继续...", cycle_count);
    RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  cycle_in_progress_ = false;
  loop_grasp_running_ = false;

  if (stop_loop_flag_)
  {
    RCLCPP_INFO(get_logger(), "[loopGraspThread] 收到停止信号");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "[loopGraspThread] ROS节点已关闭");
  }
  
  RCLCPP_INFO(get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║      [循环抓取线程] 已退出 (共完成 %3d 次循环)           ║", cycle_count);
  RCLCPP_INFO(get_logger(), "╚════════════════════════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), "");
}

// ============================================================================
// 主执行流程
// ============================================================================

bool ExecuteGraspPoseWorker::run()
{
  RCLCPP_INFO(get_logger(), "ExecuteGraspPoseWorker 等待服务调用...");
  
  // 服务驱动模式：不自动执行，等待服务调用
  // 返回 true 表示节点正常运行，实际的抓取逻辑由服务回调触发
  return true;
}

}  // namespace demo_driver

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::ExecuteGraspPoseWorker::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  // add_node 会注册节点上全部 callback group（含 estimate_pose 的 Reentrant 组），勿再 add_callback_group 同一组
  executor.add_node(node);

  std::thread spinner([&executor]() { executor.spin(); });

  demo_driver::g_worker_for_signal = node.get();
  std::signal(SIGINT, demo_driver::sigintHandler);
  std::signal(SIGTERM, demo_driver::sigintHandler);

  const int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "ExecuteGraspPoseWorker 就绪，等待服务调用 (/execute_single_grasp, /loop_grasp_control)");
  node->run();

  // 保持节点运行，等待 Ctrl+C 信号
  spinner.join();

  demo_driver::g_worker_for_signal = nullptr;
  return 0;
}
