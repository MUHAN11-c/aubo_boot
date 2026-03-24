/**
 * @file execute_grasp_pose_worker.cpp
 * @brief 执行抓取位姿 Worker（服务驱动模式）：支持视觉估计和参数常量两种抓取模式
 */

#include "demo_driver/execute_grasp_pose_worker.h"

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
#include <interface/srv/estimate_pose.hpp>

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

/** 子类内实现：基类 scaleTrajectoryTime 为 static 不可访问 */
static void scaleTrajectoryTimeLocal(moveit_msgs::msg::RobotTrajectory& traj, double velocity_factor)
{
  if (velocity_factor <= 0.0 || velocity_factor > 1.0)
    return;
  const double scale = 1.0 / velocity_factor;
  auto& pts = traj.joint_trajectory.points;
  for (size_t i = 0; i < pts.size(); ++i)
  {
    double sec =
        static_cast<double>(pts[i].time_from_start.sec) + 1e-9 * static_cast<double>(pts[i].time_from_start.nanosec);
    sec *= scale;
    pts[i].time_from_start.sec = static_cast<int32_t>(sec);
    pts[i].time_from_start.nanosec = static_cast<uint32_t>((sec - pts[i].time_from_start.sec) * 1e9);
    if (!pts[i].velocities.empty())
      for (auto& v : pts[i].velocities)
        v /= scale;
    if (!pts[i].accelerations.empty())
      for (auto& a : pts[i].accelerations)
        a /= (scale * scale);
  }
}

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
// | egp_joint_velocity_scaling | double   | 1.0            | 关节/笛卡尔速度缩放 [0~1]                           |
// | egp_joint_acceleration_scaling| double| 0.1            | 关节/笛卡尔加速度缩放 [0~1]                         |
// | egp_home_velocity_scaling  | double   | 0.7            | moveToHome 回安全位速度缩放 [0~1]                   |
// | egp_home_acceleration_scaling| double| 0.45           | moveToHome 回安全位加速度缩放 [0~1]                 |
// | egp_gripper_io_index       | int      | 7              | Aubo 夹爪 IO pin 号                                  |
// | egp_lift_offset            | double   | 0.2            | 抓取后沿 Z 轴抬起高度 (m)                            |
// | egp_place_mode             | string   | home_offset    | pose/joints/home_offset                             |
// | egp_place_offset_y         | double   | -0.2           | 安全位 y 偏移 (m)                                   |
// | egp_place_offset_z         | double   | -0.15          | 安全位 z 偏移 (m)                                   |
// | egp_place_pose             | double[7]| 见代码         | (x,y,z,qx,qy,qz,qw)                                 |
// | egp_place_joints           | double[6]| 见代码         | 放置关节角 (rad)                                     |
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
    declare_parameter("egp_joint_velocity_scaling", 1.0);
  if (!has_parameter("egp_joint_acceleration_scaling"))
    declare_parameter("egp_joint_acceleration_scaling", 0.1);
  if (!has_parameter("egp_home_velocity_scaling"))
    declare_parameter("egp_home_velocity_scaling", 0.7);
  if (!has_parameter("egp_home_acceleration_scaling"))
    declare_parameter("egp_home_acceleration_scaling", 0.45);
  if (!has_parameter("egp_gripper_io_index"))
    declare_parameter("egp_gripper_io_index", kGripperIoIndex);
  if (!has_parameter("egp_lift_offset"))
    declare_parameter("egp_lift_offset", 0.2);
  if (!has_parameter("egp_place_mode"))
    declare_parameter("egp_place_mode", std::string("home_offset"));
  if (!has_parameter("egp_place_offset_y"))
    declare_parameter("egp_place_offset_y", -0.2);
  if (!has_parameter("egp_place_offset_z"))
    declare_parameter("egp_place_offset_z", -0.15);
  if (!has_parameter("egp_cartesian_max_points"))
    declare_parameter("egp_cartesian_max_points", 40);
  if (!has_parameter("egp_place_pose"))
    declare_parameter("egp_place_pose", std::vector<double>({ 0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0 }));
  if (!has_parameter("egp_place_joints"))
    declare_parameter("egp_place_joints", std::vector<double>({ 1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540 }));

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
  home_velocity_scaling_ = get_parameter("egp_home_velocity_scaling").as_double();
  home_acceleration_scaling_ = get_parameter("egp_home_acceleration_scaling").as_double();
  gripper_io_index_ = get_parameter("egp_gripper_io_index").as_int();
  lift_offset_ = get_parameter("egp_lift_offset").as_double();
  place_mode_ = get_parameter("egp_place_mode").as_string();
  place_offset_y_ = get_parameter("egp_place_offset_y").as_double();
  place_offset_z_ = get_parameter("egp_place_offset_z").as_double();
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

  place_pose_ = { 0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0 };
  place_joints_ = { 1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540 };
  {
    auto p = get_parameter("egp_place_pose").as_double_array();
    for (size_t i = 0; i < 7 && i < p.size(); ++i)
      place_pose_[i] = p[i];
  }
  {
    auto p = get_parameter("egp_place_joints").as_double_array();
    for (size_t i = 0; i < 6 && i < p.size(); ++i)
      place_joints_[i] = p[i];
  }

  RCLCPP_INFO(get_logger(),
              "ExecuteGraspPoseWorker 初始化完成：抓取位置=[%.3f, %.3f, %.3f], Z轴旋转=%.4f rad (%.2f°)",
              grasp_position_[0], grasp_position_[1], grasp_position_[2], grasp_z_rotation_, grasp_z_rotation_ * 180.0 / M_PI);

  // 创建服务（放入独立 callback group，避免阻塞 MoveIt current_state_monitor 默认组）
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 创建服务
  execute_single_grasp_service_ = create_service<demo_interface::srv::ExecuteGraspPose>(
      "/execute_single_grasp",
      std::bind(&ExecuteGraspPoseWorker::handleExecuteSingleGrasp, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_cb_group_);

  loop_grasp_control_service_ = create_service<std_srvs::srv::SetBool>(
      "/loop_grasp_control",
      std::bind(&ExecuteGraspPoseWorker::handleLoopGraspControl, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_cb_group_);

  // 视觉 client 使用 Reentrant 回调组，避免服务回调内 wait_for 时与默认互斥组死锁；executor.add_node 会注册本组，勿重复 add_callback_group
  estimate_pose_client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  estimate_pose_client_ = create_client<interface::srv::EstimatePose>(
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
  (void)acc;

  const std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty())
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] 无法获取末端 link");
    return false;
  }

  geometry_msgs::msg::PoseStamped current_stamped = move_group_->getCurrentPose(eef_link);
  geometry_msgs::msg::Pose current_pose = current_stamped.pose;

  RCLCPP_INFO(get_logger(), "[runGraspApproach] 当前位姿 Position: [%.6f, %.6f, %.6f]",
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(get_logger(), "[runGraspApproach] 当前位姿 Orientation (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
              current_pose.orientation.x, current_pose.orientation.y,
              current_pose.orientation.z, current_pose.orientation.w);

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

  geometry_msgs::msg::Quaternion grasp_ori = pose_ee.orientation;
  geometry_msgs::msg::Quaternion grasp_ori_short = quatSameHemisphere(current_pose.orientation, grasp_ori);

  // 定义各个路径点
  geometry_msgs::msg::Pose p_xy;
  p_xy.position.x = gx;
  p_xy.position.y = gy;
  p_xy.position.z = current_pose.position.z;
  p_xy.orientation = current_pose.orientation;

  geometry_msgs::msg::Pose p_up;
  p_up.position.x = gx;
  p_up.position.y = gy;
  p_up.position.z = z_above;
  p_up.orientation = current_pose.orientation;

  geometry_msgs::msg::Pose p_rot;
  p_rot.position.x = gx;
  p_rot.position.y = gy;
  p_rot.position.z = z_above;
  p_rot.orientation = grasp_ori_short;

  geometry_msgs::msg::Pose p_down;
  p_down.position.x = gx;
  p_down.position.y = gy;
  p_down.position.z = gz;
  // p_down的姿态将在第4段时从第3段的结果继承

  // 分段执行笛卡尔路径
  struct Segment {
    std::string name;
    geometry_msgs::msg::Pose target;
    bool use_current_orientation;  // 是否基于当前姿态计算
  };
  
  std::vector<Segment> segments = {
    {"XY平移并上升到安全高度", p_up, false},
    {"相对Z轴旋转", {}, true},  // 第2段特殊处理
    {"下降到抓取点", {}, true}   // 第3段继承第2段姿态
  };

  for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx)
  {
    auto segment = segments[seg_idx];  // 拷贝，因为可能需要修改
    RCLCPP_INFO(get_logger(), "[runGraspApproach] 执行第%zu/%zu段: %s",
                seg_idx + 1, segments.size(), segment.name.c_str());
    
    geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose(eef_link);
    
    // 第2段：在当前姿态基础上叠加Z轴旋转
    if (segment.use_current_orientation && seg_idx == 1)
    {
      segment.target.position.x = gx;
      segment.target.position.y = gy;
      segment.target.position.z = z_above;
      
      // 打印当前姿态
      RCLCPP_INFO(get_logger(), "  ========== Z轴旋转前后姿态对比 ==========");
      RCLCPP_INFO(get_logger(), "  当前位置: [%.6f, %.6f, %.6f]",
                  current.pose.position.x, current.pose.position.y, current.pose.position.z);
      RCLCPP_INFO(get_logger(), "  当前姿态 (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
                  current.pose.orientation.x, current.pose.orientation.y,
                  current.pose.orientation.z, current.pose.orientation.w);
      
      // 将当前姿态转换为四元数
      Eigen::Quaterniond q_current(current.pose.orientation.w,
                                    current.pose.orientation.x,
                                    current.pose.orientation.y,
                                    current.pose.orientation.z);
      
      // 目标姿态（绝对目标）：来自抓取位姿，只在第2段完成“当前 -> 目标”的旋转
      Eigen::Quaterniond q_goal(grasp_ori_short.w, grasp_ori_short.x, grasp_ori_short.y, grasp_ori_short.z);
      q_goal.normalize();
      q_current.normalize();

      // 计算当前到目标的相对旋转：q_delta = q_current^{-1} * q_goal
      Eigen::Quaterniond q_delta = q_current.conjugate() * q_goal;
      q_delta.normalize();

      Eigen::Matrix3d delta_rot = q_delta.toRotationMatrix();
      const double delta_yaw = std::atan2(delta_rot(1, 0), delta_rot(0, 0));
      const double delta_yaw_short = normalizeYawToShortEquivalent(delta_yaw);

      RCLCPP_INFO(get_logger(), "  当前->目标 相对Z旋转: %.4f rad (%.2f°)",
                  delta_yaw, delta_yaw * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "  使用短角等价旋转: %.4f rad (%.2f°)",
                  delta_yaw_short, delta_yaw_short * 180.0 / M_PI);
      RCLCPP_INFO(get_logger(), "  相对旋转四元数 (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
                  q_delta.x(), q_delta.y(), q_delta.z(), q_delta.w());

      // 只在旋转段使用短角等价姿态，避免大角度绕行；位置 xyz 保持不变。
      Eigen::Quaterniond q_delta_short(Eigen::AngleAxisd(delta_yaw_short, Eigen::Vector3d::UnitZ()));
      Eigen::Quaterniond q_target = q_current * q_delta_short;
      q_target.normalize();
      
      segment.target.orientation.x = q_target.x();
      segment.target.orientation.y = q_target.y();
      segment.target.orientation.z = q_target.z();
      segment.target.orientation.w = q_target.w();
      
      RCLCPP_INFO(get_logger(), "  目标位置: [%.6f, %.6f, %.6f]",
                  segment.target.position.x, segment.target.position.y, segment.target.position.z);
      RCLCPP_INFO(get_logger(), "  目标姿态 (qx,qy,qz,qw): [%.6f, %.6f, %.6f, %.6f]",
                  segment.target.orientation.x, segment.target.orientation.y,
                  segment.target.orientation.z, segment.target.orientation.w);
      RCLCPP_INFO(get_logger(), "  =========================================");
    }
    // 第3段：继承当前姿态，只改变Z位置
    else if (segment.use_current_orientation && seg_idx == 2)
    {
      segment.target.position.x = gx;
      segment.target.position.y = gy;
      segment.target.position.z = gz;
      segment.target.orientation = current.pose.orientation;  // 继承当前姿态
      
      RCLCPP_INFO(get_logger(), "  保持当前姿态，下降到目标高度");
    }
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current.pose);
    if (!segment.use_current_orientation && seg_idx == 0)
    {
      waypoints.push_back(p_xy);
      waypoints.push_back(p_up);
    }
    else
    {
      waypoints.push_back(segment.target);
    }
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = false;
    
    for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
    {
      double fraction = move_group_->computeCartesianPath(
          waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
      size_t num_points = trajectory.joint_trajectory.points.size();

      RCLCPP_INFO(get_logger(), "  第%zu段笛卡尔: fraction=%.2f%%, 点数=%zu (尝试 %d/%d)",
                  seg_idx + 1, fraction * 100.0, num_points, attempt, kArcPathMaxRetries);

      if (fraction < 1.0)
      {
        RCLCPP_WARN(get_logger(), "  第%zu段笛卡尔路径未达100%%, fraction=%.2f", seg_idx + 1, fraction);
        if (attempt < kArcPathMaxRetries)
          std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
        continue;
      }
      if (num_points > static_cast<size_t>(cartesian_max_points_))
      {
        RCLCPP_ERROR(get_logger(), "  第%zu段笛卡尔轨迹点数过多 (%zu > %d)", seg_idx + 1, num_points,
                     cartesian_max_points_);
        if (attempt < kArcPathMaxRetries)
          std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
        continue;
      }

      scaleTrajectoryTimeLocal(trajectory, static_cast<double>(vel));
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      auto exec_ok = move_group_->execute(plan);
      if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "  第%zu段执行失败，错误码=%d", seg_idx + 1, exec_ok.val);
        if (attempt < kArcPathMaxRetries)
          std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
        continue;
      }
      RCLCPP_INFO(get_logger(), "  第%zu段完成", seg_idx + 1);
      success = true;
      break;
    }
    
    if (!success)
    {
      RCLCPP_ERROR(get_logger(), "[runGraspApproach] 第%zu段失败: %s", seg_idx + 1, segment.name.c_str());
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "抓取接近完成（所有%zu段执行成功）", segments.size());
  return true;
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
  if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
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
  RCLCPP_INFO(get_logger(), "► 步骤 3/9: 抓取前开夹爪 (IO=%d, 状态=true)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过抓取前开夹爪 IO 设置（未调用 /aubo_driver/set_io）。"
                "如需真实机械臂夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
    RCLCPP_INFO(get_logger(), "✓ 步骤 3 完成（已跳过IO）");
  }
  else
  {
    if (!setGripperIo(gripper_io_index_, true))  // IO反转：true=打开
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 3 失败：抓取前开夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 3 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 4/9: 抓取接近 (多段笛卡尔路径)");
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
    if (!setGripperIo(gripper_io_index_, false))  // IO反转：false=闭合
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 5 失败：闭夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 5 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 6/9: 抬起 (z=%.2f m)", lift_offset_);
  if (!runArcPath('z', lift_offset_, joint_velocity_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "✗ 步骤 6 失败：抬起失败");
    return false;
  }
  RCLCPP_INFO(get_logger(), "✓ 步骤 6 完成");

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 7/9: 移动到放置位 (place_mode=%s)", place_mode_.c_str());
  if (place_mode_ == "home_offset")
  {
    RCLCPP_INFO(get_logger(), "  放置模式：home_offset (回安全位 + 偏移)");
    if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：回安全位失败");
      return false;
    }
    const std::vector<CartesianSegment> place_segments = {
      { 'y', place_offset_y_ },
      { 'x', -0.2 },
      { 'z', place_offset_z_ },
    };
    RCLCPP_INFO(get_logger(), "  执行偏移：y=%.2f, x=-0.2, z=%.2f", place_offset_y_, place_offset_z_);
    if (!runArcPathSequence(place_segments, joint_velocity_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：放置位多段笛卡尔失败");
      return false;
    }
  }
  else if (place_mode_ == "pose")
  {
    RCLCPP_INFO(get_logger(), "  放置模式：pose (直接移动到目标位姿)");
    if (!moveToPose(place_pose_[0], place_pose_[1], place_pose_[2], place_pose_[3], place_pose_[4], place_pose_[5],
                    place_pose_[6], false, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：移动到放置位失败");
      return false;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "  放置模式：joints (移动到关节角度)");
    if (!moveToJoints(place_joints_, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 7 失败：移动到放置关节失败");
      return false;
    }
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
    if (!setGripperIo(gripper_io_index_, true))  // IO反转：true=打开
    {
      RCLCPP_ERROR(get_logger(), "✗ 步骤 8 失败：开夹爪失败");
      return false;
    }
    RCLCPP_INFO(get_logger(), "✓ 步骤 8 完成");
  }

  RCLCPP_INFO(get_logger(), "");
  RCLCPP_INFO(get_logger(), "► 步骤 9/9: 回安全位");
  if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
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

  auto request = std::make_shared<interface::srv::EstimatePose::Request>();
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
    const std::shared_ptr<demo_interface::srv::ExecuteGraspPose::Request> request,
    std::shared_ptr<demo_interface::srv::ExecuteGraspPose::Response> response)
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
