/**
 * @file moveit_gripper_io_base.cpp
 * @brief 夹爪 IO 控制基类实现
 *
 * 提供基于 MoveIt2 的关节运动、位姿运动、笛卡尔路径、夹爪 IO 控制（开关夹爪）等能力，
 * 供 GripperSwapWorker 等子类继承扩展。
 */

#include "demo_driver/moveit_gripper_io_base.h"

#include <chrono>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>

/** 表达式检查宏：若 expr 为 false 则立即 return false，用于链式调用 */
#define CHECK(expr) do { if (!(expr)) return false; } while (0)

namespace demo_driver
{
// #region agent log
namespace {
constexpr const char* kDebugLogPath = "/home/mu/IVG2.0/.cursor/debug-a15b92.log";
constexpr const char* kDebugSessionId = "a15b92";
inline void dbg_log(const char* run_id, const char* hypothesis_id, const char* location,
                    const char* message, const std::string& data_json)
{
  try
  {
    std::ofstream f(kDebugLogPath, std::ios::app);
    if (!f.is_open())
      return;
    const auto ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    f << "{\"sessionId\":\"" << kDebugSessionId
      << "\",\"runId\":\"" << run_id
      << "\",\"hypothesisId\":\"" << hypothesis_id
      << "\",\"location\":\"" << location
      << "\",\"message\":\"" << message
      << "\",\"data\":" << data_json
      << ",\"timestamp\":" << ts_ms << "}\n";
  }
  catch (...)
  {
  }
}
}  // namespace
// #endregion


// ============================================================================
// 静态成员定义
// ============================================================================

/** 回零关节角 (rad)，6 轴 */
const std::array<double, 6> MoveitGripperIoBase::kHomeJointsRad1 = {
  1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540
};

/** Aubo 驱动 SetIO 服务名 */
const std::string MoveitGripperIoBase::kAuboSetIOService = "/aubo_driver/set_io";

// ============================================================================
// 笛卡尔路径相关常量
// ============================================================================

static constexpr int kArcPathMaxRetries = 5;           /**< 笛卡尔路径最大重试次数 */
static constexpr int kArcPathRetryDelaySec = 2;       /**< 重试间隔 (s) */
static constexpr double kArcPathInitialDelaySec = 0.2; /**< 首次规划前延迟 (s) */
static constexpr double kCartesianEefStep = 0.01;     /**< 笛卡尔路径步长 (m) */
static constexpr double kCartesianJumpThreshold = 0.0;/**< 跳跃阈值，0 表示不限制 */

/** 缩放轨迹时间：computeCartesianPath 生成的轨迹 time_from_start 已固定，setMaxVelocityScalingFactor 无效，
 *  需手动缩放 time_from_start。factor=1/velocity_factor，如 0.15 速度 -> factor≈6.67，轨迹变慢 */
static void scaleTrajectoryTime(moveit_msgs::msg::RobotTrajectory& traj, double velocity_factor)
{
  if (velocity_factor <= 0.0 || velocity_factor > 1.0)
    return;
  const double scale = 1.0 / velocity_factor;
  auto& pts = traj.joint_trajectory.points;
  for (size_t i = 0; i < pts.size(); ++i)
  {
    double sec = static_cast<double>(pts[i].time_from_start.sec) + 1e-9 * pts[i].time_from_start.nanosec;
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

// ============================================================================
// 构造与初始化
// ============================================================================

MoveitGripperIoBase::MoveitGripperIoBase(const rclcpp::NodeOptions& options)
  : rclcpp::Node("moveit_gripper_io_base", options)
{
  aubo_set_io_client_ = create_client<aubo_msgs::srv::SetIO>(kAuboSetIOService);
}

std::shared_ptr<MoveitGripperIoBase> MoveitGripperIoBase::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<MoveitGripperIoBase>(options);
  node->initMoveGroup();
  return node;
}

void MoveitGripperIoBase::initMoveGroup()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "manipulator");
  move_group_->allowReplanning(true);
  move_group_->setMaxVelocityScalingFactor(0.5);
}

// ============================================================================
// 服务等待
// ============================================================================

bool MoveitGripperIoBase::waitForServices(std::chrono::seconds timeout)
{
  if (!aubo_set_io_client_->wait_for_service(timeout))
  {
    RCLCPP_WARN(get_logger(), "服务 %s 未就绪，setGripperIo 不可用", kAuboSetIOService.c_str());
  }
  RCLCPP_INFO(get_logger(), "将通过 %s 设置夹爪 IO", kAuboSetIOService.c_str());
  RCLCPP_INFO(get_logger(), "所需服务已就绪");
  return true;
}

// ============================================================================
// 关节空间运动
// ============================================================================

bool MoveitGripperIoBase::moveToJoints(const std::array<double, 6>& joint_positions_rad,
                                    float velocity_factor,
                                    float acceleration_factor)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_joints] MoveGroup 未初始化");
    return false;
  }

  move_group_->setMaxVelocityScalingFactor(velocity_factor);
  move_group_->setMaxAccelerationScalingFactor(acceleration_factor);
  std::vector<double> joints(joint_positions_rad.begin(), joint_positions_rad.end());
  move_group_->setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_ok = move_group_->plan(plan);
  if (plan_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_joints] 规划失败，错误码=%d", plan_ok.val);
    return false;
  }

  auto exec_ok = move_group_->execute(plan);
  if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_joints] 执行失败，错误码=%d", exec_ok.val);
    return false;
  }

  RCLCPP_INFO(get_logger(), "[move_to_joints] 成功");
  return true;
}

// ============================================================================
// 位姿空间运动
// ============================================================================

bool MoveitGripperIoBase::moveToPose(double x, double y, double z,
                                  double qx, double qy, double qz, double qw,
                                  bool use_joints,
                                  float velocity_factor,
                                  float acceleration_factor)
{
  (void)use_joints;  /* 直接使用 MoveGroup 位姿目标，MoveIt 内部做 IK */

  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] MoveGroup 未初始化");
    return false;
  }

  if (z < kZMinLimit)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] Z 轴安全限位: z=%.3f < %.2f m，拒绝执行", z, kZMinLimit);
    return false;
  }

  move_group_->setMaxVelocityScalingFactor(velocity_factor);
  move_group_->setMaxAccelerationScalingFactor(acceleration_factor);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  target_pose.orientation.x = qx;
  target_pose.orientation.y = qy;
  target_pose.orientation.z = qz;
  target_pose.orientation.w = qw;

  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_ok = move_group_->plan(plan);
  if (plan_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] 规划失败，错误码=%d", plan_ok.val);
    return false;
  }

  auto exec_ok = move_group_->execute(plan);
  if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] 执行失败，错误码=%d", exec_ok.val);
    return false;
  }

  logCurrentState();
  RCLCPP_INFO(get_logger(), "[move_to_pose] 成功");
  return true;
}

bool MoveitGripperIoBase::logCurrentState()
{
  if (!move_group_)
    return false;

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
  if (!current_state)
    return false;

  const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup("manipulator");
  if (!jmg)
    return false;

  std::vector<double> joint_positions;
  current_state->copyJointGroupPositions(jmg, joint_positions);
  if (joint_positions.size() < 6)
    return false;

  RCLCPP_INFO(get_logger(),
              "当前关节(rad) 可用于 moveToJoints: { %.6f, %.6f, %.6f, %.6f, %.6f, %.6f }",
              joint_positions[0], joint_positions[1], joint_positions[2],
              joint_positions[3], joint_positions[4], joint_positions[5]);

  /* 关节名 + 位置 */
  std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
  std::stringstream ss;
  ss << "  Joint positions (rad): ";
  for (size_t i = 0; i < joint_names.size() && i < joint_positions.size(); ++i)
    ss << joint_names[i] << "=" << std::fixed << std::setprecision(4) << joint_positions[i] << " ";
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  /* 关节速度 (来自 joint_states，若未发布则为 0 或未初始化) */
  std::vector<double> joint_velocities;
  current_state->copyJointGroupVelocities(jmg, joint_velocities);
  if (joint_velocities.size() >= joint_names.size())
  {
    ss.str("");
    ss << "  Joint velocities (rad/s): ";
    for (size_t i = 0; i < joint_names.size() && i < joint_velocities.size(); ++i)
      ss << joint_names[i] << "=" << std::fixed << std::setprecision(4) << joint_velocities[i] << " ";
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  /* 末端位姿 */
  const std::string eef_link = move_group_->getEndEffectorLink();
  if (!eef_link.empty())
  {
    current_state->update();
    const Eigen::Isometry3d& T = current_state->getGlobalLinkTransform(eef_link);
    Eigen::Vector3d p = T.translation();
    Eigen::Quaterniond q(T.rotation());
    RCLCPP_INFO(get_logger(),
                "  End effector (%s) pose: xyz=[%.4f, %.4f, %.4f] xyzw=[%.4f, %.4f, %.4f, %.4f]",
                eef_link.c_str(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
  }

  /* 所有变量位置 (用于调试) */
  const std::vector<std::string>& var_names = current_state->getVariableNames();
  const double* var_pos = current_state->getVariablePositions();
  if (var_pos && !var_names.empty())
  {
    ss.str("");
    ss << "  All variable positions: ";
    for (size_t i = 0; i < var_names.size(); ++i)
      ss << var_names[i] << "=" << std::fixed << std::setprecision(4) << var_pos[i] << " ";
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }

  return true;
}

// ============================================================================
// 夹爪 IO 控制
// ============================================================================

bool MoveitGripperIoBase::setGripperIo(int32_t io_index, bool high)
{
  if (io_index < 0)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] io_index 必须 >= 0，当前为 %d", io_index);
    return false;
  }

  if (!aubo_set_io_client_->service_is_ready())
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] 服务 %s 不可用", kAuboSetIOService.c_str());
    return false;
  }

  auto req = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  req->fun = aubo_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  req->pin = static_cast<int8_t>(io_index);
  req->state = high ? aubo_msgs::srv::SetIO::Request::STATE_ON : aubo_msgs::srv::SetIO::Request::STATE_OFF;

  auto future = aubo_set_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] 调用 %s 超时或失败", kAuboSetIOService.c_str());
    return false;
  }

  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] %s 返回失败", kAuboSetIOService.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "[set_gripper_io] pin %d -> %s (via %s)",
              io_index, high ? "HIGH" : "LOW", kAuboSetIOService.c_str());
  return true;
}

// ============================================================================
// 笛卡尔路径运动
// ============================================================================

bool MoveitGripperIoBase::runArcPath(double z_offset, float velocity_factor)
{
  return runArcPath('z', z_offset, velocity_factor);
}

bool MoveitGripperIoBase::runArcPath(char axis, double offset, float velocity_factor)
{
  const char* axis_label = (axis == 'x') ? "X" : (axis == 'y') ? "Y" : "Z";
  RCLCPP_INFO(get_logger(), "笛卡尔路径: 沿 %s 轴 %+.2f m", axis_label, offset);

  if (kArcPathInitialDelaySec > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));

  const std::string eef_link = move_group_->getEndEffectorLink();
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose.pose);

  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  if (axis == 'x')
    target_pose.position.x += offset;
  else if (axis == 'y')
    target_pose.position.y += offset;
  else
    target_pose.position.z += offset;

  if (axis == 'z' && target_pose.position.z < kZMinLimit)
  {
    RCLCPP_ERROR(get_logger(), "笛卡尔路径 Z 轴安全限位: 目标 z=%.3f < %.2f m，拒绝执行",
                 target_pose.position.z, kZMinLimit);
    return false;
  }
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）完成度: %.2f%% (尝试 %d/%d)",
                axis_label, offset, fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      scaleTrajectoryTime(trajectory, static_cast<double>(velocity_factor));
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "%s 轴直线移动执行完成 (速度因子 %.2f)", axis_label, velocity_factor);
      return true;
    }

    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }

  RCLCPP_ERROR(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）在 %d 次尝试后仍未达 100%%",
               axis_label, offset, kArcPathMaxRetries);
  return false;
}

bool MoveitGripperIoBase::runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor)
{
  if (segments.empty())
  {
    RCLCPP_INFO(get_logger(), "[runArcPathSequence] segments 为空，不运动");
    return true;
  }

  if (kArcPathInitialDelaySec > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));

  const std::string eef_link = move_group_->getEndEffectorLink();
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose.pose);

  geometry_msgs::msg::Pose p = current_pose.pose;
  for (const CartesianSegment& seg : segments)
  {
    if (seg.axis == 'x')
      p.position.x += seg.offset;
    else if (seg.axis == 'y')
      p.position.y += seg.offset;
    else
      p.position.z += seg.offset;

    if (seg.axis == 'z' && p.position.z < kZMinLimit)
    {
      RCLCPP_ERROR(get_logger(), "多段笛卡尔路径 Z 轴安全限位: 段后 z=%.3f < %.2f m，拒绝执行",
                   p.position.z, kZMinLimit);
      return false;
    }
    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "多段笛卡尔路径完成度: %.2f%% (尝试 %d/%d)",
                fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      scaleTrajectoryTime(trajectory, static_cast<double>(velocity_factor));
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      double planned_sec = 0.0;
      const auto& pts = plan.trajectory_.joint_trajectory.points;
      if (!pts.empty())
      {
        planned_sec = static_cast<double>(pts.back().time_from_start.sec) +
                      1e-9 * static_cast<double>(pts.back().time_from_start.nanosec);
      }
      {
        std::ostringstream oss;
        oss << "{\"attempt\":" << attempt
            << ",\"velocity_factor\":" << static_cast<double>(velocity_factor)
            << ",\"planned_points\":" << pts.size()
            << ",\"planned_duration_sec\":" << planned_sec
            << ",\"fraction\":" << fraction << "}";
        // #region agent log
        dbg_log("pre-fix-1", "H1", "moveit_gripper_io_base.cpp:runArcPathSequence:before_execute",
                "about to execute cartesian trajectory", oss.str());
        // #endregion
      }
      const auto exec_wall_start = std::chrono::steady_clock::now();
      const auto exec_ok = move_group_->execute(plan);
      const auto exec_wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - exec_wall_start).count();
      {
        std::ostringstream oss;
        oss << "{\"exec_code\":" << exec_ok.val
            << ",\"exec_wall_ms\":" << exec_wall_ms
            << ",\"planned_duration_sec\":" << planned_sec << "}";
        // #region agent log
        dbg_log("pre-fix-1", "H1", "moveit_gripper_io_base.cpp:runArcPathSequence:after_execute",
                "cartesian trajectory execute returned", oss.str());
        // #endregion
      }
      RCLCPP_INFO(get_logger(), "多段笛卡尔路径执行完成 (速度因子 %.2f)", velocity_factor);
      return true;
    }

    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }

  RCLCPP_ERROR(get_logger(), "多段笛卡尔路径在 %d 次尝试后仍未达 100%%", kArcPathMaxRetries);
  return false;
}

// ============================================================================
// 命名目标与默认流程
// ============================================================================

bool MoveitGripperIoBase::moveToHome(float velocity_factor, float acceleration_factor)
{
  move_group_->setMaxVelocityScalingFactor(velocity_factor);
  move_group_->setMaxAccelerationScalingFactor(acceleration_factor);
  move_group_->setNamedTarget("camera_pose");
  move_group_->move();
  return true;
}

bool MoveitGripperIoBase::moveToArcStart()
{
  return moveToPose(0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0, false, 0.2f, 0.2f);
}

bool MoveitGripperIoBase::run()
{
  CHECK(moveToJoints(kHomeJointsRad1, 0.2f, 0.1f));
  CHECK(runArcPath(-0.255));
  CHECK(runArcPath(0.255));
  CHECK(moveToHome());
  return true;
}

}  // namespace demo_driver
