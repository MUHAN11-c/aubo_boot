/**
 * @file moveit_arcline_demo.cpp
 * @brief 三夹爪快换 Demo 基类实现。
 */

#include "demo_driver/moveit_arcline_demo.h"

#include <chrono>
#include <future>
#include <thread>
#include <vector>

#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>

#define CHECK(expr)                                                                                                       \
  do                                                                                                                      \
  {                                                                                                                       \
    if (!(expr))                                                                                                          \
      return false;                                                                                                       \
  } while (0)

namespace demo_driver
{

const std::array<double, 6> MoveitArclineDemo::kHomeJointsRad1 = { 1.210212, 0.129677, 1.925533, 0.225356, 1.571783,
                                                                    1.209540 };
const std::string MoveitArclineDemo::kMoveToPoseService = "/move_to_pose";
const std::string MoveitArclineDemo::kAuboSetIOService = "/aubo_driver/set_io";

static constexpr int kArcPathMaxRetries = 5;
static constexpr int kArcPathRetryDelaySec = 2;
static constexpr double kArcPathInitialDelaySec = 0.2;
static constexpr double kCartesianEefStep = 0.01;
static constexpr double kCartesianJumpThreshold = 0.0;

MoveitArclineDemo::MoveitArclineDemo(const rclcpp::NodeOptions& options) : rclcpp::Node("moveit_arcline_demo", options)
{
  move_to_pose_client_ = create_client<demo_interface::srv::MoveToPose>(kMoveToPoseService);
  aubo_set_io_client_ = create_client<aubo_msgs::srv::SetIO>(kAuboSetIOService);
}

std::shared_ptr<MoveitArclineDemo> MoveitArclineDemo::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<MoveitArclineDemo>(options);
  node->initMoveGroup();
  return node;
}

void MoveitArclineDemo::initMoveGroup()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
  move_group_->allowReplanning(true);
  move_group_->setMaxVelocityScalingFactor(0.5);
}

bool MoveitArclineDemo::waitForServices(std::chrono::seconds timeout)
{
  if (!move_to_pose_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(get_logger(), "服务 %s 未就绪，请先启动 move_to_pose_server_node", kMoveToPoseService.c_str());
    return false;
  }
  if (!aubo_set_io_client_->wait_for_service(timeout))
  {
    RCLCPP_WARN(get_logger(), "服务 %s 未就绪，setDigitalOutput 不可用", kAuboSetIOService.c_str());
  }
  RCLCPP_INFO(get_logger(), "将通过 %s 设置数字输出", kAuboSetIOService.c_str());
  RCLCPP_INFO(get_logger(), "所需服务已就绪");
  return true;
}

bool MoveitArclineDemo::moveToJoints(const std::array<double, 6>& joint_positions_rad, float velocity_factor,
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

bool MoveitArclineDemo::moveToPose(double x, double y, double z, double qx, double qy, double qz, double qw,
                                   bool use_joints, float velocity_factor, float acceleration_factor)
{
  auto req = std::make_shared<demo_interface::srv::MoveToPose::Request>();
  req->target_pose.position.x = x;
  req->target_pose.position.y = y;
  req->target_pose.position.z = z;
  req->target_pose.orientation.x = qx;
  req->target_pose.orientation.y = qy;
  req->target_pose.orientation.z = qz;
  req->target_pose.orientation.w = qw;
  req->use_joints = use_joints;
  req->velocity_factor = velocity_factor;
  req->acceleration_factor = acceleration_factor;

  auto future = move_to_pose_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] 调用超时或失败");
    return false;
  }

  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] 失败: error_code=%d %s", res->error_code, res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[move_to_pose] 成功");
  return true;
}

bool MoveitArclineDemo::setDigitalOutput(int32_t io_index, bool high)
{
  if (io_index < 0)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] io_index 必须 >= 0，当前为 %d", io_index);
    return false;
  }

  if (!aubo_set_io_client_->service_is_ready())
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] 服务 %s 不可用", kAuboSetIOService.c_str());
    return false;
  }

  auto req = std::make_shared<aubo_msgs::srv::SetIO::Request>();
  req->fun = aubo_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  req->pin = static_cast<int8_t>(io_index);
  req->state = high ? aubo_msgs::srv::SetIO::Request::STATE_ON : aubo_msgs::srv::SetIO::Request::STATE_OFF;

  auto future = aubo_set_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] 调用 %s 超时或失败", kAuboSetIOService.c_str());
    return false;
  }

  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] %s 返回失败", kAuboSetIOService.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[set_digital_output] pin %d -> %s (via %s)", io_index, high ? "HIGH" : "LOW",
              kAuboSetIOService.c_str());
  return true;
}

bool MoveitArclineDemo::runArcPath(double z_offset)
{
  return runArcPath('z', z_offset);
}

bool MoveitArclineDemo::runArcPath(char axis, double offset)
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
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）完成度: %.2f%% (尝试 %d/%d)", axis_label, offset, fraction * 100.0,
                attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "%s 轴直线移动执行完成", axis_label);
      return true;
    }

    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }

  RCLCPP_ERROR(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）在 %d 次尝试后仍未达 100%%", axis_label, offset,
               kArcPathMaxRetries);
  return false;
}

bool MoveitArclineDemo::runArcPathSequence(const std::vector<CartesianSegment>& segments)
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
    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "多段笛卡尔路径完成度: %.2f%% (尝试 %d/%d)", fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "多段笛卡尔路径执行完成");
      return true;
    }

    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }

  RCLCPP_ERROR(get_logger(), "多段笛卡尔路径在 %d 次尝试后仍未达 100%%", kArcPathMaxRetries);
  return false;
}

bool MoveitArclineDemo::moveToHome()
{
  move_group_->setNamedTarget("camera_pose");
  move_group_->move();
  return true;
}

bool MoveitArclineDemo::moveToArcStart()
{
  return moveToPose(0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0, false, 0.2f, 0.2f);
}

bool MoveitArclineDemo::run()
{
  CHECK(moveToJoints(kHomeJointsRad1, 0.15f, 0.1f));
  CHECK(runArcPath(-0.255));
  CHECK(runArcPath(0.255));
  CHECK(moveToHome());
  return true;
}

}  // namespace demo_driver
