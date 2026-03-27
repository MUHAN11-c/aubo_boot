/**
 * @file gripper_swap_worker.cpp
 * @brief 夹爪更换 Worker 节点：继承 MoveitGripperIoBase，实现夹爪快换及 IO 控制。
 */

#include "demo_driver/gripper_swap_worker.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#define CHECK(expr)                                                                                                       \
  do                                                                                                                      \
  {                                                                                                                       \
    if (!(expr))                                                                                                          \
      return false;                                                                                                       \
  } while (0)

namespace demo_driver
{
namespace
{
static constexpr int kArcPathMaxRetries = 5;
static constexpr int kArcPathRetryDelaySec = 2;
static constexpr double kArcPathInitialDelaySec = 0.2;
static constexpr double kCartesianEefStep = 0.01;
static constexpr double kCartesianJumpThreshold = 0.0;
// 仿真临时开关：
// true  -> 跳过本文件全部 setGripperIo 调用（不调用 /aubo_driver/set_io）
// false -> 恢复真实夹爪快换 IO 控制
static constexpr bool kSkipTemporaryGripperIo = false;
}  // namespace

GripperSwapWorker::GripperSwapWorker(const rclcpp::NodeOptions& options) : MoveitGripperIoBase(options)
{
  declare_parameter("joint_velocity_scaling", 0.7f);
  declare_parameter("joint_acceleration_scaling", 0.3f);
  declare_parameter("home_velocity_scaling", 0.7f);
  declare_parameter("home_acceleration_scaling", 0.3f);
  declare_parameter("gripper_io_index", kQuickSwapIoIndex);
  declare_parameter("joint_cartesian_switch_delay_sec", 0.05);

  joint_velocity_scaling_ = get_parameter("joint_velocity_scaling").as_double();
  joint_acceleration_scaling_ = get_parameter("joint_acceleration_scaling").as_double();
  home_velocity_scaling_ = get_parameter("home_velocity_scaling").as_double();
  home_acceleration_scaling_ = get_parameter("home_acceleration_scaling").as_double();
  gripper_io_index_ = static_cast<int32_t>(get_parameter("gripper_io_index").as_int());
  joint_cartesian_switch_delay_sec_ = std::max(0.0, get_parameter("joint_cartesian_switch_delay_sec").as_double());

  // 服务回调放入独立 CallbackGroup，避免长耗时换爪流程占住默认组，导致 MoveIt current_state_monitor 无法及时更新。
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_swap_srv_ = create_service<demo_interface::srv::RunGripperSwap>(
      "run_gripper_swap",
      std::bind(&GripperSwapWorker::onGripperSwapRequest, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_cb_group_);
  RCLCPP_INFO(get_logger(),
              "服务 run_gripper_swap 已创建，通过 direction 选择 gripper0_to_gripper2、gripper2_to_gripper0 或 gripper2；"
              "joint_vel=%.2f joint_acc=%.2f home_vel=%.2f home_acc=%.2f j↔c_switch_delay=%.3f io_index=%d",
              joint_velocity_scaling_, joint_acceleration_scaling_, home_velocity_scaling_, home_acceleration_scaling_,
              joint_cartesian_switch_delay_sec_, gripper_io_index_);
}

bool GripperSwapWorker::sleepJointCartesianSwitchDelay(const char* where)
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

std::shared_ptr<GripperSwapWorker> GripperSwapWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<GripperSwapWorker>(options);
  node->initMoveGroup();
  return node;
}

bool GripperSwapWorker::runArcPath(double z_offset, float velocity_factor, float acceleration_factor)
{
  return runArcPath('z', z_offset, velocity_factor, acceleration_factor);
}

bool GripperSwapWorker::runArcPath(char axis, double offset, float velocity_factor, float acceleration_factor)
{
  (void)velocity_factor;
  (void)acceleration_factor;
  const char* axis_label = (axis == 'x') ? "X" : (axis == 'y') ? "Y" : "Z";
  RCLCPP_INFO(get_logger(), "[gripper_swap_worker] 笛卡尔路径(无Z限位): 沿 %s 轴 %+.3f m", axis_label, offset);

  if (kArcPathInitialDelaySec > 0.0)
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
    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "[gripper_swap_worker] 笛卡尔完成度 %.2f%% (尝试 %d/%d)",
                fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      auto exec_ok = move_group_->execute(plan);
      if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "[gripper_swap_worker] 笛卡尔执行失败，错误码=%d", exec_ok.val);
        return false;
      }
      return true;
    }
    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }
  return false;
}

bool GripperSwapWorker::runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor,
                                           float acceleration_factor)
{
  (void)velocity_factor;
  (void)acceleration_factor;
  if (segments.empty())
    return true;

  if (kArcPathInitialDelaySec > 0.0)
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
    // 注意：这里故意不做 Z 安全下限裁剪（夹爪快换工位需要）
    waypoints.push_back(p);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "[gripper_swap_worker] 多段笛卡尔完成度 %.2f%% (尝试 %d/%d)",
                fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      auto exec_ok = move_group_->execute(plan);
      if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "[gripper_swap_worker] 多段笛卡尔执行失败，错误码=%d", exec_ok.val);
        return false;
      }
      return true;
    }
    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
  }
  return false;
}

bool GripperSwapWorker::swapToGripper0()
{
  RCLCPP_INFO(get_logger(), "swapToGripper0 (gripper2->gripper0) 开始执行");
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  const double y_step = 0.1;
  std::vector<CartesianSegment> segments = { { 'y', y_step }, { 'z', -0.243 }, { 'y', -y_step }, { 'z', -0.012 } };
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper0: 关节→笛卡尔（首段多轴）"));
  CHECK(runArcPathSequence(segments, joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)。"
                "如需真实夹爪快换，请将 kSkipTemporaryGripperIo 改为 false。",
                gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.3));
  CHECK(runArcPath('z', 0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

  CHECK(sleepJointCartesianSwitchDelay("swapToGripper0: 笛卡尔→关节"));
  CHECK(moveToJoints({ 1.004674, -0.050534, 1.752202, 0.231544, 1.571618, 1.004515 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper0: 关节→笛卡尔（Z 下降）"));
  CHECK(runArcPath('z', -0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath('z', 0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper0: 笛卡尔→回安全位（关节）"));
  CHECK(moveToHome(home_velocity_scaling_, home_acceleration_scaling_));
  return true;
}

bool GripperSwapWorker::swapToGripper2()
{
  RCLCPP_INFO(get_logger(), "swapToGripper2 (gripper0->gripper2) 开始执行");
  CHECK(moveToJoints({ 1.004674, -0.050534, 1.752202, 0.231544, 1.571618, 1.004515 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper2: 关节→笛卡尔"));
  CHECK(runArcPath('z', -0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath('z', 0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }

  const double y_step = 0.1;
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper2: 笛卡尔→关节"));
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper2: 关节→笛卡尔"));
  CHECK(runArcPath('z', -0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  std::vector<CartesianSegment> segments1 = { { 'z', 0.012 }, { 'y', y_step }, { 'z', 0.243 } };
  CHECK(runArcPathSequence(segments1, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(sleepJointCartesianSwitchDelay("swapToGripper2: 笛卡尔→回安全位（关节）"));
  CHECK(moveToHome(home_velocity_scaling_, home_acceleration_scaling_));
  return true;
}

bool GripperSwapWorker::switchToGripper2()
{
  RCLCPP_INFO(get_logger(), "switchToGripper2 开始执行");
  const double y_step = 0.1;
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  CHECK(sleepJointCartesianSwitchDelay("switchToGripper2: 关节→笛卡尔"));
  CHECK(runArcPath('z', -0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  std::vector<CartesianSegment> segments1 = { { 'z', 0.012 }, { 'y', y_step }, { 'z', 0.243 } };
  CHECK(runArcPathSequence(segments1, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(sleepJointCartesianSwitchDelay("switchToGripper2: 笛卡尔→回安全位（关节）"));
  CHECK(moveToHome(home_velocity_scaling_, home_acceleration_scaling_));
  return true;
}

bool GripperSwapWorker::run()
{
  CHECK(moveToJoints({ 0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643 },
                     joint_velocity_scaling_, joint_acceleration_scaling_));
  const double y_step = 0.1;
  std::vector<CartesianSegment> segments = { { 'y', y_step }, { 'z', -0.243 }, { 'y', -y_step }, { 'z', -0.012 } };
  CHECK(sleepJointCartesianSwitchDelay("run: 关节→笛卡尔"));
  CHECK(runArcPathSequence(segments, joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(1));
  std::vector<CartesianSegment> segments1 = { { 'z', 0.012 }, { 'y', y_step }, { 'z', 0.243 } };
  // 连续笛卡尔段之间不加 j↔c 延时
  CHECK(runArcPathSequence(segments1, joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }

  CHECK(sleepJointCartesianSwitchDelay("run: 笛卡尔→关节（moveToPose）"));
  CHECK(moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false,
                   joint_velocity_scaling_, joint_acceleration_scaling_));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, true)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, true));
  }
  CHECK(sleepJointCartesianSwitchDelay("run: 关节→笛卡尔"));
  CHECK(runArcPath('z', -0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(), "⚠ 仿真临时模式：跳过 setGripperIo(%d, false)", gripper_io_index_);
  }
  else
  {
    CHECK(setGripperIo(gripper_io_index_, false));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  CHECK(runArcPath('z', 0.255, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(sleepJointCartesianSwitchDelay("run: 笛卡尔→回安全位（关节）"));
  CHECK(moveToHome(home_velocity_scaling_, home_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return true;
}

void GripperSwapWorker::onGripperSwapRequest(
    const std::shared_ptr<demo_interface::srv::RunGripperSwap::Request> request,
    std::shared_ptr<demo_interface::srv::RunGripperSwap::Response> response)
{
  const std::string& direction = request->direction;
  if (direction != "gripper0_to_gripper2" && direction != "gripper2_to_gripper0" && direction != "gripper2")
  {
    response->success = false;
    response->message = "未知 direction，仅支持 gripper0_to_gripper2、gripper2_to_gripper0 或 gripper2";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  bool expected = false;
  if (!swap_in_progress_.compare_exchange_strong(expected, true))
  {
    response->success = false;
    response->message = "夹爪切换正在执行中，请勿并发调用";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "收到 run_gripper_swap 请求 direction=%s，开始同步执行（阻塞返回）", direction.c_str());
  bool ok = false;
  try
  {
    if (direction == "gripper0_to_gripper2")
      ok = swapToGripper2();
    else if (direction == "gripper2_to_gripper0")
      ok = swapToGripper0();
    else if (direction == "gripper2")
      ok = switchToGripper2();
  }
  catch (const std::exception& e)
  {
    ok = false;
    RCLCPP_ERROR(get_logger(), "run_gripper_swap 执行异常: %s", e.what());
  }
  catch (...)
  {
    ok = false;
    RCLCPP_ERROR(get_logger(), "run_gripper_swap 执行异常: 未知异常");
  }

  swap_in_progress_.store(false);

  response->success = ok;
  response->message = ok ? ("切换完成: " + direction) : ("切换失败: " + direction);
  if (ok)
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  else
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::GripperSwapWorker::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "夹爪更换 Worker 已就绪，调用服务 run_gripper_swap 执行夹爪快换");
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
