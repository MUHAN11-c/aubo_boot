/**
 * @file gripper_swap_worker.cpp
 * @brief 夹爪快换 Worker（仅物理运动 + IO，场景显示由 scene_attach_worker 管理）
 */

#include "tool_changer/gripper_swap_worker.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <thread>

#define CHECK(expr) do { if (!(expr)) return false; } while (0)

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tool_changer
{
namespace
{

// ═══════════════════════════════════════════════════════════════════
// 快换工位几何
// ═══════════════════════════════════════════════════════════════════

constexpr double kDockDepth  = 0.206;
constexpr double kDockSeat   = 0.012;
constexpr double kDockLift   = 0.210;
constexpr double kDockSlideY = 0.100;

// ═══════════════════════════════════════════════════════════════════
// 动作时序
// ═══════════════════════════════════════════════════════════════════

constexpr double kReleaseWaitSec = 0.3;
constexpr double kLockWaitSec    = 0.5;

// ═══════════════════════════════════════════════════════════════════
// 关节位姿
// ═══════════════════════════════════════════════════════════════════

constexpr std::array<double, 6> kJoints_DockStation = {{
    0.936893, 0.016616, 1.419053, -0.167867, 1.571655, 0.935894}};

constexpr std::array<double, 6> kJoints_ReleaseGripper0 = {{
    1.137820, 0.222690, 1.598043, -0.194970, 1.571688, 1.136957}};

// ═══════════════════════════════════════════════════════════════════
// 笛卡尔位姿
// ═══════════════════════════════════════════════════════════════════

constexpr double kPos_Gripper0DockX = 0.27017;
constexpr double kPos_Gripper0DockY = 0.29517;
constexpr double kPos_Gripper2DockX = 0.3741;
constexpr double kPos_Gripper2DockY = 0.30394;
constexpr double kPos_SafeZ = 0.4755;

// ═══════════════════════════════════════════════════════════════════
// 笛卡尔 / IO
// ═══════════════════════════════════════════════════════════════════

constexpr int    kCartMaxRetries     = 5;
constexpr double kCartRetryWaitSec   = 2.0;
constexpr double kCartInitWaitSec    = 0.2;
constexpr double kCartEefStep        = 0.01;
constexpr double kCartJumpThreshold  = 0.0;
constexpr int    kIOTimeoutSec       = 60;
constexpr const char* kSetIOService = "/aubo_driver/set_io";

}  // namespace

// ═══════════════════════════════════════════════════════════════════
// 信号 & 睡眠
// ═══════════════════════════════════════════════════════════════════

static GripperSwapWorker* g_worker_for_signal = nullptr;

static void sigintHandler(int)
{
  if (g_worker_for_signal)
    g_worker_for_signal->requestShutdown();
  rclcpp::shutdown();
}

static void sleepInterruptible(GripperSwapWorker* worker, double seconds)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && (!worker || !worker->isShutdownRequested()) &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

struct SpinnerJoinGuard
{
  std::thread& th;
  ~SpinnerJoinGuard() { if (th.joinable()) th.join(); }
  SpinnerJoinGuard(const SpinnerJoinGuard&) = delete;
  SpinnerJoinGuard& operator=(const SpinnerJoinGuard&) = delete;
};

// ═══════════════════════════════════════════════════════════════════
// 工具定义
// ═══════════════════════════════════════════════════════════════════

const GripperSwapWorker::ToolInfo GripperSwapWorker::kToolGripper0 = {
    "gripper0", "气动夹爪 φ40", "gripper",
    R"({"stroke":40,"force":120,"weight":0.8})"};

const GripperSwapWorker::ToolInfo GripperSwapWorker::kToolGripper2 = {
    "gripper2", "电动夹爪 φ60", "gripper",
    R"({"stroke":60,"force":250,"weight":1.5})"};

const GripperSwapWorker::ToolInfo GripperSwapWorker::kToolNone = {"", "", "", ""};

// ═══════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════

GripperSwapWorker::GripperSwapWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("gripper_swap_worker", options)
{
  declare_parameter("joint_velocity_scaling", 0.7);
  declare_parameter("joint_acceleration_scaling", 0.3);
  declare_parameter("home_velocity_scaling", 0.7);
  declare_parameter("home_acceleration_scaling", 0.3);
  declare_parameter("gripper_io_index", 7);
  declare_parameter("joint_cartesian_switch_delay_sec", 0.05);
  declare_parameter("simulation_skip_io", false);

  joint_velocity_scaling_  = static_cast<float>(get_parameter("joint_velocity_scaling").as_double());
  joint_acceleration_scaling_ = static_cast<float>(get_parameter("joint_acceleration_scaling").as_double());
  home_velocity_scaling_   = static_cast<float>(get_parameter("home_velocity_scaling").as_double());
  home_acceleration_scaling_ = static_cast<float>(get_parameter("home_acceleration_scaling").as_double());
  gripper_io_index_ = static_cast<int32_t>(get_parameter("gripper_io_index").as_int());
  simulation_skip_io_ = get_parameter("simulation_skip_io").as_bool();
  joint_cartesian_switch_delay_sec_ =
      std::max(0.0, get_parameter("joint_cartesian_switch_delay_sec").as_double());

  set_io_client_ = create_client<demo_interface::srv::SetRobotIO>(kSetIOService);

  tool_status_pub_ = create_publisher<tool_changer_interface::msg::ToolChangerStatus>(
      "/tool_changer_status", 10);

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  gripper_swap_srv_ = create_service<tool_changer_interface::srv::RunGripperSwap>(
      "/run_gripper_swap",
      std::bind(&GripperSwapWorker::onGripperSwapRequest, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  change_tool_srv_ = create_service<tool_changer_interface::srv::ChangeTool>(
      "/change_tool",
      std::bind(&GripperSwapWorker::onChangeTool, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  get_tool_srv_ = create_service<tool_changer_interface::srv::GetCurrentTool>(
      "/get_current_tool",
      std::bind(&GripperSwapWorker::onGetCurrentTool, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  RCLCPP_INFO(get_logger(),
              "就绪 | run_gripper_swap / change_tool / get_current_tool"
              " | vel=%.2f acc=%.2f home_vel=%.2f home_acc=%.2f delay=%.3f io=%d sim_skip_io=%s",
              joint_velocity_scaling_, joint_acceleration_scaling_,
              home_velocity_scaling_, home_acceleration_scaling_,
              joint_cartesian_switch_delay_sec_, gripper_io_index_,
              simulation_skip_io_ ? "true" : "false");

  publishToolStatus(false);
}

std::shared_ptr<GripperSwapWorker> GripperSwapWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<GripperSwapWorker>(options);
  node->initMoveGroup();
  return node;
}

void GripperSwapWorker::initMoveGroup()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "manipulator");
  move_group_->allowReplanning(true);
  move_group_->setMaxVelocityScalingFactor(0.5);
  move_group_->setMaxAccelerationScalingFactor(0.5);
}

bool GripperSwapWorker::waitForServices(std::chrono::seconds timeout)
{
  if (!set_io_client_->wait_for_service(timeout))
  {
    RCLCPP_WARN(get_logger(), "服务 %s 未在 %lds 内就绪", kSetIOService, timeout.count());
    return false;
  }
  RCLCPP_INFO(get_logger(), "服务 %s 已就绪", kSetIOService);
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 运动原语
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::moveToJoints(const std::array<double, 6>& joints, float vel, float acc)
{
  if (!move_group_) { RCLCPP_ERROR(get_logger(), "MoveGroup 未初始化"); return false; }

  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
  move_group_->setJointValueTarget(std::vector<double>(joints.begin(), joints.end()));

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[moveToJoints] 规划失败");
    return false;
  }
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool GripperSwapWorker::moveToHome(float vel, float acc)
{
  if (!move_group_) { RCLCPP_ERROR(get_logger(), "MoveGroup 未初始化"); return false; }
  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
  move_group_->setNamedTarget("camera_pose");
  return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
}

bool GripperSwapWorker::moveToTargetXYZ(double target_x, double target_y, double target_z,
                                         float vel, float acc)
{
  const std::string eef_link = move_group_->getEndEffectorLink();
  const auto current_pose = move_group_->getCurrentPose(eef_link).pose;
  const double delta_x = target_x - current_pose.position.x;
  const double delta_y = target_y - current_pose.position.y;
  const double delta_z = target_z - current_pose.position.z;

  std::vector<CartesianSegment> segments;
  constexpr double kMinDeltaM = 1e-9;
  if (std::fabs(delta_x) > kMinDeltaM) segments.push_back({'x', delta_x});
  if (std::fabs(delta_y) > kMinDeltaM) segments.push_back({'y', delta_y});
  if (std::fabs(delta_z) > kMinDeltaM) segments.push_back({'z', delta_z});
  if (segments.empty()) return true;

  RCLCPP_INFO(get_logger(), "moveToXYZ: target(%.4f,%.4f,%.4f) delta(%.4f,%.4f,%.4f)",
              target_x, target_y, target_z, delta_x, delta_y, delta_z);
  return runCartesianPath(segments, vel, acc);
}

// ═══════════════════════════════════════════════════════════════════
// 笛卡尔路径
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::runCartesianPath(char axis, double offset, float vel, float acc)
{
  return runCartesianPath({{axis, offset}}, vel, acc);
}

bool GripperSwapWorker::runCartesianPath(const std::vector<CartesianSegment>& segments,
                                         float vel, float acc)
{
  if (!move_group_) { RCLCPP_ERROR(get_logger(), "MoveGroup 未初始化"); return false; }
  if (segments.empty()) return true;

  if (kCartInitWaitSec > 0.0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kCartInitWaitSec));

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kCartMaxRetries; ++attempt)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);

    const std::string eef_link = move_group_->getEndEffectorLink();
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose.pose);

    geometry_msgs::msg::Pose waypoint = current_pose.pose;
    for (const auto& seg : segments)
    {
      if (seg.axis == 'x')      waypoint.position.x += seg.offset;
      else if (seg.axis == 'y') waypoint.position.y += seg.offset;
      else                      waypoint.position.z += seg.offset;
      waypoints.push_back(waypoint);
    }

    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartEefStep, kCartJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔路径: %.1f%% (第 %d/%d 次)", fraction * 100.0, attempt, kCartMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "笛卡尔执行失败");
        return false;
      }
      return true;
    }
    if (attempt < kCartMaxRetries)
      std::this_thread::sleep_for(std::chrono::duration<double>(kCartRetryWaitSec));
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════
// IO
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::setGripperIoSafe(bool open_gripper)
{
  if (simulation_skip_io_)
  {
    RCLCPP_WARN(get_logger(), "仿真模式: 跳过 setGripperIo(%d, %s)",
                gripper_io_index_, open_gripper ? "释放(开)" : "锁定(关)");
    return true;
  }
  return setGripperIo(gripper_io_index_, open_gripper);
}

bool GripperSwapWorker::setGripperIo(int32_t io_index, bool high)
{
  if (!set_io_client_->service_is_ready())
  {
    RCLCPP_ERROR(get_logger(), "服务 %s 不可用", kSetIOService);
    return false;
  }

  auto req = std::make_shared<demo_interface::srv::SetRobotIO::Request>();
  req->io_type = "digital_output";
  req->io_index = io_index;
  req->value = high ? 1.0 : 0.0;

  auto future = set_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kIOTimeoutSec)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "setGripperIo(%d, %s) 超时", io_index, high ? "HIGH" : "LOW");
    return false;
  }

  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "setGripperIo 失败: %s", res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "setGripperIo(%d, %s) OK", io_index, high ? "HIGH" : "LOW");
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 工具状态
// ═══════════════════════════════════════════════════════════════════

void GripperSwapWorker::publishToolStatus(bool connected)
{
  auto msg = tool_changer_interface::msg::ToolChangerStatus();
  msg.header.stamp = now();
  msg.header.frame_id = "tool_changer";
  msg.tool_id        = current_tool_.id;
  msg.tool_name      = current_tool_.name;
  msg.tool_type      = current_tool_.type;
  msg.is_connected   = connected;
  msg.tool_parameters = current_tool_.parameters;
  tool_status_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "工具状态: id=%s name=%s connected=%s",
              current_tool_.id.c_str(), current_tool_.name.c_str(),
              connected ? "true" : "false");
}

// ═══════════════════════════════════════════════════════════════════
// 关节↔笛卡尔切换延时
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::sleepJointCartesianSwitchDelay(const char* where)
{
  if (joint_cartesian_switch_delay_sec_ <= 0.0) return true;
  RCLCPP_INFO(get_logger(), "%s: 关节↔笛卡尔切换延时 %.3f s", where,
              joint_cartesian_switch_delay_sec_);
  sleepInterruptible(this, joint_cartesian_switch_delay_sec_);
  return rclcpp::ok() && !shutdown_requested_;
}

// ═══════════════════════════════════════════════════════════════════
// 快换原语：放 / 取
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::releaseGripper(
    const std::vector<CartesianSegment>& approach,
    const std::vector<CartesianSegment>& depart,
    double settle_release_sec,
    double settle_close_sec)
{
  CHECK(runCartesianPath(approach, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(setGripperIoSafe(true));
  std::this_thread::sleep_for(std::chrono::duration<double>(settle_release_sec));
  CHECK(runCartesianPath(depart, joint_velocity_scaling_, joint_acceleration_scaling_));
  CHECK(setGripperIoSafe(false));
  std::this_thread::sleep_for(std::chrono::duration<double>(settle_close_sec));
  return true;
}

bool GripperSwapWorker::pickGripper(
    const std::vector<CartesianSegment>& approach,
    const std::vector<CartesianSegment>& depart,
    double settle_lock_sec)
{
  CHECK(setGripperIoSafe(true));
  CHECK(runCartesianPath(approach, joint_velocity_scaling_, joint_acceleration_scaling_));
  std::this_thread::sleep_for(std::chrono::duration<double>(settle_lock_sec));
  CHECK(setGripperIoSafe(false));
  std::this_thread::sleep_for(std::chrono::duration<double>(settle_lock_sec));
  CHECK(runCartesianPath(depart, joint_velocity_scaling_, joint_acceleration_scaling_));
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 流程函数（仅物理运动）
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::swapToGripper0()
{
  RCLCPP_INFO(get_logger(), "── 放gripper2 → 取gripper0 ──");

  CHECK(moveToJoints(kJoints_DockStation, joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("放 gripper2: 关节→笛卡尔"));
  CHECK(releaseGripper(
      {{'y',  kDockSlideY},
       {'z', -(kDockDepth - kDockSeat)},
       {'y', -kDockSlideY},
       {'z', -kDockSeat}},
      {{'z',  kDockLift}},
      kReleaseWaitSec, kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper0: 笛卡尔→关节"));
  CHECK(moveToTargetXYZ(kPos_Gripper0DockX, kPos_Gripper0DockY, kPos_SafeZ,
                        joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper0: 关节→笛卡尔"));
  CHECK(pickGripper({{'z', -kDockDepth}}, {{'z', kDockLift}}, kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("归位: 笛卡尔→关节"));
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

bool GripperSwapWorker::swapToGripper2()
{
  RCLCPP_INFO(get_logger(), "── 放gripper0 → 取gripper2 ──");

  CHECK(moveToJoints(kJoints_ReleaseGripper0, joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("放 gripper0: 关节→笛卡尔"));
  CHECK(releaseGripper({{'z', -kDockDepth}}, {{'z', kDockLift}}, kLockWaitSec, kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper2: 笛卡尔→关节"));
  CHECK(moveToJoints(kJoints_DockStation, joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper2: 关节→笛卡尔"));
  CHECK(pickGripper(
      {{'z', -kDockDepth}},
      {{'z',  kDockSeat}, {'y', kDockSlideY}, {'z', kDockLift}},
      kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("归位: 笛卡尔→关节"));
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

bool GripperSwapWorker::switchToGripper2()
{
  RCLCPP_INFO(get_logger(), "── 无工具 → gripper2 ──");

  CHECK(moveToTargetXYZ(kPos_Gripper2DockX, kPos_Gripper2DockY, kPos_SafeZ,
                        joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper2: 关节→笛卡尔"));
  CHECK(pickGripper(
      {{'z', -kDockDepth}},
      {{'z',  kDockSeat}, {'y', kDockSlideY}, {'z', kDockLift}},
      kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("归位: 笛卡尔→关节"));
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

bool GripperSwapWorker::switchToGripper0()
{
  RCLCPP_INFO(get_logger(), "── 无工具 → gripper0 ──");

  CHECK(moveToTargetXYZ(kPos_Gripper0DockX, kPos_Gripper0DockY, kPos_SafeZ,
                        joint_velocity_scaling_, joint_acceleration_scaling_));

  CHECK(sleepJointCartesianSwitchDelay("取 gripper0: 关节→笛卡尔"));
  CHECK(pickGripper({{'z', -kDockDepth}}, {{'z', kDockLift}}, kLockWaitSec));

  CHECK(sleepJointCartesianSwitchDelay("归位: 笛卡尔→关节"));
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

bool GripperSwapWorker::changeToTool(const std::string& target_id)
{
  RCLCPP_INFO(get_logger(), "changeToTool: %s → %s", current_tool_.id.c_str(), target_id.c_str());

  if (target_id == current_tool_.id)
  {
    RCLCPP_INFO(get_logger(), "已是 %s，跳过切换", target_id.c_str());
    return true;
  }

  bool ok = false;

  if (current_tool_.id == "gripper2" && target_id == "gripper0")
  {
    ok = swapToGripper0();
    if (ok) current_tool_ = kToolGripper0;
  }
  else if (current_tool_.id == "gripper0" && target_id == "gripper2")
  {
    ok = swapToGripper2();
    if (ok) current_tool_ = kToolGripper2;
  }
  else if (current_tool_.id.empty() && target_id == "gripper2")
  {
    ok = switchToGripper2();
    if (ok) current_tool_ = kToolGripper2;
  }
  else if (current_tool_.id.empty() && target_id == "gripper0")
  {
    ok = switchToGripper0();
    if (ok) current_tool_ = kToolGripper0;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "不支持的切换: %s → %s", current_tool_.id.c_str(), target_id.c_str());
    return false;
  }

  // 发布工具状态 → scene_attach_worker 订阅后自动更新 PlanningScene
  publishToolStatus(ok && !current_tool_.id.empty());
  return ok;
}

// ═══════════════════════════════════════════════════════════════════
// 服务回调
// ═══════════════════════════════════════════════════════════════════

GripperSwapWorker::SwapResult GripperSwapWorker::runSwapOperation(std::function<bool()> operation)
{
  bool expected = false;
  if (!swap_in_progress_.compare_exchange_strong(expected, true))
    return SwapResult::Busy;

  bool ok = false;
  try { ok = operation(); }
  catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "快换异常: %s", e.what()); }
  catch (...)                      { RCLCPP_ERROR(get_logger(), "快换异常: 未知"); }

  swap_in_progress_.store(false);
  return ok ? SwapResult::Success : SwapResult::Failed;
}

void GripperSwapWorker::onChangeTool(
    const std::shared_ptr<tool_changer_interface::srv::ChangeTool::Request> request,
    std::shared_ptr<tool_changer_interface::srv::ChangeTool::Response> response)
{
  RCLCPP_INFO(get_logger(), "━━ /change_tool: target=%s ━━", request->tool_id.c_str());

  auto result = runSwapOperation([&]() { return changeToTool(request->tool_id); });

  switch (result) {
    case SwapResult::Busy:
      response->success = false; response->error_code = -1;
      response->message = "快换正忙，拒绝并发"; break;
    case SwapResult::Success:
      response->success = true; response->error_code = 0;
      response->message = "已切换到: " + current_tool_.id + " (" + current_tool_.name + ")"; break;
    case SwapResult::Failed:
      response->success = false; response->error_code = -1;
      response->message = "切换失败: " + request->tool_id; break;
  }
  RCLCPP_INFO(get_logger(), "换刀结果: %s", response->message.c_str());
}

void GripperSwapWorker::onGetCurrentTool(
    const std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Request>,
    std::shared_ptr<tool_changer_interface::srv::GetCurrentTool::Response> response)
{
  response->success         = true;
  response->tool_id         = current_tool_.id;
  response->tool_name       = current_tool_.name;
  response->tool_type       = current_tool_.type;
  response->tool_parameters = current_tool_.parameters;
  response->message = current_tool_.id.empty()
      ? "当前无工具" : ("当前工具: " + current_tool_.id + " (" + current_tool_.name + ")");
}

void GripperSwapWorker::onGripperSwapRequest(
    const std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Request> request,
    std::shared_ptr<tool_changer_interface::srv::RunGripperSwap::Response> response)
{
  static const std::map<std::string, std::string> kDirToTarget = {
      {"gripper0_to_gripper2", "gripper2"},
      {"gripper2_to_gripper0", "gripper0"},
      {"gripper2",            "gripper2"},
  };

  RCLCPP_INFO(get_logger(), "━━ run_gripper_swap: direction=%s ━━", request->direction.c_str());

  auto it = kDirToTarget.find(request->direction);
  if (it == kDirToTarget.end())
  {
    response->success = false;
    response->message = "未知 direction: " + request->direction;
    return;
  }

  auto result = runSwapOperation([&]() { return changeToTool(it->second); });

  switch (result) {
    case SwapResult::Busy:
      response->success = false; response->message = "快换正忙，拒绝并发"; break;
    case SwapResult::Success:
      response->success = true; response->message = "完成: " + request->direction; break;
    case SwapResult::Failed:
      response->success = false; response->message = "失败: " + request->direction; break;
  }
}

// ═══════════════════════════════════════════════════════════════════
// 生命周期
// ═══════════════════════════════════════════════════════════════════

void GripperSwapWorker::run()
{
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(shared_from_this());
  std::thread spinner([&exec]() { exec.spin(); });
  SpinnerJoinGuard join_spinner{ spinner };

  if (simulation_skip_io_)
  {
    RCLCPP_INFO(get_logger(), "仿真模式: 跳过 /aubo_driver/set_io 服务等待");
  }
  else if (!waitForServices(10s))
  {
    RCLCPP_ERROR(get_logger(), "依赖服务未就绪，退出");
    return;
  }

  RCLCPP_INFO(get_logger(), "夹爪快换 Worker 就绪");

  while (rclcpp::ok() && !shutdown_requested_)
    sleepInterruptible(this, 0.5);
}

void GripperSwapWorker::onShutdown()
{
  RCLCPP_INFO(get_logger(), "onShutdown: 回安全位、关 IO");
  moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

void GripperSwapWorker::requestShutdown()
{
  shutdown_requested_ = true;
  RCLCPP_INFO(get_logger(), "收到退出请求");
}

}  // namespace tool_changer

// ═══════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;

  auto node = tool_changer::GripperSwapWorker::create(opts);

  tool_changer::g_worker_for_signal = node.get();
  std::signal(SIGINT, tool_changer::sigintHandler);
  std::signal(SIGTERM, tool_changer::sigintHandler);

  node->run();

  tool_changer::g_worker_for_signal = nullptr;
  node->onShutdown();
  rclcpp::shutdown();
  return 0;
}
