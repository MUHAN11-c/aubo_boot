/**
 * @file gripper_swap_worker.cpp
 * @brief 夹爪快换 Worker — 四类轨迹原语组合，每函数独立常量，独立修改
 *
 *   1. 回 home      — moveToHome
 *   2. 到固定点位    — moveToDockStation / moveToReleaseGripper0 / moveToGripper*DockAbove
 *   3. 取轨迹       — pickGripper0 / pickGripper2
 *   4. 放轨迹       — releaseGripper0 / releaseGripper2
 *
 *   综合流程 = 2 → (3|4) → 1
 */

#include "tool_changer/gripper_swap_worker.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>
#include <std_msgs/msg/string.hpp>
#include <ivg_interfaces/srv/move_to_pose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tool_changer
{
namespace
{

constexpr int    kCartMaxRetries    = 5;
constexpr double kCartRetryWaitSec  = 2.0;
constexpr double kCartInitWaitSec   = 0.2;
constexpr double kCartEefStep       = 0.01;
constexpr double kCartJumpThreshold = 0.0;
constexpr const char* kSetIOService = "/aubo_driver/set_io";
constexpr int    kIOTimeoutSec      = 60;

}  // namespace

// ═══════════════════════════════════════════════════════════════════
// 信号 & 辅助
// ═══════════════════════════════════════════════════════════════════

static GripperSwapWorker* g_worker_for_signal = nullptr;

/** SIGINT/SIGTERM 处理：通知 Worker 优雅退出并 shutdown rclcpp */
static void sigintHandler(int)
{
  if (g_worker_for_signal) g_worker_for_signal->requestShutdown();
  rclcpp::shutdown();
}

/** 可被 shutdown 信号中断的 sleep */
static void sleepInterruptible(GripperSwapWorker* worker, double seconds)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && (!worker || !worker->isShutdownRequested()) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/** 析构时 join spinner 线程，防止 std::terminate */
struct SpinnerJoinGuard
{
  std::thread& th;
  ~SpinnerJoinGuard() { if (th.joinable()) th.join(); }
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

  joint_velocity_scaling_     = static_cast<float>(get_parameter("joint_velocity_scaling").as_double());
  joint_acceleration_scaling_ = static_cast<float>(get_parameter("joint_acceleration_scaling").as_double());
  home_velocity_scaling_      = static_cast<float>(get_parameter("home_velocity_scaling").as_double());
  home_acceleration_scaling_  = static_cast<float>(get_parameter("home_acceleration_scaling").as_double());
  gripper_io_index_           = static_cast<int32_t>(get_parameter("gripper_io_index").as_int());
  simulation_skip_io_         = get_parameter("simulation_skip_io").as_bool();
  joint_cartesian_switch_delay_sec_ = std::max(0.0,
      get_parameter("joint_cartesian_switch_delay_sec").as_double());

  set_io_client_ = create_client<ivg_interfaces::srv::SetRobotIO>(kSetIOService);
  tool_status_pub_ = create_publisher<ivg_interfaces::msg::ToolChangerStatus>(
      "/tool_changer_status", 10);

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  gripper_swap_srv_ = create_service<ivg_interfaces::srv::RunGripperSwap>(
      "/run_gripper_swap",
      std::bind(&GripperSwapWorker::onGripperSwapRequest, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  change_tool_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
      "/change_tool",
      std::bind(&GripperSwapWorker::onChangeTool, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  get_tool_srv_ = create_service<ivg_interfaces::srv::GetCurrentTool>(
      "/get_current_tool",
      std::bind(&GripperSwapWorker::onGetCurrentTool, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  debug_move_xyz_srv_ = create_service<ivg_interfaces::srv::MoveToPose>(
      "/debug/move_to_xyz",
      std::bind(&GripperSwapWorker::onDebugMoveToXYZ, this, _1, _2),
      rmw_qos_profile_services_default, service_cb_group_);

  auto mode_qos = rclcpp::QoS(1).transient_local().reliable();
  mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/aubo/mode", mode_qos,
      [this](const std_msgs::msg::String& msg) {
        if (msg.data == "simulation" && !simulation_skip_io_) {
          simulation_skip_io_ = true;
          RCLCPP_INFO(get_logger(), "[仿真] IO 控制已禁用");
        }
      });

  RCLCPP_INFO(get_logger(),
      "就绪 | vel=%.2f acc=%.2f home_vel=%.2f home_acc=%.2f delay=%.3f io=%d sim=%s",
      joint_velocity_scaling_, joint_acceleration_scaling_,
      home_velocity_scaling_, home_acceleration_scaling_,
      joint_cartesian_switch_delay_sec_, gripper_io_index_,
      simulation_skip_io_ ? "true" : "false");

  publishToolStatus(false);
}

/** 工厂：构造节点并初始化 MoveGroup */
std::shared_ptr<GripperSwapWorker> GripperSwapWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<GripperSwapWorker>(options);
  node->initMoveGroup();
  return node;
}

/** 初始化 MoveGroupInterface，绑定 "manipulator" 规划组 */
void GripperSwapWorker::initMoveGroup()
{
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "manipulator");
  move_group_->allowReplanning(true);
  move_group_->setMaxVelocityScalingFactor(0.5);
  move_group_->setMaxAccelerationScalingFactor(0.5);
}

/** 等待 SetRobotIO 服务就绪，超时返回 false */
bool GripperSwapWorker::waitForServices(std::chrono::seconds timeout)
{
  if (!set_io_client_->wait_for_service(timeout)) {
    RCLCPP_WARN(get_logger(), "服务 %s 未在 %lds 内就绪", kSetIOService, timeout.count());
    return false;
  }
  RCLCPP_INFO(get_logger(), "服务 %s 已就绪", kSetIOService);
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 1 — 回 home
// ═══════════════════════════════════════════════════════════════════

/** 关节空间运动到命名目标 camera_pose */
bool GripperSwapWorker::moveToHome(float vel, float acc)
{
  if (!move_group_) return false;
  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
  move_group_->setNamedTarget("camera_pose");
  return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 2 — 到固定点位
// ═══════════════════════════════════════════════════════════════════

/** 关节空间运动到指定 6 轴关节角 (rad) */
bool GripperSwapWorker::moveToJoints(const std::array<double, 6>& joints, float vel, float acc)
{
  if (!move_group_) return false;
  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
  move_group_->setJointValueTarget(std::vector<double>(joints.begin(), joints.end()));

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "[moveToJoints] 规划失败");
    return false;
  }
  return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

/** 笛卡尔直线平移到目标 XYZ，姿态保持不变 */
bool GripperSwapWorker::moveToTargetXYZ(double target_x, double target_y, double target_z,
                                         float vel, float acc)
{
  const std::string eef_link = move_group_->getEndEffectorLink();
  const auto current_pose = move_group_->getCurrentPose(eef_link).pose;

  std::vector<CartesianSegment> segments;
  constexpr double kMinDeltaM = 1e-9;
  if (std::fabs(target_x - current_pose.position.x) > kMinDeltaM)
    segments.push_back({'x', target_x - current_pose.position.x});
  if (std::fabs(target_y - current_pose.position.y) > kMinDeltaM)
    segments.push_back({'y', target_y - current_pose.position.y});
  if (std::fabs(target_z - current_pose.position.z) > kMinDeltaM)
    segments.push_back({'z', target_z - current_pose.position.z});
  if (segments.empty()) return true;

  RCLCPP_INFO(get_logger(), "moveToXYZ: (%.4f,%.4f,%.4f)", target_x, target_y, target_z);
  return runCartesianPath(segments, vel, acc);
}

/** 关节运动到快换站（两夹爪共用 dock） */
bool GripperSwapWorker::moveToDockStation()
{
  constexpr std::array<double, 6> kJoints = {{
      0.936893, 0.016616, 1.419053, -0.167867, 1.571655, 0.935894}};
  return moveToJoints(kJoints, joint_velocity_scaling_, joint_acceleration_scaling_);
}

/** 关节运动到 gripper0 释放/抓取位 */
bool GripperSwapWorker::moveToReleaseGripper0()
{
  constexpr std::array<double, 6> kJoints = {{
      1.137820, 0.222690, 1.598043, -0.194970, 1.571688, 1.136957}};
  return moveToJoints(kJoints, joint_velocity_scaling_, joint_acceleration_scaling_);
}

/** 笛卡尔平移到 gripper0 工位上方安全点 */
bool GripperSwapWorker::moveToGripper0DockAbove()
{
  constexpr double kX = 0.27017;
  constexpr double kY = 0.29517;
  constexpr double kZ = 0.4755;
  return moveToTargetXYZ(kX, kY, kZ, joint_velocity_scaling_, joint_acceleration_scaling_);
}

/** 笛卡尔平移到 gripper2 工位上方安全点 */
bool GripperSwapWorker::moveToGripper2DockAbove()
{
  constexpr double kX = 0.3741;
  constexpr double kY = 0.30394;
  constexpr double kZ = 0.4755;
  return moveToTargetXYZ(kX, kY, kZ, joint_velocity_scaling_, joint_acceleration_scaling_);
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 3 — 取轨迹
// ═══════════════════════════════════════════════════════════════════

/** gripper0 取：开IO → 垂直下降 → 关IO锁定 → 等待稳定 → 垂直抬起 */
bool GripperSwapWorker::pickGripper0()
{
  constexpr double kDepth     = 0.210;  // 下降深度 (m)
  constexpr double kLift      = 0.210;  // 抬起高度 (m)
  constexpr double kSettleSec = 0.5;    // IO 动作后稳定延时 (s)

  if (!setGripperIoSafe(true)) return false;
  if (!runCartesianPath('z', -kDepth, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  return runCartesianPath('z', kLift, joint_velocity_scaling_, joint_acceleration_scaling_);
}

/** gripper2 取：开IO → 下降 → 关IO锁定 → 微抬脱扣 → 侧滑退出 → 升起 */
bool GripperSwapWorker::pickGripper2()
{
  constexpr double kDepth     = 0.210;  // 下降深度 (m)
  constexpr double kSeat      = 0.012;  // 脱扣微抬高度 (m)
  constexpr double kSlideY    = 0.100;  // 侧滑退出距离 (m)
  constexpr double kLift      = 0.210;  // 最终抬起高度 (m)
  constexpr double kSettleSec = 0.5;    // IO 动作后稳定延时 (s)

  if (!setGripperIoSafe(true)) return false;
  if (!runCartesianPath('z', -kDepth, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  return runCartesianPath({{'z', kSeat}, {'y', kSlideY}, {'z', kLift}},
                          joint_velocity_scaling_, joint_acceleration_scaling_);
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 4 — 放轨迹
// ═══════════════════════════════════════════════════════════════════

/** gripper0 放：垂直下降 → 开IO释放 → 垂直抬起 → 关IO */
bool GripperSwapWorker::releaseGripper0()
{
  constexpr double kDepth     = 0.210;  // 下降深度 (m)
  constexpr double kLift      = 0.210;  // 抬起高度 (m)
  constexpr double kSettleSec = 0.5;    // IO 动作后稳定延时 (s)

  if (!runCartesianPath('z', -kDepth, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(true)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  if (!runCartesianPath('z', kLift, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleSec));
  return true;
}

/** gripper2 放：侧滑进入 → 半降 → 侧回对齐 → 坐到底 → 开IO释放 → 抬起 → 关IO */
bool GripperSwapWorker::releaseGripper2()
{
  constexpr double kDepth      = 0.210;  // 总下降深度 (m)
  constexpr double kSeat       = 0.012;  // 最后坐入深度 (m)
  constexpr double kSlideY     = 0.100;  // 侧滑距离 (m)
  constexpr double kLift       = 0.210;  // 抬起高度 (m)
  constexpr double kReleaseSec = 0.3;    // 释放后稳定延时 (s)
  constexpr double kLockSec    = 0.5;    // 关 IO 后稳定延时 (s)

  if (!runCartesianPath({{'y', kSlideY},
                         {'z', -(kDepth - kSeat)},
                         {'y', -kSlideY},
                         {'z', -kSeat}},
                        joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(true)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kReleaseSec));
  if (!runCartesianPath('z', kLift, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(kLockSec));
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 笛卡尔路径（底层）
// ═══════════════════════════════════════════════════════════════════

/** 单轴笛卡尔路径（委托多段版本） */
bool GripperSwapWorker::runCartesianPath(char axis, double offset, float vel, float acc)
{
  return runCartesianPath({{axis, offset}}, vel, acc);
}

/** 多段笛卡尔路点一次 computeCartesianPath + execute，最多重试 kCartMaxRetries 次 */
bool GripperSwapWorker::runCartesianPath(const std::vector<CartesianSegment>& segments,
                                         float vel, float acc)
{
  if (!move_group_ || segments.empty()) return false;

  if (kCartInitWaitSec > 0.0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kCartInitWaitSec));

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kCartMaxRetries; ++attempt)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->setMaxVelocityScalingFactor(vel);
    move_group_->setMaxAccelerationScalingFactor(acc);

    const std::string eef_link = move_group_->getEndEffectorLink();
    geometry_msgs::msg::Pose waypoint = move_group_->getCurrentPose(eef_link).pose;
    std::vector<geometry_msgs::msg::Pose> waypoints = { waypoint };

    for (const auto& seg : segments)
    {
      if (seg.axis == 'x')      waypoint.position.x += seg.offset;
      else if (seg.axis == 'y') waypoint.position.y += seg.offset;
      else                      waypoint.position.z += seg.offset;
      waypoints.push_back(waypoint);
    }

    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartEefStep, kCartJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔: %.1f%% (第%d/%d次)", fraction * 100.0, attempt, kCartMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
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

/** 设置快换 IO：仿真模式跳过，真实模式委托 setGripperIo */
bool GripperSwapWorker::setGripperIoSafe(bool open_gripper)
{
  if (simulation_skip_io_) {
    RCLCPP_INFO(get_logger(), "[仿真] 跳过 IO(%d, %s)", gripper_io_index_,
                open_gripper ? "开" : "关");
    return true;
  }
  return setGripperIo(gripper_io_index_, open_gripper);
}

/** 调用 /aubo_driver/set_io 服务，digital_output 模式，超时 kIOTimeoutSec 秒 */
bool GripperSwapWorker::setGripperIo(int32_t io_index, bool high)
{
  if (!set_io_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "IO 服务不可用");
    return false;
  }

  auto req = std::make_shared<ivg_interfaces::srv::SetRobotIO::Request>();
  req->io_type  = "digital_output";
  req->io_index = io_index;
  req->value    = high ? 1.0 : 0.0;

  auto future = set_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kIOTimeoutSec)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "IO(%d, %s) 超时", io_index, high ? "HIGH" : "LOW");
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    RCLCPP_ERROR(get_logger(), "IO 失败: %s", res->message.c_str());
    return false;
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 工具状态
// ═══════════════════════════════════════════════════════════════════

/** 发布 /tool_changer_status，scene_attach_worker 订阅后自动更新 PlanningScene */
void GripperSwapWorker::publishToolStatus(bool connected)
{
  auto msg = ivg_interfaces::msg::ToolChangerStatus();
  msg.header.stamp    = now();
  msg.header.frame_id = "tool_changer";
  msg.tool_id         = current_tool_.id;
  msg.tool_name       = current_tool_.name;
  msg.tool_type       = current_tool_.type;
  msg.is_connected    = connected;
  msg.tool_parameters = current_tool_.parameters;
  tool_status_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "工具: id=%s name=%s connected=%s",
              current_tool_.id.c_str(), current_tool_.name.c_str(), connected ? "true" : "false");
}

// ═══════════════════════════════════════════════════════════════════
// 关节↔笛卡尔切换延时
// ═══════════════════════════════════════════════════════════════════

/** 关节与笛卡尔运动切换前插入短暂延时，避免状态未及时刷新导致规划异常；可被 shutdown 中断 */
bool GripperSwapWorker::sleepJointCartesianSwitchDelay(const char* where)
{
  if (joint_cartesian_switch_delay_sec_ <= 0.0) return true;
  RCLCPP_INFO(get_logger(), "%s: 延时 %.3fs", where, joint_cartesian_switch_delay_sec_);
  sleepInterruptible(this, joint_cartesian_switch_delay_sec_);
  return rclcpp::ok() && !shutdown_requested_;
}

// ═══════════════════════════════════════════════════════════════════
// 综合流程 — 四类轨迹原语组合
// ═══════════════════════════════════════════════════════════════════

/** 放 gripper2 → 取 gripper0: DockStation → releaseGripper2 → ReleaseGripper0 → pickGripper0 → home */
bool GripperSwapWorker::swapToGripper0()
{
  RCLCPP_INFO(get_logger(), "── 放gripper2 → 取gripper0 ──");

  if (!moveToDockStation())     { RCLCPP_ERROR(get_logger(), "moveToDockStation 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("放gripper2: J→C")) return false;
  if (!releaseGripper2())       { RCLCPP_ERROR(get_logger(), "releaseGripper2 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("取gripper0: C→J")) return false;
  if (!moveToReleaseGripper0()) { RCLCPP_ERROR(get_logger(), "moveToReleaseGripper0 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("取gripper0: J→C")) return false;
  if (!pickGripper0())          { RCLCPP_ERROR(get_logger(), "pickGripper0 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("归位: C→J")) return false;
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

/** 放 gripper0 → 取 gripper2: ReleaseGripper0 → releaseGripper0 → DockStation → pickGripper2 → home */
bool GripperSwapWorker::swapToGripper2()
{
  RCLCPP_INFO(get_logger(), "── 放gripper0 → 取gripper2 ──");

  if (!moveToReleaseGripper0()) { RCLCPP_ERROR(get_logger(), "moveToReleaseGripper0 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("放gripper0: J→C")) return false;
  if (!releaseGripper0())       { RCLCPP_ERROR(get_logger(), "releaseGripper0 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("取gripper2: C→J")) return false;
  if (!moveToDockStation())     { RCLCPP_ERROR(get_logger(), "moveToDockStation 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("取gripper2: J→C")) return false;
  if (!pickGripper2())          { RCLCPP_ERROR(get_logger(), "pickGripper2 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("归位: C→J")) return false;
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

/** 无工具 → gripper2: DockStation → pickGripper2 → home */
bool GripperSwapWorker::switchToGripper2()
{
  RCLCPP_INFO(get_logger(), "── 无工具 → gripper2 ──");

  if (!moveToDockStation())     { RCLCPP_ERROR(get_logger(), "moveToDockStation 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("取gripper2: J→C")) return false;
  if (!pickGripper2())          { RCLCPP_ERROR(get_logger(), "pickGripper2 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("归位: C→J")) return false;
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

/** 无工具 → gripper0: ReleaseGripper0 → pickGripper0 → home */
bool GripperSwapWorker::switchToGripper0()
{
  RCLCPP_INFO(get_logger(), "── 无工具 → gripper0 ──");

  if (!moveToReleaseGripper0()) { RCLCPP_ERROR(get_logger(), "moveToReleaseGripper0 失败"); return false; }
  if (!sleepJointCartesianSwitchDelay("取gripper0: J→C")) return false;
  if (!pickGripper0())          { RCLCPP_ERROR(get_logger(), "pickGripper0 失败"); return false; }

  if (!sleepJointCartesianSwitchDelay("归位: C→J")) return false;
  return moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

/** 根据当前工具和目标工具选择对应的综合流程 */
bool GripperSwapWorker::changeToTool(const std::string& target_id)
{
  RCLCPP_INFO(get_logger(), "changeToTool: %s → %s", current_tool_.id.c_str(), target_id.c_str());

  if (target_id == current_tool_.id) {
    RCLCPP_INFO(get_logger(), "已是 %s，跳过", target_id.c_str());
    return true;
  }

  // 切换开始前清除夹爪状态: 让 scene_attach_worker 脱离碰撞模型,
  // 避免运动过程中附着的夹爪网格与机械臂自身发生碰撞导致规划失败
  publishToolStatus(false);

  bool ok = false;

  if (current_tool_.id == "gripper2" && target_id == "gripper0") {
    ok = swapToGripper0();
    if (ok) current_tool_ = kToolGripper0;
  } else if (current_tool_.id == "gripper0" && target_id == "gripper2") {
    ok = swapToGripper2();
    if (ok) current_tool_ = kToolGripper2;
  } else if (current_tool_.id.empty() && target_id == "gripper2") {
    ok = switchToGripper2();
    if (ok) current_tool_ = kToolGripper2;
  } else if (current_tool_.id.empty() && target_id == "gripper0") {
    ok = switchToGripper0();
    if (ok) current_tool_ = kToolGripper0;
  } else {
    RCLCPP_ERROR(get_logger(), "不支持: %s → %s", current_tool_.id.c_str(), target_id.c_str());
    return false;
  }

  publishToolStatus(ok && !current_tool_.id.empty());
  return ok;
}

// ═══════════════════════════════════════════════════════════════════
// 服务回调
// ═══════════════════════════════════════════════════════════════════

/** /change_tool 服务回调：同步执行快换并返回结果 */
void GripperSwapWorker::onChangeTool(
    const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> request,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> response)
{
  RCLCPP_INFO(get_logger(), "━━ /change_tool: target=%s ━━", request->tool_id.c_str());

  bool ok = false;
  try { ok = changeToTool(request->tool_id); }
  catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "异常: %s", e.what()); }

  response->success    = ok;
  response->error_code = ok ? 0 : -1;
  response->message    = ok ? ("已切换到: " + current_tool_.id)
                            : ("切换失败: " + request->tool_id);
}

/** /get_current_tool 服务回调：返回当前挂载的工具信息 */
void GripperSwapWorker::onGetCurrentTool(
    const std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Request>,
    std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Response> response)
{
  response->success         = true;
  response->tool_id         = current_tool_.id;
  response->tool_name       = current_tool_.name;
  response->tool_type       = current_tool_.type;
  response->tool_parameters = current_tool_.parameters;
  response->message         = current_tool_.id.empty()
      ? "当前无工具" : ("当前工具: " + current_tool_.id);
}

/** /run_gripper_swap 服务回调：将 direction 映射为目标夹爪后执行切换 */
void GripperSwapWorker::onGripperSwapRequest(
    const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> request,
    std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> response)
{
  static const std::map<std::string, std::string> kDirToTarget = {
      {"gripper0_to_gripper2", "gripper2"},
      {"gripper2_to_gripper0", "gripper0"},
      {"gripper2",             "gripper2"},
  };

  RCLCPP_INFO(get_logger(), "━━ run_gripper_swap: direction=%s ━━", request->direction.c_str());

  auto it = kDirToTarget.find(request->direction);
  if (it == kDirToTarget.end()) {
    response->success = false;
    response->message = "未知 direction: " + request->direction;
    return;
  }

  bool ok = false;
  try { ok = changeToTool(it->second); }
  catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "异常: %s", e.what()); }

  response->success = ok;
  response->message = ok ? ("完成: " + request->direction) : ("失败: " + request->direction);
}

// ═══════════════════════════════════════════════════════════════════
// 调试 — 笛卡尔直线运动
// ═══════════════════════════════════════════════════════════════════

/** /debug/move_to_xyz 服务回调：手动测试笛卡尔平移到指定 XYZ */
void GripperSwapWorker::onDebugMoveToXYZ(
    const std::shared_ptr<ivg_interfaces::srv::MoveToPose::Request> req,
    std::shared_ptr<ivg_interfaces::srv::MoveToPose::Response> resp)
{
  if (!move_group_) {
    resp->success = false; resp->error_code = -1;
    resp->message = "MoveGroup 未初始化"; return;
  }
  float vel = std::max(0.05f, std::min(1.0f, req->velocity_factor));
  float acc = std::max(0.05f, std::min(1.0f, req->acceleration_factor));

  bool ok = moveToTargetXYZ(req->target_pose.position.x,
                            req->target_pose.position.y,
                            req->target_pose.position.z, vel, acc);
  resp->success    = ok;
  resp->error_code = ok ? 0 : -2;
  resp->message    = ok ? "完成" : "失败";
}

// ═══════════════════════════════════════════════════════════════════
// 生命周期
// ═══════════════════════════════════════════════════════════════════

/** 主循环：等待 /aubo/mode → 启动 MultiThreadedExecutor(2线程) → 阻塞等待退出信号 */
void GripperSwapWorker::run()
{
  // 等待 /aubo/mode 消息（最长 8s），超时默认非仿真
  auto t0 = std::chrono::steady_clock::now();
  while (!simulation_skip_io_ &&
         std::chrono::steady_clock::now() - t0 < std::chrono::seconds(8)) {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(shared_from_this());
  std::thread spinner([&exec]() { exec.spin(); });
  SpinnerJoinGuard join_spinner{ spinner };

  if (!simulation_skip_io_ && !waitForServices(10s)) {
    RCLCPP_ERROR(get_logger(), "依赖服务未就绪，退出");
    return;
  }

  RCLCPP_INFO(get_logger(), "夹爪快换 Worker 就绪");

  while (rclcpp::ok() && !shutdown_requested_)
    sleepInterruptible(this, 0.5);
}

/** 退出前回 home */
void GripperSwapWorker::onShutdown()
{
  RCLCPP_INFO(get_logger(), "onShutdown: 回 home");
  moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

/** 标记退出请求，run() 主循环检测到后退出 */
void GripperSwapWorker::requestShutdown()
{
  shutdown_requested_ = true;
}

}  // namespace tool_changer

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;

  auto node = tool_changer::GripperSwapWorker::create(opts);

  tool_changer::g_worker_for_signal = node.get();
  std::signal(SIGINT,  tool_changer::sigintHandler);
  std::signal(SIGTERM, tool_changer::sigintHandler);

  node->run();

  tool_changer::g_worker_for_signal = nullptr;
  node->onShutdown();
  rclcpp::shutdown();
  return 0;
}
