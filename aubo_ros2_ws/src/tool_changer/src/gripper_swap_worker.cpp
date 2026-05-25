/**
 * @file gripper_swap_worker.cpp
 * @brief 夹爪快换 Worker — 数据驱动，轨迹参数全部来自 tools.yaml
 *
 *   1. 回 home           — moveToHome
 *   2. 到固定点位         — moveToJoints / moveToTargetXYZ / moveToDockApproach
 *   3. 取轨迹（泛型）      — pickTool(config) 按 strategy 分发
 *   4. 放轨迹（泛型）      — releaseTool(config) 按 strategy 分发
 *
 *   综合流程 = 释放当前 → 取目标 → 回 home（无硬编码分支）
 */

#include "tool_changer/gripper_swap_worker.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>
#include <yaml-cpp/yaml.h>
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
constexpr const char* kSetIOService = "/set_robot_io";
constexpr const char* kSceneAttachService = "/scene_attach";
constexpr const char* kSceneDetachService = "/scene_detach";
constexpr int    kIOTimeoutSec      = 60;
constexpr int    kSceneTimeoutSec   = 5;

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
// 构造
// ═══════════════════════════════════════════════════════════════════

GripperSwapWorker::GripperSwapWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("gripper_swap_worker", options)
{
  // RobotController 构造安全（只存指针），move_group_ 由 robot_->init() 延后创建
  robot_ = std::make_shared<demo_driver::RobotController>(this);
  move_group_ = nullptr;

  set_io_client_ = create_client<ivg_interfaces::srv::SetRobotIO>(kSetIOService);
  scene_attach_client_ = create_client<ivg_interfaces::srv::ChangeTool>(kSceneAttachService);
  scene_detach_client_ = create_client<ivg_interfaces::srv::ChangeTool>(kSceneDetachService);

  // launch 可能通过 automatically_declare_parameters_from_overrides 注入参数，用 has_parameter() 守卫避免重复声明喵~
  if (!has_parameter("joint_velocity_scaling"))
    declare_parameter("joint_velocity_scaling", 0.7);
  if (!has_parameter("joint_acceleration_scaling"))
    declare_parameter("joint_acceleration_scaling", 0.3);
  if (!has_parameter("home_velocity_scaling"))
    declare_parameter("home_velocity_scaling", 0.7);
  if (!has_parameter("home_acceleration_scaling"))
    declare_parameter("home_acceleration_scaling", 0.3);
  if (!has_parameter("gripper_io_index"))
    declare_parameter("gripper_io_index", 7);
  if (!has_parameter("joint_cartesian_switch_delay_sec"))
    declare_parameter("joint_cartesian_switch_delay_sec", 0.05);
  if (!has_parameter("simulation_skip_io"))
    declare_parameter("simulation_skip_io", false);
  if (!has_parameter("initial_tool_id"))
    declare_parameter("initial_tool_id", "");

  joint_velocity_scaling_     = static_cast<float>(get_parameter("joint_velocity_scaling").as_double());
  joint_acceleration_scaling_ = static_cast<float>(get_parameter("joint_acceleration_scaling").as_double());
  home_velocity_scaling_      = static_cast<float>(get_parameter("home_velocity_scaling").as_double());
  home_acceleration_scaling_  = static_cast<float>(get_parameter("home_acceleration_scaling").as_double());
  gripper_io_index_           = static_cast<int32_t>(get_parameter("gripper_io_index").as_int());
  simulation_skip_io_         = get_parameter("simulation_skip_io").as_bool();
  joint_cartesian_switch_delay_sec_ = std::max(0.0,
      get_parameter("joint_cartesian_switch_delay_sec").as_double());

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

  loadToolConfig();

  // 检查是否配置了初始工具（因为末端夹爪类型没有硬件反馈）喵~
  std::string initial_tool_id = get_parameter("initial_tool_id").as_string();
  if (!initial_tool_id.empty()) {
    auto it = tool_configs_.find(initial_tool_id);
    if (it != tool_configs_.end()) {
      current_tool_.id         = it->second.id;
      current_tool_.name       = it->second.name;
      current_tool_.type       = it->second.type;
      current_tool_.parameters = it->second.parameters;
      RCLCPP_INFO(get_logger(), "启动时设定初始工具: %s (%s)",
                  current_tool_.id.c_str(), current_tool_.name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "initial_tool_id '%s' 不在 tools.yaml 中，将以无工具状态启动",
                  initial_tool_id.c_str());
    }
  }

  RCLCPP_INFO(get_logger(),
      "就绪 | vel=%.2f acc=%.2f home_vel=%.2f home_acc=%.2f delay=%.3f io=%d sim=%s tools=%zu",
      joint_velocity_scaling_, joint_acceleration_scaling_,
      home_velocity_scaling_, home_acceleration_scaling_,
      joint_cartesian_switch_delay_sec_, gripper_io_index_,
      simulation_skip_io_ ? "true" : "false", tool_configs_.size());

  // 周期性发布 /tool_changer_status（默认 VOLATILE QoS 不保留历史，后连接的前端需定时推送）喵~
  // 首次立即发布，之后每 5 秒重发，确保后连接的 rosbridge 订阅者最多等 5 秒获知状态喵~
  publishToolStatus(!current_tool_.id.empty());
  status_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      [this]() { publishToolStatus(!current_tool_.id.empty()); });
}

/** 工厂：构造节点并两阶段初始化 RobotController */
std::shared_ptr<GripperSwapWorker> GripperSwapWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<GripperSwapWorker>(options);
  node->robot_->init();
  node->move_group_ = node->robot_->moveGroup();
  return node;
}

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

/** 关节空间运动到命名目标 camera_pose — 委托 RobotController */
bool GripperSwapWorker::moveToHome(float vel, float acc)
{
  if (!robot_) return false;
  return robot_->moveToHome(vel, acc);
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 2 — 到固定点位
// ═══════════════════════════════════════════════════════════════════

/** 关节空间运动到指定 6 轴关节角 (rad) — 委托 RobotController */
bool GripperSwapWorker::moveToJoints(const std::array<double, 6>& joints, float vel, float acc)
{
  if (!robot_) return false;
  return robot_->moveToJoints(joints, vel, acc);
}

/** 笛卡尔直线平移到目标 XYZ，姿态保持不变 */
bool GripperSwapWorker::moveToTargetXYZ(double target_x, double target_y, double target_z,
                                         float vel, float acc)
{
  if (!robot_) return false;
  const auto current_pose = robot_->getCurrentPose();

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
  return robot_->moveCartesianPath(segments, vel, acc);
}

/** 运动到工具 dock 接近位（关节角优先，否则笛卡尔直线） */
bool GripperSwapWorker::moveToDockApproach(const ToolConfig& tool)
{
  if (!robot_) return false;
  if (tool.has_dock_approach_xyz) {
    return moveToTargetXYZ(tool.dock_approach_xyz[0],
                           tool.dock_approach_xyz[1],
                           tool.dock_approach_xyz[2],
                           joint_velocity_scaling_, joint_acceleration_scaling_);
  }
  return robot_->moveToJoints(tool.dock_approach_joints,
                      joint_velocity_scaling_, joint_acceleration_scaling_);
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 3 — 取轨迹（泛型，按 strategy 分发）
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::pickTool(const ToolConfig& tool)
{
  if (!robot_) return false;
  if (tool.strategy == TrajectoryStrategy::kSlide)
  {
    const auto& p = tool.slide;
    if (!setGripperIoSafe(true)) return false;
    if (!robot_->moveCartesianPath({{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
    std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
    if (!setGripperIoSafe(false)) return false;
    std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
    return robot_->moveCartesianPath({{'z', p.seat}, {'y', p.slide_y}, {'z', p.lift}},
                            joint_velocity_scaling_, joint_acceleration_scaling_);
  }
  // vertical (default)
  const auto& p = tool.vertical;
  if (!setGripperIoSafe(true)) return false;
  if (!robot_->moveCartesianPath({{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  return robot_->moveCartesianPath({{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_);
}

// ═══════════════════════════════════════════════════════════════════
// 轨迹原语 4 — 放轨迹（泛型，按 strategy 分发）
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::releaseTool(const ToolConfig& tool, bool* tool_released)
{
  if (tool_released) *tool_released = false;
  if (!robot_) return false;
  if (tool.strategy == TrajectoryStrategy::kSlide)
  {
    const auto& p = tool.slide;
    if (!robot_->moveCartesianPath({{'y', p.slide_y},
                           {'z', -(p.depth - p.seat)},
                           {'y', -p.slide_y},
                           {'z', -p.seat}},
                          joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
    if (!setGripperIoSafe(true)) return false;
    if (tool_released) *tool_released = true;
    std::this_thread::sleep_for(std::chrono::duration<double>(p.release_sec));
    if (!robot_->moveCartesianPath({{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
    if (!setGripperIoSafe(false)) return false;
    std::this_thread::sleep_for(std::chrono::duration<double>(p.lock_sec));
    return true;
  }
  // vertical (default)
  const auto& p = tool.vertical;
  if (!robot_->moveCartesianPath({{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(true)) return false;
  if (tool_released) *tool_released = true;
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  if (!robot_->moveCartesianPath({{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_)) return false;
  if (!setGripperIoSafe(false)) return false;
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 笛卡尔路径（底层）
// ═══════════════════════════════════════════════════════════════════

/** 单轴笛卡尔路径（委托多段版本） */
bool GripperSwapWorker::runCartesianPath(char axis, double offset, float vel, float acc)
{
  return robot_->moveCartesianPath({{axis, offset}}, vel, acc);
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
    const auto current_pose = move_group_->getCurrentPose(eef_link);
    geometry_msgs::msg::Pose waypoint = current_pose.pose;
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

/** 设置快换 IO：仿真模式跳过，真实模式委托 RobotController */
bool GripperSwapWorker::setGripperIoSafe(bool open_gripper)
{
  if (!robot_) return false;
  if (simulation_skip_io_) {
    RCLCPP_INFO(get_logger(), "[仿真] 跳过 IO(%d, %s)", gripper_io_index_,
                open_gripper ? "开" : "关");
    return true;
  }
  return robot_->setGripper(gripper_io_index_, open_gripper);
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

bool GripperSwapWorker::updateSceneAttachment(const std::string& tool_id, bool attached)
{
  // 委托 scene_attach_worker：ACO 走 /attached_collision_object，detach 顺带 /planning_scene world REMOVE喵~
  auto client = attached ? scene_attach_client_ : scene_detach_client_;
  const char* service_name = attached ? kSceneAttachService : kSceneDetachService;
  if (!client->wait_for_service(std::chrono::seconds(kSceneTimeoutSec))) {
    RCLCPP_ERROR(get_logger(), "%s 服务未就绪，无法%s %s 的规划场景碰撞",
                 service_name, attached ? "附着" : "移除", tool_id.c_str());
    return false;
  }

  auto req = std::make_shared<ivg_interfaces::srv::ChangeTool::Request>();
  req->tool_id = tool_id;
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kSceneTimeoutSec)) != std::future_status::ready) {
    RCLCPP_ERROR(get_logger(), "%s 调用超时: %s", service_name, tool_id.c_str());
    return false;
  }
  auto res = future.get();
  if (!res->success) {
    RCLCPP_ERROR(get_logger(), "%s 调用失败: %s", service_name, res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "规划场景碰撞%s: %s",
              attached ? "附着" : "移除", tool_id.c_str());
  return true;
}

// ═══════════════════════════════════════════════════════════════════
// 工具状态
// ═══════════════════════════════════════════════════════════════════

/** 发布 /tool_changer_status；scene_attach_worker 订阅后同步 ACO +（必要时）world REMOVE喵~ */
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
// 配置加载 — 从 tools.yaml 读取所有工具的轨迹参数
// ═══════════════════════════════════════════════════════════════════

void GripperSwapWorker::loadToolConfig()
{
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("tool_changer")
                + "/config/tools.yaml";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径获取失败: %s", e.what());
    return;
  }

  YAML::Node config;
  try { config = YAML::LoadFile(config_path); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "YAML 加载失败 %s: %s", config_path.c_str(), e.what());
    return;
  }

  auto tools = config["tools"];
  if (!tools) {
    RCLCPP_ERROR(get_logger(), "tools.yaml 缺少 'tools' 节点");
    return;
  }

  for (const auto& kv : tools)
  {
    std::string tid = kv.first.as<std::string>();
    const auto& t = kv.second;
    ToolConfig cfg;

    cfg.id         = tid;
    cfg.name       = t["name"].as<std::string>("");
    cfg.type       = t["type"].as<std::string>("");
    cfg.parameters = t["parameters"].as<std::string>("");

    // dock_approach_joints（优先）或 dock_above XYZ 笛卡尔直线（回退）
    if (t["dock_approach_joints"] && t["dock_approach_joints"].size() == 6) {
      for (int i = 0; i < 6; ++i)
        cfg.dock_approach_joints[i] = t["dock_approach_joints"][i].as<double>();
    } else if (t["dock_above"] && t["dock_above"].size() == 3) {
      cfg.has_dock_approach_xyz = true;
      cfg.dock_approach_xyz[0] = t["dock_above"]["x"].as<double>();
      cfg.dock_approach_xyz[1] = t["dock_above"]["y"].as<double>();
      cfg.dock_approach_xyz[2] = t["dock_above"]["z"].as<double>();
    } else {
      RCLCPP_WARN(get_logger(), "工具 %s: 缺少 dock_approach_joints (6关节角) 或 dock_above (XYZ坐标)", tid.c_str());
      continue;
    }

    // trajectory strategy + params
    if (t["trajectory"]) {
      const auto& tp = t["trajectory"];
      std::string strat = tp["strategy"].as<std::string>("vertical");
      if (strat == "slide") {
        cfg.strategy = TrajectoryStrategy::kSlide;
        cfg.slide.depth       = tp["depth"].as<double>(cfg.slide.depth);
        cfg.slide.lift        = tp["lift"].as<double>(cfg.slide.lift);
        cfg.slide.slide_y     = tp["slide_y"].as<double>(cfg.slide.slide_y);
        cfg.slide.seat        = tp["seat"].as<double>(cfg.slide.seat);
        cfg.slide.settle_sec  = tp["settle_sec"].as<double>(cfg.slide.settle_sec);
        cfg.slide.release_sec = tp["release_sec"].as<double>(cfg.slide.release_sec);
        cfg.slide.lock_sec    = tp["lock_sec"].as<double>(cfg.slide.lock_sec);
      } else {
        cfg.vertical.depth      = tp["depth"].as<double>(cfg.vertical.depth);
        cfg.vertical.lift       = tp["lift"].as<double>(cfg.vertical.lift);
        cfg.vertical.settle_sec = tp["settle_sec"].as<double>(cfg.vertical.settle_sec);
      }
    }

    tool_configs_[tid] = cfg;
    if (cfg.has_dock_approach_xyz) {
      RCLCPP_INFO(get_logger(), "工具轨迹: %s strategy=%s dock_above=[%.3f,%.3f,%.3f] (笛卡尔直线)",
        tid.c_str(),
        (cfg.strategy == TrajectoryStrategy::kSlide) ? "slide" : "vertical",
        cfg.dock_approach_xyz[0], cfg.dock_approach_xyz[1], cfg.dock_approach_xyz[2]);
    } else {
      RCLCPP_INFO(get_logger(), "工具轨迹: %s strategy=%s joints=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
        tid.c_str(),
        (cfg.strategy == TrajectoryStrategy::kSlide) ? "slide" : "vertical",
        cfg.dock_approach_joints[0], cfg.dock_approach_joints[1],
        cfg.dock_approach_joints[2], cfg.dock_approach_joints[3],
        cfg.dock_approach_joints[4], cfg.dock_approach_joints[5]);
    }
  }
  RCLCPP_INFO(get_logger(), "共加载 %zu 个工具轨迹配置", tool_configs_.size());
}

// ═══════════════════════════════════════════════════════════════════
// 综合流程：释放当前 → 取目标 → 回 home（数据驱动，无硬编码分支）
// ═══════════════════════════════════════════════════════════════════

bool GripperSwapWorker::changeToTool(const std::string& target_id)
{
  auto target_it = tool_configs_.find(target_id);
  if (target_it == tool_configs_.end()) {
    RCLCPP_ERROR(get_logger(), "未知工具: %s", target_id.c_str());
    return false;
  }
  const auto& target = target_it->second;

  RCLCPP_INFO(get_logger(), "changeToTool: %s → %s", current_tool_.id.c_str(), target_id.c_str());

  if (target_id == current_tool_.id) {
    RCLCPP_INFO(get_logger(), "已是 %s，跳过", target_id.c_str());
    return true;
  }

  // 1. 释放当前工具喵~
  if (!current_tool_.id.empty()) {
    auto current_it = tool_configs_.find(current_tool_.id);
    if (current_it == tool_configs_.end()) {
      RCLCPP_ERROR(get_logger(), "当前工具 %s 不在配置中", current_tool_.id.c_str());
      return false;
    }
    const auto& current = current_it->second;

    if (!moveToDockApproach(current)) {
      RCLCPP_ERROR(get_logger(), "释放 %s: dock approach 失败", current.id.c_str());
      return false;
    }
    if (!sleepJointCartesianSwitchDelay("释放: J→C")) return false;
    if (!updateSceneAttachment(current.id, false)) {
      RCLCPP_ERROR(get_logger(), "释放 %s: 提前移除规划场景碰撞失败", current.id.c_str());
      return false;
    }
    // 立即清除 current_tool_ 并发布空状态，防止周期性定时器（5s）在
    // releaseTool 执行期间发布旧工具 ID，导致 scene_attach_worker 重新附着已脱离的 ACO
    current_tool_ = ToolInfo{};
    publishToolStatus(false);

    bool tool_released = false;
    if (!releaseTool(current, &tool_released)) {
      if (!tool_released) {
        // 工具仍物理连接时恢复状态和碰撞体喵~
        current_tool_ = {current.id, current.name, current.type, current.parameters};
        publishToolStatus(true);
        (void)updateSceneAttachment(current.id, true);
      }
      RCLCPP_ERROR(get_logger(), "释放 %s 失败", current.id.c_str());
      return false;
    }
    if (!sleepJointCartesianSwitchDelay("释放→取: C→J")) return false;
  }

  // 2. 取目标工具喵~
  if (!moveToDockApproach(target)) {
    RCLCPP_ERROR(get_logger(), "取 %s: dock approach 失败", target.id.c_str());
    return false;
  }
  if (!sleepJointCartesianSwitchDelay("取: J→C")) return false;
  if (!pickTool(target)) {
    RCLCPP_ERROR(get_logger(), "取 %s 失败", target.id.c_str());
    return false;
  }

  // pickTool 完成时夹爪已物理锁紧，立即更新状态（不等 home）喵~
  current_tool_.id         = target.id;
  current_tool_.name       = target.name;
  current_tool_.type       = target.type;
  current_tool_.parameters = target.parameters;
  publishToolStatus(true);

  // 3. 回 home 喵~
  if (!sleepJointCartesianSwitchDelay("归位: C→J")) return false;
  if (!robot_->moveToHome(home_velocity_scaling_, home_acceleration_scaling_)) {
    RCLCPP_ERROR(get_logger(), "归位失败");
    return false;
  }

  return true;
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

/** /run_gripper_swap 服务回调：通用方向解析 → changeToTool */
void GripperSwapWorker::onGripperSwapRequest(
    const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> request,
    std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> response)
{
  // 通用解析: "gripper0_to_gripper2" → source=gripper0, target=gripper2
  //           "gripper2"           → target=gripper2 (无源工具)
  std::string source_id, target_id;
  auto pos = request->direction.find("_to_");
  if (pos != std::string::npos) {
    source_id = request->direction.substr(0, pos);
    target_id = request->direction.substr(pos + 4);
  } else {
    target_id = request->direction;
  }

  RCLCPP_INFO(get_logger(), "━━ run_gripper_swap: direction=%s source=%s target=%s ━━",
              request->direction.c_str(), source_id.c_str(), target_id.c_str());

  if (tool_configs_.find(target_id) == tool_configs_.end()) {
    response->success = false;
    response->message = "未知 direction 或工具: " + request->direction;
    return;
  }

  // 检查源工具合法性 ("_to_" 格式中包含源工具时)
  if (!source_id.empty() && tool_configs_.find(source_id) == tool_configs_.end()) {
    response->success = false;
    response->message = "未知源工具: " + source_id;
    return;
  }

  // 后端无当前工具但请求指定了源工具 → 用源工具填充 current_tool_,
  // 确保 changeToTool 能正确执行释放步骤喵~
  // (仿真模式下用户手动选择工具, 后端 current_tool_ 为空, 必须从 direction 获知源工具)
  const bool fill_source = current_tool_.id.empty() && !source_id.empty();
  if (fill_source) {
    const auto& src = tool_configs_[source_id];
    current_tool_.id         = src.id;
    current_tool_.name       = src.name;
    current_tool_.type       = src.type;
    current_tool_.parameters = src.parameters;
    RCLCPP_INFO(get_logger(), "[swap] 从 direction 填充源工具: %s", source_id.c_str());
    publishToolStatus(true);
  }

  bool ok = false;
  try { ok = changeToTool(target_id); }
  catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "异常: %s", e.what()); }

  // 从 direction 填充的源工具且 changeToTool 失败 → 回退 current_tool_
  if (!ok && fill_source) {
    current_tool_ = ToolInfo{};
    publishToolStatus(false);
  }

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
  robot_->moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
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
