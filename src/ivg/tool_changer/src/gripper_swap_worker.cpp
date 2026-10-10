// 夹爪快换 Worker 实现 — 源自 aubo_boot tool_changer（2026-09-30 移植）。
// 适配点见头文件注释；综合流程与轨迹参数保真。
#include "tool_changer/gripper_swap_worker.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <thread>

using namespace std::chrono_literals;

namespace tool_changer
{
namespace
{

constexpr const char * kSceneAttachService = "/scene_attach";
constexpr const char * kSceneDetachService = "/scene_detach";
constexpr int kSceneTimeoutSec = 5;

}  // namespace

// ═════════════════════════════════ 信号 & 辅助 ═════════════════════════════════

static GripperSwapWorker * g_worker_for_signal = nullptr;

static void sigintHandler(int)
{
  if (g_worker_for_signal) {g_worker_for_signal->requestShutdown();}
  rclcpp::shutdown();
}

static void sleepInterruptible(GripperSwapWorker * worker, double seconds)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && (!worker || !worker->isShutdownRequested()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

struct SpinnerJoinGuard
{
  std::thread & th;
  ~SpinnerJoinGuard()
  {
    if (th.joinable()) {th.join();}
  }
};

// ═════════════════════════════════ 构造 ═════════════════════════════════

GripperSwapWorker::GripperSwapWorker(const rclcpp::NodeOptions & options)
: rclcpp::Node("gripper_swap_worker", options)
{
  scene_attach_client_ = create_client<ivg_interfaces::srv::ChangeTool>(kSceneAttachService);
  scene_detach_client_ = create_client<ivg_interfaces::srv::ChangeTool>(kSceneDetachService);

  if (!has_parameter("joint_velocity_scaling")) {
    declare_parameter("joint_velocity_scaling", 0.7);
  }
  if (!has_parameter("joint_acceleration_scaling")) {
    declare_parameter("joint_acceleration_scaling", 0.3);
  }
  if (!has_parameter("home_velocity_scaling")) {declare_parameter("home_velocity_scaling", 0.7);}
  if (!has_parameter("home_acceleration_scaling")) {
    declare_parameter("home_acceleration_scaling", 0.3);
  }
  if (!has_parameter("gripper_io_index")) {declare_parameter("gripper_io_index", 7);}
  if (!has_parameter("joint_cartesian_switch_delay_sec")) {
    declare_parameter("joint_cartesian_switch_delay_sec", 0.05);
  }
  if (!has_parameter("simulation_skip_io")) {declare_parameter("simulation_skip_io", false);}
  if (!has_parameter("initial_tool_id")) {declare_parameter("initial_tool_id", "");}
  if (!has_parameter("io_simulated")) {declare_parameter("io_simulated", false);}
  if (!has_parameter("planning_group")) {
    declare_parameter("planning_group", std::string("manipulator_e5"));
  }
  if (!has_parameter("home_target")) {declare_parameter("home_target", std::string("camera_pose"));}
  // 本仓 moveit 默认管线=pilz，Pilz 需显式 planner_id（移植适配点）
  if (!has_parameter("planning_pipeline")) {
    declare_parameter("planning_pipeline", std::string("pilz_industrial_motion_planner"));
  }
  if (!has_parameter("planner_id")) {declare_parameter("planner_id", std::string("PTP"));}

  joint_velocity_scaling_ =
    static_cast<float>(get_parameter("joint_velocity_scaling").as_double());
  joint_acceleration_scaling_ =
    static_cast<float>(get_parameter("joint_acceleration_scaling").as_double());
  home_velocity_scaling_ =
    static_cast<float>(get_parameter("home_velocity_scaling").as_double());
  home_acceleration_scaling_ =
    static_cast<float>(get_parameter("home_acceleration_scaling").as_double());
  gripper_io_index_ = static_cast<int32_t>(get_parameter("gripper_io_index").as_int());
  simulation_skip_io_ = get_parameter("simulation_skip_io").as_bool() ||
    get_parameter("io_simulated").as_bool();
  joint_cartesian_switch_delay_sec_ = std::max(
    0.0, get_parameter("joint_cartesian_switch_delay_sec").as_double());

  tool_status_pub_ = create_publisher<ivg_interfaces::msg::ToolChangerStatus>(
    "/tool_changer_status", 10);

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  gripper_swap_srv_ = create_service<ivg_interfaces::srv::RunGripperSwap>(
    "/run_gripper_swap",
    [this](const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> req,
    std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> resp) {
      onGripperSwapRequest(req, resp);
    },
    rclcpp::ServicesQoS(), service_cb_group_);

  change_tool_srv_ = create_service<ivg_interfaces::srv::ChangeTool>(
    "/change_tool",
    [this](const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> resp) {
      onChangeTool(req, resp);
    },
    rclcpp::ServicesQoS(), service_cb_group_);

  get_tool_srv_ = create_service<ivg_interfaces::srv::GetCurrentTool>(
    "/get_current_tool",
    [this](const std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Request> req,
    std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Response> resp) {
      onGetCurrentTool(req, resp);
    },
    rclcpp::ServicesQoS(), service_cb_group_);

  auto mode_qos = rclcpp::QoS(1).transient_local().reliable();
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    "/aubo/mode", mode_qos,
    [this](const std_msgs::msg::String & msg) {
      if (msg.data == "simulation" && !simulation_skip_io_) {
        simulation_skip_io_ = true;
        RCLCPP_INFO(get_logger(), "[仿真] IO 控制已禁用");
      }
    });

  // 加载 tools.yaml（纯核解析）
  try {
    const std::string config_path =
      ament_index_cpp::get_package_share_directory("tool_changer") + "/config/tools.yaml";
    std::string err;
    if (!loadToolConfigs(config_path, tool_configs_, err)) {
      RCLCPP_ERROR(get_logger(), "tools.yaml 加载失败: %s", err.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "tool_changer 包路径获取失败: %s", e.what());
  }

  const std::string initial_tool_id = get_parameter("initial_tool_id").as_string();
  if (!initial_tool_id.empty()) {
    auto it = tool_configs_.find(initial_tool_id);
    if (it != tool_configs_.end()) {
      current_tool_.id = it->second.id;
      current_tool_.name = it->second.name;
      current_tool_.type = it->second.type;
      current_tool_.parameters = it->second.parameters;
      RCLCPP_INFO(
        get_logger(), "启动时设定初始工具: %s (%s)",
        current_tool_.id.c_str(), current_tool_.name.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "initial_tool_id '%s' 不在 tools.yaml 中，将以无工具状态启动",
        initial_tool_id.c_str());
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "就绪 | vel=%.2f acc=%.2f home_vel=%.2f home_acc=%.2f delay=%.3f io=%d sim=%s tools=%zu",
    joint_velocity_scaling_, joint_acceleration_scaling_,
    home_velocity_scaling_, home_acceleration_scaling_,
    joint_cartesian_switch_delay_sec_, gripper_io_index_,
    simulation_skip_io_ ? "true" : "false", tool_configs_.size());

  publishToolStatus(!current_tool_.id.empty());
  status_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    [this]() {publishToolStatus(!current_tool_.id.empty());});
}

std::shared_ptr<GripperSwapWorker> GripperSwapWorker::create(const rclcpp::NodeOptions & options)
{
  auto node = std::make_shared<GripperSwapWorker>(options);
  node->robot_ = std::make_unique<ivg_demo_services::RobotController>(
    node.get(), node->get_parameter("planning_group").as_string());
  node->robot_->init();
  return node;
}

// ═════════════════════════════════ 轨迹原语 ═════════════════════════════════

bool GripperSwapWorker::moveToHome(float vel, float acc)
{
  if (!robot_) {return false;}
  return robot_->moveToHome(vel, acc);
}

bool GripperSwapWorker::moveToJoints(
  const std::array<double, 6> & joints, float vel, float acc)
{
  if (!robot_) {return false;}
  return robot_->moveToJoints(joints, vel, acc);
}

bool GripperSwapWorker::moveToTargetXYZ(
  double target_x, double target_y, double target_z, float vel, float acc)
{
  if (!robot_) {return false;}
  const auto current_pose = robot_->getCurrentPose();

  std::vector<CartesianSegment> segments;
  constexpr double kMinDeltaM = 1e-9;
  if (std::fabs(target_x - current_pose.position.x) > kMinDeltaM) {
    segments.push_back({'x', target_x - current_pose.position.x});
  }
  if (std::fabs(target_y - current_pose.position.y) > kMinDeltaM) {
    segments.push_back({'y', target_y - current_pose.position.y});
  }
  if (std::fabs(target_z - current_pose.position.z) > kMinDeltaM) {
    segments.push_back({'z', target_z - current_pose.position.z});
  }
  if (segments.empty()) {return true;}

  RCLCPP_INFO(get_logger(), "moveToXYZ: (%.4f,%.4f,%.4f)", target_x, target_y, target_z);
  return robot_->moveCartesianPath(segments, vel, acc);
}

bool GripperSwapWorker::moveToDockApproach(const ToolConfig & tool)
{
  if (!robot_) {return false;}
  if (tool.has_dock_approach_xyz) {
    return moveToTargetXYZ(
      tool.dock_approach_xyz[0], tool.dock_approach_xyz[1], tool.dock_approach_xyz[2],
      joint_velocity_scaling_, joint_acceleration_scaling_);
  }
  return robot_->moveToJoints(
    tool.dock_approach_joints, joint_velocity_scaling_, joint_acceleration_scaling_);
}

bool GripperSwapWorker::pickTool(const ToolConfig & tool)
{
  if (!robot_) {return false;}
  if (tool.strategy == TrajectoryStrategy::kSlide) {
    const auto & p = tool.slide;
    if (!setGripperIoSafe(true)) {return false;}
    if (!robot_->moveCartesianPath(
        {{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
    if (!setGripperIoSafe(false)) {return false;}
    std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
    return robot_->moveCartesianPath(
      {{'z', p.seat}, {'y', p.slide_y}, {'z', p.lift}},
      joint_velocity_scaling_, joint_acceleration_scaling_);
  }
  const auto & p = tool.vertical;
  if (!setGripperIoSafe(true)) {return false;}
  if (!robot_->moveCartesianPath(
      {{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  if (!setGripperIoSafe(false)) {return false;}
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  return robot_->moveCartesianPath(
    {{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_);
}

bool GripperSwapWorker::releaseTool(const ToolConfig & tool, bool * tool_released)
{
  if (tool_released) {*tool_released = false;}
  if (!robot_) {return false;}
  if (tool.strategy == TrajectoryStrategy::kSlide) {
    const auto & p = tool.slide;
    if (!robot_->moveCartesianPath(
        {{'y', p.slide_y},
          {'z', -(p.depth - p.seat)},
          {'y', -p.slide_y},
          {'z', -p.seat}},
        joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      return false;
    }
    if (!setGripperIoSafe(true)) {return false;}
    if (tool_released) {*tool_released = true;}
    std::this_thread::sleep_for(std::chrono::duration<double>(p.release_sec));
    if (!robot_->moveCartesianPath(
        {{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      return false;
    }
    if (!setGripperIoSafe(false)) {return false;}
    std::this_thread::sleep_for(std::chrono::duration<double>(p.lock_sec));
    return true;
  }
  const auto & p = tool.vertical;
  if (!robot_->moveCartesianPath(
      {{'z', -p.depth}}, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    return false;
  }
  if (!setGripperIoSafe(true)) {return false;}
  if (tool_released) {*tool_released = true;}
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  if (!robot_->moveCartesianPath(
      {{'z', p.lift}}, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    return false;
  }
  if (!setGripperIoSafe(false)) {return false;}
  std::this_thread::sleep_for(std::chrono::duration<double>(p.settle_sec));
  return true;
}

// ═════════════════════════════════ IO / 场景 ═════════════════════════════════

bool GripperSwapWorker::setGripperIoSafe(bool open_gripper)
{
  if (!robot_) {return false;}
  if (simulation_skip_io_) {
    RCLCPP_INFO(
      get_logger(), "[仿真] 跳过 IO(%d, %s)", gripper_io_index_,
      open_gripper ? "开" : "关");
    return true;
  }
  return robot_->setGripper(gripper_io_index_, open_gripper);
}

bool GripperSwapWorker::updateSceneAttachment(const std::string & tool_id, bool attached)
{
  auto client = attached ? scene_attach_client_ : scene_detach_client_;
  const char * service_name = attached ? kSceneAttachService : kSceneDetachService;
  if (!client->wait_for_service(std::chrono::seconds(kSceneTimeoutSec))) {
    RCLCPP_ERROR(
      get_logger(), "%s 服务未就绪，无法%s %s 的规划场景碰撞",
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
  RCLCPP_INFO(
    get_logger(), "规划场景碰撞%s: %s", attached ? "附着" : "移除", tool_id.c_str());
  return true;
}

// ═════════════════════════════════ 工具状态 ═════════════════════════════════

void GripperSwapWorker::publishToolStatus(bool connected)
{
  ivg_interfaces::msg::ToolChangerStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = "tool_changer";
  msg.tool_id = current_tool_.id;
  msg.tool_name = current_tool_.name;
  msg.tool_type = current_tool_.type;
  msg.is_connected = connected;
  msg.tool_parameters = current_tool_.parameters;
  tool_status_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(), "工具: id=%s name=%s connected=%s",
    current_tool_.id.c_str(), current_tool_.name.c_str(), connected ? "true" : "false");
}

bool GripperSwapWorker::sleepJointCartesianSwitchDelay(const char * where)
{
  if (joint_cartesian_switch_delay_sec_ <= 0.0) {return true;}
  RCLCPP_INFO(get_logger(), "%s: 延时 %.3fs", where, joint_cartesian_switch_delay_sec_);
  sleepInterruptible(this, joint_cartesian_switch_delay_sec_);
  return rclcpp::ok() && !shutdown_requested_;
}

// ═════════════════════════════════ 综合流程 ═════════════════════════════════

bool GripperSwapWorker::changeToTool(const std::string & target_id)
{
  auto target_it = tool_configs_.find(target_id);
  if (target_it == tool_configs_.end()) {
    RCLCPP_ERROR(get_logger(), "未知工具: %s", target_id.c_str());
    return false;
  }
  const auto & target = target_it->second;

  RCLCPP_INFO(
    get_logger(), "changeToTool: %s → %s", current_tool_.id.c_str(), target_id.c_str());

  if (target_id == current_tool_.id) {
    RCLCPP_INFO(get_logger(), "已是 %s，跳过", target_id.c_str());
    return true;
  }

  // 1. 释放当前工具
  if (!current_tool_.id.empty()) {
    auto current_it = tool_configs_.find(current_tool_.id);
    if (current_it == tool_configs_.end()) {
      RCLCPP_ERROR(get_logger(), "当前工具 %s 不在配置中", current_tool_.id.c_str());
      return false;
    }
    const auto & current = current_it->second;

    if (!moveToDockApproach(current)) {
      RCLCPP_ERROR(get_logger(), "释放 %s: dock approach 失败", current.id.c_str());
      return false;
    }
    if (!sleepJointCartesianSwitchDelay("释放: J→C")) {return false;}
    if (!updateSceneAttachment(current.id, false)) {
      RCLCPP_ERROR(get_logger(), "释放 %s: 提前移除规划场景碰撞失败", current.id.c_str());
      return false;
    }
    // 立即清除 current_tool_ 并发布空状态，防止周期定时器在 releaseTool
    // 执行期间发布旧工具 ID，导致 scene_attach_worker 重新附着已脱离的 ACO
    current_tool_ = ToolInfo{};
    publishToolStatus(false);

    bool tool_released = false;
    if (!releaseTool(current, &tool_released)) {
      if (!tool_released) {
        // 工具仍物理连接时恢复状态和碰撞体
        current_tool_ = {current.id, current.name, current.type, current.parameters};
        publishToolStatus(true);
        (void)updateSceneAttachment(current.id, true);
      }
      RCLCPP_ERROR(get_logger(), "释放 %s 失败", current.id.c_str());
      return false;
    }
    if (!sleepJointCartesianSwitchDelay("释放→取: C→J")) {return false;}
  }

  // 2. 取目标工具
  if (!moveToDockApproach(target)) {
    RCLCPP_ERROR(get_logger(), "取 %s: dock approach 失败", target.id.c_str());
    return false;
  }
  if (!sleepJointCartesianSwitchDelay("取: J→C")) {return false;}
  if (!pickTool(target)) {
    RCLCPP_ERROR(get_logger(), "取 %s 失败", target.id.c_str());
    return false;
  }

  // pickTool 完成时夹爪已物理锁紧，立即更新状态（不等 home）
  current_tool_.id = target.id;
  current_tool_.name = target.name;
  current_tool_.type = target.type;
  current_tool_.parameters = target.parameters;
  publishToolStatus(true);

  // 3. 回 home
  if (!sleepJointCartesianSwitchDelay("归位: C→J")) {return false;}
  if (!robot_->moveToHome(home_velocity_scaling_, home_acceleration_scaling_)) {
    RCLCPP_ERROR(get_logger(), "归位失败");
    return false;
  }

  return true;
}

// ═════════════════════════════════ 服务回调 ═════════════════════════════════

void GripperSwapWorker::onChangeTool(
  const std::shared_ptr<ivg_interfaces::srv::ChangeTool::Request> request,
  std::shared_ptr<ivg_interfaces::srv::ChangeTool::Response> response)
{
  RCLCPP_INFO(get_logger(), "━━ /change_tool: target=%s ━━", request->tool_id.c_str());

  bool ok = false;
  try {
    ok = changeToTool(request->tool_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "异常: %s", e.what());
  }

  response->success = ok;
  response->error_code = ok ? 0 : -1;
  response->message = ok ? ("已切换到: " + current_tool_.id) :
    ("切换失败: " + request->tool_id);
}

void GripperSwapWorker::onGetCurrentTool(
  const std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Request>,
  std::shared_ptr<ivg_interfaces::srv::GetCurrentTool::Response> response)
{
  response->success = true;
  response->tool_id = current_tool_.id;
  response->tool_name = current_tool_.name;
  response->tool_type = current_tool_.type;
  response->tool_parameters = current_tool_.parameters;
  response->message = current_tool_.id.empty() ?
    "当前无工具" : ("当前工具: " + current_tool_.id);
}

void GripperSwapWorker::onGripperSwapRequest(
  const std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Request> request,
  std::shared_ptr<ivg_interfaces::srv::RunGripperSwap::Response> response)
{
  std::string source_id, target_id;
  parseSwapDirection(request->direction, source_id, target_id);

  RCLCPP_INFO(
    get_logger(), "━━ run_gripper_swap: direction=%s source=%s target=%s ━━",
    request->direction.c_str(), source_id.c_str(), target_id.c_str());

  if (tool_configs_.find(target_id) == tool_configs_.end()) {
    response->success = false;
    response->message = "未知 direction 或工具: " + request->direction;
    return;
  }
  if (!source_id.empty() && tool_configs_.find(source_id) == tool_configs_.end()) {
    response->success = false;
    response->message = "未知源工具: " + source_id;
    return;
  }

  // 后端无当前工具但请求指定了源工具 → 用源工具填充 current_tool_
  // （仿真模式下用户手动选择工具，必须从 direction 获知源工具）
  const bool fill_source = current_tool_.id.empty() && !source_id.empty();
  if (fill_source) {
    const auto & src = tool_configs_[source_id];
    current_tool_.id = src.id;
    current_tool_.name = src.name;
    current_tool_.type = src.type;
    current_tool_.parameters = src.parameters;
    RCLCPP_INFO(get_logger(), "[swap] 从 direction 填充源工具: %s", source_id.c_str());
    publishToolStatus(true);
  }

  bool ok = false;
  try {
    ok = changeToTool(target_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "异常: %s", e.what());
  }

  if (!ok && fill_source) {
    current_tool_ = ToolInfo{};
    publishToolStatus(false);
  }

  response->success = ok;
  response->message = ok ? ("完成: " + request->direction) : ("失败: " + request->direction);
}

// ═════════════════════════════════ 生命周期 ═════════════════════════════════

void GripperSwapWorker::run()
{
  // 等待 /aubo/mode 消息（最长 8s），超时默认非仿真
  auto t0 = std::chrono::steady_clock::now();
  while (!simulation_skip_io_ &&
    std::chrono::steady_clock::now() - t0 < std::chrono::seconds(8))
  {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(shared_from_this());
  std::thread spinner([&exec]() {exec.spin();});
  SpinnerJoinGuard join_spinner{spinner};

  RCLCPP_INFO(get_logger(), "夹爪快换 Worker 就绪");

  while (rclcpp::ok() && !shutdown_requested_) {
    sleepInterruptible(this, 0.5);
  }
}

void GripperSwapWorker::onShutdown()
{
  RCLCPP_INFO(get_logger(), "onShutdown: 回 home");
  if (robot_) {
    robot_->moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
  }
}

void GripperSwapWorker::requestShutdown()
{
  shutdown_requested_ = true;
}

}  // namespace tool_changer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = tool_changer::GripperSwapWorker::create(rclcpp::NodeOptions());

  tool_changer::g_worker_for_signal = node.get();
  std::signal(SIGINT, tool_changer::sigintHandler);
  std::signal(SIGTERM, tool_changer::sigintHandler);

  node->run();

  tool_changer::g_worker_for_signal = nullptr;
  node->onShutdown();
  rclcpp::shutdown();
  return 0;
}
