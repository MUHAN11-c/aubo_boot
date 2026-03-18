/**
 * @file publish_grasps_client_worker.cpp
 * @brief GraspNet 抓取放置 Worker：订阅 PoseArray，循环执行抓取→放置→回安全位。
 */

#include "demo_driver/publish_grasps_client_worker.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <sstream>
#include <thread>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>

#define CHECK(expr)                                                                                                       \
  do                                                                                                                      \
  {                                                                                                                       \
    if (!(expr))                                                                                                          \
      return false;                                                                                                       \
  } while (0)

namespace demo_driver
{

/** 供 SIGINT 处理访问，在 main 中设置 */
static PublishGraspsClientWorker* g_worker_for_signal = nullptr;

static void sigintHandler(int)
{
  if (g_worker_for_signal)
    g_worker_for_signal->requestShutdown();
  rclcpp::shutdown();
}

/** 可中断睡眠：每 100ms 检查 rclcpp::ok() 与 shutdown_requested，超时或收到退出则返回 */
static void sleepInterruptible(PublishGraspsClientWorker* worker, double seconds)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && (!worker || !worker->isShutdownRequested()) &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

static constexpr int kArcPathMaxRetries = 3;
static constexpr double kArcPathRetryDelaySec = 0.5;
static constexpr double kCartesianEefStep = 0.015;
static constexpr double kCartesianJumpThreshold = 0.0;

/** GraspNet Z 轴 180° 修正四元数 (qx,qy,qz,qw) = (0,0,1,0) */
static const geometry_msgs::msg::Quaternion kQuatZ180 = []() {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 1.0;
  q.w = 0.0;
  return q;
}();

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

// ============================================================================
// 构造与初始化
// ============================================================================
//
// 参数说明（可通过 ros2 run 或 launch 覆盖）：
//
// | 参数                     | 类型    | 默认           | 含义                                               |
// |--------------------------|---------|----------------|----------------------------------------------------|
// | prefer_vertical          | bool    | true           | 选优策略：true 取垂直度最高，false 取最新组第一个                    |
// | grasp_z_offset           | double  | 0.15           | gripper_tip→end_effector 沿 z 轴补偿 (m)            |
// | height_above             | double  | 0.05           | 抓取点上方安全高度 (m)，笛卡尔路径先到该高度再下降                  |
// | joint_velocity_scaling   | float   | 1.0            | 关节/笛卡尔速度缩放 [0~1]，用于抓取接近、抬起、放置位姿等   |
// | joint_acceleration_scaling| float   | 0.1            | 关节/笛卡尔加速度缩放 [0~1]，用于抓取接近、放置位姿等       |
// | home_velocity_scaling    | float   | 0.5            | moveToHome 回安全位速度缩放 [0~1]，与笛卡尔分开               |
// | home_acceleration_scaling| float   | 0.5            | moveToHome 回安全位加速度缩放 [0~1]，与笛卡尔分开             |
// | grasp_poses_topic        | string  | grasp_poses_base| 抓取位姿话题，与 graspnet_demo_points_node 发布一致        |
// | grasp_capture_service_name | string| /graspnet_capture_control | 感知采集控制服务（SetBool）                 |
// | grasp_capture_service_timeout_sec | double | 2.0    | 采集控制服务等待超时 (s)                      |
// | loop_control_service_name | string| /publish_grasps_worker_loop_control | Worker 循环控制服务（SetBool） |
// | auto_start_loop         | bool    | false          | 启动后是否立即进入循环；false 时等待服务开启               |
// | wait_poses_timeout_sec   | double  | 30.0           | 等待窗口就绪超时 (s)                                   |
// | grasp_window_size        | int     | 5              | 滑动窗口大小：缓存最近 N 组 PoseArray                      |
// | min_groups_before_pick   | int     | 3              | 至少 M 组后再选优                                      |
// | gripper_io_index         | int     | 7              | Aubo 夹爪 IO pin 号                                  |
// | lift_offset              | double  | 0.2            | 抓取后沿 Z 轴抬起高度 (m)                               |
// | place_mode               | string  | "home_offset"  | "pose"/"joints"/"home_offset"（安全位+偏移）           |
// | place_offset_y           | double  | -0.2          | 安全位 y 偏移 (m)，place_mode=home_offset 时使用        |
// | place_offset_z           | double  | -0.20         | 安全位 z 偏移 (m)，place_mode=home_offset 时使用       |
// | place_pose               | double[7]| 见下           | (x,y,z,qx,qy,qz,qw)，place_mode=pose 时使用             |
// | place_joints             | double[6]| 见下           | 放置关节角 (rad)，place_mode=joints 时使用               |
// | cycle_delay_sec          | double  | 1.0            | 每周期结束后等待 (s)                                    |
// | fail_retry_delay_sec     | double  | 1.0            | 失败后额外等待 (s)                                     |
// | max_cycles               | int     | -1             | 最大周期数，-1 无限循环                                  |
// | status_topic             | string  | grasp_place_status | 状态监控话题，发布 JSON 状态                         |
// | cartesian_max_points     | int     | 50                | 抓取接近笛卡尔轨迹点数上限，超过则拒绝执行（即使规划100%%）   |
//
// ============================================================================

PublishGraspsClientWorker::PublishGraspsClientWorker(const rclcpp::NodeOptions& options)
  : MoveitGripperIoBase(options)
{
  declare_parameter("prefer_vertical", true);
  declare_parameter("grasp_z_offset", 0.15);
  declare_parameter("height_above", 0.1);
  declare_parameter("joint_velocity_scaling", 1.0f);
  declare_parameter("joint_acceleration_scaling", 0.1f);
  declare_parameter("home_velocity_scaling", 0.7f);
  declare_parameter("home_acceleration_scaling", 0.45f);
  declare_parameter("grasp_poses_topic", std::string("grasp_poses_base"));
  declare_parameter("grasp_capture_service_name", std::string("/graspnet_capture_control"));
  declare_parameter("grasp_capture_service_timeout_sec", 2.0);
  declare_parameter("loop_control_service_name", std::string("/publish_grasps_worker_loop_control"));
  declare_parameter("auto_start_loop", false);
  declare_parameter("wait_poses_timeout_sec", 30.0);
  declare_parameter("grasp_window_size", 5);
  declare_parameter("min_groups_before_pick", 3);
  declare_parameter("gripper_io_index", 6);
  declare_parameter("lift_offset", 0.2);
  declare_parameter("place_mode", std::string("home_offset"));
  declare_parameter("place_offset_y", -0.2);
  declare_parameter("place_offset_z", -0.15);
  declare_parameter("cycle_delay_sec", 1.0);
  declare_parameter("fail_retry_delay_sec", 1.0);
  declare_parameter("max_cycles", -1);
  declare_parameter("status_topic", std::string("grasp_place_status"));
  declare_parameter("cartesian_max_points", 50);
  declare_parameter("place_pose", std::vector<double>({ 0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0 }));
  declare_parameter("place_joints",
                    std::vector<double>({ 1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540 }));

  prefer_vertical_ = get_parameter("prefer_vertical").as_bool();
  grasp_z_offset_ = get_parameter("grasp_z_offset").as_double();
  height_above_ = get_parameter("height_above").as_double();
  joint_velocity_scaling_ = get_parameter("joint_velocity_scaling").as_double();
  joint_acceleration_scaling_ = get_parameter("joint_acceleration_scaling").as_double();
  home_velocity_scaling_ = get_parameter("home_velocity_scaling").as_double();
  home_acceleration_scaling_ = get_parameter("home_acceleration_scaling").as_double();
  grasp_poses_topic_ = get_parameter("grasp_poses_topic").as_string();
  grasp_capture_service_name_ = get_parameter("grasp_capture_service_name").as_string();
  grasp_capture_service_timeout_sec_ = get_parameter("grasp_capture_service_timeout_sec").as_double();
  loop_control_service_name_ = get_parameter("loop_control_service_name").as_string();
  auto_start_loop_ = get_parameter("auto_start_loop").as_bool();
  wait_poses_timeout_sec_ = get_parameter("wait_poses_timeout_sec").as_double();
  grasp_window_size_ = static_cast<size_t>(std::max<int64_t>(1, get_parameter("grasp_window_size").as_int()));
  min_groups_before_pick_ = static_cast<size_t>(std::max<int64_t>(1, get_parameter("min_groups_before_pick").as_int()));
  gripper_io_index_ = get_parameter("gripper_io_index").as_int();
  lift_offset_ = get_parameter("lift_offset").as_double();
  place_mode_ = get_parameter("place_mode").as_string();
  place_offset_y_ = get_parameter("place_offset_y").as_double();
  place_offset_z_ = get_parameter("place_offset_z").as_double();
  cycle_delay_sec_ = get_parameter("cycle_delay_sec").as_double();
  fail_retry_delay_sec_ = get_parameter("fail_retry_delay_sec").as_double();
  max_cycles_ = get_parameter("max_cycles").as_int();
  status_topic_ = get_parameter("status_topic").as_string();
  {
    int val = static_cast<int>(get_parameter("cartesian_max_points").as_int());
    cartesian_max_points_ = (val < 1) ? 1 : val;
  }

  place_pose_ = { 0.4, 0.0, 0.45, 0.0, 1.0, 0.0, 0.0 };
  place_joints_ = { 1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540 };
  {
    auto p = get_parameter("place_pose").as_double_array();
    for (size_t i = 0; i < 7 && i < p.size(); ++i)
      place_pose_[i] = p[i];
  }
  {
    auto p = get_parameter("place_joints").as_double_array();
    for (size_t i = 0; i < 6 && i < p.size(); ++i)
      place_joints_[i] = p[i];
  }

  grasp_poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      grasp_poses_topic_, 10, std::bind(&PublishGraspsClientWorker::graspPosesCallback, this, std::placeholders::_1));
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
  grasp_capture_client_ = create_client<std_srvs::srv::SetBool>(grasp_capture_service_name_);
  loop_control_service_ = create_service<std_srvs::srv::SetBool>(
      loop_control_service_name_,
      std::bind(&PublishGraspsClientWorker::handleLoopControl, this, std::placeholders::_1, std::placeholders::_2));
  loop_enabled_.store(auto_start_loop_);
  stop_after_cycle_.store(false);

  RCLCPP_INFO(get_logger(),
              "订阅抓取位姿: %s, 窗口大小=%zu, 最少%zu组后选优, 触发服务=%s, 循环控制服务=%s, auto_start_loop=%s",
              grasp_poses_topic_.c_str(), grasp_window_size_, min_groups_before_pick_, grasp_capture_service_name_.c_str(),
              loop_control_service_name_.c_str(), auto_start_loop_ ? "true" : "false");
}

std::shared_ptr<PublishGraspsClientWorker> PublishGraspsClientWorker::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<PublishGraspsClientWorker>(options);
  node->initMoveGroup();
  return node;
}

// ============================================================================
// 订阅回调
// ============================================================================

void PublishGraspsClientWorker::graspPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.empty())
    return;
  std::lock_guard<std::mutex> lock(window_mutex_);
  latest_grasp_poses_ = *msg;
  grasp_groups_window_.push_back(*msg);
  while (grasp_groups_window_.size() > grasp_window_size_)
    grasp_groups_window_.pop_front();
}

// ============================================================================
// 窗口操作
// ============================================================================

void PublishGraspsClientWorker::clearGraspWindow()
{
  std::lock_guard<std::mutex> lock(window_mutex_);
  grasp_groups_window_.clear();
  latest_grasp_poses_.poses.clear();
  RCLCPP_INFO(get_logger(), "已清空抓取窗口");
}

bool PublishGraspsClientWorker::waitForGraspWindowReady()
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_poses_timeout_sec_);
  while (rclcpp::ok() && !shutdown_requested_ && std::chrono::steady_clock::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      if (grasp_groups_window_.size() >= min_groups_before_pick_)
      {
        RCLCPP_INFO(get_logger(), "抓取窗口就绪: %zu/%zu 组",
                   grasp_groups_window_.size(), grasp_window_size_);
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_ERROR(get_logger(), "等待抓取窗口超时 (%.1f s)", wait_poses_timeout_sec_);
  return false;
}

bool PublishGraspsClientWorker::requestGraspCapture(bool enable)
{
  if (!grasp_capture_client_)
  {
    RCLCPP_ERROR(get_logger(), "采集控制服务客户端未初始化");
    return false;
  }
  if (!grasp_capture_client_->wait_for_service(std::chrono::duration<double>(grasp_capture_service_timeout_sec_)))
  {
    RCLCPP_ERROR(get_logger(), "采集控制服务不可用: %s (timeout=%.2fs)", grasp_capture_service_name_.c_str(),
                 grasp_capture_service_timeout_sec_);
    return false;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;
  auto future = grasp_capture_client_->async_send_request(req);
  auto wait_ret = future.wait_for(std::chrono::duration<double>(grasp_capture_service_timeout_sec_));
  if (wait_ret != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "采集控制服务调用超时: %s", enable ? "start" : "stop");
    return false;
  }
  std::shared_ptr<std_srvs::srv::SetBool::Response> resp;
  try
  {
    resp = future.get();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "采集控制服务调用异常: %s", e.what());
    return false;
  }
  if (!resp->success)
  {
    RCLCPP_ERROR(get_logger(), "采集控制服务返回失败: %s", resp->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "采集控制服务成功: %s (%s)", enable ? "start" : "stop", resp->message.c_str());
  return true;
}

void PublishGraspsClientWorker::handleLoopControl(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data)
  {
    if (shutdown_requested_)
    {
      response->success = false;
      response->message = "节点已进入退出流程，无法重新开启循环";
      return;
    }
    loop_enabled_.store(true);
    stop_after_cycle_.store(false);
    response->success = true;
    response->message = "已开启循环抓取";
    RCLCPP_INFO(get_logger(), "收到循环控制: start，已开启循环抓取");
    return;
  }

  // false: 当前周期结束后退出；若当前不在周期中则立即退出
  loop_enabled_.store(false);
  stop_after_cycle_.store(true);
  if (!cycle_in_progress_.load())
    shutdown_requested_.store(true);

  response->success = true;
  response->message = cycle_in_progress_.load() ? "已收到关闭请求，将在当前周期结束后退出"
                                                : "已收到关闭请求，当前无活动周期，立即退出";
  RCLCPP_INFO(get_logger(), "收到循环控制: stop，%s", response->message.c_str());
}

std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> PublishGraspsClientWorker::selectBestFromWindow()
{
  std::lock_guard<std::mutex> lock(window_mutex_);
  if (prefer_vertical_)
  {
    std::string best_tag;
    geometry_msgs::msg::Pose best_pose;
    double best_score = -1.0;
    for (size_t gi = 0; gi < grasp_groups_window_.size(); ++gi)
    {
      for (size_t pi = 0; pi < grasp_groups_window_[gi].poses.size(); ++pi)
      {
        const auto& p = grasp_groups_window_[gi].poses[pi];
        double s = verticalityScore(p);
        if (s > best_score)
        {
          best_score = s;
          best_tag = "group" + std::to_string(gi) + "_grasp" + std::to_string(pi);
          best_pose = p;
        }
      }
    }
    if (best_score >= 0)
    {
      RCLCPP_INFO(get_logger(), "选优: %s, 垂直度=%.3f", best_tag.c_str(), best_score);
      return std::make_pair(best_tag, best_pose);
    }
  }
  else
  {
    if (!latest_grasp_poses_.poses.empty())
    {
      RCLCPP_INFO(get_logger(), "选优: latest_grasp_0");
      return std::make_pair(std::string("latest_grasp_0"), latest_grasp_poses_.poses[0]);
    }
  }
  return std::nullopt;
}

// ============================================================================
// 变换与得分
// ============================================================================

Eigen::Matrix4d PublishGraspsClientWorker::buildGraspToEndEffectorTransform()
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(2, 3) = -grasp_z_offset_;
  return T;
}

geometry_msgs::msg::Pose PublishGraspsClientWorker::applyTransformationToPose(
    const geometry_msgs::msg::Pose& pose, const Eigen::Matrix4d& transform_local)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix4d T_base_grasp = Eigen::Matrix4d::Identity();
  T_base_grasp.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_base_grasp(0, 3) = pose.position.x;
  T_base_grasp(1, 3) = pose.position.y;
  T_base_grasp(2, 3) = pose.position.z;

  Eigen::Matrix4d T_base_target = T_base_grasp * transform_local;

  geometry_msgs::msg::Pose out;
  out.position.x = T_base_target(0, 3);
  out.position.y = T_base_target(1, 3);
  out.position.z = T_base_target(2, 3);
  Eigen::Quaterniond q_out(T_base_target.block<3, 3>(0, 0));
  out.orientation.x = q_out.x();
  out.orientation.y = q_out.y();
  out.orientation.z = q_out.z();
  out.orientation.w = q_out.w();
  return out;
}

double PublishGraspsClientWorker::verticalityScore(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d z_axis = q.toRotationMatrix().col(2);
  return std::abs(z_axis.dot(Eigen::Vector3d(0, 0, -1)));
}

geometry_msgs::msg::Quaternion PublishGraspsClientWorker::quatSameHemisphere(
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

geometry_msgs::msg::Pose PublishGraspsClientWorker::applyGraspZFlip180(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond q_ori(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Quaterniond q_z180(kQuatZ180.w, kQuatZ180.x, kQuatZ180.y, kQuatZ180.z);
  Eigen::Quaterniond q_new = q_ori * q_z180;

  geometry_msgs::msg::Pose out;
  out.position = pose.position;
  out.orientation.x = q_new.x();
  out.orientation.y = q_new.y();
  out.orientation.z = q_new.z();
  out.orientation.w = q_new.w();
  return out;
}

// ============================================================================
// 抓取接近
// ============================================================================

bool PublishGraspsClientWorker::runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above,
                                                 float vel, float acc)
{
  (void)acc;
  geometry_msgs::msg::Pose pose_for_plan = applyGraspZFlip180(pose_ee);

  const std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty())
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] 无法获取末端 link");
    return false;
  }

  geometry_msgs::msg::PoseStamped current_stamped = move_group_->getCurrentPose(eef_link);
  geometry_msgs::msg::Pose current_pose = current_stamped.pose;

  double gx = pose_for_plan.position.x;
  double gy = pose_for_plan.position.y;
  double gz = pose_for_plan.position.z;
  double z_above = gz + height_above;

  if (gz < kZMinLimit)
  {
    RCLCPP_WARN(get_logger(), "[runGraspApproach] Z 轴安全限位: 抓取点 z=%.3f < %.2f m，覆盖为 %.2f m 执行", gz,
                kZMinLimit, kZMinLimit);
    gz = kZMinLimit;
    z_above = gz + height_above;
  }

  geometry_msgs::msg::Quaternion grasp_ori = pose_for_plan.orientation;
  geometry_msgs::msg::Quaternion grasp_ori_short = quatSameHemisphere(current_pose.orientation, grasp_ori);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);

  // 多段笛卡尔：按 x -> y -> up(z_above) -> rotate -> down 顺序
  geometry_msgs::msg::Pose p_up;
  p_up.position.x = gx;
  p_up.position.y = gy;
  p_up.position.z = z_above;
  p_up.orientation = current_pose.orientation;
  geometry_msgs::msg::Pose p_x;
  p_x.position.x = gx;
  p_x.position.y = current_pose.position.y;
  p_x.position.z = current_pose.position.z;
  p_x.orientation = current_pose.orientation;
  waypoints.push_back(p_x);

  geometry_msgs::msg::Pose p_y;
  p_y.position.x = gx;
  p_y.position.y = gy;
  p_y.position.z = current_pose.position.z;
  p_y.orientation = current_pose.orientation;
  waypoints.push_back(p_y);

  waypoints.push_back(p_up);

  geometry_msgs::msg::Pose p_rot;
  p_rot.position.x = gx;
  p_rot.position.y = gy;
  p_rot.position.z = z_above;
  p_rot.orientation = grasp_ori_short;
  waypoints.push_back(p_rot);

  geometry_msgs::msg::Pose p_down;
  p_down.position.x = gx;
  p_down.position.y = gy;
  p_down.position.z = gz;
  p_down.orientation = grasp_ori_short;
  waypoints.push_back(p_down);

  moveit_msgs::msg::RobotTrajectory trajectory;
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    double fraction = move_group_->computeCartesianPath(
        waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    size_t num_points = trajectory.joint_trajectory.points.size();

    RCLCPP_INFO(get_logger(), "抓取接近笛卡尔: fraction=%.2f%%, 点数=%zu (尝试 %d/%d)",
                fraction * 100.0, num_points, attempt, kArcPathMaxRetries);

    if (fraction < 1.0)
    {
      const int segment_count = static_cast<int>(waypoints.size()) - 1;
      int seg_idx = std::min(segment_count - 1, static_cast<int>(fraction * segment_count));
      RCLCPP_ERROR(get_logger(), "笛卡尔路径未达100%%, fraction=%.2f, 截断于第%d段。不回退，直接失败。", fraction,
                   seg_idx + 1);
      if (attempt < kArcPathMaxRetries)
        std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
      continue;
    }
    if (num_points > static_cast<size_t>(cartesian_max_points_))
    {
      RCLCPP_ERROR(get_logger(), "笛卡尔轨迹点数过多 (%zu > %d)，规划100%%也不执行。", num_points,
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
      RCLCPP_ERROR(get_logger(), "[runGraspApproach] 执行失败，错误码=%d", exec_ok.val);
      return false;
    }
    RCLCPP_INFO(get_logger(), "抓取接近完成");
    return true;
  }
  return false;
}

bool PublishGraspsClientWorker::runGraspMotion(const geometry_msgs::msg::Pose& target_pose)
{
  return runGraspApproach(target_pose, height_above_, joint_velocity_scaling_, joint_acceleration_scaling_);
}

// ============================================================================
// runOneCycle
// ============================================================================

bool PublishGraspsClientWorker::runOneCycle()
{
  struct CycleProgressGuard
  {
    explicit CycleProgressGuard(std::atomic<bool>& in_progress) : in_progress_(in_progress) { in_progress_.store(true); }
    ~CycleProgressGuard() { in_progress_.store(false); }
    std::atomic<bool>& in_progress_;
  } cycle_progress_guard(cycle_in_progress_);

  bool capture_started = false;
  const auto stopCaptureIfNeeded = [this, &capture_started]() {
    if (capture_started)
      (void)requestGraspCapture(false);
  };
  const auto failCycle = [&stopCaptureIfNeeded]() {
    stopCaptureIfNeeded();
    return false;
  };

  if (!rclcpp::ok() || shutdown_requested_)
    return false;
  RCLCPP_INFO(get_logger(), "周期开始: 预清空抓取窗口（清理上周期残留）");
  clearGraspWindow();
  if (!rclcpp::ok() || shutdown_requested_)
    return false;
  RCLCPP_INFO(get_logger(), "步骤 -1: 触发感知开始采集");
  if (!requestGraspCapture(true))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤-1 触发感知采集失败");
    return false;
  }
  capture_started = true;
  RCLCPP_INFO(get_logger(), "步骤 0: 回安全位（识别前到位）");
  if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤0 回安全位失败，需在识别前到位");
    return failCycle();
  }
  if (!rclcpp::ok() || shutdown_requested_)
    return failCycle();

  RCLCPP_INFO(get_logger(), "步骤 1: 清空抓取窗口");
  clearGraspWindow();
  if (!rclcpp::ok() || shutdown_requested_)
    return failCycle();

  RCLCPP_INFO(get_logger(), "步骤 2: 等待抓取窗口就绪 (>=%zu 组)", min_groups_before_pick_);
  if (!waitForGraspWindowReady())
    return failCycle();

  if (!rclcpp::ok() || shutdown_requested_)
    return failCycle();
  RCLCPP_INFO(get_logger(), "步骤 3: 从窗口选优 (prefer_vertical=%s)", prefer_vertical_ ? "true" : "false");
  auto selected = selectBestFromWindow();
  if (!selected)
  {
    RCLCPP_ERROR(get_logger(), "步骤 3 选优失败，窗口无有效抓取");
    return failCycle();
  }

  if (!rclcpp::ok() || shutdown_requested_)
    return failCycle();
  auto [tag, grasp_pose] = *selected;
  RCLCPP_INFO(get_logger(), "步骤 4: gripper_tip 变换为 end_effector 目标 (选用 %s)", tag.c_str());
  Eigen::Matrix4d T_local = buildGraspToEndEffectorTransform();
  geometry_msgs::msg::Pose pose_ee = applyTransformationToPose(grasp_pose, T_local);
  RCLCPP_INFO(get_logger(), "  目标位姿 xyz=[%.3f, %.3f, %.3f]", pose_ee.position.x, pose_ee.position.y,
              pose_ee.position.z);

  RCLCPP_INFO(get_logger(), "步骤 5: 抓取接近 (4 点笛卡尔)");
  if (!runGraspApproach(pose_ee, height_above_, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤5 抓取接近失败");
    return failCycle();
  }

  // RCLCPP_INFO(get_logger(), "步骤 6: 闭夹爪");
  // if (!setGripperIo(gripper_io_index_, true))
  // {
  //   RCLCPP_ERROR(get_logger(), "runOneCycle 步骤6 闭夹爪失败");
  //   return false;
  // }

  RCLCPP_INFO(get_logger(), "步骤 7: 抬起 (z=%.2f m)", lift_offset_);
  if (!runArcPath('z', lift_offset_, joint_velocity_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤7 抬起失败");
    return failCycle();
  }

  RCLCPP_INFO(get_logger(), "步骤 8: 移动到放置位 (place_mode=%s)", place_mode_.c_str());
  if (place_mode_ == "home_offset")
  {
    if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤8 回安全位失败");
      return failCycle();
    }
    const std::vector<CartesianSegment> place_segments = {
      { 'y', place_offset_y_ },
      { 'x', -0.2 },
      { 'z', place_offset_z_ },
    };
    if (!runArcPathSequence(place_segments, joint_velocity_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤8 放置位多段笛卡尔失败 (y=%.2f, z=%.2f)",
                   place_offset_y_, place_offset_z_);
      return failCycle();
    }
  }
  else if (place_mode_ == "pose")
  {
    if (!moveToPose(place_pose_[0], place_pose_[1], place_pose_[2], place_pose_[3], place_pose_[4], place_pose_[5],
                    place_pose_[6], false, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤8 移动到放置位失败");
      return failCycle();
    }
  }
  else
  {
    if (!moveToJoints(place_joints_, joint_velocity_scaling_, joint_acceleration_scaling_))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤8 移动到放置关节失败");
      return failCycle();
    }
  }

  // RCLCPP_INFO(get_logger(), "步骤 9: 开夹爪");
  // if (!setGripperIo(gripper_io_index_, false))
  // {
  //   RCLCPP_ERROR(get_logger(), "runOneCycle 步骤9 开夹爪失败");
  //   return false;
  // }

  RCLCPP_INFO(get_logger(), "步骤 10: 回安全位");
  if (!moveToHome(home_velocity_scaling_, home_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤10 回安全位失败");
    return failCycle();
  }

  stopCaptureIfNeeded();
  return true;
}

// ============================================================================
// 状态发布与主循环
// ============================================================================

void PublishGraspsClientWorker::publishStatus(int cycle_count, int success_count, int fail_count, int last_step,
                                               bool is_running)
{
  std_msgs::msg::String msg;
  std::ostringstream ss;
  ss << "{\"cycle_count\":" << cycle_count << ",\"success_count\":" << success_count
     << ",\"fail_count\":" << fail_count << ",\"last_step\":" << last_step << ",\"is_running\":" << (is_running ? "true" : "false")
     << "}";
  msg.data = ss.str();
  status_pub_->publish(msg);
}

void PublishGraspsClientWorker::onShutdown()
{
  RCLCPP_INFO(get_logger(), "onShutdown: 回安全位、开夹爪");
  setGripperIo(gripper_io_index_, false);
  moveToHome(home_velocity_scaling_, home_acceleration_scaling_);
}

void PublishGraspsClientWorker::requestShutdown()
{
  shutdown_requested_ = true;
  RCLCPP_INFO(get_logger(), "收到退出请求，主循环将尽快退出");
}

bool PublishGraspsClientWorker::run()
{
  RCLCPP_INFO(get_logger(), "主循环启动，max_cycles=%d，Ctrl+C 可随时退出，初始循环状态=%s",
              max_cycles_, loop_enabled_.load() ? "running" : "idle");
  while (rclcpp::ok() && !shutdown_requested_ && (max_cycles_ < 0 || cycle_count_ < max_cycles_))
  {
    if (!loop_enabled_.load())
    {
      publishStatus(cycle_count_, success_count_, fail_count_, 0, false);
      sleepInterruptible(this, 0.2);
      if (stop_after_cycle_.load() && !cycle_in_progress_.load())
        shutdown_requested_.store(true);
      continue;
    }

    bool success = runOneCycle();
    if (success)
    {
      cycle_count_++;
      success_count_++;
      publishStatus(cycle_count_, success_count_, fail_count_, 0, true);
      RCLCPP_INFO(get_logger(), "周期 %d 成功", cycle_count_);
    }
    else
    {
      fail_count_++;
      publishStatus(cycle_count_, success_count_, fail_count_, -1, true);
      RCLCPP_WARN(get_logger(), "周期失败，fail_count=%d", fail_count_);
      sleepInterruptible(this, fail_retry_delay_sec_);
    }
    if (stop_after_cycle_.load())
      shutdown_requested_.store(true);
    sleepInterruptible(this, cycle_delay_sec_);
  }
  publishStatus(cycle_count_, success_count_, fail_count_, 0, false);
  RCLCPP_INFO(get_logger(), "主循环退出，共 %d 周期，成功 %d，失败 %d", cycle_count_, success_count_, fail_count_);
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

  auto node = demo_driver::PublishGraspsClientWorker::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
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

  RCLCPP_INFO(node->get_logger(), "PublishGraspsClientWorker 就绪，进入循环抓取放置");
  node->run();

  demo_driver::g_worker_for_signal = nullptr;
  node->onShutdown();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
