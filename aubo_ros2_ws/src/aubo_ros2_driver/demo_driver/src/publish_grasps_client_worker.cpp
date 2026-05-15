/**
 * @file publish_grasps_client_worker.cpp
 * @brief GraspNet 抓取放置 Worker：订阅抓取位姿话题，在循环中完成「选优 → 笛卡尔接近 → 夹爪 → 抬起 → B 点放置 → 回 A 点安全位」。
 *
 * 【数据流】感知 PoseArray → 滑窗 → 选优 → tip→EEF 变换 → MoveIt 运动链 → 夹爪 IO。
 * 【控制流】loop_control(SetBool) / auto_start_loop 决定是否进入 runOneCycle；SIGINT 置 shutdown 标志。
 *
 * 【源码结构（自上而下）】
 *   1) 匿名命名空间：笛卡尔辅助、路径常量、仿真开关、姿态小工具函数
 *   2) 进程级：SIGINT、可中断睡眠
 *   3) 构造 / create / 参数表
 *   4) MoveIt + Aubo IO：preparePlanningState → plan/execute → 笛卡尔弧段
 *   5) ROS 实体回调：抓取话题、窗口、采集与循环服务
 *   6) 几何：选优得分、坐标变换、抓取接近（多段笛卡尔）
 *   7) runOneCycle 步骤链 → publishStatus / run / main
 *
 * MoveIt2：规划前 setStartStateToCurrentState + 速度/加速度缩放（对 plan() 生效）；笛卡尔直线轨迹按 MoveIt
 * computeCartesianPath 默认时间戳直接 execute，不做手动 time_from_start 缩放。
 */

#include "demo_driver/publish_grasps_client_worker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <cstdint>
#include <future>
#include <sstream>
#include <thread>
#include <tuple>
#include <utility>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>

/** 周期步骤链：条件不满足则 return false */
#define CHECK(expr)                                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(expr))                                                                                                       \
      return false;                                                                                                    \
  } while (0)

namespace demo_driver
{

const int32_t PublishGraspsClientWorker::kGripperIoIndex = 6;

namespace
{
// ---------------------------------------------------------------------------
// 服务名、放置路径常量、笛卡尔路径数值（本文件内 linkage）
// ---------------------------------------------------------------------------
/** Aubo 数字 IO 服务，setGripperIo 经此调用夹爪开闭 */
const char* kAuboSetIOService = "/aubo_driver/set_io";
/** SetRobotIO 异步请求最长等待 (s)，超时则本次 IO 判失败 */
constexpr int kIoCallTimeoutSeconds = 60;
/** runArcPath / runArcPathSequence 首次 computeCartesianPath 前短延迟 (s)，给 TF/状态一点稳定时间；0 可关闭 */
constexpr double kArcPathInitialDelaySec = 0.2;
/** 放置段第二段沿 base X 偏移 (m) */
constexpr double kPlaceHomeOffsetSegmentX = -0.2;

/** 笛卡尔 fraction<1 或执行失败时的最大重试次数（抓取接近、单轴弧、多段弧共用） */
constexpr int kArcPathMaxRetries = 3;
/** 两次笛卡尔重试之间的休眠 (s) */
constexpr double kArcPathRetryDelaySec = 0.5;
/** computeCartesianPath 末端直线插值步长 (m)，越小轨迹点越多、越密 */
constexpr double kCartesianEefStep = 0.015;
/** 关节空间跳变阈值；0 表示按 MoveIt 默认语义不额外限制（与历史配置一致） */
constexpr double kCartesianJumpThreshold = 0.0;

/** true：跳过夹爪 IO，便于无 /aubo_driver/set_io 的仿真环境 */
constexpr bool kSkipTemporaryGripperIo = false;

/**
 * GraspNet 预测抓取专属：网络输出的抓取系与真实夹爪/末端工具系在绕接近轴（局部 Z）上常差 π，
 * 仅对来自 GraspNet 的位姿在规划前右乘此四元数。(qx,qy,qz,qw) = (0,0,1,0)
 */
const geometry_msgs::msg::Quaternion kQuatZ180 = []() {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 1.0;
  q.w = 0.0;
  return q;
}();

/** 将绕 Z 的 twist 角先收到 (-π, π]，再折叠到 (-π/2, π/2]（等价 ±90° 短路径）。 */
double normalizeYawToPlusMinusHalfPi(double yaw_rad)
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

void offsetPoseAxis(geometry_msgs::msg::Pose& pose, char axis, double offset)
{
  if (axis == 'x')
    pose.position.x += offset;
  else if (axis == 'y')
    pose.position.y += offset;
  else
    pose.position.z += offset;  // 非 'x'/'y' 均按 Z 处理，与历史逻辑一致
}

}  // namespace

// ---------------------------------------------------------------------------
// 进程级：信号与睡眠（非成员，供 main / run 协作）
// ---------------------------------------------------------------------------
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
  while (rclcpp::ok() && (!worker || !worker->isShutdownRequested()) && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// =============================================================================
// 构造与初始化
// =============================================================================
//
// 参数说明（可通过 ros2 run 或 launch 覆盖）：
//
// | 参数                     | 类型    | 默认           | 含义                                               |
// |--------------------------|---------|----------------|----------------------------------------------------|
// | prefer_vertical          | bool    | true           | 选优策略：true 取垂直度最高，false 取最新组第一个 | |
// grasp_z_offset           | double  | 0.15           | gripper_tip→end_effector 沿 z 轴补偿 (m)            | |
// height_above             | double  | 0.05           | 抓取点上方安全高度 (m)，笛卡尔路径先到该高度再下降 | |
// joint_velocity_scaling   | float   | 0.5            | 速度缩放 [0~1]，用于关节规划、moveToHome 等（笛卡尔轨迹不缩放）
// | | joint_acceleration_scaling| float   | 0.3            | 加速度缩放 [0~1]，与 joint_velocity_scaling 共用场景一致 |
// | grasp_poses_topic        | string  | grasp_poses_base| 抓取位姿话题，与 graspnet_demo_points_node 发布一致        |
// | grasp_capture_service_name | string| /graspnet_capture_control | 感知采集控制服务（SetBool）                 |
// | grasp_capture_service_timeout_sec | double | 2.0    | 采集控制服务等待超时 (s)                      |
// | loop_control_service_name | string| /publish_grasps_worker_loop_control | Worker 循环控制服务（SetBool） |
// | auto_start_loop         | bool    | false          | 启动后是否立即进入循环；false 时等待服务开启               |
// | wait_poses_timeout_sec   | double  | 5.0            | 等待窗口就绪超时 (s)                                   |
// | grasp_window_size        | int     | 5              | 滑动窗口大小：缓存最近 N 组 PoseArray                      |
// | min_groups_before_pick   | int     | 3              | 至少 M 组后再选优                                      |
// | gripper_io_index         | int     | 7              | Aubo 夹爪 IO pin 号                                  |
// | lift_offset              | double  | 0.2            | 抓取后沿 Z 轴抬起高度 (m)                               |
// | place_offset_y           | double  | -0.2          | 安全位后笛卡尔 y 偏移 (m)                              |
// | place_offset_z           | double  | -0.20         | 安全位后笛卡尔 z 偏移 (m)                              |
// | joint_cartesian_switch_delay_sec | double | 0.2   | 关节↔笛卡尔切换时衔接延时 (s)，每处 j↔c 边界各一次；0 关闭   |
// | cycle_delay_sec          | double  | 1.0            | 每周期结束后等待 (s)                                    |
// | fail_retry_delay_sec     | double  | 1.0            | 失败后额外等待 (s)                                     |
// | max_cycles               | int     | -1             | 最大周期数，-1 无限循环                                  |
// | status_topic             | string  | grasp_place_status | 状态监控话题，发布 JSON 状态                         |
// | cartesian_max_points     | int     | 50                |
// 抓取接近笛卡尔轨迹点数上限，超过则拒绝执行（即使规划100%%）   |
//
// =============================================================================

PublishGraspsClientWorker::PublishGraspsClientWorker(const rclcpp::NodeOptions& options)
  : rclcpp::Node("publish_grasps_client_worker", options)
{
  // RobotController 构造安全（只存指针），init() 延后到 main
  robot_ = std::make_shared<RobotController>(this);
  move_group_ = nullptr;
  aubo_set_io_client_ = create_client<ivg_interfaces::srv::SetRobotIO>(kAuboSetIOService);

  // --- 声明参数（详见本文件上方参数表）---
  declare_parameter("prefer_vertical", true);
  declare_parameter("grasp_z_offset", 0.15);
  declare_parameter("height_above", 0.1);
  declare_parameter("joint_velocity_scaling", 0.7f);
  declare_parameter("joint_acceleration_scaling", 0.3f);
  declare_parameter("grasp_poses_topic", std::string("grasp_poses_base"));
  declare_parameter("grasp_capture_service_name", std::string("/graspnet_capture_control"));
  declare_parameter("grasp_capture_service_timeout_sec", 2.0);
  declare_parameter("loop_control_service_name", std::string("/publish_grasps_worker_loop_control"));
  declare_parameter("auto_start_loop", false);
  declare_parameter("wait_poses_timeout_sec", 5.0);
  declare_parameter("grasp_window_size", 5);
  declare_parameter("min_groups_before_pick", 3);
  declare_parameter("gripper_io_index", kGripperIoIndex);
  declare_parameter("lift_offset", 0.2);
  declare_parameter("place_offset_y", -0.2);
  declare_parameter("place_offset_z", -0.15);
  declare_parameter("joint_cartesian_switch_delay_sec", 0.2);
  declare_parameter("cycle_delay_sec", 1.0);
  declare_parameter("fail_retry_delay_sec", 1.0);
  declare_parameter("max_cycles", -1);
  declare_parameter("status_topic", std::string("grasp_place_status"));
  declare_parameter("cartesian_max_points", 50);

  // --- 读入成员（与 declare 顺序对应）---
  prefer_vertical_ = get_parameter("prefer_vertical").as_bool();
  grasp_z_offset_ = get_parameter("grasp_z_offset").as_double();
  height_above_ = get_parameter("height_above").as_double();
  joint_velocity_scaling_ = get_parameter("joint_velocity_scaling").as_double();
  joint_acceleration_scaling_ = get_parameter("joint_acceleration_scaling").as_double();
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
  place_offset_y_ = get_parameter("place_offset_y").as_double();
  place_offset_z_ = get_parameter("place_offset_z").as_double();
  joint_cartesian_switch_delay_sec_ = std::max(0.0, get_parameter("joint_cartesian_switch_delay_sec").as_double());
  cycle_delay_sec_ = get_parameter("cycle_delay_sec").as_double();
  fail_retry_delay_sec_ = get_parameter("fail_retry_delay_sec").as_double();
  max_cycles_ = get_parameter("max_cycles").as_int();
  status_topic_ = get_parameter("status_topic").as_string();
  {
    int val = static_cast<int>(get_parameter("cartesian_max_points").as_int());
    cartesian_max_points_ = (val < 1) ? 1 : val;
  }

  // --- 通信实体：订阅/发布/服务客户端（MoveGroup 在 create() 里 init）---
  const rclcpp::QoS qos_default(10);
  grasp_poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      grasp_poses_topic_, qos_default,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) { graspPosesCallback(std::move(msg)); });
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, qos_default);
  grasp_capture_client_ = create_client<std_srvs::srv::SetBool>(grasp_capture_service_name_);
  loop_control_service_ = create_service<std_srvs::srv::SetBool>(
      loop_control_service_name_,
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr resp) {
        handleLoopControl(req, resp);
      });
  loop_enabled_.store(auto_start_loop_);
  stop_after_cycle_.store(false);

  RCLCPP_INFO(
      get_logger(), "订阅抓取位姿: %s, 窗口大小=%zu, 最少%zu组后选优, 触发服务=%s, 循环控制服务=%s, auto_start_loop=%s",
      grasp_poses_topic_.c_str(), grasp_window_size_, min_groups_before_pick_, grasp_capture_service_name_.c_str(),
      loop_control_service_name_.c_str(), auto_start_loop_ ? "true" : "false");
}

void PublishGraspsClientWorker::initRobot()
{
  robot_->init();
  move_group_ = robot_->moveGroup();
}

// =============================================================================
// MoveIt 运动 + Aubo 夹爪 IO（runOneCycle 的步骤链均调用此层）
// =============================================================================

void PublishGraspsClientWorker::preparePlanningState(float velocity_factor, float acceleration_factor)
{
  if (!move_group_)
    return;
  // MoveGroupInterface：规划前用当前观测状态作为起点，并设置 OMPL 等规划器使用的速度/加速度缩放
  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(velocity_factor);
  move_group_->setMaxAccelerationScalingFactor(acceleration_factor);
}

bool PublishGraspsClientWorker::waitForServices(std::chrono::seconds timeout)
{
  if (!aubo_set_io_client_->wait_for_service(timeout))
  {
    RCLCPP_WARN(get_logger(), "服务 %s 未就绪，setGripperIo 不可用", kAuboSetIOService);
  }
  RCLCPP_INFO(get_logger(), "将通过 %s 设置夹爪 IO", kAuboSetIOService);
  RCLCPP_INFO(get_logger(), "所需服务已就绪");
  return true;
}

bool PublishGraspsClientWorker::setGripperIo(int32_t io_index, bool high)
{
  if (io_index < 0)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] io_index 必须 >= 0，当前为 %d", io_index);
    return false;
  }
  if (!aubo_set_io_client_->service_is_ready())
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] 服务 %s 不可用", kAuboSetIOService);
    return false;
  }
  auto req = std::make_shared<ivg_interfaces::srv::SetRobotIO::Request>();
  req->io_type = "digital_output";
  req->io_index = io_index;
  req->value = high ? 1.0 : 0.0;
  auto future = aubo_set_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kIoCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] 调用 %s 超时或失败", kAuboSetIOService);
    return false;
  }
  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[set_gripper_io] %s 返回失败: code=%d, msg=%s", kAuboSetIOService, res->error_code,
                 res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[set_gripper_io] pin %d -> %s (via %s)", io_index, high ? "HIGH" : "LOW",
              kAuboSetIOService);
  return true;
}

// 安全位 A 点：SRDF 命名状态 camera_pose（与 moveit 配置一致）
bool PublishGraspsClientWorker::moveToHome(float velocity_factor, float acceleration_factor)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_home_A] MoveGroup 未初始化");
    return false;
  }
  preparePlanningState(velocity_factor, acceleration_factor);
  move_group_->setNamedTarget("camera_pose");

  static constexpr double kHomeJointEpsilonRad = 0.01;
  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(1.0);
  if (current_state)
  {
    const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(move_group_->getName());
    if (jmg)
    {
      std::vector<double> current_joints;
      current_state->copyJointGroupPositions(jmg, current_joints);
      std::vector<double> target_joints;
      move_group_->getJointValueTarget(target_joints);
      if (current_joints.size() == target_joints.size() && !current_joints.empty())
      {
        bool at_home = true;
        for (size_t i = 0; i < current_joints.size(); ++i)
        {
          if (std::fabs(current_joints[i] - target_joints[i]) > kHomeJointEpsilonRad)
          {
            at_home = false;
            break;
          }
        }
        if (at_home)
        {
          RCLCPP_INFO(get_logger(), "[move_to_home_A] 已在 A 点（camera_pose），跳过规划与执行");
          return true;
        }
      }
    }
  }

  if (const moveit::core::MoveItErrorCode move_ok = move_group_->move();
      move_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_home_A] 执行失败，错误码=%d", move_ok.val);
    return false;
  }
  return true;
}

bool PublishGraspsClientWorker::runArcPath(char axis, double offset, float velocity_factor, float acceleration_factor)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[runArcPath] MoveGroup 未初始化");
    return false;
  }
  const char* axis_label = (axis == 'x') ? "X" : (axis == 'y') ? "Y" : "Z";
  RCLCPP_INFO(get_logger(), "笛卡尔路径: 沿 %s 轴 %+.2f m", axis_label, offset);
  if (kArcPathInitialDelaySec > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));

  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    preparePlanningState(velocity_factor, acceleration_factor);
    const std::string eef_link = move_group_->getEndEffectorLink();
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    offsetPoseAxis(target_pose, axis, offset);
    if (axis == 'z' && target_pose.position.z < kZMinLimit)
    {
      RCLCPP_WARN(get_logger(), "笛卡尔路径 Z 轴安全限位: 目标 z=%.3f < %.2f m，覆盖为 %.2f m 执行",
                  target_pose.position.z, kZMinLimit, kZMinLimit);
      target_pose.position.z = kZMinLimit;
    }
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）完成度: %.2f%% (尝试 %d/%d)", axis_label, offset,
                fraction * 100.0, attempt, kArcPathMaxRetries);
    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      const moveit::core::MoveItErrorCode exec_ok = move_group_->execute(plan);
      if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "笛卡尔路径执行失败，错误码=%d", exec_ok.val);
        return false;
      }
      RCLCPP_INFO(get_logger(), "%s 轴直线移动执行完成", axis_label);
      return true;
    }
    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
  }
  RCLCPP_ERROR(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）在 %d 次尝试后仍未达 100%%", axis_label, offset,
               kArcPathMaxRetries);
  return false;
}

bool PublishGraspsClientWorker::runArcPathSequence(const std::vector<CartesianSegment>& segments, float velocity_factor,
                                                   float acceleration_factor)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[runArcPathSequence] MoveGroup 未初始化");
    return false;
  }
  if (segments.empty())
  {
    RCLCPP_INFO(get_logger(), "[runArcPathSequence] segments 为空，不运动");
    return true;
  }
  if (kArcPathInitialDelaySec > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));

  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    preparePlanningState(velocity_factor, acceleration_factor);
    const std::string eef_link = move_group_->getEndEffectorLink();
    geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    geometry_msgs::msg::Pose p = current_pose.pose;
    for (const CartesianSegment& seg : segments)
    {
      offsetPoseAxis(p, seg.axis, seg.offset);
      if (seg.axis == 'z' && p.position.z < kZMinLimit)
      {
        RCLCPP_WARN(get_logger(), "多段笛卡尔路径 Z 轴安全限位: 段后 z=%.3f < %.2f m，覆盖为 %.2f m 执行", p.position.z,
                    kZMinLimit, kZMinLimit);
        p.position.z = kZMinLimit;
      }
      waypoints.push_back(p);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "多段笛卡尔路径完成度: %.2f%% (尝试 %d/%d)", fraction * 100.0, attempt,
                kArcPathMaxRetries);
    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      const moveit::core::MoveItErrorCode exec_ok = move_group_->execute(plan);
      if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "多段笛卡尔路径执行失败，错误码=%d", exec_ok.val);
        return false;
      }
      RCLCPP_INFO(get_logger(), "多段笛卡尔路径执行完成");
      return true;
    }
    if (attempt < kArcPathMaxRetries)
      std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathRetryDelaySec));
  }
  RCLCPP_ERROR(get_logger(), "多段笛卡尔路径在 %d 次尝试后仍未达 100%%", kArcPathMaxRetries);
  return false;
}

// =============================================================================
// ROS：抓取话题、滑窗、感知采集与循环控制服务
// =============================================================================

void PublishGraspsClientWorker::graspPosesCallback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  // 每组消息入队，超过窗口长度则丢弃最旧一组
  if (msg->poses.empty())
    return;
  geometry_msgs::msg::PoseArray snapshot = *msg;
  std::lock_guard<std::mutex> lock(window_mutex_);
  latest_grasp_poses_ = snapshot;
  grasp_groups_window_.push_back(std::move(snapshot));
  while (grasp_groups_window_.size() > grasp_window_size_)
    grasp_groups_window_.pop_front();
}

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
        RCLCPP_INFO(get_logger(), "抓取窗口就绪: %zu/%zu 组", grasp_groups_window_.size(), grasp_window_size_);
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

// SetBool：true 开始循环；false 请求在本周期结束后停止（节点不退出）
void PublishGraspsClientWorker::handleLoopControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
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

  // false: 当前周期结束后停止循环；若当前不在周期中则立即转为空闲（不退出节点）
  loop_enabled_.store(false);
  stop_after_cycle_.store(true);

  response->success = true;
  response->message = cycle_in_progress_.load() ? "已收到关闭请求，将在当前周期结束后停止循环" :
                                                  "已收到关闭请求，当前无活动周期，已停止循环";
  RCLCPP_INFO(get_logger(), "收到循环控制: stop，%s", response->message.c_str());
}

// =============================================================================
// 位姿几何、选优得分与抓取接近（tip→EEF、多段笛卡尔）
// =============================================================================

// 从滑窗中选一组抓取：垂直模式取「末端竖直度」最高；否则取最新一组第一个位姿
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

// tip（GraspNet 输出）→ 实际规划用末端：沿局部 Z 平移 grasp_z_offset
Eigen::Matrix4d PublishGraspsClientWorker::buildGraspToEndEffectorTransform()
{
  // 抓取点在 tip 系下沿 Z 平移到 EEF（与 grasp_z_offset 参数一致）
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(2, 3) = -grasp_z_offset_;
  return T;
}

geometry_msgs::msg::Pose PublishGraspsClientWorker::applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                                              const Eigen::Matrix4d& transform_local)
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

// GraspNet 预测抓取专属：ori * q_z180，在抓取/工具局部绕 Z 转 π；位置不变。非 GraspNet 来源勿用。
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

// 多段笛卡尔：x→y→抬升到 z_above → 姿态对齐 → 下降至抓取高度；每轮重试前 preparePlanningState + 刷新当前位姿
bool PublishGraspsClientWorker::runGraspApproach(const geometry_msgs::msg::Pose& pose_ee, double height_above,
                                                 float vel, float acc)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] MoveGroup 未初始化");
    return false;
  }

  // pose_ee 来自 GraspNet 话题时做 Z180；见 applyGraspZFlip180 注释。
  geometry_msgs::msg::Pose pose_for_plan = applyGraspZFlip180(pose_ee);
  const std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty())
  {
    RCLCPP_ERROR(get_logger(), "[runGraspApproach] 无法获取末端 link");
    return false;
  }

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
  const geometry_msgs::msg::Quaternion grasp_ori = pose_for_plan.orientation;

  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)
  {
    preparePlanningState(vel, acc);
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose(eef_link).pose;

    Eigen::Quaterniond q_current(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y,
                                 current_pose.orientation.z);
    q_current.normalize();
    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();

    const auto computeGraspApproachOrientation =
        [&](const Eigen::Quaterniond& q_goal_in) -> std::tuple<geometry_msgs::msg::Quaternion, double, double> {
      Eigen::Quaterniond q_goal = q_goal_in;
      q_goal.normalize();
      Eigen::Quaterniond q_delta = q_current.conjugate() * q_goal;
      q_delta.normalize();
      const Eigen::Vector3d q_vec(q_delta.x(), q_delta.y(), q_delta.z());
      const Eigen::Vector3d proj_on_z = z_axis * q_vec.dot(z_axis);
      Eigen::Quaterniond q_twist(q_delta.w(), proj_on_z.x(), proj_on_z.y(), proj_on_z.z());
      if (q_twist.norm() < 1e-9)
        q_twist = Eigen::Quaterniond::Identity();
      else
        q_twist.normalize();
      Eigen::Quaterniond q_swing = q_delta * q_twist.conjugate();
      q_swing.normalize();
      const double delta_yaw_raw = 2.0 * std::atan2(q_twist.z(), q_twist.w());
      const double delta_yaw = normalizeYawToPlusMinusHalfPi(delta_yaw_raw);
      Eigen::Quaterniond q_twist_applied(Eigen::AngleAxisd(delta_yaw, z_axis));
      Eigen::Quaterniond q_delta_applied = q_swing * q_twist_applied;
      q_delta_applied.normalize();
      Eigen::Quaterniond q_target = q_current * q_delta_applied;
      q_target.normalize();
      geometry_msgs::msg::Quaternion grasp_ori_rot;
      grasp_ori_rot.x = q_target.x();
      grasp_ori_rot.y = q_target.y();
      grasp_ori_rot.z = q_target.z();
      grasp_ori_rot.w = q_target.w();
      return { grasp_ori_rot, delta_yaw_raw, delta_yaw };
    };

    Eigen::Quaterniond q_goal(grasp_ori.w, grasp_ori.x, grasp_ori.y, grasp_ori.z);
    geometry_msgs::msg::Quaternion grasp_ori_rot_short;
    double delta_yaw_raw = 0.0;
    double delta_yaw = 0.0;
    std::tie(grasp_ori_rot_short, delta_yaw_raw, delta_yaw) = computeGraspApproachOrientation(q_goal);
    RCLCPP_INFO(get_logger(), "抓取接近旋转: swing–twist 绕世界Z raw=%.4f rad (%.2f°), 归一化±90°=%.4f rad (%.2f°)",
                delta_yaw_raw, delta_yaw_raw * 180.0 / M_PI, delta_yaw, delta_yaw * 180.0 / M_PI);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);

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
    p_rot.orientation = grasp_ori_rot_short;
    waypoints.push_back(p_rot);

    geometry_msgs::msg::Pose p_down;
    p_down.position.x = gx;
    p_down.position.y = gy;
    p_down.position.z = gz;
    p_down.orientation = grasp_ori_rot_short;
    waypoints.push_back(p_down);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction =
        move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    size_t num_points = trajectory.joint_trajectory.points.size();

    RCLCPP_INFO(get_logger(), "抓取接近笛卡尔: fraction=%.2f%%, 点数=%zu (尝试 %d/%d)", fraction * 100.0, num_points,
                attempt, kArcPathMaxRetries);

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

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const moveit::core::MoveItErrorCode exec_ok = move_group_->execute(plan);
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

// 对外封装：用当前 height_above 与关节速度/加速度缩放做抓取接近
bool PublishGraspsClientWorker::runGraspMotion(const geometry_msgs::msg::Pose& target_pose)
{
  return runGraspApproach(target_pose, height_above_, joint_velocity_scaling_, joint_acceleration_scaling_);
}

// 放置 B 点：固定关节角（shoulder…wrist3，与 manipulator 组变量顺序一致）
bool PublishGraspsClientWorker::moveToPlacePointB(float velocity_factor, float acceleration_factor)
{
  if (!move_group_)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_place_B] MoveGroup 未初始化");
    return false;
  }
  preparePlanningState(velocity_factor, acceleration_factor);
  const std::vector<double> joints = { -1.1784, 0.2477, 1.6223, -0.1973, 1.5708, -1.1791 };
  move_group_->setJointValueTarget(joints);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode plan_ok = move_group_->plan(plan);
  if (plan_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_place_B] 规划失败，错误码=%d", plan_ok.val);
    return false;
  }
  const moveit::core::MoveItErrorCode exec_ok = move_group_->execute(plan);
  if (exec_ok != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_place_B] 执行失败，错误码=%d", exec_ok.val);
    return false;
  }
  RCLCPP_INFO(get_logger(), "[move_to_place_B] 已到达 B 点（放置关节目标）");
  return true;
}

void PublishGraspsClientWorker::logCurrentEefPoseAndJoints(const char* stage_label)
{
  if (!stage_label || !move_group_)
    return;
  const std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty())
    return;

  const geometry_msgs::msg::PoseStamped ps = move_group_->getCurrentPose(eef_link);
  const auto& p = ps.pose.position;
  const auto& o = ps.pose.orientation;
  RCLCPP_INFO(get_logger(), "%s frame=%s eef=%s pos=[%.4f, %.4f, %.4f] m quat_xyzw=[%.5f, %.5f, %.5f, %.5f]",
              stage_label, ps.header.frame_id.c_str(), eef_link.c_str(), p.x, p.y, p.z, o.x, o.y, o.z, o.w);

  moveit::core::RobotStatePtr st = move_group_->getCurrentState(1.0);
  if (!st)
    return;
  const moveit::core::JointModelGroup* jmg = st->getJointModelGroup(move_group_->getName());
  if (!jmg)
    return;
  std::vector<double> joints;
  st->copyJointGroupPositions(jmg, joints);
  const std::vector<std::string>& jnames = jmg->getVariableNames();
  std::ostringstream sr;
  std::ostringstream sd;
  sr << std::fixed << std::setprecision(4);
  sd << std::fixed << std::setprecision(2);
  for (size_t i = 0; i < joints.size() && i < jnames.size(); ++i)
  {
    if (i)
    {
      sr << ' ';
      sd << ' ';
    }
    sr << jnames[i] << '=' << joints[i] << "rad";
    sd << jnames[i] << '=' << (joints[i] * 180.0 / M_PI) << "deg";
  }
  RCLCPP_INFO(get_logger(), "%s 关节 %s", stage_label, sr.str().c_str());
  RCLCPP_INFO(get_logger(), "%s 关节 %s", stage_label, sd.str().c_str());
}

// =============================================================================
// 单周期：步骤 -1～11（与文档/状态机一一对应，任一步失败则 failCycle）
// =============================================================================
// 0 回 A 点 → -1 开采集 → 1 清窗 → 2 等窗口 → 3 选优 → 4 tip→EEF → 5 开爪 → 6 接近 → 7 闭爪
// → 8 抬起 → 9 B 点（关节+可选笛卡尔）→ 10 开爪 → 11 回 A 点
bool PublishGraspsClientWorker::runOneCycle()
{
  if (!robot_ || !move_group_) return false;
  struct CycleProgressGuard
  {
    explicit CycleProgressGuard(std::atomic<bool>& in_progress) : in_progress_(in_progress)
    {
      in_progress_.store(true);
    }
    ~CycleProgressGuard()
    {
      in_progress_.store(false);
    }
    CycleProgressGuard(const CycleProgressGuard&) = delete;
    CycleProgressGuard& operator=(const CycleProgressGuard&) = delete;
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

  const auto sleepJointCartesianSwitch = [this, &stopCaptureIfNeeded](const char* where) {
    if (joint_cartesian_switch_delay_sec_ <= 0.0)
      return true;
    RCLCPP_INFO(get_logger(), "%s: 关节↔笛卡尔切换延时 %.3f s", where, joint_cartesian_switch_delay_sec_);
    sleepInterruptible(this, joint_cartesian_switch_delay_sec_);
    if (!rclcpp::ok() || shutdown_requested_)
    {
      stopCaptureIfNeeded();
      return false;
    }
    return true;
  };

  if (!rclcpp::ok() || shutdown_requested_)
    return false;
  RCLCPP_INFO(get_logger(), "周期开始: 预清空抓取窗口（清理上周期残留）");
  clearGraspWindow();
  if (!rclcpp::ok() || shutdown_requested_)
    return false;
  RCLCPP_INFO(get_logger(), "步骤 0: 回 A 点安全位（识别前到位，camera_pose）");
  if (!robot_->moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤0 回 A 点失败，需在识别前到位");
    return failCycle();
  }
  if (!rclcpp::ok() || shutdown_requested_)
    return failCycle();

  RCLCPP_INFO(get_logger(), "步骤 1: 触发感知开始采集");
  if (!requestGraspCapture(true))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤-1 触发感知采集失败");
    return false;
  }
  capture_started = true;

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

  RCLCPP_INFO(get_logger(), "步骤 5: 抓取前开夹爪 (IO=%d, false=打开, 注意与 ExecuteGraspPoseWorker 语义相反, 参见 ivg_utils.io)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过步骤5 夹爪IO（未调用 /aubo_driver/set_io）。"
                "如需真实夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
  }
  else
  {
    if (!robot_->setGripper(gripper_io_index_, false))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤5 抓取前开夹爪失败");
      return failCycle();
    }
  }

  if (!sleepJointCartesianSwitch("步骤 6 前（关节→笛卡尔）"))
    return false;

  RCLCPP_INFO(get_logger(), "步骤 6: 抓取接近 (4 点笛卡尔)");
  if (!runGraspApproach(pose_ee, height_above_, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤6 抓取接近失败");
    return failCycle();
  }

  RCLCPP_INFO(get_logger(), "步骤 7: 闭夹爪 (IO=%d, true=闭合, 语义与开夹爪相反)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过步骤7 夹爪IO（未调用 /aubo_driver/set_io）。"
                "如需真实夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
  }
  else
  {
    if (!robot_->setGripper(gripper_io_index_, true))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤7 闭夹爪失败");
      return failCycle();
    }
  }

  RCLCPP_INFO(get_logger(), "步骤 8: 抬起 (z=%.2f m)", lift_offset_);
  if (!robot_->moveCartesianZ(lift_offset_, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤8 抬起失败");
    return failCycle();
  }

  if (!sleepJointCartesianSwitch("步骤 9 前（笛卡尔→关节，前往 B 点）"))
    return false;

  RCLCPP_INFO(get_logger(), "步骤 9: 移动到 B 点（放置关节目标）");
  if (!moveToPlacePointB(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤9 运动至 B 点失败");
    return failCycle();
  }
  if (!sleepJointCartesianSwitch("步骤 9 B 点关节后（关节→笛卡尔，微调）"))
    return false;
  // B 点后笛卡尔微调（如沿 z）
  const std::vector<CartesianSegment> place_segments = { { 'z', place_offset_z_ } };
  if (!robot_->moveCartesianPath(place_segments, joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤9 B 点后笛卡尔微调失败");
    return failCycle();
  }
  logCurrentEefPoseAndJoints("步骤 9 B 点（开夹爪前）");

  RCLCPP_INFO(get_logger(), "步骤 10: 开夹爪 (IO=%d)", gripper_io_index_);
  if (kSkipTemporaryGripperIo)
  {
    RCLCPP_WARN(get_logger(),
                "⚠ 仿真临时模式：已跳过步骤10 夹爪IO（未调用 /aubo_driver/set_io）。"
                "如需真实夹爪控制，请将 kSkipTemporaryGripperIo 改为 false。");
  }
  else
  {
    if (!robot_->setGripper(gripper_io_index_, false))
    {
      RCLCPP_ERROR(get_logger(), "runOneCycle 步骤10 开夹爪失败");
      return failCycle();
    }
  }

  if (!sleepJointCartesianSwitch("步骤 11 前（笛卡尔→关节，回 A 点）"))
    return false;

  RCLCPP_INFO(get_logger(), "步骤 11: 回 A 点安全位（camera_pose）");
  if (!robot_->moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_))
  {
    RCLCPP_ERROR(get_logger(), "runOneCycle 步骤11 回 A 点失败");
    return failCycle();
  }

  stopCaptureIfNeeded();
  return true;
}

// =============================================================================
// 周期状态 JSON、优雅停机与主循环（loop_enabled / stop_after_cycle / max_cycles）
// =============================================================================

void PublishGraspsClientWorker::publishStatus(int cycle_count, int success_count, int fail_count, int last_step,
                                              bool is_running)
{
  std_msgs::msg::String msg;
  std::ostringstream ss;
  ss << "{\"cycle_count\":" << cycle_count << ",\"success_count\":" << success_count << ",\"fail_count\":" << fail_count
     << ",\"last_step\":" << last_step << ",\"is_running\":" << (is_running ? "true" : "false") << "}";
  msg.data = ss.str();
  status_pub_->publish(msg);
}

void PublishGraspsClientWorker::onShutdown()
{
  RCLCPP_INFO(get_logger(), "onShutdown: 回 A 点安全位%s",
              kSkipTemporaryGripperIo ? "（仿真模式：跳过开夹爪IO）" : "、开夹爪");
  if (!kSkipTemporaryGripperIo)
  {
    robot_->setGripper(gripper_io_index_, false);
  }
  robot_->moveToHome(joint_velocity_scaling_, joint_acceleration_scaling_);
}

void PublishGraspsClientWorker::requestShutdown()
{
  shutdown_requested_ = true;
  RCLCPP_INFO(get_logger(), "收到退出请求，主循环将尽快退出");
}

// loop_enabled 为 false 时空转并发布状态；为 true 时反复调用 runOneCycle，支持「本周期结束后停止」
bool PublishGraspsClientWorker::run()
{
  RCLCPP_INFO(get_logger(), "主循环启动，max_cycles=%d，Ctrl+C 可随时退出，初始循环状态=%s", max_cycles_,
              loop_enabled_.load() ? "running" : "idle");
  while (rclcpp::ok() && !shutdown_requested_ && (max_cycles_ < 0 || cycle_count_ < max_cycles_))
  {
    if (!loop_enabled_.load())
    {
      publishStatus(cycle_count_, success_count_, fail_count_, 0, false);
      sleepInterruptible(this, 0.2);
      // stop_after_cycle_ 仅用于“当前周期后停循环”，停稳后清标志，节点保持 idle 可再次 start。
      if (stop_after_cycle_.load() && !cycle_in_progress_.load())
        stop_after_cycle_.store(false);
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
    {
      loop_enabled_.store(false);
      stop_after_cycle_.store(false);
      RCLCPP_INFO(get_logger(), "已按请求在当前周期结束后停止循环，节点保持空闲可再次启动");
    }
    sleepInterruptible(this, cycle_delay_sec_);
  }
  publishStatus(cycle_count_, success_count_, fail_count_, 0, false);
  RCLCPP_INFO(get_logger(), "主循环退出，共 %d 周期，成功 %d，失败 %d", cycle_count_, success_count_, fail_count_);
  return true;
}

}  // namespace demo_driver

// =============================================================================
// 程序入口：多线程 executor + 信号 + JoinGuard，保证任一路径退出均 join spin
// =============================================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<demo_driver::PublishGraspsClientWorker>(options);
  node->initRobot();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);

  std::thread spinner([&executor]() { executor.spin(); });
  // main 任一路径返回前 join spin 线程（含早退与异常栈展开）
  struct SpinnerJoinGuard
  {
    std::thread& th;
    ~SpinnerJoinGuard()
    {
      if (th.joinable())
        th.join();
    }
    SpinnerJoinGuard(const SpinnerJoinGuard&) = delete;
    SpinnerJoinGuard& operator=(const SpinnerJoinGuard&) = delete;
  } join_spinner{ spinner };

  demo_driver::g_worker_for_signal = node.get();
  std::signal(SIGINT, demo_driver::sigintHandler);
  std::signal(SIGTERM, demo_driver::sigintHandler);

  const int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "PublishGraspsClientWorker 就绪，进入循环抓取放置");
  node->run();

  demo_driver::g_worker_for_signal = nullptr;
  node->onShutdown();
  rclcpp::shutdown();
  return 0;
}
