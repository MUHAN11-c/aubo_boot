/**
 * @file moveit_arcline_demo.cpp
 * @brief 三夹爪快换 Demo：通过 MoveGroup 笛卡尔路径与位姿/关节服务，依次对三个夹爪执行到位、下降、延时、上升、回 home。
 */

#include "demo_driver/moveit_arcline_demo.h"

#include <cmath>
#include <future>
#include <thread>
#include <vector>

#include <moveit/robot_state/robot_state.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>

/** 任一步失败则 return false，用于 run() 中链式检查 */
#define CHECK(expr) do { if (!(expr)) return false; } while (0)

namespace demo_driver
{
// -----------------------------------------------------------------------------
//  静态常量定义
// -----------------------------------------------------------------------------

/** 夹爪 1 快换用 home 关节角（6 轴弧度），用于 moveToJoints */
const std::array<double, 6> MoveitArclineDemo::kHomeJointsRad1 = {
  1.210212, 0.129677, 1.925533, 0.225356, 1.571783, 1.209540
};
/** 位姿运动服务名，由 move_to_pose_server 提供 */
const std::string MoveitArclineDemo::kMoveToPoseService = "/move_to_pose";
/** 关节运动服务名，由 move_to_pose_server 提供 */
const std::string MoveitArclineDemo::kMoveToJointsService = "/move_to_joints";
/** 机器人 IO 设置服务名，由 set_robot_io_server 提供，用于数字输出等 */
const std::string MoveitArclineDemo::kSetRobotIOService = "/demo_driver/set_io";

/** 笛卡尔路径规划未达 100% 时最大重试次数 */
static constexpr int kArcPathMaxRetries = 5;
/** 每次重试前等待秒数，便于关节/场景状态更新 */
static constexpr int kArcPathRetryDelaySec = 2;
/** 首次规划前等待秒数，使 joint_states 与规划场景同步 */
static constexpr double kArcPathInitialDelaySec = 0.2;
/** 笛卡尔路径步长（m），末端每步位移，越小路径越密 */
static constexpr double kCartesianEefStep = 0.01;
/** 笛卡尔路径跳变阈值，0 表示不允许跳变 */
static constexpr double kCartesianJumpThreshold = 0.0;

// -----------------------------------------------------------------------------
//  构造与初始化
// -----------------------------------------------------------------------------

MoveitArclineDemo::MoveitArclineDemo(const rclcpp::NodeOptions& options)
  : rclcpp::Node("moveit_arcline_demo", options)
{
  move_to_pose_client_ = create_client<demo_interface::srv::MoveToPose>(kMoveToPoseService);
  move_to_joints_client_ = create_client<demo_interface::srv::MoveToJoints>(kMoveToJointsService);
  set_robot_io_client_ = create_client<demo_interface::srv::SetRobotIO>(kSetRobotIOService);
}

std::shared_ptr<MoveitArclineDemo> MoveitArclineDemo::create(const rclcpp::NodeOptions& options)
{
  auto node = std::make_shared<MoveitArclineDemo>(options);
  // 构造完成后创建 MoveGroup，避免在构造函数内使用 shared_from_this() 导致 bad_weak_ptr
  node->initMoveGroup();
  return node;
}

void MoveitArclineDemo::initMoveGroup()
{
  move_group_ =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
  move_group_->allowReplanning(true);
  move_group_->setMaxVelocityScalingFactor(0.5);
}

// -----------------------------------------------------------------------------
//  服务就绪检查
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::waitForServices(std::chrono::seconds timeout)
{
  // timeout: 等待服务就绪的最长时间，超时则返回 false
  if (!move_to_pose_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(get_logger(), "服务 %s 未就绪，请先启动 move_to_pose_server_node", kMoveToPoseService.c_str());
    return false;
  }
  if (!move_to_joints_client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(get_logger(), "服务 %s 未就绪，请先启动 move_to_joints 相关节点", kMoveToJointsService.c_str());
    return false;
  }
  if (!set_robot_io_client_->wait_for_service(timeout))
  {
    RCLCPP_WARN(get_logger(), "服务 %s 未就绪，setDigitalOutput 不可用，请先启动 set_robot_io_server_node",
                kSetRobotIOService.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "SetRobotIO 服务已就绪");
  }
  RCLCPP_INFO(get_logger(), "所需服务已就绪");
  return true;
}

// -----------------------------------------------------------------------------
//  运动服务封装：move_to_joints / move_to_pose
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::moveToJoints(const std::array<double, 6>& joint_positions_rad,
                                     float velocity_factor,
                                     float acceleration_factor)
{
  // joint_positions_rad: 目标 6 轴关节角（弧度）
  // velocity_factor: 速度缩放 [0,1]，相对最大速度的比例
  // acceleration_factor: 加速度缩放 [0,1]，相对最大加速度的比例
  auto req = std::make_shared<demo_interface::srv::MoveToJoints::Request>();
  req->joint_positions_rad.assign(joint_positions_rad.begin(), joint_positions_rad.end());
  req->velocity_factor = velocity_factor;
  req->acceleration_factor = acceleration_factor;

  auto future = move_to_joints_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_joints] 调用超时或失败");
    return false;
  }

  auto res = future.get();  // res->success: 是否成功; res->error_code, res->message: 错误信息
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_joints] 失败: error_code=%d %s", res->error_code, res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[move_to_joints] 成功");
  return true;
}

bool MoveitArclineDemo::moveToPose(double x, double y, double z,
                                   double qx, double qy, double qz, double qw,
                                   bool use_joints,
                                   float velocity_factor,
                                   float acceleration_factor)
{
  // x,y,z: 目标位置（m），在基坐标系下
  // qx,qy,qz,qw: 目标姿态四元数
  // use_joints: true 用关节插值，false 用笛卡尔/位姿插值
  // velocity_factor, acceleration_factor: 速度/加速度缩放 [0,1]
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

  auto res = future.get();  // res->success, res->error_code, res->message
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[move_to_pose] 失败: error_code=%d %s", res->error_code, res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[move_to_pose] 成功");
  return true;
}

// -----------------------------------------------------------------------------
//  数字输出（SetRobotIO 客户端）
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::setDigitalOutput(int32_t io_index, bool high)
{
  // io_index: 数字输出引脚索引（>=0），对应机器人控制柜 DO 点
  // high: true 输出高电平(1)，false 输出低电平(0)，可用于夹爪/电磁阀等
  if (!set_robot_io_client_->service_is_ready())
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] 服务 %s 不可用", kSetRobotIOService.c_str());
    return false;
  }
  if (io_index < 0)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] io_index 必须 >= 0，当前为 %d", io_index);
    return false;
  }

  auto req = std::make_shared<demo_interface::srv::SetRobotIO::Request>();
  req->io_type = "digital_output";
  req->io_index = io_index;
  req->value = high ? 1.0 : 0.0;

  auto future = set_robot_io_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(kCallTimeoutSeconds)) != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] 调用超时或失败");
    return false;
  }

  auto res = future.get();
  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "[set_digital_output] 失败: error_code=%d %s", res->error_code, res->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[set_digital_output] pin %d -> %s", io_index, high ? "HIGH" : "LOW");
  return true;
}

// -----------------------------------------------------------------------------
//  笛卡尔直线路径（沿单轴位移，用于快换时的下降/上升）
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::runArcPath(double z_offset)
{
  // z_offset: 沿 Z 轴位移量（m），负值下降、正值上升
  return runArcPath('z', z_offset);
}

bool MoveitArclineDemo::runArcPath(char axis, double offset)
{
  // axis: 位移轴 'x'/'y'/'z'；offset: 该轴方向位移量（m），正负表示方向
  const char* axis_label = (axis == 'x') ? "X" : (axis == 'y') ? "Y" : "Z";
  RCLCPP_INFO(get_logger(), "笛卡尔路径: 沿 %s 轴 %+.2f m", axis_label, offset);

  if (kArcPathInitialDelaySec > 0)
  {
    RCLCPP_INFO(get_logger(), "等待 %.1f 秒以便当前状态同步...", kArcPathInitialDelaySec);
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));
  }

  const std::string eef_link = move_group_->getEndEffectorLink();  // 末端连杆名，如 tool_tcp
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);  // 当前末端位姿
  RCLCPP_INFO(get_logger(),
              "当前末端 '%s': pos(%.4f, %.4f, %.4f) ori(%.4f, %.4f, %.4f, %.4f)",
              eef_link.c_str(),
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
              current_pose.pose.orientation.x, current_pose.pose.orientation.y,
              current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  moveit::core::RobotStatePtr robot_state = move_group_->getCurrentState();  // 当前机器人状态
  if (robot_state)
  {
    const moveit::core::JointModelGroup* jmg = robot_state->getJointModelGroup("manipulator");  // 规划组
    if (jmg)
    {
      std::vector<double> joint_positions_rad;   // 当前 6 轴关节角（弧度）
      robot_state->copyJointGroupPositions(jmg, joint_positions_rad);
      std::vector<std::string> joint_names = jmg->getActiveJointModelNames();  // 关节名顺序
      RCLCPP_INFO(get_logger(), "当前关节 (rad):");
      for (size_t i = 0; i < joint_positions_rad.size() && i < joint_names.size(); ++i)
        RCLCPP_INFO(get_logger(), "  %s = %.6f", joint_names[i].c_str(), joint_positions_rad[i]);
      if (joint_positions_rad.size() >= 6)
      {
        RCLCPP_INFO(get_logger(),
                    "moveToJoints({%.6f, %.6f, %.6f, %.6f, %.6f, %.6f}, 0.5f, 0.5f);",
                    joint_positions_rad[0], joint_positions_rad[1], joint_positions_rad[2],
                    joint_positions_rad[3], joint_positions_rad[4], joint_positions_rad[5]);
      }
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "无法获取当前关节状态 (getCurrentState 为空)");
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;   // 笛卡尔路径途经点：起点 + 终点
  waypoints.push_back(current_pose.pose);
  geometry_msgs::msg::Pose target_pose = current_pose.pose;  // 终点 = 当前位姿 + 沿 axis 偏移 offset
  if (axis == 'x')
    target_pose.position.x += offset;
  else if (axis == 'y')
    target_pose.position.y += offset;
  else
    target_pose.position.z += offset;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;  // 规划得到的关节轨迹
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)  // attempt: 当前重试次数
  {
    double fraction =  // 规划完成度 [0,1]，1 表示 100% 可达
      move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）完成度: %.2f%% (尝试 %d/%d)",
                axis_label, offset, fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;  // 可执行的运动计划
      plan.trajectory_ = trajectory;
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "%s 轴直线移动执行完成", axis_label);
      return true;
    }

    if (attempt < kArcPathMaxRetries)
    {
      RCLCPP_WARN(get_logger(), "规划未达 100%%，%d 秒后重试...", kArcPathRetryDelaySec);
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
    }
  }

  RCLCPP_ERROR(get_logger(), "笛卡尔路径（%s 轴 %+.2f m）在 %d 次尝试后仍未达 100%%",
               axis_label, offset, kArcPathMaxRetries);
  return false;
}

// -----------------------------------------------------------------------------
//  多段单轴笛卡尔路径一次规划、一次执行
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::runArcPathSequence(const std::vector<CartesianSegment>& segments)
{
  // segments: 多段 (axis, offset)，按顺序累加得到 waypoints；空则视为成功且不运动
  if (segments.empty())
  {
    RCLCPP_INFO(get_logger(), "[runArcPathSequence] segments 为空，不运动");
    return true;
  }

  RCLCPP_INFO(get_logger(), "多段笛卡尔路径，共 %zu 段", segments.size());
  for (size_t i = 0; i < segments.size(); ++i)
  {
    const char* label = (segments[i].axis == 'x') ? "X" : (segments[i].axis == 'y') ? "Y" : "Z";
    RCLCPP_INFO(get_logger(), "  段 %zu: %s 轴 %+.3f m", i + 1, label, segments[i].offset);
  }

  // 首次规划前等待，使 joint_states 与规划场景同步（与 runArcPath 一致）
  if (kArcPathInitialDelaySec > 0)
  {
    RCLCPP_INFO(get_logger(), "等待 %.1f 秒以便当前状态同步...", kArcPathInitialDelaySec);
    std::this_thread::sleep_for(std::chrono::duration<double>(kArcPathInitialDelaySec));
  }

  const std::string eef_link = move_group_->getEndEffectorLink();   // 末端连杆名，如 tool_tcp
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(eef_link);  // 当前末端位姿

  // 构建 waypoints：起点 + 每段累加后的位姿（每段仅沿 seg.axis 偏移 seg.offset）
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

  moveit_msgs::msg::RobotTrajectory trajectory;  // 规划得到的关节轨迹
  for (int attempt = 1; attempt <= kArcPathMaxRetries; ++attempt)  // attempt: 当前重试次数
  {
    double fraction =  // 规划完成度 [0,1]，1 表示 100% 可达
      move_group_->computeCartesianPath(waypoints, kCartesianEefStep, kCartesianJumpThreshold, trajectory);
    RCLCPP_INFO(get_logger(), "多段笛卡尔路径完成度: %.2f%% (尝试 %d/%d)",
                fraction * 100.0, attempt, kArcPathMaxRetries);

    if (fraction >= 1.0)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;  // 可执行的运动计划
      plan.trajectory_ = trajectory;
      move_group_->execute(plan);  // 只执行一次
      RCLCPP_INFO(get_logger(), "多段笛卡尔路径执行完成");
      return true;
    }

    if (attempt < kArcPathMaxRetries)
    {
      RCLCPP_WARN(get_logger(), "规划未达 100%%，%d 秒后重试...", kArcPathRetryDelaySec);
      std::this_thread::sleep_for(std::chrono::seconds(kArcPathRetryDelaySec));
    }
  }

  RCLCPP_ERROR(get_logger(), "多段笛卡尔路径在 %d 次尝试后仍未达 100%%", kArcPathMaxRetries);
  return false;
}

// -----------------------------------------------------------------------------
//  三夹爪快换主流程 run()
//
//  每个夹爪流程：到位 -> 沿 Z 下降 -> 延时 -> 沿 Z 上升 ->（最后一个夹爪）回 home(camera_pose)。
//  夹爪 0/1 的代码已注释，当前仅执行夹爪 2 的快换。
// -----------------------------------------------------------------------------

bool MoveitArclineDemo::run()
{
  // ----- 夹爪 0 快换换下 -----
  // moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.15f, 0.1f);
  // runArcPath(-0.255);
  // setDigitalOutput(7, true);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  // runArcPath(0.255);
  // std::this_thread::sleep_for(std::chrono::seconds(2));
  // setDigitalOutput(7, false);

    // ----- 夹爪 0 快换换上 -----
  // moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.1f, 0.1f);
  // setDigitalOutput(7, true);
  // runArcPath(-0.255);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  // setDigitalOutput(7, false);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  // runArcPath(0.255);
  // std::this_thread::sleep_for(std::chrono::seconds(2));


//   ----- 夹爪 1 快换（已注释） -----
  // moveToJoints(kHomeJointsRad1, 0.15f, 0.1f);
  // runArcPath(-0.255);
  // std::this_thread::sleep_for(std::chrono::seconds(2));
  // runArcPath(0.255);
  // // runArcPathSequence({{'z', -0.255}, {'z', 0.255}});
  // move_group_->setNamedTarget("camera_pose");
  // move_group_->move();

  // ----- 夹爪 2 快换（当前执行） -----
  //当前末端 'tool_tcp': pos(0.4690, 0.3594, 0.1204) ori(0.7067, 0.7075, -0.0000, -0.0003)
  // moveToPose(0.36767 - 0.003 + 0.105, 0.24267 + 0.105 + 0.012, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002,
  //                 -0.0005, false, 0.15f, 0.1f);
  moveToJoints({0.860766, -0.265055, 1.501074, 0.195106, 1.571464, 0.859643}, 0.15f, 0.1f);
  // runArcPath('y',0.1);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  // runArcPath('z',-0.243);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  // runArcPath('y',-0.1);
  // std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
  // runArcPath('z',-0.10);
  // std::this_thread::sleep_for(std::chrono::duration<double>(5));
  const double y_step = 0.1;
  std::vector<CartesianSegment> segments = {
    {'y', y_step},
    {'z', -0.243},
    {'y', -y_step},
    {'z', -0.012}
  };
  CHECK(runArcPathSequence(segments));
  setDigitalOutput(7, true);
  std::this_thread::sleep_for(std::chrono::duration<double>(1));
  std::vector<CartesianSegment> segments1 = {
    {'z', 0.012},
    {'y', y_step},
    {'z', 0.243}
  };
  CHECK(runArcPathSequence(segments1));
  setDigitalOutput(7, false);

      // ----- 夹爪 0 快换换上 -----
  moveToPose(0.36767 - 0.003, 0.24267 + 0.105, 0.0405 + 0.185 + 0.1, 0.7068, 0.7074, 0.0002, -0.0005, false, 0.1f, 0.1f);
  setDigitalOutput(7, true);
  runArcPath(-0.255);
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  setDigitalOutput(7, false);
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
  runArcPath(0.255);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // RCLCPP_INFO(get_logger(), "夹爪 2 快换完成，回到 camera_pose");
  // move_group_->setNamedTarget("camera_pose");
  // move_group_->move();

  return true;
}

}  // namespace demo_driver

// -----------------------------------------------------------------------------
//  main：创建节点、多线程执行、等待服务、执行三夹爪快换
// -----------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = demo_driver::MoveitArclineDemo::create(options);  // 三夹爪快换 Demo 节点

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);  // 2 线程处理回调
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });  // 后台旋转，使服务客户端等可用

  const int kServiceWaitSec = 10;  // 等待 move_to_pose / move_to_joints / set_io 就绪的秒数
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool ok = node->run();  // 执行三夹爪快换流程，成功为 true

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
