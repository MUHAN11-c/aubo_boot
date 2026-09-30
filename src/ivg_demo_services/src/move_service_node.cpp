// move_service_node — 演示栈手动运动/状态/IO 读服务。
// 合并自 aubo_boot demo_driver 的 move_to_pose / get_current_state /
// read_robot_io / set_robot_enable / set_speed_factor 五个 server
// （2026-09-30 按需补缺移植，语义映射见包 README）。
#include <aubo_msgs/msg/io_state.hpp>
#include <ivg_demo_services/robot_controller.hpp>
#include <ivg_interfaces/srv/get_current_state.hpp>
#include <ivg_interfaces/srv/move_to_pose.hpp>
#include <ivg_interfaces/srv/read_robot_io.hpp>
#include <ivg_interfaces/srv/set_robot_enable.hpp>
#include <ivg_interfaces/srv/set_speed_factor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace ivg_demo_services
{

using MoveToPose = ivg_interfaces::srv::MoveToPose;
using GetCurrentState = ivg_interfaces::srv::GetCurrentState;
using ReadRobotIO = ivg_interfaces::srv::ReadRobotIO;
using SetRobotEnable = ivg_interfaces::srv::SetRobotEnable;
using SetSpeedFactor = ivg_interfaces::srv::SetSpeedFactor;

class MoveServiceNode : public rclcpp::Node
{
public:
  MoveServiceNode()
  : Node("move_service_node")
  {
    // 参数先声明，RobotController 构造时读取 io_simulated/home_target
    const std::string planning_group =
      declare_parameter<std::string>("planning_group", "manipulator_e5");
    declare_parameter<bool>("io_simulated", false);
    declare_parameter<std::string>("home_target", "camera_pose");
    // 本仓 moveit 默认管线=pilz，Pilz 需显式 planner_id（移植适配点）
    declare_parameter<std::string>(
      "planning_pipeline", "pilz_industrial_motion_planner");
    declare_parameter<std::string>("planner_id", "PTP");
    default_vel_ = declare_parameter<double>("default_velocity_scaling", 0.5);
    default_acc_ = declare_parameter<double>("default_acceleration_scaling", 0.5);
    robot_ = std::make_unique<RobotController>(this, planning_group);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        latest_joint_state_ = *msg;
      });
    io_state_sub_ = create_subscription<aubo_msgs::msg::IOState>(
      "/aubo_io_controller/io_states", 10,
      [this](aubo_msgs::msg::IOState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(io_mutex_);
        latest_io_state_ = *msg;
      });
    startup_client_ = create_client<std_srvs::srv::Trigger>("/aubo/startup");
    stop_client_ = create_client<std_srvs::srv::Trigger>("/aubo/stop");

    // 每个服务独立回调组：长运动不互相阻塞（MultiThreadedExecutor）
    move_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    query_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto move_srv_cb = [this](
      const MoveToPose::Request::SharedPtr req,
      MoveToPose::Response::SharedPtr resp) {handleMoveToPose(req, resp);};
    move_to_pose_srv_ = create_service<MoveToPose>(
      "/move_to_pose", move_srv_cb,
      rclcpp::ServicesQoS(), move_grp_);
    debug_move_srv_ = create_service<MoveToPose>(
      "/debug/move_to_xyz", move_srv_cb,
      rclcpp::ServicesQoS(), move_grp_);

    get_state_srv_ = create_service<GetCurrentState>(
      "/get_current_state",
      [this](const GetCurrentState::Request::SharedPtr, GetCurrentState::Response::SharedPtr resp) {
        handleGetCurrentState(resp);
      }, rclcpp::ServicesQoS(), query_grp_);
    read_io_srv_ = create_service<ReadRobotIO>(
      "/read_robot_io",
      [this](const ReadRobotIO::Request::SharedPtr req, ReadRobotIO::Response::SharedPtr resp) {
        handleReadRobotIO(req, resp);
      }, rclcpp::ServicesQoS(), query_grp_);
    set_enable_srv_ = create_service<SetRobotEnable>(
      "/set_robot_enable",
      [this](const SetRobotEnable::Request::SharedPtr req,
      SetRobotEnable::Response::SharedPtr resp) {
        handleSetRobotEnable(req, resp);
      }, rclcpp::ServicesQoS(), query_grp_);
    set_speed_srv_ = create_service<SetSpeedFactor>(
      "/set_speed_factor",
      [this](const SetSpeedFactor::Request::SharedPtr req,
      SetSpeedFactor::Response::SharedPtr resp) {
        handleSetSpeedFactor(req, resp);
      }, rclcpp::ServicesQoS(), query_grp_);
  }

  bool initController() {return robot_->init();}

private:
  float clampFactor(float v, float fallback) const
  {
    if (v <= 0.0f) {return static_cast<float>(fallback);}
    return v > 1.0f ? 1.0f : v;
  }

  void handleMoveToPose(
    const MoveToPose::Request::SharedPtr req,
    MoveToPose::Response::SharedPtr resp)
  {
    if (!robot_->init()) {
      resp->success = false;
      resp->message = "MoveGroupInterface 初始化失败（move_group 未运行？）";
      return;
    }
    const float vel = clampFactor(req->velocity_factor, default_vel_);
    const float acc = clampFactor(req->acceleration_factor, default_acc_);
    bool ok = false;
    if (req->use_joints) {
      std::array<double, 6> joints{};
      for (size_t i = 0; i < 6; ++i) {joints[i] = req->target_joints[i];}
      ok = robot_->moveToJoints(joints, vel, acc);
    } else {
      ok = robot_->moveCartesianStraight(req->target_pose, vel, acc);
    }
    resp->success = ok;
    resp->error_code = ok ? 0 : 1;
    resp->message = ok ? "运动完成" : "运动失败（规划或执行失败，见日志）";
  }

  void handleGetCurrentState(GetCurrentState::Response::SharedPtr resp)
  {
    auto joints = robot_->getCurrentJoints();
    if (joints.empty()) {
      // 状态不可用时不再调 getCurrentPose（其内部同样依赖 CSM，防空引用）
      resp->success = false;
      resp->message = "当前状态不可用（/joint_states 未达或 CSM 冷启动）";
      return;
    }
    auto pose = robot_->getCurrentPose();  // 可能阻塞 ~1s，勿持 joint_mutex_
    std::lock_guard<std::mutex> lock(joint_mutex_);
    resp->success = true;
    resp->joint_position_rad = joints;
    resp->cartesian_position = pose;
    if (latest_joint_state_.has_value() &&
      latest_joint_state_->velocity.size() == joints.size())
    {
      resp->velocity = latest_joint_state_->velocity;
    } else {
      resp->velocity.assign(joints.size(), 0.0);
    }
    resp->message = resp->success ? "OK" : "获取失败（move_group 未运行？）";
  }

  void handleReadRobotIO(
    const ReadRobotIO::Request::SharedPtr req,
    ReadRobotIO::Response::SharedPtr resp)
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!latest_io_state_.has_value()) {
      resp->success = false;
      resp->message = "尚无 /aubo_io_controller/io_states（mock 栈无 IO 控制器）";
      return;
    }
    const auto & io = *latest_io_state_;
    const int idx = req->io_index;
    const bool is_analog =
      (req->io_type == "analog") || (req->io_type == "analog_out");
    const bool is_do = (req->io_type == "digital_out");
    if (is_analog) {
      const auto & ch =
        (req->io_type == "analog") ? io.analog_in_states : io.analog_out_states;
      if (idx < 0 || static_cast<size_t>(idx) >= ch.size()) {
        resp->success = false;
        resp->message = "索引越界";
        return;
      }
      resp->success = true;
      resp->value = ch[idx].state;
    } else {
      const auto & ch = is_do ? io.digital_out_states : io.digital_in_states;
      if (idx < 0 || static_cast<size_t>(idx) >= ch.size()) {
        resp->success = false;
        resp->message = "索引越界";
        return;
      }
      resp->success = true;
      resp->value = ch[idx].state ? 1.0 : 0.0;
    }
    resp->message = "OK";
  }

  void handleSetRobotEnable(
    const SetRobotEnable::Request::SharedPtr req, SetRobotEnable::Response::SharedPtr resp)
  {
    auto client = req->enable ? startup_client_ : stop_client_;
    const char * name = req->enable ? "/aubo/startup" : "/aubo/stop";
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      resp->success = false;
      resp->error_code = 1;
      resp->message = std::string(name) + " 不可达（aubo_dashboard 未运行）";
      return;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
      resp->success = false;
      resp->error_code = 2;
      resp->message = std::string(name) + " 超时";
      return;
    }
    resp->success = future.get()->success;
    resp->error_code = resp->success ? 0 : 3;
    resp->message = future.get()->message;
    RCLCPP_INFO(get_logger(), "set_robot_enable(%d) → %s: %s",
      req->enable ? 1 : 0, name, resp->message.c_str());
  }

  void handleSetSpeedFactor(
    const SetSpeedFactor::Request::SharedPtr req, SetSpeedFactor::Response::SharedPtr resp)
  {
    float v = req->velocity_factor;
    if (v <= 0.0f) {
      v = static_cast<float>(default_vel_);
    } else if (v > 1.0f) {
      v = 1.0f;
    }
    default_vel_ = v;
    resp->success = true;
    resp->message = "默认速度倍率已设为 " + std::to_string(v);
  }

  std::unique_ptr<RobotController> robot_;
  double default_vel_;
  double default_acc_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<aubo_msgs::msg::IOState>::SharedPtr io_state_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr startup_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

  rclcpp::CallbackGroup::SharedPtr move_grp_;
  rclcpp::CallbackGroup::SharedPtr query_grp_;
  rclcpp::Service<MoveToPose>::SharedPtr move_to_pose_srv_;
  rclcpp::Service<MoveToPose>::SharedPtr debug_move_srv_;
  rclcpp::Service<GetCurrentState>::SharedPtr get_state_srv_;
  rclcpp::Service<ReadRobotIO>::SharedPtr read_io_srv_;
  rclcpp::Service<SetRobotEnable>::SharedPtr set_enable_srv_;
  rclcpp::Service<SetSpeedFactor>::SharedPtr set_speed_srv_;

  std::mutex joint_mutex_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  std::mutex io_mutex_;
  std::optional<aubo_msgs::msg::IOState> latest_io_state_;
};

}  // namespace ivg_demo_services

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ivg_demo_services::MoveServiceNode>();
  // MoveGroupInterface 构造耗时长（等 move_group）。顺序必须是先 initController
  // 再 add_node：MGI 构造内部对已挂执行器的节点会抛异常，先挂执行器=初始化永败。
  if (!node->initController()) {
    RCLCPP_WARN(
      node->get_logger(),
      "MoveGroupInterface 初始化失败（move_group 未起？）；首个请求会重试");
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
