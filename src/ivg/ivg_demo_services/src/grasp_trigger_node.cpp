// grasp_trigger_node — 视觉抓取触发（单次 + 循环）。
// 源自 aubo_boot demo_driver 的 execute_grasp_pose_worker 核心序列
// （2026-09-30 按需补缺移植）：视觉估计源从旧 VPE 客户端改为订阅
// ivg_graspnet 的 /grasp_poses_base（PoseArray，base 系），选最垂直抓取；
// 运动序列保真：回安全位→构建抓取位姿→Z 偏移→开夹爪→X→Y→抬升→旋转→下降
// →闭夹爪→抬起→放置偏移→开夹爪。
#include <ivg_demo_services/motion_utils.hpp>
#include <ivg_demo_services/robot_controller.hpp>
#include <ivg_interfaces/srv/execute_grasp_pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace ivg_demo_services
{

using ExecuteGraspPose = ivg_interfaces::srv::ExecuteGraspPose;

class GraspTriggerNode : public rclcpp::Node
{
public:
  GraspTriggerNode()
  : Node("grasp_trigger_node")
  {
    const std::string planning_group =
      declare_parameter<std::string>("planning_group", "manipulator_e5");
    declare_parameter<bool>("io_simulated", false);
    declare_parameter<std::string>("home_target", "camera_pose");
    // 本仓 moveit 默认管线=pilz，Pilz 需显式 planner_id（移植适配点）
    declare_parameter<std::string>(
      "planning_pipeline", "pilz_industrial_motion_planner");
    declare_parameter<std::string>("planner_id", "PTP");

    // egp_* 参数名与 aubo_boot 一致（操作员肌肉记忆零破坏）
    grasp_z_offset_ = declare_parameter<double>("egp_grasp_z_offset", 0.01);
    vel_ = declare_parameter<double>("egp_joint_velocity_scaling", 0.7);
    acc_ = declare_parameter<double>("egp_joint_acceleration_scaling", 0.7);
    gripper_io_index_ = declare_parameter<int>("egp_gripper_io_index", 6);
    lift_offset_ = declare_parameter<double>("egp_lift_offset", 0.2);
    place_offset_x_ = declare_parameter<double>("egp_place_offset_x", -0.2);
    place_offset_y_ = declare_parameter<double>("egp_place_offset_y", -0.2);
    place_offset_z_ = declare_parameter<double>("egp_place_offset_z", -0.15);
    cartesian_max_points_ = declare_parameter<int>("egp_cartesian_max_points", 40);
    height_above_ = declare_parameter<double>("egp_height_above", 0.05);
    z_min_limit_ = declare_parameter<double>("egp_z_min_limit", 0.05);
    wait_poses_timeout_ = declare_parameter<double>("egp_wait_poses_timeout_sec", 10.0);
    loop_interval_sec_ = declare_parameter<double>("egp_loop_interval_sec", 1.0);
    grasp_poses_topic_ = declare_parameter<std::string>("grasp_poses_topic", "/grasp_poses_base");
    // 预设抓取位姿（use_visual_estimation=false 时使用；对齐 aubo_boot 旧参数用途）
    preset_x_ = declare_parameter<double>("egp_preset_grasp_x", 0.0);
    preset_y_ = declare_parameter<double>("egp_preset_grasp_y", 0.0);
    preset_z_ = declare_parameter<double>("egp_preset_grasp_z", 0.0);

    robot_ = std::make_unique<RobotController>(this, planning_group);
    robot_->setZMinLimit(z_min_limit_);

    poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      grasp_poses_topic_, 10,
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        latest_poses_ = msg;
      });

    exec_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ctrl_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    execute_srv_ = create_service<ExecuteGraspPose>(
      "/execute_single_grasp",
      [this](
        const ExecuteGraspPose::Request::SharedPtr req,
        ExecuteGraspPose::Response::SharedPtr resp) {handleExecute(req, resp);},
      rclcpp::ServicesQoS(), exec_grp_);
    auto loop_cb = [this](
      const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr resp) {
        loop_running_ = req->data;
        resp->success = true;
        resp->message = req->data ? "抓取循环已启动" : "抓取循环已停止";
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
      };
    loop_srv_ = create_service<std_srvs::srv::SetBool>(
      "/loop_grasp_control", loop_cb,
      rclcpp::ServicesQoS(), ctrl_grp_);
    publish_loop_srv_ = create_service<std_srvs::srv::SetBool>(
      "/publish_grasps_worker_loop_control", loop_cb,
      rclcpp::ServicesQoS(), ctrl_grp_);

    loop_thread_ = std::thread([this] {loopWorker();});
    RCLCPP_INFO(get_logger(), "grasp_trigger_node 就绪（订 %s）", grasp_poses_topic_.c_str());
  }

  ~GraspTriggerNode() override
  {
    loop_running_ = false;
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
  }

  bool initController() {return robot_->init();}

private:
  /// 等待新的抓取位姿并选最垂直的一个；超时返回 false
  bool waitFreshGraspPose(geometry_msgs::msg::Pose & out)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(wait_poses_timeout_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        if (latest_poses_ && !latest_poses_->poses.empty()) {
          // 选最垂直：抓取 Z 轴与世界 -Z 的对齐度最高（与 ivg_graspnet 选优同口径）
          const geometry_msgs::msg::Pose * best = nullptr;
          double best_score = -1.0;
          for (const auto & p : latest_poses_->poses) {
            double score = verticality(p);
            if (score > best_score) {
              best_score = score;
              best = &p;
            }
          }
          out = *best;
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  /// 抓取 Z 轴（旋转矩阵第三列 z 分量 R(2,2)=1-2(x²+y²)）与世界 -Z 的对齐度
  static double verticality(const geometry_msgs::msg::Pose & p)
  {
    const auto & q = p.orientation;
    return std::abs(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  }

  void handleExecute(
    const ExecuteGraspPose::Request::SharedPtr req,
    ExecuteGraspPose::Response::SharedPtr resp)
  {
    if (busy_.exchange(true)) {
      resp->success = false;
      resp->message = "上一抓取周期仍在执行";
      return;
    }
    // 异常路径也必须释放 busy_，否则一次异常后单次抓取永久拒绝
    struct BusyGuard
    {
      std::atomic<bool> & flag;
      ~BusyGuard() {flag = false;}
    } guard{busy_};
    runOneCycle(req, resp);
  }

  void loopWorker()
  {
    while (rclcpp::ok()) {
      if (loop_running_) {
        if (!busy_.exchange(true)) {
          struct BusyGuard
          {
            std::atomic<bool> & flag;
            ~BusyGuard() {flag = false;}
          } guard{busy_};
          auto resp = std::make_shared<ExecuteGraspPose::Response>();
          auto req = std::make_shared<ExecuteGraspPose::Request>();
          req->use_visual_estimation = true;
          runOneCycle(req, resp);
          RCLCPP_INFO(get_logger(), "循环抓取周期结果: success=%d (%s)",
            resp->success ? 1 : 0, resp->message.c_str());
        }
        std::this_thread::sleep_for(
          std::chrono::duration<double>(loop_interval_sec_));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
  }

  /// 一周期（保真 aubo_boot ExecuteGraspPoseWorker::runOneCycle）
  void runOneCycle(
    const ExecuteGraspPose::Request::SharedPtr req,
    ExecuteGraspPose::Response::SharedPtr resp)
  {
    if (!robot_->init()) {
      resp->success = false;
      resp->message = "Step 0 failed: MoveGroupInterface 初始化失败";
      return;
    }

    // 步骤 0：回安全位
    if (!robot_->moveToHome(vel_, acc_)) {
      resp->success = false;
      resp->message = "Step 0 failed: 回安全位失败";
      return;
    }

    // 步骤 1/2：构建抓取位姿 + gripper_tip→end_effector Z 偏移
    geometry_msgs::msg::Pose grasp_pose;
    if (req->use_visual_estimation) {
      if (!waitFreshGraspPose(grasp_pose)) {
        resp->success = false;
        resp->message =
          "Step 1 failed: " + grasp_poses_topic_ + " 超时无有效抓取位姿";
        return;
      }
    } else {
      grasp_pose.position.x = preset_x_;
      grasp_pose.position.y = preset_y_;
      grasp_pose.position.z = preset_z_;
      grasp_pose.orientation.w = 1.0;
    }
    const auto pose_ee = applyGraspZOffset(grasp_pose, grasp_z_offset_);
    resp->final_position = pose_ee.position;
    resp->final_orientation = pose_ee.orientation;

    // 步骤 3：开夹爪
    if (!robot_->setGripper(gripper_io_index_, true)) {
      resp->success = false;
      resp->message = "Step 3 failed: 开夹爪 IO 失败";
      return;
    }

    // 步骤 4：抓取接近（当前→X→Y→抬升→旋转→下降，一条笛卡尔轨迹）
    if (!runGraspApproach(pose_ee)) {
      resp->success = false;
      resp->message = "Step 4 failed: 抓取接近失败";
      return;
    }

    // 步骤 5：闭夹爪
    if (!robot_->setGripper(gripper_io_index_, false)) {
      resp->success = false;
      resp->message = "Step 5 failed: 闭夹爪 IO 失败";
      return;
    }

    // 步骤 6：抬起
    if (!robot_->moveCartesianZ(lift_offset_, vel_, acc_)) {
      resp->success = false;
      resp->message = "Step 6 failed: 抬起失败";
      return;
    }

    // 步骤 7：移动到放置位（y/x/z 偏移多段笛卡尔）
    const std::vector<CartesianSegment> place_segments = {
      {'y', place_offset_y_},
      {'x', place_offset_x_},
      {'z', place_offset_z_},
    };
    if (!robot_->moveCartesianPath(place_segments, vel_, acc_)) {
      resp->success = false;
      resp->message = "Step 7 failed: 放置位多段笛卡尔失败";
      return;
    }

    // 步骤 8：开夹爪（放置）
    if (!robot_->setGripper(gripper_io_index_, true)) {
      resp->success = false;
      resp->message = "Step 8 failed: 开夹爪 IO 失败";
      return;
    }

    resp->success = true;
    resp->message = "All steps completed";
  }

  /// 抓取接近（保真 aubo_boot runGraspApproach：单次 computeCartesianPath + execute）
  bool runGraspApproach(const geometry_msgs::msg::Pose & pose_ee)
  {
    constexpr int kMaxRetries = 3;
    constexpr double kRetryDelaySec = 0.5;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
      robot_->moveGroup().setStartStateToCurrentState();
      robot_->setVelocityScaling(vel_);
      robot_->setAccelerationScaling(acc_);
      const auto current = robot_->getCurrentPose();
      auto waypoints = buildApproachWaypoints(current, pose_ee, height_above_, z_min_limit_);
      waypoints.insert(waypoints.begin(), current);

      moveit_msgs::msg::RobotTrajectory traj;
      moveit_msgs::msg::MoveItErrorCodes error_code;
      const double fraction = robot_->moveGroup().computeCartesianPath(
        waypoints, 0.015, traj, true, &error_code);
      const size_t num_points = traj.joint_trajectory.points.size();
      RCLCPP_INFO(get_logger(),
        "抓取接近笛卡尔: fraction=%.2f%%, 点数=%zu (尝试 %d/%d)",
        fraction * 100.0, num_points, attempt, kMaxRetries);
      if (fraction < 1.0 || num_points > static_cast<size_t>(cartesian_max_points_)) {
        if (attempt < kMaxRetries) {
          std::this_thread::sleep_for(std::chrono::duration<double>(kRetryDelaySec));
        }
        continue;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = traj;
      if (robot_->moveGroup().execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        if (attempt < kMaxRetries) {
          std::this_thread::sleep_for(std::chrono::duration<double>(kRetryDelaySec));
        }
        continue;
      }
      return true;
    }
    return false;
  }

  std::unique_ptr<RobotController> robot_;
  double grasp_z_offset_;
  double vel_;
  double acc_;
  int gripper_io_index_;
  double lift_offset_;
  double place_offset_x_;
  double place_offset_y_;
  double place_offset_z_;
  int cartesian_max_points_;
  double height_above_;
  double z_min_limit_;
  double wait_poses_timeout_;
  double loop_interval_sec_;
  std::string grasp_poses_topic_;
  // 预设抓取位姿（use_visual_estimation=false 时使用）
  double preset_x_{0.0};
  double preset_y_{0.0};
  double preset_z_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_sub_;
  rclcpp::CallbackGroup::SharedPtr exec_grp_;
  rclcpp::CallbackGroup::SharedPtr ctrl_grp_;
  rclcpp::Service<ExecuteGraspPose>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr loop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_loop_srv_;

  std::mutex poses_mutex_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_poses_;
  std::atomic<bool> loop_running_{false};
  std::atomic<bool> busy_{false};
  std::thread loop_thread_;
};

}  // namespace ivg_demo_services

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ivg_demo_services::GraspTriggerNode>();
  // 先 initController 再 add_node（MGI 构造对已挂执行器的节点会抛异常）
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
