#include "demo_driver/moveit_gripper_io_base.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace demo_driver
{

namespace mtc = moveit::task_constructor;

static const geometry_msgs::msg::Quaternion kQuatZ180 = []() {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 1.0;
  q.w = 0.0;
  return q;
}();

class PublishGraspsClientWorkerMtcReplacementOnce : public MoveitGripperIoBase
{
public:
  explicit PublishGraspsClientWorkerMtcReplacementOnce(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : MoveitGripperIoBase(options)
  {
    declare_parameter("prefer_vertical", true);
    declare_parameter("grasp_z_offset", 0.15);
    declare_parameter("height_above", 0.1);
    declare_parameter("joint_velocity_scaling", 0.5);
    declare_parameter("joint_acceleration_scaling", 0.25);
    declare_parameter("grasp_poses_topic", std::string("grasp_poses_base"));
    declare_parameter("wait_poses_timeout_sec", 30.0);
    declare_parameter("grasp_window_size", 5);
    declare_parameter("min_groups_before_pick", 3);
    declare_parameter("lift_offset", 0.2);
    declare_parameter("place_offset_y", -0.2);
    declare_parameter("place_offset_z", -0.2);

    prefer_vertical_ = get_parameter("prefer_vertical").as_bool();
    grasp_z_offset_ = get_parameter("grasp_z_offset").as_double();
    height_above_ = get_parameter("height_above").as_double();
    joint_velocity_scaling_ = get_parameter("joint_velocity_scaling").as_double();
    joint_acceleration_scaling_ = get_parameter("joint_acceleration_scaling").as_double();
    grasp_poses_topic_ = get_parameter("grasp_poses_topic").as_string();
    wait_poses_timeout_sec_ = get_parameter("wait_poses_timeout_sec").as_double();
    grasp_window_size_ = static_cast<size_t>(std::max<int64_t>(1, get_parameter("grasp_window_size").as_int()));
    min_groups_before_pick_ = static_cast<size_t>(std::max<int64_t>(1, get_parameter("min_groups_before_pick").as_int()));
    lift_offset_ = get_parameter("lift_offset").as_double();
    place_offset_y_ = get_parameter("place_offset_y").as_double();
    place_offset_z_ = get_parameter("place_offset_z").as_double();

    grasp_poses_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        grasp_poses_topic_, 10,
        std::bind(&PublishGraspsClientWorkerMtcReplacementOnce::graspPosesCallback, this, std::placeholders::_1));
  }

  static std::shared_ptr<PublishGraspsClientWorkerMtcReplacementOnce> create(const rclcpp::NodeOptions& options)
  {
    auto node = std::make_shared<PublishGraspsClientWorkerMtcReplacementOnce>(options);
    node->initMoveGroup();
    return node;
  }

  bool run() override
  {
    clearGraspWindow();
    if (!waitForGraspWindowReady())
    {
      RCLCPP_ERROR(get_logger(), "抓取窗口未就绪，无法执行 MTC 单次任务");
      return false;
    }

    auto selected = selectBestFromWindow();
    if (!selected)
    {
      RCLCPP_ERROR(get_logger(), "抓取窗口无可用目标");
      return false;
    }

    auto [tag, grasp_pose] = *selected;
    RCLCPP_INFO(get_logger(), "MTC 单次任务选中目标: %s", tag.c_str());

    geometry_msgs::msg::Pose pose_ee = applyTransformationToPose(grasp_pose, buildGraspToEndEffectorTransform());
    pose_ee = applyGraspZFlip180(pose_ee);

    if (!planAndExecuteOnce(pose_ee))
      return false;

    RCLCPP_INFO(get_logger(), "MTC 替换版：单次规划并执行完成");
    return true;
  }

private:
  void graspPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty())
      return;
    std::lock_guard<std::mutex> lock(window_mutex_);
    latest_grasp_poses_ = *msg;
    grasp_groups_window_.push_back(*msg);
    while (grasp_groups_window_.size() > grasp_window_size_)
      grasp_groups_window_.pop_front();
  }

  void clearGraspWindow()
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    latest_grasp_poses_.poses.clear();
    grasp_groups_window_.clear();
  }

  bool waitForGraspWindowReady() const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_poses_timeout_sec_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(window_mutex_);
        if (grasp_groups_window_.size() >= min_groups_before_pick_)
          return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> selectBestFromWindow() const
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
          const double s = verticalityScore(p);
          if (s > best_score)
          {
            best_score = s;
            best_pose = p;
            best_tag = "group" + std::to_string(gi) + "_grasp" + std::to_string(pi);
          }
        }
      }
      if (best_score >= 0.0)
        return std::make_pair(best_tag, best_pose);
    }
    else if (!latest_grasp_poses_.poses.empty())
    {
      return std::make_pair(std::string("latest_grasp_0"), latest_grasp_poses_.poses.front());
    }
    return std::nullopt;
  }

  Eigen::Matrix4d buildGraspToEndEffectorTransform() const
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(2, 3) = -grasp_z_offset_;
    return T;
  }

  geometry_msgs::msg::Pose applyTransformationToPose(const geometry_msgs::msg::Pose& pose,
                                                     const Eigen::Matrix4d& transform_local) const
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

  static geometry_msgs::msg::Pose applyGraspZFlip180(const geometry_msgs::msg::Pose& pose)
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

  static double verticalityScore(const geometry_msgs::msg::Pose& pose)
  {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d z_axis = q.toRotationMatrix().col(2);
    return std::abs(z_axis.dot(Eigen::Vector3d(0.0, 0.0, -1.0)));
  }

  bool planAndExecuteOnce(const geometry_msgs::msg::Pose& grasp_pose)
  {
    if (!move_group_)
      return false;

    const std::string planning_group = "manipulator";
    const std::string eef_link = move_group_->getEndEffectorLink();
    if (eef_link.empty())
    {
      RCLCPP_ERROR(get_logger(), "无法获取末端执行器 link");
      return false;
    }

    geometry_msgs::msg::Pose pre_grasp = grasp_pose;
    pre_grasp.position.z += height_above_;
    if (grasp_pose.position.z < kZMinLimit)
    {
      RCLCPP_WARN(get_logger(), "抓取 z=%.3f 低于安全下限 %.2f，按限位执行", grasp_pose.position.z, kZMinLimit);
      pre_grasp.position.z = kZMinLimit + height_above_;
    }

    mtc::Task task;
    task.stages()->setName("publish_grasp_mtc_replacement_once");
    task.loadRobotModel(shared_from_this());

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    sampling_planner->setProperty("max_velocity_scaling_factor", joint_velocity_scaling_);
    sampling_planner->setProperty("max_acceleration_scaling_factor", joint_acceleration_scaling_);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(joint_velocity_scaling_);
    cartesian_planner->setMaxAccelerationScalingFactor(joint_acceleration_scaling_);
    cartesian_planner->setStepSize(0.01);

    {
      auto stage = std::make_unique<mtc::stages::CurrentState>("current_state");
      task.add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("to_home", sampling_planner);
      stage->setGroup(planning_group);
      stage->setGoal("camera_pose");
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "base_link";
      p.pose = pre_grasp;
      auto stage = std::make_unique<mtc::stages::MoveTo>("to_pre_grasp", sampling_planner);
      stage->setGroup(planning_group);
      stage->setGoal(p);
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "base_link";
      p.pose = grasp_pose;
      auto stage = std::make_unique<mtc::stages::MoveTo>("to_grasp", sampling_planner);
      stage->setGroup(planning_group);
      stage->setGoal(p);
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = lift_offset_;
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->setGroup(planning_group);
      stage->setIKFrame(eef_link);
      stage->setDirection(direction);
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.y = place_offset_y_;
      auto stage = std::make_unique<mtc::stages::MoveRelative>("place_y", cartesian_planner);
      stage->setGroup(planning_group);
      stage->setIKFrame(eef_link);
      stage->setDirection(direction);
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.x = -0.2;
      auto stage = std::make_unique<mtc::stages::MoveRelative>("place_x", cartesian_planner);
      stage->setGroup(planning_group);
      stage->setIKFrame(eef_link);
      stage->setDirection(direction);
      task.add(std::move(stage));
    }
    {
      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = place_offset_z_;
      auto stage = std::make_unique<mtc::stages::MoveRelative>("place_z", cartesian_planner);
      stage->setGroup(planning_group);
      stage->setIKFrame(eef_link);
      stage->setDirection(direction);
      task.add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("back_home", sampling_planner);
      stage->setGroup(planning_group);
      stage->setGoal("camera_pose");
      task.add(std::move(stage));
    }

    try
    {
      task.init();
    }
    catch (const mtc::InitStageException& e)
    {
      RCLCPP_ERROR(get_logger(), "MTC task.init() 失败: %s", e.what());
      return false;
    }

    if (!task.plan(1))
    {
      RCLCPP_ERROR(get_logger(), "MTC task.plan() 失败，未生成解");
      return false;
    }

    const auto& solutions = task.solutions();
    if (solutions.empty())
    {
      RCLCPP_ERROR(get_logger(), "MTC 规划无解");
      return false;
    }

    const auto exec_code = task.execute(*solutions.front());
    if (exec_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "MTC task.execute() 失败，错误码=%d", exec_code.val);
      return false;
    }
    return true;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr grasp_poses_sub_;

  mutable std::mutex window_mutex_;
  std::deque<geometry_msgs::msg::PoseArray> grasp_groups_window_;
  geometry_msgs::msg::PoseArray latest_grasp_poses_;

  bool prefer_vertical_{ true };
  double grasp_z_offset_{ 0.15 };
  double height_above_{ 0.1 };
  double joint_velocity_scaling_{ 0.5 };
  double joint_acceleration_scaling_{ 0.25 };
  std::string grasp_poses_topic_{ "grasp_poses_base" };
  double wait_poses_timeout_sec_{ 30.0 };
  size_t grasp_window_size_{ 5 };
  size_t min_groups_before_pick_{ 3 };
  double lift_offset_{ 0.2 };
  double place_offset_y_{ -0.2 };
  double place_offset_z_{ -0.2 };
};

static PublishGraspsClientWorkerMtcReplacementOnce* g_worker_for_signal = nullptr;

static void sigintHandler(int)
{
  (void)g_worker_for_signal;
  rclcpp::shutdown();
}

}  // namespace demo_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = demo_driver::PublishGraspsClientWorkerMtcReplacementOnce::create(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  demo_driver::g_worker_for_signal = node.get();
  std::signal(SIGINT, demo_driver::sigintHandler);
  std::signal(SIGTERM, demo_driver::sigintHandler);

  constexpr int kServiceWaitSec = 10;
  if (!node->waitForServices(std::chrono::seconds(kServiceWaitSec)))
  {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const bool ok = node->run();
  demo_driver::g_worker_for_signal = nullptr;
  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}

