// 复刻自 xarm_moveit_demo moveit_beeline_demo，适配 Aubo（规划组 manipulator，与 graspnet_demo.launch.py 配合使用）
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

/*
主要功能：
    1. 将机械臂移动到 home 位置
    2. 移动到三角形起点 (0.4, 0.0, 0.5)
    3. 规划并执行三角形笛卡尔路径：
       - 向下移动 20 cm
       - 向右移动 20 cm
       - 返回起始点
    4. 最后返回 home 位置

运行方式：先启动 graspnet_demo.launch.py，再在另一终端执行
  ros2 run demo_driver moveit_beeline_demo_node
*/

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_beeline_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Aubo 规划组为 manipulator
  moveit::planning_interface::MoveGroupInterface arm(node, "manipulator");

  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.8);


  arm.setStartStateToCurrentState();
  // arm.setPoseTarget(target_pose);
  // arm.setNamedTarget("camera_pose");
  // arm.move();

  // 记录当前位姿作为起点；终点与 test_movel_service.py (24-25) 一致
  geometry_msgs::msg::Pose start_pose = arm.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Start pose: x=%.3f, y=%.3f, z=%.3f", start_pose.position.x, start_pose.position.y, start_pose.position.z);
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = -0.4345;
  end_pose.position.y = 0.0843;
  end_pose.position.z = 0.3398;
  end_pose.orientation.x = 0.7074;
  end_pose.orientation.y = -0.7068;
  end_pose.orientation.z = 0.0005;
  end_pose.orientation.w = 0.0001;
  RCLCPP_INFO(node->get_logger(), "End pose: x=%.3f, y=%.3f, z=%.3f", end_pose.position.x, end_pose.position.y, end_pose.position.z);

  // 分三个轴添加：先 x，再 y，再 z，各一个 waypoint
  const double dx = end_pose.position.x - start_pose.position.x;
  const double dy = end_pose.position.y - start_pose.position.y;
  const double dz = end_pose.position.z - start_pose.position.z;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::msg::Pose wppose = start_pose;
  wppose.position.x += dx;  // x 轴
  waypoints.push_back(wppose);

  wppose.position.y += dy;  // y 轴
  waypoints.push_back(wppose);

  wppose.position.z += dz;  // z 轴
  wppose.orientation = end_pose.orientation;
  waypoints.push_back(wppose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(node->get_logger(),
              "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  if (fraction == 1.0)
  {
    arm.execute(trajectory);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Cartesian path only %.2f%% achievable, skip execution", fraction * 100.0);
  }
  // arm.setNamedTarget("home");
  // arm.move();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
