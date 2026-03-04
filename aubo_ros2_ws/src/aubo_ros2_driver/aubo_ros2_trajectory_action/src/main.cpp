#include "aubo_ros2_trajectory_action.h"

using namespace aubo_ros2_trajectory_action;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 使用 MoveIt 配置中的控制器名称，这样 MoveIt 的轨迹才能到达这里
  std::string controller_name = "joint_trajectory_controller/follow_joint_trajectory";
  auto action_server = std::make_shared<JointTrajectoryAction>(controller_name);

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
