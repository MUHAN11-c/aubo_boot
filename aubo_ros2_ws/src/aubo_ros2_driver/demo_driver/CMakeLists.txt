cmake_minimum_required(VERSION 3.8)
project(demo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# Include directories
include_directories(include)

add_executable(move_to_pose_server_node
  src/moveit_gripper_io_base.cpp
  src/move_to_pose_server.cpp
)
ament_target_dependencies(move_to_pose_server_node
  rclcpp
  geometry_msgs
  ivg_interfaces
  ivg_interfaces
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(plan_trajectory_server_node src/plan_trajectory_server.cpp)
ament_target_dependencies(plan_trajectory_server_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  ivg_interfaces
  ivg_interfaces
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(execute_trajectory_server_node src/execute_trajectory_server.cpp)
ament_target_dependencies(execute_trajectory_server_node
  rclcpp
  trajectory_msgs
  ivg_interfaces
  moveit_core
  moveit_ros_planning_interface
)

add_executable(get_current_state_server_node src/get_current_state_server.cpp)
ament_target_dependencies(get_current_state_server_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ivg_interfaces
  ivg_interfaces
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(set_speed_factor_server_node src/set_speed_factor_server.cpp)
ament_target_dependencies(set_speed_factor_server_node
  rclcpp
  ivg_interfaces
  moveit_core
  moveit_ros_planning_interface
)

add_executable(set_robot_pose_server_node src/set_robot_pose_server.cpp)
ament_target_dependencies(set_robot_pose_server_node
  rclcpp
  geometry_msgs
  ivg_interfaces
  ivg_interfaces
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# GraspNet 抓取放置 worker（订阅 grasp_poses_base，循环抓取→放置→回安全位；运动逻辑自包含）
add_executable(publish_grasps_client_worker_node
  src/publish_grasps_client_worker.cpp
)
ament_target_dependencies(publish_grasps_client_worker_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  ivg_interfaces
  ivg_interfaces
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

# 执行常量抓取位姿 worker（服务驱动模式：支持视觉估计和参数常量两种抓取模式）
add_executable(execute_grasp_pose_worker_node
  src/moveit_gripper_io_base.cpp
  src/execute_grasp_pose_worker.cpp
)
ament_target_dependencies(execute_grasp_pose_worker_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  ivg_interfaces
  ivg_interfaces
  ivg_interfaces
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

# 系统状态监控节点（发布 /system/node_status + /system/log）
add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node rclcpp ivg_interfaces)

# Install targets
install(TARGETS
  move_to_pose_server_node
  plan_trajectory_server_node
  execute_trajectory_server_node
  get_current_state_server_node
  set_speed_factor_server_node
  set_robot_pose_server_node
  publish_grasps_client_worker_node
  execute_grasp_pose_worker_node
  system_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/moveit2_tcp_pose_publisher.py
  scripts/verify_kinematics.py
  scripts/test_ros2_communication.py
  scripts/run_tracetools_analysis.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
