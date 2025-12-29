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
find_package(demo_interface REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)

# Include directories
include_directories(include)

# Declare executables (server files now contain main functions)
add_executable(robot_status_publisher_node src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  demo_interface
  aubo_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# add_executable(robot_io_status_publisher_node src/robot_io_status_publisher.cpp)
# ament_target_dependencies(robot_io_status_publisher_node
#   rclcpp
#   demo_interface
#   aubo_msgs
# )

add_executable(move_to_pose_server_node src/move_to_pose_server.cpp)
ament_target_dependencies(move_to_pose_server_node
  rclcpp
  geometry_msgs
  demo_interface
  aubo_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(plan_trajectory_server_node src/plan_trajectory_server.cpp)
ament_target_dependencies(plan_trajectory_server_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  demo_interface
  aubo_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(execute_trajectory_server_node src/execute_trajectory_server.cpp)
ament_target_dependencies(execute_trajectory_server_node
  rclcpp
  trajectory_msgs
  demo_interface
  moveit_core
  moveit_ros_planning_interface
)

add_executable(get_current_state_server_node src/get_current_state_server.cpp)
ament_target_dependencies(get_current_state_server_node
  rclcpp
  sensor_msgs
  geometry_msgs
  demo_interface
  aubo_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(set_speed_factor_server_node src/set_speed_factor_server.cpp)
ament_target_dependencies(set_speed_factor_server_node
  rclcpp
  demo_interface
  moveit_core
  moveit_ros_planning_interface
)

# Note: The following executables are commented out because source files don't exist yet
# add_executable(set_robot_enable_server_node src/set_robot_enable_server.cpp)
# ament_target_dependencies(set_robot_enable_server_node
#   rclcpp
#   std_msgs
#   demo_interface
# )

# add_executable(set_robot_io_server_node src/set_robot_io_server.cpp)
# ament_target_dependencies(set_robot_io_server_node
#   rclcpp
#   demo_interface
#   aubo_msgs
# )

# add_executable(read_robot_io_server_node src/read_robot_io_server.cpp)
# ament_target_dependencies(read_robot_io_server_node
#   rclcpp
#   demo_interface
#   aubo_msgs
# )

# Install targets
install(TARGETS
  robot_status_publisher_node
  # robot_io_status_publisher_node
  move_to_pose_server_node
  plan_trajectory_server_node
  execute_trajectory_server_node
  get_current_state_server_node
  set_speed_factor_server_node
  # set_robot_enable_server_node
  # set_robot_io_server_node
  # read_robot_io_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
