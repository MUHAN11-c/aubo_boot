cmake_minimum_required(VERSION 3.8)
project(aubo_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(demo_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Service client executables
add_executable(move_to_pose_client src/move_to_pose_client.cpp)
ament_target_dependencies(move_to_pose_client
  rclcpp
  demo_interface
  geometry_msgs
)

add_executable(plan_trajectory_client src/plan_trajectory_client.cpp)
ament_target_dependencies(plan_trajectory_client
  rclcpp
  demo_interface
  geometry_msgs
  trajectory_msgs
)

add_executable(execute_trajectory_client src/execute_trajectory_client.cpp)
ament_target_dependencies(execute_trajectory_client
  rclcpp
  demo_interface
  trajectory_msgs
)

add_executable(get_current_state_client src/get_current_state_client.cpp)
ament_target_dependencies(get_current_state_client
  rclcpp
  demo_interface
)

add_executable(get_current_pose_client src/get_current_pose_client.cpp)
ament_target_dependencies(get_current_pose_client
  rclcpp
  demo_interface
  geometry_msgs
)

add_executable(get_current_pose_moveit2 src/get_current_pose_moveit2.cpp)
ament_target_dependencies(get_current_pose_moveit2
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(set_speed_factor_client src/set_speed_factor_client.cpp)
ament_target_dependencies(set_speed_factor_client
  rclcpp
  demo_interface
)

add_executable(set_robot_enable_client src/set_robot_enable_client.cpp)
ament_target_dependencies(set_robot_enable_client
  rclcpp
  demo_interface
)

add_executable(set_robot_io_client src/set_robot_io_client.cpp)
ament_target_dependencies(set_robot_io_client
  rclcpp
  demo_interface
)

add_executable(read_robot_io_client src/read_robot_io_client.cpp)
ament_target_dependencies(read_robot_io_client
  rclcpp
  demo_interface
)

# Topic subscriber executables
add_executable(robot_status_subscriber src/robot_status_subscriber.cpp)
ament_target_dependencies(robot_status_subscriber
  rclcpp
  demo_interface
)

add_executable(robot_io_status_subscriber src/robot_io_status_subscriber.cpp)
ament_target_dependencies(robot_io_status_subscriber
  rclcpp
  demo_interface
)

# Comprehensive example
add_executable(comprehensive_example src/comprehensive_example.cpp)
ament_target_dependencies(comprehensive_example
  rclcpp
  demo_interface
  geometry_msgs
  trajectory_msgs
)

# Install targets
install(TARGETS
  move_to_pose_client
  plan_trajectory_client
  execute_trajectory_client
  get_current_state_client
  get_current_pose_client
  get_current_pose_moveit2
  set_speed_factor_client
  set_robot_enable_client
  set_robot_io_client
  read_robot_io_client
  robot_status_subscriber
  robot_io_status_subscriber
  comprehensive_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

