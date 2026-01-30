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

# Declare a C++ library
add_library(demo_driver
  src/robot_status_publisher.cpp
  src/robot_io_status_publisher.cpp
  src/move_to_pose_server.cpp
  src/plan_trajectory_server.cpp
  src/execute_trajectory_server.cpp
  src/get_current_state_server.cpp
  src/set_speed_factor_server.cpp
  src/set_robot_enable_server.cpp
  src/set_robot_io_server.cpp
  src/read_robot_io_server.cpp
)

ament_target_dependencies(demo_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  demo_interface
  aubo_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# Declare executables
add_executable(robot_status_publisher_node src/main.cpp)
ament_target_dependencies(robot_status_publisher_node
  rclcpp
)
target_link_libraries(robot_status_publisher_node demo_driver)

add_executable(robot_io_status_publisher_node src/robot_io_status_main.cpp)
ament_target_dependencies(robot_io_status_publisher_node
  rclcpp
)
target_link_libraries(robot_io_status_publisher_node demo_driver)

add_executable(move_to_pose_server_node src/move_to_pose_main.cpp)
ament_target_dependencies(move_to_pose_server_node
  rclcpp
)
target_link_libraries(move_to_pose_server_node demo_driver)

add_executable(plan_trajectory_server_node src/plan_trajectory_main.cpp)
ament_target_dependencies(plan_trajectory_server_node
  rclcpp
)
target_link_libraries(plan_trajectory_server_node demo_driver)

add_executable(execute_trajectory_server_node src/execute_trajectory_main.cpp)
ament_target_dependencies(execute_trajectory_server_node
  rclcpp
)
target_link_libraries(execute_trajectory_server_node demo_driver)

add_executable(get_current_state_server_node src/get_current_state_main.cpp)
ament_target_dependencies(get_current_state_server_node
  rclcpp
)
target_link_libraries(get_current_state_server_node demo_driver)

add_executable(set_speed_factor_server_node src/set_speed_factor_main.cpp)
ament_target_dependencies(set_speed_factor_server_node
  rclcpp
)
target_link_libraries(set_speed_factor_server_node demo_driver)

add_executable(set_robot_enable_server_node src/set_robot_enable_main.cpp)
ament_target_dependencies(set_robot_enable_server_node
  rclcpp
)
target_link_libraries(set_robot_enable_server_node demo_driver)

add_executable(set_robot_io_server_node src/set_robot_io_main.cpp)
ament_target_dependencies(set_robot_io_server_node
  rclcpp
)
target_link_libraries(set_robot_io_server_node demo_driver)

add_executable(read_robot_io_server_node src/read_robot_io_main.cpp)
ament_target_dependencies(read_robot_io_server_node
  rclcpp
)
target_link_libraries(read_robot_io_server_node demo_driver)

# Install targets
install(TARGETS
  demo_driver
  robot_status_publisher_node
  robot_io_status_publisher_node
  move_to_pose_server_node
  plan_trajectory_server_node
  execute_trajectory_server_node
  get_current_state_server_node
  set_speed_factor_server_node
  set_robot_enable_server_node
  set_robot_io_server_node
  read_robot_io_server_node
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
