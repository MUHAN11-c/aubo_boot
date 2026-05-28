cmake_minimum_required(VERSION 3.10)
project(latte_cartesian_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

include_directories(include)

add_executable(cartesian_planner_node
  src/cartesian_planner_node.cpp
  src/cartesian_planner_main.cpp
)
ament_target_dependencies(cartesian_planner_node
  rclcpp
  geometry_msgs
  trajectory_msgs
  ivg_interfaces
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  moveit_visual_tools
)

install(TARGETS cartesian_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
