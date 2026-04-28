cmake_minimum_required(VERSION 3.8)
project(tool_changer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tool_changer_interface REQUIRED)
find_package(demo_interface REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(include)

# 夹爪快换 worker（独立节点，提供 run_gripper_swap / change_tool / get_current_tool 服务）
add_executable(gripper_swap_worker_node
  src/gripper_swap_worker.cpp
)
ament_target_dependencies(gripper_swap_worker_node
  rclcpp
  geometry_msgs
  tool_changer_interface
  demo_interface
  aubo_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

install(TARGETS
  gripper_swap_worker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
