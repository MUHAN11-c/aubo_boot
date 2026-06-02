cmake_minimum_required(VERSION 3.8)
project(latte_backend)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(demo_driver REQUIRED)

# ── latte_workflow_node: 拉花工作流编排 + 心形轨迹生成 ──
add_executable(latte_workflow_node
  src/latte_workflow_node.cpp
  src/latte_trajectory.cpp
)

target_include_directories(latte_workflow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(latte_workflow_node
  demo_driver::robot_controller
)
ament_target_dependencies(latte_workflow_node
  rclcpp
  geometry_msgs
  std_srvs
  ivg_interfaces
  moveit_core
  moveit_ros_planning_interface
  demo_driver
)

install(TARGETS latte_workflow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS scripts/spout_calibrator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
