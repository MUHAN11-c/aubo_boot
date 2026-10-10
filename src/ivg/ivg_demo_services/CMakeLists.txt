cmake_minimum_required(VERSION 3.10)
project(ivg_demo_services)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# ── 运动薄层库（latte_backend / grasp_trigger / move_service 共用）──
add_library(demo_motion SHARED
  src/motion_utils.cpp
  src/robot_controller.cpp
)
target_include_directories(demo_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(demo_motion
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
)
ament_target_dependencies(demo_motion
  aubo_msgs
  geometry_msgs
  moveit_msgs
  rclcpp
)
target_compile_definitions(demo_motion PRIVATE ${EIGEN3_DEFINITIONS})

# ── 节点 ──
add_executable(system_monitor_node src/system_monitor_node.cpp)
target_include_directories(system_monitor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(system_monitor_node
  aubo_msgs ivg_interfaces rclcpp)

add_executable(move_service_node src/move_service_node.cpp)
target_include_directories(move_service_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(move_service_node demo_motion)
ament_target_dependencies(move_service_node
  aubo_msgs geometry_msgs ivg_interfaces rclcpp sensor_msgs std_srvs)

add_executable(grasp_trigger_node src/grasp_trigger_node.cpp)
target_include_directories(grasp_trigger_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(grasp_trigger_node demo_motion)
ament_target_dependencies(grasp_trigger_node
  geometry_msgs ivg_interfaces rclcpp sensor_msgs std_srvs)

install(TARGETS demo_motion
  EXPORT export_ivg_demo_services
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

install(TARGETS system_monitor_node move_service_node grasp_trigger_node
  DESTINATION lib/${PROJECT_NAME})

# IVG 演示栈辅助脚本（2026-09-30 自 aubo_boot 移植；按 IVG/peach 隔离裁定
# 从 aubo_e5_moveit_config 迁入：aubo_mode / limit_workspace /
# publish_gripper_joint_states / publish_obstacle + workspace_limits.yaml）
install(PROGRAMS
  scripts/aubo_mode.py
  scripts/limit_workspace.py
  scripts/publish_gripper_joint_states.py
  scripts/publish_obstacle.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/workspace_limits.yaml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头版权项目结束再补（同 peach_arm）
  # cpplint 整测项跳过（testing.md 口径）：其 legal/copyright 与 Google
  # 头要求与本仓现有包冲突，C++ 风格门以 uncrustify 为准。
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_utils test/test_motion_utils.cpp)
  target_include_directories(test_motion_utils PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_motion_utils demo_motion)
  ament_target_dependencies(test_motion_utils geometry_msgs rclcpp)
endif()

ament_export_targets(export_ivg_demo_services HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp aubo_msgs geometry_msgs ivg_interfaces moveit_msgs
  moveit_ros_planning_interface)
ament_package()
