cmake_minimum_required(VERSION 3.10)
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

# ═══════════════════════════════════════════════════════════════════════════
# robot_controller 共享库 — MoveIt 运动 + IO 控制基础设施
# ═══════════════════════════════════════════════════════════════════════════

add_library(robot_controller SHARED
  src/robot_controller.cpp
)
target_include_directories(robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_controller
  rclcpp
  geometry_msgs
  ivg_interfaces
  tf2
  moveit_core
  moveit_ros_planning_interface
)

# ═══════════════════════════════════════════════════════════════════════════
# 可执行节点
# ═══════════════════════════════════════════════════════════════════════════

add_executable(move_to_pose_server_node src/move_to_pose_server.cpp)
target_link_libraries(move_to_pose_server_node robot_controller)
ament_target_dependencies(move_to_pose_server_node
  rclcpp geometry_msgs ivg_interfaces
  moveit_core moveit_ros_planning_interface
)

add_executable(plan_trajectory_server_node src/plan_trajectory_server.cpp)
ament_target_dependencies(plan_trajectory_server_node
  rclcpp geometry_msgs trajectory_msgs
  ivg_interfaces tf2_geometry_msgs
  moveit_core moveit_ros_planning_interface
)

add_executable(execute_trajectory_server_node src/execute_trajectory_server.cpp)
ament_target_dependencies(execute_trajectory_server_node
  rclcpp trajectory_msgs ivg_interfaces
  moveit_core moveit_ros_planning_interface
)

add_executable(get_current_state_server_node src/get_current_state_server.cpp)
ament_target_dependencies(get_current_state_server_node
  rclcpp sensor_msgs geometry_msgs
  ivg_interfaces tf2_geometry_msgs
  moveit_core moveit_ros_planning_interface
)

add_executable(set_speed_factor_server_node src/set_speed_factor_server.cpp)
ament_target_dependencies(set_speed_factor_server_node
  rclcpp ivg_interfaces
  moveit_core moveit_ros_planning_interface
)

add_executable(set_robot_pose_server_node src/set_robot_pose_server.cpp)
ament_target_dependencies(set_robot_pose_server_node
  rclcpp geometry_msgs ivg_interfaces
  tf2 tf2_geometry_msgs
  moveit_core moveit_ros_planning_interface
)

# GraspNet 抓取放置 worker
add_executable(publish_grasps_client_worker_node src/publish_grasps_client_worker.cpp)
target_link_libraries(publish_grasps_client_worker_node robot_controller)
ament_target_dependencies(publish_grasps_client_worker_node
  rclcpp geometry_msgs std_msgs std_srvs
  ivg_interfaces moveit_msgs
  moveit_core moveit_ros_planning_interface
)

# 执行常量抓取位姿 worker
add_executable(execute_grasp_pose_worker_node src/execute_grasp_pose_worker.cpp)
target_link_libraries(execute_grasp_pose_worker_node robot_controller)
ament_target_dependencies(execute_grasp_pose_worker_node
  rclcpp geometry_msgs sensor_msgs std_srvs
  ivg_interfaces moveit_msgs
  moveit_core moveit_ros_planning_interface
)

# 系统状态监控节点
add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node rclcpp ivg_interfaces)

# 机器人使能服务
add_executable(set_robot_enable_server_node src/set_robot_enable_server.cpp)
ament_target_dependencies(set_robot_enable_server_node rclcpp std_srvs ivg_interfaces)

# IO 状态读取服务
add_executable(read_robot_io_server_node src/read_robot_io_server.cpp)
ament_target_dependencies(read_robot_io_server_node rclcpp ivg_interfaces)

# ═══════════════════════════════════════════════════════════════════════════
# 安装
# ═══════════════════════════════════════════════════════════════════════════

install(TARGETS robot_controller
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
  set_robot_enable_server_node
  read_robot_io_server_node
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

# ═══════════════════════════════════════════════════════════════════════════
# 导出 — 供下游包 (latte_backend) 使用
# ═══════════════════════════════════════════════════════════════════════════

ament_export_include_directories(include)
ament_export_libraries(robot_controller)
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  ivg_interfaces
  tf2
  moveit_core
  moveit_ros_planning_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
