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
find_package(geometric_shapes REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# 夹爪快换 worker（物理运动 + IO 控制）
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

# 场景附着 worker（订阅 /tool_changer_status，更新 PlanningScene 显示）
add_executable(scene_attach_worker_node
  src/scene_attach_worker.cpp
)
ament_target_dependencies(scene_attach_worker_node
  rclcpp
  tool_changer_interface
  moveit_msgs
  geometric_shapes
  yaml_cpp_vendor
  ament_index_cpp
)

install(TARGETS
  gripper_swap_worker_node
  scene_attach_worker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
