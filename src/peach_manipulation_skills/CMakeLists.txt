cmake_minimum_required(VERSION 3.16)
project(peach_manipulation_skills)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(peach_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Threads REQUIRED)

# 纯逻辑核（零 ROS/MoveIt 依赖）：质量门、视点规划、安全门、目标缓存。
add_library(${PROJECT_NAME}_core
  src/quality_gate.cpp
  src/safety_gate.cpp
  src/target_cache.cpp
  src/view_planner.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Eigen 出现在公共头（target_cache.hpp / view_planner.hpp 等），必须 PUBLIC。
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen Threads::Threads)

# 参数定义单一事实源（声明/类型/默认值/校验）：生成
# peach_manipulation_skills/approach_grasp_node_parameters.hpp（ParamListener/Params）。
generate_parameter_library(approach_grasp_node_parameters
  src/approach_grasp_node_parameters.yaml
)

# 节点按职责拆为多个编译单元，共享私有类声明 approach_grasp_node_impl.hpp。
add_executable(approach_grasp_node
  src/approach_grasp_node.cpp
  src/bt_nodes.cpp
  src/cycle_action.cpp
  src/grasp_task.cpp
  src/main.cpp
  src/motion_interface.cpp
  src/motion_interface_impl.cpp
)
target_include_directories(approach_grasp_node PRIVATE include)
# 命名空间 target / ${pkg_TARGETS}（Jazzy 现代 CMake 惯例，替代
# ament_target_dependencies）；节点非导出目标，一律 PRIVATE。
target_link_libraries(approach_grasp_node PRIVATE
  ${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
  approach_grasp_node_parameters
  behaviortree_cpp::behaviortree_cpp
  moveit_ros_planning_interface::moveit_ros_planning_interface
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${aubo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${peach_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_core approach_grasp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
