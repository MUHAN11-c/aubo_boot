cmake_minimum_required(VERSION 3.16)
project(peach_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# clangd 需要编译数据库才能带上 Eigen 的 -isystem /usr/include/eigen3；
# 否则打开公共头时会报 'Eigen/Geometry' file not found。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(peach_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Threads REQUIRED)

# 纯逻辑核（零 ROS/MoveIt 依赖）：质量门、视点规划、安全门、目标缓存。
add_library(${PROJECT_NAME}_core
  src/quality_gate.cpp
  src/safety_gate.cpp
  src/view_planner.cpp
  src/target_cache.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Eigen 出现在公共头（target_cache.hpp / view_planner.hpp 等），必须 PUBLIC。
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen Threads::Threads)

# 参数定义单一事实源（声明/类型/默认值/校验）：生成
# peach_manipulation/manipulation_parameters.hpp（ParamListener/Params）。
generate_parameter_library(manipulation_parameters
  config/manipulation_parameters.yaml
)

# 节点按职责拆为多个编译单元，共享 include/peach_manipulation/ 下的节点头。
add_executable(peach_manipulation_node
  src/manipulation_skills_node.cpp
  src/cycle.cpp
  src/stages.cpp
  src/grasp_task.cpp
  src/main.cpp
  src/motion.cpp
  src/tool_actuator.cpp
)
target_include_directories(peach_manipulation_node PRIVATE include)
# 命名空间 target / ${pkg_TARGETS}（Jazzy 现代 CMake 惯例，替代
# ament_target_dependencies）；节点非导出目标，一律 PRIVATE。
target_link_libraries(peach_manipulation_node PRIVATE
  ${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
  manipulation_parameters
  moveit_ros_planning_interface::moveit_ros_planning_interface
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${aubo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_task_constructor_msgs_TARGETS}
  ${peach_interfaces_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_core peach_manipulation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头 BSD 等项目结束再补
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
# _core 公共头使用 Eigen / nlohmann_json，按 ament 官方惯例导出依赖。
ament_export_dependencies(Eigen3 nlohmann_json)
ament_package()
