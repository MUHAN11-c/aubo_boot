cmake_minimum_required(VERSION 3.8)
project(peach_harvest_orchestrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(peach_harvest_msgs REQUIRED)
find_package(peach_pose_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(harvest_state_machine src/state_machine.cpp)
target_include_directories(harvest_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS harvest_state_machine EXPORT export_harvest_state_machine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# 参数定义单一事实源（声明/类型/默认值/校验）：生成
# peach_harvest_orchestrator/harvest_orchestrator_node_parameters.hpp。
generate_parameter_library(harvest_orchestrator_node_parameters
  src/harvest_orchestrator_node_parameters.yaml
)

add_executable(harvest_orchestrator_node src/harvest_orchestrator_node.cpp)
target_include_directories(harvest_orchestrator_node PRIVATE include)
# 命名空间 target / ${pkg_TARGETS}（Jazzy 现代 CMake 惯例，替代
# ament_target_dependencies）；节点非导出目标，一律 PRIVATE。
target_link_libraries(harvest_orchestrator_node PRIVATE
  harvest_state_machine
  harvest_orchestrator_node_parameters
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${aubo_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${peach_harvest_msgs_TARGETS}
  ${peach_pose_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
install(TARGETS harvest_orchestrator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
  target_link_libraries(test_state_machine harvest_state_machine)

  # ament_add_gtest 内部已用 plain 签名 target_link_libraries，测试目标只能沿用
  # plain 签名（CMake 禁止同一 target 混用 plain/keyword）。
  ament_add_gtest(test_policy_service test/test_policy_service.cpp)
  target_link_libraries(test_policy_service
    rclcpp::rclcpp
    ${lifecycle_msgs_TARGETS}
    ${peach_harvest_msgs_TARGETS})
  target_compile_definitions(test_policy_service PRIVATE
    ORCHESTRATOR_EXECUTABLE="$<TARGET_FILE:harvest_orchestrator_node>")
  add_dependencies(test_policy_service harvest_orchestrator_node)

  find_package(rcl_yaml_param_parser REQUIRED)
  ament_add_gtest(test_yaml_defaults test/test_yaml_defaults.cpp)
  target_link_libraries(test_yaml_defaults
    rclcpp::rclcpp
    rcl_yaml_param_parser::rcl_yaml_param_parser)
  target_compile_definitions(test_yaml_defaults PRIVATE
    ORCHESTRATOR_EXECUTABLE="$<TARGET_FILE:harvest_orchestrator_node>"
    ORCHESTRATOR_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/orchestrator.yaml")
  add_dependencies(test_yaml_defaults harvest_orchestrator_node)

  # 参数定义校验：启动期非法覆盖抛异常、运行期非法 set 拒绝。
  ament_add_gtest(test_parameter_validation test/test_parameter_validation.cpp)
  target_link_libraries(test_parameter_validation
    rclcpp::rclcpp
    harvest_orchestrator_node_parameters)
endif()

ament_export_targets(export_harvest_state_machine HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
