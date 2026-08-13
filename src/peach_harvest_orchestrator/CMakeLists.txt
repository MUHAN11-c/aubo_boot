cmake_minimum_required(VERSION 3.8)
project(peach_harvest_orchestrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(peach_harvest_msgs REQUIRED)
find_package(peach_pose_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(harvest_state_machine src/state_machine.cpp)
target_include_directories(harvest_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS harvest_state_machine EXPORT export_harvest_state_machine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

add_executable(harvest_orchestrator_node src/harvest_orchestrator_node.cpp)
target_include_directories(harvest_orchestrator_node PRIVATE include)
ament_target_dependencies(harvest_orchestrator_node
  aubo_msgs diagnostic_msgs peach_harvest_msgs peach_pose_msgs rclcpp
  rclcpp_action rclcpp_lifecycle std_msgs std_srvs)
target_link_libraries(harvest_orchestrator_node harvest_state_machine)
install(TARGETS harvest_orchestrator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE include)
  target_link_libraries(test_state_machine harvest_state_machine)

  ament_add_gtest(test_policy_service test/test_policy_service.cpp)
  ament_target_dependencies(test_policy_service
    lifecycle_msgs peach_harvest_msgs rclcpp)
  target_compile_definitions(test_policy_service PRIVATE
    ORCHESTRATOR_EXECUTABLE="$<TARGET_FILE:harvest_orchestrator_node>")
  add_dependencies(test_policy_service harvest_orchestrator_node)
endif()

ament_export_targets(export_harvest_state_machine HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
