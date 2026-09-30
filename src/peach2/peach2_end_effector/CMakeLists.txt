cmake_minimum_required(VERSION 3.22)
project(peach2_end_effector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pure core: no ROS. Unit-tested directly; the plugin classes live here.
add_library(${PROJECT_NAME}_core SHARED
  src/cutter_end_effector.cpp
  src/mock_io_backend.cpp
  src/plugins.cpp
  src/tool_profile.cpp
  src/tool_state_machine.cpp
  src/types.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)

# pluginlib registration only; plugins.xml points at this library.
add_library(${PROJECT_NAME}_plugins SHARED src/plugin_registration.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PUBLIC ${PROJECT_NAME}_core pluginlib::pluginlib)

# ROS IO backend (aubo_io_controller SetIO + IOState).
add_library(${PROJECT_NAME}_aubo_io SHARED src/aubo_io_backend.cpp)
target_link_libraries(${PROJECT_NAME}_aubo_io PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  ${aubo_msgs_TARGETS})

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_plugins ${PROJECT_NAME}_aubo_io
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(aubo_description REQUIRED)
  set(PEACH2_TOOL_CONFIG_DIR "${aubo_description_DIR}/../config")

  ament_add_gtest(test_tool_state_machine test/test_tool_state_machine.cpp)
  target_link_libraries(test_tool_state_machine ${PROJECT_NAME}_core)

  ament_add_gtest(test_tool_profile test/test_tool_profile.cpp)
  target_link_libraries(test_tool_profile ${PROJECT_NAME}_core)
  target_compile_definitions(test_tool_profile PRIVATE
    PEACH2_TOOL_CONFIG_DIR="${PEACH2_TOOL_CONFIG_DIR}")

  ament_add_gtest(test_plugins test/test_plugins.cpp)
  target_link_libraries(test_plugins ${PROJECT_NAME}_core)
  target_compile_definitions(test_plugins PRIVATE
    PEACH2_TOOL_CONFIG_DIR="${PEACH2_TOOL_CONFIG_DIR}")

  ament_add_gtest(test_mock_flow test/test_mock_flow.cpp)
  target_link_libraries(test_mock_flow ${PROJECT_NAME}_core)
  target_compile_definitions(test_mock_flow PRIVATE
    PEACH2_TOOL_CONFIG_DIR="${PEACH2_TOOL_CONFIG_DIR}")

  ament_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  target_link_libraries(test_pluginlib_load ${PROJECT_NAME}_core pluginlib::pluginlib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 aubo_msgs eigen3_cmake_module pluginlib rclcpp yaml-cpp)
ament_package()
