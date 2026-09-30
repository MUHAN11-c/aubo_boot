cmake_minimum_required(VERSION 3.22)
project(peach2_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(peach2_end_effector REQUIRED)
find_package(peach2_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)

generate_parameter_library(peach2_manipulation_parameters
  params/peach2_manipulation_parameters.yaml)

# Pure core: zero ROS (command gate, reverse trajectories, staging geometry, harvest cycle,
# MoveTo, neighbour-bag collision objects). Unit-tested against fakes.
add_library(${PROJECT_NAME}_core SHARED
  src/bag_obstacles.cpp
  src/command_gate.cpp
  src/harvest_cycle.cpp
  src/motion_backend.cpp
  src/move_to.cpp
  src/staging_geometry.cpp
  src/trajectory_reverse.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  peach2_end_effector::peach2_end_effector_core)

# IDL <-> core conversions (message types only, no executor).
add_library(${PROJECT_NAME}_conversions SHARED src/conversions.cpp)
target_link_libraries(${PROJECT_NAME}_conversions PUBLIC
  ${PROJECT_NAME}_core
  ${aubo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${peach2_interfaces_TARGETS})

# Lifecycle node + MoveIt backend (private headers under src/).
add_executable(peach2_manipulation_node
  src/main.cpp
  src/manipulation_node.cpp
  src/moveit_motion_backend.cpp
)
target_link_libraries(peach2_manipulation_node
  ${PROJECT_NAME}_conversions
  peach2_manipulation_parameters
  ament_index_cpp::ament_index_cpp
  bondcpp::bondcpp
  ${diagnostic_updater_TARGETS}
  moveit_core::moveit_robot_state
  moveit_ros_planning_interface::moveit_move_group_interface
  ${moveit_msgs_TARGETS}
  peach2_end_effector::peach2_end_effector_aubo_io
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_eigen::tf2_eigen
  ${trajectory_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS peach2_manipulation_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(aubo_description REQUIRED)
  set(PEACH2_TOOL_CONFIG_DIR "${aubo_description_DIR}/../config")

  foreach(test_name
      test_bag_obstacles
      test_command_gate
      test_harvest_cycle
      test_motion_backend
      test_move_to
      test_staging_geometry
      test_trajectory_reverse)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    target_compile_definitions(${test_name} PRIVATE
      PEACH2_TOOL_CONFIG_DIR="${PEACH2_TOOL_CONFIG_DIR}")
  endforeach()

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME}_conversions)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 aubo_msgs eigen3_cmake_module geometry_msgs peach2_end_effector peach2_interfaces)
ament_package()
