cmake_minimum_required(VERSION 3.22)
project(aubo_e5_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  trajectory_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  aubo_msgs
  generate_parameter_library
)

generate_parameter_library(
  aubo_passthrough_trajectory_controller_parameters
  src/aubo_passthrough_trajectory_controller_parameters.yaml
)

generate_parameter_library(
  aubo_io_controller_parameters
  src/aubo_io_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/aubo_passthrough_trajectory_controller.cpp
  src/aubo_io_controller.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  aubo_passthrough_trajectory_controller_parameters
  aubo_io_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface aubo_e5_controllers_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES aubo_e5_controllers_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
