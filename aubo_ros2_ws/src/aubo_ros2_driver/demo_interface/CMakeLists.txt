cmake_minimum_required(VERSION 3.8)
project(demo_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/RobotIOStatus.msg"
  "msg/ToolIOStatus.msg"
  "srv/MoveToPose.srv"
  "srv/PlanTrajectory.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/GetCurrentState.srv"
  "srv/SetSpeedFactor.srv"
  "srv/SetRobotEnable.srv"
  "srv/SetRobotIO.srv"
  "srv/ReadRobotIO.srv"
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
