cmake_minimum_required(VERSION 3.22)
project(aubo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Analog.msg"
  "msg/Digital.msg"
  "msg/GoalPoint.msg"
  "msg/IOState.msg"
  "msg/JointPos.msg"
  "msg/JointStatus.msg"
  "msg/TraPoint.msg"
  "msg/RobotStatus.msg"
  "srv/GetFK.srv"
  "srv/GetIK.srv"
  "srv/SetIO.srv"
  "srv/SetPayload.srv"
  "action/RunHandEyeCalibration.action"
  "srv/ActivateHandEyeCalibration.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
