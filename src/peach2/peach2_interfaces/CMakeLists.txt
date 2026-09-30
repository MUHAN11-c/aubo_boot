cmake_minimum_required(VERSION 3.22)
project(peach2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BagLandmark.msg"
  "msg/BatchState.msg"
  "msg/Enables.msg"
  "msg/FailureCode.msg"
  "msg/GraspDecision.msg"
  "msg/HarvestResult.msg"
  "msg/TargetModel.msg"
  "msg/TargetModelArray.msg"
  "msg/TargetObservation.msg"
  "msg/TargetObservationArray.msg"
  "msg/ToolState.msg"
  "srv/BeginScene.srv"
  "srv/BuildSceneSnapshot.srv"
  "srv/CheckReachability.srv"
  "srv/GetDecision.srv"
  "srv/SetEnables.srv"
  "action/HarvestTarget.action"
  "action/MoveTo.action"
  "action/ObserveTarget.action"
  "action/RunBatch.action"
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
