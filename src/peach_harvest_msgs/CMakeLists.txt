cmake_minimum_required(VERSION 3.8)
project(peach_harvest_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspDecision.msg"
  "msg/HarvestEvent.msg"
  "msg/HarvestState.msg"
  "msg/HarvestSummary.msg"
  "msg/ReconstructionStatus.msg"
  "msg/TargetOutcome.msg"
  "srv/ControlHarvest.srv"
  "srv/SetOperationPolicy.srv"
  "action/RunHarvest.action"
  "action/RunTargetCycle.action"
  DEPENDENCIES action_msgs builtin_interfaces diagnostic_msgs geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
