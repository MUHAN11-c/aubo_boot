cmake_minimum_required(VERSION 3.22)
project(peach_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BagFitting.msg"
  "msg/BagFittingArray.msg"
  "msg/BagGrasp2D.msg"
  "msg/BagGraspCandidate.msg"
  "msg/BagGraspCandidateArray.msg"
  "msg/CanonicalEvent.msg"
  "msg/DepositResult.msg"
  "msg/GraspDecision.msg"
  "msg/GraspHypothesis.msg"
  "msg/HarvestEvent.msg"
  "msg/HarvestResult.msg"
  "msg/HarvestState.msg"
  "msg/HarvestSummary.msg"
  "msg/JobIntent.msg"
  "msg/MatchStatus.msg"
  "msg/PeachTargetObservation.msg"
  "msg/PeachTargetObservationArray.msg"
  "msg/ReconstructionStatus.msg"
  "msg/SceneSnapshot.msg"
  "msg/ShapeHypothesis.msg"
  "msg/TargetModel.msg"
  "msg/TargetOutcome.msg"
  "msg/TargetQuality.msg"
  "msg/Verification.msg"
  "srv/BeginScene.srv"
  "srv/ControlTask.srv"
  "action/BuildTargetModel.action"
  "action/ExecuteTarget.action"
  "action/RunHarvest.action"
  "action/SurveyScene.action"
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_interface_manifest.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
