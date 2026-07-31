cmake_minimum_required(VERSION 3.22)
project(peach_pose_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BagGraspCandidate.msg"
  "msg/BagGraspCandidateArray.msg"
  "msg/BagGrasp2D.msg"
  "msg/BagGrasp2DArray.msg"
  "msg/BagFitting.msg"
  "msg/BagFittingArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
