cmake_minimum_required(VERSION 3.10)
project(ivg_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 旁路估姿 IDL：仅 visual_pose_estimation_python 使用。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianPosition.msg"
  "srv/EstimatePose.srv"
  "srv/EstimatePose2D.srv"
  "srv/ListTemplates.srv"
  "srv/StandardizeTemplate.srv"
  "srv/UpdateParams.srv"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
