cmake_minimum_required(VERSION 3.10)
project(ivg_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 旁路估姿 IDL（EstimatePose 族）+ IVG 演示栈 IDL（拉花/快换/抓取/监控，源自 aubo_boot 移植）。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianPosition.msg"
  "msg/NodeStatus.msg"
  "msg/SystemLog.msg"
  "msg/ToolChangerStatus.msg"
  "srv/EstimatePose.srv"
  "srv/EstimatePose2D.srv"
  "srv/ListTemplates.srv"
  "srv/StandardizeTemplate.srv"
  "srv/UpdateParams.srv"
  "srv/RunLatteWorkflow.srv"
  "srv/RunGripperSwap.srv"
  "srv/ChangeTool.srv"
  "srv/GetCurrentTool.srv"
  "srv/ExecuteGraspPose.srv"
  "srv/MoveToPose.srv"
  "srv/GetCurrentState.srv"
  "srv/ReadRobotIO.srv"
  "srv/SetRobotEnable.srv"
  "srv/SetSpeedFactor.srv"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
