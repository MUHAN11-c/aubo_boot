cmake_minimum_required(VERSION 2.8.3)
project(demo_interface)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs trajectory_msgs)

add_message_files(
   FILES
   RobotStatus.msg
   RobotIOStatus.msg
   ToolIOStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   MoveToPose.srv
   PlanTrajectory.srv
   ExecuteTrajectory.srv
   GetCurrentState.srv
   SetSpeedFactor.srv
   SetRobotEnable.srv
   SetRobotIO.srv
   ReadRobotIO.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

