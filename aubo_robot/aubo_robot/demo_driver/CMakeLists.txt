cmake_minimum_required(VERSION 2.8.3)
project(demo_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  industrial_msgs
  demo_interface
  aubo_msgs
  message_generation
  moveit_core
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES demo_driver
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs trajectory_msgs industrial_msgs demo_interface aubo_msgs moveit_core moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(demo_driver
  src/robot_status_publisher.cpp
  src/robot_io_status_publisher.cpp
  src/move_to_pose_server.cpp
  src/plan_trajectory_server.cpp
  src/execute_trajectory_server.cpp
  src/get_current_state_server.cpp
  src/set_speed_factor_server.cpp
  src/set_robot_enable_server.cpp
  src/set_robot_io_server.cpp
  src/read_robot_io_server.cpp
)

## Add cmake target dependencies of the library
add_dependencies(demo_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(robot_status_publisher_node src/main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(robot_status_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_status_publisher_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for robot IO status publisher
add_executable(robot_io_status_publisher_node src/robot_io_status_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(robot_io_status_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_io_status_publisher_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for move_to_pose service
add_executable(move_to_pose_server_node src/move_to_pose_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(move_to_pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(move_to_pose_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for plan_trajectory service
add_executable(plan_trajectory_server_node src/plan_trajectory_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(plan_trajectory_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(plan_trajectory_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for execute_trajectory service
add_executable(execute_trajectory_server_node src/execute_trajectory_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(execute_trajectory_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(execute_trajectory_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for get_current_state service
add_executable(get_current_state_server_node src/get_current_state_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(get_current_state_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(get_current_state_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for set_speed_factor service
add_executable(set_speed_factor_server_node src/set_speed_factor_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(set_speed_factor_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(set_speed_factor_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for set_robot_enable service
add_executable(set_robot_enable_server_node src/set_robot_enable_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(set_robot_enable_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(set_robot_enable_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for set_robot_io service
add_executable(set_robot_io_server_node src/set_robot_io_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(set_robot_io_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(set_robot_io_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for read_robot_io service
add_executable(read_robot_io_server_node src/read_robot_io_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(read_robot_io_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(read_robot_io_server_node
  demo_driver
  ${catkin_LIBRARIES}
)

