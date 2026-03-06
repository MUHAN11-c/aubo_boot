cmake_minimum_required(VERSION 2.8.3)
project(demo_driver)

## C++17 required for std::shared_mutex used by system log4cxx (via ros.h)
add_compile_options(-std=c++17)

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

