cmake_minimum_required(VERSION 3.5)
project(aubo_driver_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(demo_interface REQUIRED)

include_directories(include include/otg ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/log4cplus
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/config
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/aubocontroller
  )
else()
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib32)
endif()

add_executable(aubo_driver_ros2
  src/aubo_driver_ros2.cpp
  src/driver_node_ros2.cpp
)
target_link_libraries(aubo_driver_ros2
  pthread
  auborobotcontroller
  otgLib
  log4cplus
  config
)
ament_target_dependencies(aubo_driver_ros2
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  aubo_msgs
  demo_interface
)

install(TARGETS aubo_driver_ros2 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

# Copy config to lib dir for runtime (driver may load config from CWD or relative path)
install(DIRECTORY config/ DESTINATION lib/${PROJECT_NAME}/config)

ament_package()
