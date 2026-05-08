cmake_minimum_required(VERSION 3.5)
project(aubo_driver_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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

# Dashboard 独立可执行文件 (LifecycleNode)
add_executable(aubo_dashboard_node
  src/aubo_dashboard_node.cpp
  src/dashboard_node_main.cpp
  src/aubo_hardware_interface.cpp
)
target_link_libraries(aubo_dashboard_node
  pthread
  auborobotcontroller
  otgLib
  log4cplus
  config
)
ament_target_dependencies(aubo_dashboard_node
  rclcpp
  rclcpp_lifecycle
  std_srvs
  sensor_msgs
  demo_interface
  aubo_msgs
)

# State Broadcaster 独立可执行文件
add_executable(aubo_state_broadcaster
  src/aubo_state_broadcaster.cpp
  src/aubo_hardware_interface.cpp
)
target_link_libraries(aubo_state_broadcaster
  pthread auborobotcontroller otgLib log4cplus config
)
ament_target_dependencies(aubo_state_broadcaster
  rclcpp sensor_msgs control_msgs demo_interface std_msgs
)

# 回调监控节点 (独立可执行文件)
add_executable(aubo_callback_monitor
  src/aubo_callback_monitor.cpp
  src/aubo_hardware_interface.cpp
)
target_link_libraries(aubo_callback_monitor
  pthread auborobotcontroller otgLib log4cplus config
)
ament_target_dependencies(aubo_callback_monitor
  rclcpp sensor_msgs std_msgs demo_interface
)

# JointTrajectoryController 独立可执行文件 (新框架驱动核心)
add_executable(joint_trajectory_controller
  src/joint_trajectory_controller.cpp
  src/joint_trajectory_controller_main.cpp
  src/aubo_hardware_interface.cpp
)
target_link_libraries(joint_trajectory_controller
  pthread auborobotcontroller otgLib log4cplus config
)
ament_target_dependencies(joint_trajectory_controller
  rclcpp rclcpp_action sensor_msgs control_msgs trajectory_msgs demo_interface
)

install(TARGETS aubo_dashboard_node aubo_state_broadcaster
               aubo_callback_monitor joint_trajectory_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

# Copy config to lib dir for runtime (driver may load config from CWD or relative path)
install(DIRECTORY config/ DESTINATION lib/${PROJECT_NAME}/config)

ament_package()
