cmake_minimum_required(VERSION 3.10)
project(aubo_driver_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 预编译的 libconfig.a 未使用 -fPIC 编译，无法链接到 PIE
# 添加 -no-pie 强制生成非 PIE 可执行文件
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ivg_interfaces REQUIRED)

include_directories(include include/otg ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/log4cplus
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/config
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/aubocontroller
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/protobuf
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
  ivg_interfaces
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
  rclcpp sensor_msgs control_msgs ivg_interfaces std_msgs
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
  rclcpp sensor_msgs std_msgs ivg_interfaces
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
  rclcpp rclcpp_action sensor_msgs control_msgs trajectory_msgs ivg_interfaces
)

install(TARGETS aubo_dashboard_node aubo_state_broadcaster
               aubo_callback_monitor joint_trajectory_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

# Copy config to lib dir for runtime (driver may load config from CWD or relative path)
install(DIRECTORY config/ DESTINATION lib/${PROJECT_NAME}/config)

# 安装 AUBO SDK 预编译共享库 (.so) 到 lib/ (非 lib/${PROJECT_NAME})
# ament_cmake 生成的 setup.bash 仅将 <prefix>/lib 加入 LD_LIBRARY_PATH，
# 不包含 <prefix>/lib/${PROJECT_NAME}，所以 .so 必须放在 lib/ 下喵~
install(DIRECTORY lib/lib64/aubocontroller/ DESTINATION lib
  FILES_MATCHING
  PATTERN "libauborobotcontroller.so*"
  PATTERN "*.bak" EXCLUDE
  PATTERN "*.a" EXCLUDE)
install(DIRECTORY lib/lib64/log4cplus/ DESTINATION lib
  FILES_MATCHING
  PATTERN "*.so*"
  PATTERN "*.bak" EXCLUDE)
install(DIRECTORY lib/lib64/config/ DESTINATION lib
  FILES_MATCHING
  PATTERN "*.so*"
  PATTERN "*.bak" EXCLUDE)
install(DIRECTORY lib/lib64/protobuf/ DESTINATION lib
  FILES_MATCHING
  PATTERN "*.so*"
  PATTERN "*.bak" EXCLUDE)

ament_package()
