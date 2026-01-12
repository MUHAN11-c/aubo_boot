cmake_minimum_required(VERSION 3.8)
project(percipio_camera_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraStatus.msg"
  "msg/ImageData.msg"
  "srv/SetCameraParameters.srv"
  "srv/SoftwareTrigger.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# 包含目录
include_directories(include)

# 相机控制节点
add_executable(camera_control_node src/camera_control_node.cpp)
ament_target_dependencies(camera_control_node
  rclcpp
  std_msgs
  sensor_msgs
)
rosidl_target_interfaces(camera_control_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装
install(TARGETS camera_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

