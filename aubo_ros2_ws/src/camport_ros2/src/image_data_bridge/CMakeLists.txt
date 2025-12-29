cmake_minimum_required(VERSION 3.8)
project(image_data_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(percipio_camera_interface REQUIRED)
find_package(cv_bridge REQUIRED)

# 包含目录
include_directories(include)

# 图像数据桥接节点
add_executable(image_data_bridge_node src/image_data_bridge_node.cpp)
ament_target_dependencies(image_data_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  percipio_camera_interface
  cv_bridge
)

# 安装
install(TARGETS image_data_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

