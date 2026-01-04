cmake_minimum_required(VERSION 3.8)
project(depth_z_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 包含 OpenCV
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(include)

# 可执行文件
add_executable(depth_z_reader_node src/depth_z_reader_node.cpp)
ament_target_dependencies(depth_z_reader_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
)

# 安装
install(TARGETS
  depth_z_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

