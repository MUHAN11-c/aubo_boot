cmake_minimum_required(VERSION 3.8)
project(coordinate_transforms)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core: no ROS, no OpenCV
find_package(ament_cmake REQUIRED)

# Optional Eigen for core math (fallback to manual 3x3/4x4 if not found)
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
  set(COORD_TF_USE_EIGEN 1)
endif()

# Include
include_directories(include)

# 旋转转换：有 Eigen 时用 Eigen 实现，否则用手写实现
set(ROTATION_CONVERSION_SRC src/core/rotation_conversion.cpp)
if(Eigen3_FOUND)
  set(ROTATION_CONVERSION_SRC src/core/rotation_conversion_eigen.cpp)
endif()

# --- Core library (non-ROS) ---
add_library(coordinate_transforms_core STATIC
  src/core/coordinate_systems.cpp
  ${ROTATION_CONVERSION_SRC}
  src/core/point_transform.cpp
)
target_include_directories(coordinate_transforms_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(Eigen3_FOUND)
  target_include_directories(coordinate_transforms_core PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_compile_definitions(coordinate_transforms_core PUBLIC COORD_TF_USE_EIGEN=1)
endif()

# --- ROS2 library ---
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(coordinate_transforms_ros2 STATIC
  src/ros2/coordinate_systems_ros2.cpp
  src/ros2/rotation_conversion_ros2.cpp
  src/ros2/point_transform_ros2.cpp
)
target_include_directories(coordinate_transforms_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(coordinate_transforms_ros2 coordinate_transforms_core)
ament_target_dependencies(coordinate_transforms_ros2
  rclcpp geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs std_msgs
)

# --- Demo node ---
add_executable(coord_tf_demo_node src/ros2/coord_tf_demo_node.cpp)
target_link_libraries(coord_tf_demo_node coordinate_transforms_ros2 coordinate_transforms_core)
ament_target_dependencies(coord_tf_demo_node
  rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs visualization_msgs std_msgs
)

# --- Optional tools (export visualization: PCD/PLY/PNG) ---
# Minimal export without OpenCV: we can write PCD as text and PNG via stb_image_write
# For now we add a placeholder that compiles; full implementation can use OpenCV if found
option(COORD_TF_BUILD_TOOLS "Build optional tools (export_visualization)" ON)
if(COORD_TF_BUILD_TOOLS)
  add_executable(export_visualization src/tools/export_visualization.cpp)
  target_link_libraries(export_visualization PRIVATE coordinate_transforms_core)
  target_include_directories(export_visualization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

# Optional OpenCV branch
find_package(OpenCV QUIET)
if(OpenCV_FOUND AND COORD_TF_BUILD_TOOLS)
  add_executable(opencv_process_data src/tools/opencv_process_data.cpp)
  target_link_libraries(opencv_process_data PRIVATE coordinate_transforms_core ${OpenCV_LIBS})
  target_include_directories(opencv_process_data PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
endif()

# Install
install(TARGETS coordinate_transforms_core coordinate_transforms_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS coord_tf_demo_node
  DESTINATION lib/${PROJECT_NAME})
if(COORD_TF_BUILD_TOOLS)
  install(TARGETS export_visualization DESTINATION lib/${PROJECT_NAME})
endif()
if(OpenCV_FOUND AND COORD_TF_BUILD_TOOLS)
  install(TARGETS opencv_process_data DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY doc/
  DESTINATION share/${PROJECT_NAME}/doc
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
