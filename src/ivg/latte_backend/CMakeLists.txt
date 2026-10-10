cmake_minimum_required(VERSION 3.10)
project(latte_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ivg_demo_services REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# ── 心形轨迹纯核（零 ROS 图依赖，gtest 直测）──
add_library(latte_trajectory STATIC
  src/latte_trajectory.cpp
)
target_include_directories(latte_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(latte_trajectory geometry_msgs)

add_executable(latte_workflow_node src/latte_workflow_node.cpp)
target_include_directories(latte_workflow_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(latte_workflow_node latte_trajectory)
ament_target_dependencies(latte_workflow_node
  aubo_msgs geometry_msgs ivg_demo_services ivg_interfaces rclcpp std_srvs)

add_executable(latte_io_node src/latte_io_node.cpp)
target_include_directories(latte_io_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(latte_io_node
  aubo_msgs rclcpp std_srvs)

install(TARGETS latte_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS latte_workflow_node latte_io_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/spout_calibrator.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头版权项目结束再补（同 peach_arm）
  set(ament_cmake_cpplint_FOUND TRUE)  # 风格门以 uncrustify 为准（testing.md 口径）
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_latte_trajectory test/test_latte_trajectory.cpp)
  target_include_directories(test_latte_trajectory PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_latte_trajectory latte_trajectory)
  ament_target_dependencies(test_latte_trajectory geometry_msgs)
endif()

ament_package()
