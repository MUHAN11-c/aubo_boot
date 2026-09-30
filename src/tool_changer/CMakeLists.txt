cmake_minimum_required(VERSION 3.10)
project(tool_changer)

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
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ivg_demo_services REQUIRED)
find_package(ivg_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(gripper_swap_worker_node
  src/gripper_swap_worker.cpp
  src/tool_config.cpp
)
target_include_directories(gripper_swap_worker_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(gripper_swap_worker_node ${YAML_CPP_LIBRARIES})
ament_target_dependencies(gripper_swap_worker_node
  ament_index_cpp geometry_msgs ivg_demo_services ivg_interfaces
  moveit_msgs rclcpp shape_msgs std_msgs)

add_executable(scene_attach_worker_node src/scene_attach_worker.cpp)
target_include_directories(scene_attach_worker_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(scene_attach_worker_node ${YAML_CPP_LIBRARIES})
ament_target_dependencies(scene_attach_worker_node
  ament_index_cpp geometry_msgs ivg_interfaces
  moveit_msgs rclcpp resource_retriever shape_msgs std_msgs)

install(TARGETS gripper_swap_worker_node scene_attach_worker_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/attach_test.py
  scripts/compute_dock_ik.py
  scripts/scene_monitor.py
  scripts/test_tool_change.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf meshes DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头版权项目结束再补（同 peach_arm）
  set(ament_cmake_cpplint_FOUND TRUE)  # 风格门以 uncrustify 为准（testing.md 口径）
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tool_config test/test_tool_config.cpp src/tool_config.cpp)
  target_include_directories(test_tool_config PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_tool_config ${YAML_CPP_LIBRARIES})
  ament_target_dependencies(test_tool_config ament_index_cpp rclcpp std_msgs)
endif()

ament_package()
