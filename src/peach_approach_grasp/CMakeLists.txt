cmake_minimum_required(VERSION 3.16)
project(peach_approach_grasp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(peach_pose_msgs REQUIRED)
find_package(peach_harvest_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/quality_gate.cpp
  src/view_planner.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

add_executable(approach_grasp_node
  src/approach_grasp_node.cpp
  src/grasp_task.cpp
)
target_include_directories(approach_grasp_node PRIVATE include)
ament_target_dependencies(approach_grasp_node
  aubo_msgs
  behaviortree_cpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  peach_pose_msgs
  peach_harvest_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)
target_link_libraries(approach_grasp_node
  ${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
)

install(TARGETS ${PROJECT_NAME}_core approach_grasp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_quality_gate test/test_quality_gate.cpp)
  target_link_libraries(test_quality_gate ${PROJECT_NAME}_core)
  ament_add_gtest(test_behavior_tree test/test_behavior_tree.cpp)
  ament_target_dependencies(test_behavior_tree behaviortree_cpp)
  target_compile_definitions(test_behavior_tree PRIVATE
    HARVEST_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/config/harvest_tree.xml"
  )
  ament_add_gtest(test_view_planner test/test_view_planner.cpp)
  target_link_libraries(test_view_planner ${PROJECT_NAME}_core)
  ament_add_gtest(test_action_contract test/test_action_contract.cpp)
  target_include_directories(test_action_contract PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
