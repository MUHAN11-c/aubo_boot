cmake_minimum_required(VERSION 3.22)
project(peach2_task)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bondcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(peach2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(peach2_task_parameters params/peach2_task_parameters.yaml)

# Pure core: zero ROS (selection, batch policy, enables, safety gate, ledger, batch session).
add_library(${PROJECT_NAME}_core STATIC
  src/core/batch_policy.cpp
  src/core/batch_session.cpp
  src/core/enables_policy.cpp
  src/core/ledger.cpp
  src/core/safety_gate.cpp
  src/core/selection.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core PRIVATE nlohmann_json::nlohmann_json)

# BT logic nodes: BehaviorTree.CPP + core, still zero ROS (the tree test runs on these).
add_library(${PROJECT_NAME}_bt STATIC src/bt/logic_nodes.cpp)
set_target_properties(${PROJECT_NAME}_bt PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}_bt PUBLIC
  ${PROJECT_NAME}_core behaviortree_cpp::behaviortree_cpp)

# ROS adapters: message conversions, async leaves, the lifecycle node.
add_library(${PROJECT_NAME}_ros STATIC
  src/ros/conversions.cpp
  src/ros/ros_nodes.cpp
  src/task_node.cpp
)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
  ${PROJECT_NAME}_bt
  peach2_task_parameters
  ament_index_cpp::ament_index_cpp
  ${aubo_msgs_TARGETS}
  bondcpp::bondcpp
  ${builtin_interfaces_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${peach2_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_selection test/test_selection.cpp)
  target_link_libraries(test_selection ${PROJECT_NAME}_core)

  ament_add_gtest(test_batch_policy test/test_batch_policy.cpp)
  target_link_libraries(test_batch_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_enables_policy test/test_enables_policy.cpp)
  target_link_libraries(test_enables_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_ledger test/test_ledger.cpp)
  target_link_libraries(test_ledger ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)

  ament_add_gtest(test_batch_session test/test_batch_session.cpp)
  target_link_libraries(test_batch_session ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)

  ament_add_gtest(test_harvest_batch_tree test/test_harvest_batch_tree.cpp)
  target_link_libraries(test_harvest_batch_tree ${PROJECT_NAME}_bt nlohmann_json::nlohmann_json)
  target_compile_definitions(test_harvest_batch_tree PRIVATE
    PEACH2_TASK_TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/harvest_batch.xml")

  ament_add_gtest(test_msg_mirror test/test_msg_mirror.cpp)
  target_link_libraries(test_msg_mirror ${PROJECT_NAME}_ros)
endif()

ament_package()
