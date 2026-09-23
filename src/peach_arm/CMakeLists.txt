cmake_minimum_required(VERSION 3.16)
project(peach_arm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# clangd 需要编译数据库才能带上 Eigen 的 -isystem /usr/include/eigen3；
# 否则打开公共头时会报 'Eigen/Geometry' file not found。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(peach_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Threads REQUIRED)

find_package(generate_parameter_library REQUIRED)
# 全量参数单一事实源（清洁重写轮 2c：手写 params.hpp 逐条转写后删除；
# 含原 execution_contract 两键，键名/默认值/校验冻结）。
generate_parameter_library(arm_parameters
  src/arm_parameters.yaml)

# 纯逻辑核（零 ROS/MoveIt 依赖）：质量门、视点规划、安全门、目标缓存。
add_library(${PROJECT_NAME}_core
  src/quality_gate.cpp
  src/safety_gate.cpp
  src/view_planner.cpp
  src/target_cache.cpp
  src/tool_actuator.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Eigen 出现在公共头（target_cache.hpp / view_planner.hpp 等），必须 PUBLIC。
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen Threads::Threads)

# 参数声明/兜底默认/校验单一事实源：include/peach_arm/params.hpp
# （手写模块，决策 0017）；部署值事实源为 config/peach_arm.yaml。

# 节点按职责拆为多个编译单元，共享 include/peach_arm/ 下的节点头。
add_executable(peach_arm
  src/manipulation_skills_node.cpp
  src/cycle.cpp
  src/move_to.cpp
  src/stages.cpp
  src/grasp_task.cpp
  src/main.cpp
  src/motion.cpp
)
target_include_directories(peach_arm PRIVATE include)
# 命名空间 target / ${pkg_TARGETS}（Jazzy 现代 CMake 惯例，替代
# ament_target_dependencies）；节点非导出目标，一律 PRIVATE。
target_link_libraries(peach_arm PRIVATE
  ${PROJECT_NAME}_core
  ${bondcpp_TARGETS}
  ${diagnostic_updater_TARGETS}
  arm_parameters
  Threads::Threads
  nlohmann_json::nlohmann_json
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${aubo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_task_constructor_msgs_TARGETS}
  ${peach_interfaces_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_core peach_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# 测试夹具（现场案册）：回放塔与脚本按源码树相对定位读取；install 份供
# 安装环境（share/peach_arm/test/fixtures）下的系统测消费。
install(DIRECTORY test/fixtures DESTINATION share/${PROJECT_NAME}/test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头 BSD 等项目结束再补
  # cpplint 整测项跳过（testing.md 口径）：其 legal/copyright 与 Google
  # include 顺序检查与本项目「own-first、文件头版权项目结束再补」约定冲突；
  # C++ 风格门以 uncrustify 为准，静态分析走 cppcheck。
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_pregrasp_level test/test_pregrasp_level.cpp)
  if(TARGET test_pregrasp_level)
    target_include_directories(test_pregrasp_level PRIVATE include)
  endif()
  # W0 安全网：纯核 gtest（重构方案 2026-09-20）。统一链 ${PROJECT_NAME}_core
  # （Eigen/Threads/公共头经 PUBLIC 传递）；trajectory_guard 公共头带
  # trajectory_msgs 依赖，另链消息目标。
  ament_add_gtest(test_grasp_geometry test/test_grasp_geometry.cpp)
  if(TARGET test_grasp_geometry)
    target_link_libraries(test_grasp_geometry ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_trajectory_guard test/test_trajectory_guard.cpp)
  if(TARGET test_trajectory_guard)
    target_link_libraries(
      test_trajectory_guard ${PROJECT_NAME}_core ${trajectory_msgs_TARGETS})
  endif()
  ament_add_gtest(test_view_planner test/test_view_planner.cpp)
  if(TARGET test_view_planner)
    target_link_libraries(test_view_planner ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_gates test/test_gates.cpp)
  if(TARGET test_gates)
    target_link_libraries(test_gates ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_reconfirm_policy test/test_reconfirm_policy.cpp)
  if(TARGET test_reconfirm_policy)
    target_link_libraries(test_reconfirm_policy ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_target_cache test/test_target_cache.cpp)
  if(TARGET test_target_cache)
    target_link_libraries(test_target_cache ${PROJECT_NAME}_core)
  endif()
  # W5 纯核安全网：帧率自适应超时族 / 预抓取残差。
  ament_add_gtest(test_frame_timeouts test/test_frame_timeouts.cpp)
  if(TARGET test_frame_timeouts)
    target_link_libraries(test_frame_timeouts ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_staging_waypoints test/test_staging_waypoints.cpp)
  if(TARGET test_staging_waypoints)
    target_link_libraries(test_staging_waypoints ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_pregrasp_residual test/test_pregrasp_residual.cpp)
  if(TARGET test_pregrasp_residual)
    target_link_libraries(test_pregrasp_residual ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_tool_actuator test/test_tool_actuator.cpp)
  if(TARGET test_tool_actuator)
    target_link_libraries(test_tool_actuator ${PROJECT_NAME}_core)
  endif()
  if(TARGET test_pregrasp_residual)
    target_link_libraries(test_pregrasp_residual ${PROJECT_NAME}_core)
  endif()
  ament_add_pytest_test(test_contact_monitor test/test_contact_monitor.py)
  ament_add_pytest_test(test_constraint_grid test/test_constraint_grid.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
# _core 公共头使用 Eigen / nlohmann_json，按 ament 官方惯例导出依赖。
ament_export_dependencies(Eigen3 nlohmann_json)
ament_package()
