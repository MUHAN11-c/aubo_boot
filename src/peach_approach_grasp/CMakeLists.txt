cmake_minimum_required(VERSION 3.16)
project(peach_approach_grasp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(aubo_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(peach_pose_msgs REQUIRED)
find_package(peach_harvest_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Threads REQUIRED)

# 纯逻辑核（零 ROS/MoveIt 依赖）：质量门、视点规划、安全门、目标缓存。
add_library(${PROJECT_NAME}_core
  src/quality_gate.cpp
  src/safety_gate.cpp
  src/target_cache.cpp
  src/view_planner.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Eigen 出现在公共头（target_cache.hpp / view_planner.hpp 等），必须 PUBLIC。
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen Threads::Threads)

# 参数定义单一事实源（声明/类型/默认值/校验）：生成
# peach_approach_grasp/approach_grasp_node_parameters.hpp（ParamListener/Params）。
generate_parameter_library(approach_grasp_node_parameters
  src/approach_grasp_node_parameters.yaml
)

# 节点按职责拆为多个编译单元，共享私有类声明 approach_grasp_node_impl.hpp。
add_executable(approach_grasp_node
  src/approach_grasp_node.cpp
  src/bt_nodes.cpp
  src/cycle_action.cpp
  src/grasp_task.cpp
  src/main.cpp
  src/motion_interface.cpp
  src/motion_interface_impl.cpp
)
target_include_directories(approach_grasp_node PRIVATE include)
# 命名空间 target / ${pkg_TARGETS}（Jazzy 现代 CMake 惯例，替代
# ament_target_dependencies）；节点非导出目标，一律 PRIVATE。
target_link_libraries(approach_grasp_node PRIVATE
  ${PROJECT_NAME}_core
  nlohmann_json::nlohmann_json
  approach_grasp_node_parameters
  behaviortree_cpp::behaviortree_cpp
  moveit_ros_planning_interface::moveit_ros_planning_interface
  moveit_task_constructor_core::moveit_task_constructor_core
  moveit_task_constructor_core::moveit_task_constructor_core_stages
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${aubo_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${peach_harvest_msgs_TARGETS}
  ${peach_pose_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_core approach_grasp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_quality_gate test/test_quality_gate.cpp)
  target_link_libraries(test_quality_gate ${PROJECT_NAME}_core)
  ament_add_gtest(test_safety_gate test/test_safety_gate.cpp)
  target_link_libraries(test_safety_gate ${PROJECT_NAME}_core)
  ament_add_gtest(test_target_cache test/test_target_cache.cpp)
  target_link_libraries(test_target_cache ${PROJECT_NAME}_core)
  # 抓取前再确认纯核策略（2.7-RECONFIRM，header-only，含 Eigen）。
  ament_add_gtest(test_reconfirm_policy test/test_reconfirm_policy.cpp)
  target_link_libraries(test_reconfirm_policy ${PROJECT_NAME}_core)
  # 观察段扫描预算（2.13-E2）与 MTC 预规划槽（2.13-E3）纯核用例
  # （header-only，链接 core 仅为继承 Eigen/include 目录）。
  ament_add_gtest(test_scan_budget test/test_scan_budget.cpp)
  target_link_libraries(test_scan_budget ${PROJECT_NAME}_core)
  ament_add_gtest(test_preplan_slot test/test_preplan_slot.cpp)
  target_link_libraries(test_preplan_slot ${PROJECT_NAME}_core)
  # ament_add_gtest 内部已用 plain 签名 target_link_libraries，本文件所有测试
  # 目标只能沿用 plain 签名（CMake 禁止同一 target 混用 plain/keyword）。
  ament_add_gtest(test_behavior_tree test/test_behavior_tree.cpp)
  target_link_libraries(test_behavior_tree behaviortree_cpp::behaviortree_cpp)
  target_compile_definitions(test_behavior_tree PRIVATE
    HARVEST_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/config/harvest_tree.xml"
  )
  ament_add_gtest(test_view_planner test/test_view_planner.cpp)
  target_link_libraries(test_view_planner ${PROJECT_NAME}_core)
  # 环境几何保护区（阶段 F1）纯核用例：stride-6 解析与点包含判定（header-only，
  # 链接 core 仅为继承 Eigen/include 目录）。
  ament_add_gtest(test_protected_zones test/test_protected_zones.cpp)
  target_link_libraries(test_protected_zones ${PROJECT_NAME}_core)
  ament_add_gtest(test_action_contract test/test_action_contract.cpp)
  target_include_directories(test_action_contract PRIVATE include)
  target_link_libraries(test_action_contract ${peach_harvest_msgs_TARGETS})
  # 阶段耗时埋点用例（重构阶段 C）：纯核 StageTimer/阶段投影 + 回调耗时注册表
  # （scoped_timer.hpp 在 src/，需 rclcpp 日志与 nlohmann_json）。
  ament_add_gtest(test_stage_timing test/test_stage_timing.cpp)
  target_include_directories(test_stage_timing PRIVATE include src)
  target_link_libraries(test_stage_timing
    rclcpp::rclcpp
    nlohmann_json::nlohmann_json)
  # 可替换接口契约测试：含 src/ 私有工厂头；编译运动接口实现体以提供
  # 工厂 inline 实例化所需的 MoveItMotionInterface 符号。
  ament_add_gtest(test_impl_contract
    test/test_impl_contract.cpp
    src/motion_interface_impl.cpp
  )
  target_include_directories(test_impl_contract PRIVATE include src)
  target_link_libraries(test_impl_contract
    ${PROJECT_NAME}_core
    moveit_ros_planning_interface::moveit_ros_planning_interface
    rclcpp::rclcpp
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS})

  # yaml 权威哲学强制：config/approach_grasp.yaml 与生成参数定义默认值逐项对齐。
  find_package(rcl_yaml_param_parser REQUIRED)
  ament_add_gtest(test_yaml_defaults test/test_yaml_defaults.cpp)
  target_link_libraries(test_yaml_defaults
    approach_grasp_node_parameters
    rclcpp::rclcpp
    rcl_yaml_param_parser::rcl_yaml_param_parser)
  target_compile_definitions(test_yaml_defaults PRIVATE
    APPROACH_GRASP_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/approach_grasp.yaml")

  # 参数校验钩子用例（依赖链 + 范围拒绝）：复用节点全部编译单元，进程内
  # 构造真实节点（不初始化 MoveIt）。
  ament_add_gtest(test_parameter_validation
    test/test_parameter_validation.cpp
    src/approach_grasp_node.cpp
    src/bt_nodes.cpp
    src/cycle_action.cpp
    src/grasp_task.cpp
    src/motion_interface.cpp
    src/motion_interface_impl.cpp
  )
  target_include_directories(test_parameter_validation PRIVATE include src)
  target_link_libraries(test_parameter_validation
    ${PROJECT_NAME}_core
    nlohmann_json::nlohmann_json
    approach_grasp_node_parameters
    behaviortree_cpp::behaviortree_cpp
    moveit_ros_planning_interface::moveit_ros_planning_interface
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
    ${aubo_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${peach_harvest_msgs_TARGETS}
    ${peach_pose_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
  )

  # 生命周期迁移（A8）用例：Unconfigured 无运动接口、Inactive 运动入口拒绝、
  # configure 依赖链违例 FAILURE、activate/deactivate 往返。复用节点全部编译
  # 单元，进程内构造真实节点（最小 URDF/SRDF 覆盖，无需硬件/仿真/move_group）。
  ament_add_gtest(test_lifecycle
    test/test_lifecycle.cpp
    src/approach_grasp_node.cpp
    src/bt_nodes.cpp
    src/cycle_action.cpp
    src/grasp_task.cpp
    src/motion_interface.cpp
    src/motion_interface_impl.cpp
  )
  target_include_directories(test_lifecycle PRIVATE include src)
  target_link_libraries(test_lifecycle
    ${PROJECT_NAME}_core
    nlohmann_json::nlohmann_json
    approach_grasp_node_parameters
    behaviortree_cpp::behaviortree_cpp
    moveit_ros_planning_interface::moveit_ros_planning_interface
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
    ${aubo_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${peach_harvest_msgs_TARGETS}
    ${peach_pose_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
  )

  # RunTargetCycle 受理门矩阵（阶段 E 残局抬质量能力端）集成用例：
  # OBSERVE_ONLY 以锁定集锚点缓存为受理依据（selected 空/非空两场景），
  # FULL/PREVIEW 钉死语义回归。编译单元与 harness 模式同 test_lifecycle。
  ament_add_gtest(test_observe_only_gate
    test/test_observe_only_gate.cpp
    src/approach_grasp_node.cpp
    src/bt_nodes.cpp
    src/cycle_action.cpp
    src/grasp_task.cpp
    src/motion_interface.cpp
    src/motion_interface_impl.cpp
  )
  target_include_directories(test_observe_only_gate PRIVATE include src)
  target_link_libraries(test_observe_only_gate
    ${PROJECT_NAME}_core
    nlohmann_json::nlohmann_json
    approach_grasp_node_parameters
    behaviortree_cpp::behaviortree_cpp
    moveit_ros_planning_interface::moveit_ros_planning_interface
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
    ${aubo_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${peach_harvest_msgs_TARGETS}
    ${peach_pose_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
  )

  # 环境几何保护区（阶段 F1）MTC 入口剔除终局路由集成用例：话题数据驱动
  # FULL 周期走到 MTC 入口校验，断言 outcome=SKIPPED_UNREACHABLE 且 reason 含
  # “入口在保护区”。编译单元与 harness 模式同 test_observe_only_gate。
  ament_add_gtest(test_protected_zone_entry
    test/test_protected_zone_entry.cpp
    src/approach_grasp_node.cpp
    src/bt_nodes.cpp
    src/cycle_action.cpp
    src/grasp_task.cpp
    src/motion_interface.cpp
    src/motion_interface_impl.cpp
  )
  target_include_directories(test_protected_zone_entry PRIVATE include src)
  target_compile_definitions(test_protected_zone_entry PRIVATE
    HARVEST_TREE_XML="${CMAKE_CURRENT_SOURCE_DIR}/config/harvest_tree.xml"
  )
  target_link_libraries(test_protected_zone_entry
    ${PROJECT_NAME}_core
    nlohmann_json::nlohmann_json
    approach_grasp_node_parameters
    behaviortree_cpp::behaviortree_cpp
    moveit_ros_planning_interface::moveit_ros_planning_interface
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
    ${aubo_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${peach_harvest_msgs_TARGETS}
    ${peach_pose_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${visualization_msgs_TARGETS}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
