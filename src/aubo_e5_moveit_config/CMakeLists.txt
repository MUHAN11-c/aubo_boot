cmake_minimum_required(VERSION 3.22)
project(aubo_e5_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
# .setup_assistant 是 MoveItConfigsBuilder 官方元数据：URDF/SRDF 定位与 setup assistant 一致。
install(FILES .setup_assistant LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 文件头 BSD 等项目结束再补
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
