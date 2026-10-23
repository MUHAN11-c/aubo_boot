cmake_minimum_required(VERSION 3.16)
project(peach_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 系统测驱动脚本（ros2 run peach_system_tests <script>）：对已起栈的
# 人工复核工具——goal 发送 / 相机流探针 / PlanningScene+ACM 巡检。
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)
install(
    DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

ament_package()
