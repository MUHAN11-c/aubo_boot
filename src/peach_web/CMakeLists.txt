cmake_minimum_required(VERSION 3.10)
project(peach_web)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    web
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
