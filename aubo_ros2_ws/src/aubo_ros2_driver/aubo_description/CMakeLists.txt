cmake_minimum_required(VERSION 3.8)
project(aubo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install meshes and urdf directories（符合 MoveIt2 标准布局：meshes/{collision,visual}/, urdf/）
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

