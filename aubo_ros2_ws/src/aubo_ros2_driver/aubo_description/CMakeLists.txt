cmake_minimum_required(VERSION 3.8)
project(aubo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories（仅包含仓库内实际存在的路径；勿写不存在的 config/launch，否则 symlink-install 会失败）
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

