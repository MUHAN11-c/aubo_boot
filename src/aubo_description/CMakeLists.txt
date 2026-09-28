cmake_minimum_required(VERSION 3.22)
project(aubo_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 文件头 BSD 等项目结束再补（五包同款例外；本机 CMake 3.28 上 _FOUND TRUE
  # 短路已失效，须走官方 AMENT_LINT_AUTO_EXCLUDE 清单）。2026-09-28 scripts/ 起。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
