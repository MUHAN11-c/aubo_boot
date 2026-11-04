cmake_minimum_required(VERSION 3.16)
project(ros2_web_link)

find_package(ament_cmake REQUIRED)

# Frontend builds are explicit and reproducible (npm ci && npm run build).
# A source checkout still builds in colcon/industrial_ci without network access;
# the server gives a precise error if the optional web assets were not built.
install(DIRECTORY launch doc config description DESTINATION share/${PROJECT_NAME})
install(DIRECTORY dist/ DESTINATION share/${PROJECT_NAME}/dist OPTIONAL)
install(DIRECTORY sdk media scene audit graph workbench web scripts third_party DESTINATION share/${PROJECT_NAME}/source)
install(FILES README.md package.json package-lock.json tsconfig.json LICENSE
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/serve.py scripts/demo_node.py scripts/bridge_node.py scripts/rosapi_node.py scripts/static_tf_compat.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/service_trace.py scripts/service_wait_trace.py scripts/service_native_trace.py
  scripts/service_late_client.py
  scripts/service_budget.py scripts/service_clients.py scripts/service_client_compat.py
  DESTINATION lib/${PROJECT_NAME})

# Initial dependency installation is explicit (npm ci), never a launch side
# effect. Once present, ordinary colcon build rebuilds the web assets as well.
find_program(NODE_EXECUTABLE node)
if(NODE_EXECUTABLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/node_modules/esbuild")
  add_custom_target(web_assets ALL
    COMMAND ${NODE_EXECUTABLE} scripts/build.mjs
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building local ROS Web SDK and optional observation components"
  )
else()
  message(STATUS "Web source only: run npm ci in ros2_web_link before building deployable web assets")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/.upstream)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
