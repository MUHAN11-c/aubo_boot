cmake_minimum_required(VERSION 3.10)
project(peach_web)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_program(NODE_EXECUTABLE NAMES node REQUIRED)
  ament_add_test(test_web_state
    COMMAND "${NODE_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/web_state.test.cjs"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
endif()

install(
  DIRECTORY
    web
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
