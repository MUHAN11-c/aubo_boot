cmake_minimum_required(VERSION 3.10)
project(peach_web)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_program(NODE_EXECUTABLE NAMES node REQUIRED)
  file(GLOB PEACH_WEB_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.test.cjs")
  foreach(test_file IN LISTS PEACH_WEB_TESTS)
    get_filename_component(test_name "${test_file}" NAME_WE)
    ament_add_test("test_${test_name}"
      COMMAND "${NODE_EXECUTABLE}" "${test_file}"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 30)
  endforeach()
endif()

install(
  DIRECTORY
    web
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
