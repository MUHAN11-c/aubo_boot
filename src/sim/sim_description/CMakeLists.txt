cmake_minimum_required(VERSION 3.8)
project(sim_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_mobile test/test_urdf_mobile.py)
endif()

ament_package()
