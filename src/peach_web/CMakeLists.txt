cmake_minimum_required(VERSION 3.10)
project(peach_web)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    web
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
