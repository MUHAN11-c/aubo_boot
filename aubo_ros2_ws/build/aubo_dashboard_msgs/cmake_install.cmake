# Install script for directory: /home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mu/IVG/aubo_ros2_ws/install/aubo_dashboard_msgs")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/aubo_dashboard_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_c/aubo_dashboard_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/lib/python3.8/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_generator_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_typesupport_fastrtps_c/aubo_dashboard_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_typesupport_fastrtps_cpp/aubo_dashboard_msgs/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_typesupport_introspection_c/aubo_dashboard_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_cpp/aubo_dashboard_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aubo_dashboard_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_typesupport_introspection_cpp/aubo_dashboard_msgs/" REGEX "/[^/]*\\.hpp$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/mu/IVG/aubo_ros2_ws/install/aubo_dashboard_msgs/lib/python3.8/site-packages/aubo_dashboard_msgs/__init__.py"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/msg" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/msg/" REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/srv" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/srv/" REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/action" TYPE DIRECTORY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/action/" REGEX "/[^/]*\\.py$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs:/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/action_msgs/cmake/../../../lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:/opt/ros/foxy/share/unique_identifier_msgs/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_fastrtps_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/aubo_dashboard_msgs__rosidl_typesupport_fastrtps_c__pyext.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs:/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/action_msgs/cmake/../../../lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:/opt/ros/foxy/share/unique_identifier_msgs/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_introspection_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/aubo_dashboard_msgs__rosidl_typesupport_introspection_c__pyext.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs:/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:/opt/ros/foxy/lib:/opt/ros/foxy/share/action_msgs/cmake/../../../lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:/opt/ros/foxy/share/unique_identifier_msgs/cmake/../../../lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/python3.8/site-packages/aubo_dashboard_msgs/aubo_dashboard_msgs_s__rosidl_typesupport_c.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/aubo_dashboard_msgs__rosidl_typesupport_c__pyext.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_generator_py/aubo_dashboard_msgs/libaubo_dashboard_msgs__python.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so"
         OLD_RPATH "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs:/opt/ros/foxy/share/action_msgs/cmake/../../../lib:/opt/ros/foxy/share/builtin_interfaces/cmake/../../../lib:/opt/ros/foxy/share/unique_identifier_msgs/cmake/../../../lib:/opt/ros/foxy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaubo_dashboard_msgs__python.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/msg/ProgramState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/msg/RobotMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/msg/SafetyMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/AddToLog.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/GetLoadedProgram.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/GetProgramState.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/GetRobotMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/GetSafetyMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/IsProgramRunning.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/IsInRemoteControl.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/IsProgramSaved.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/Load.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/Popup.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/srv/RawRequest.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/action" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_adapter/aubo_dashboard_msgs/action/SetMode.idl")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/msg/ProgramState.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/msg/RobotMode.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/msg" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/msg/SafetyMode.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/AddToLog.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/AddToLog_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/AddToLog_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/GetLoadedProgram.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetLoadedProgram_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetLoadedProgram_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/GetProgramState.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetProgramState_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetProgramState_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/GetRobotMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetRobotMode_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetRobotMode_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/GetSafetyMode.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetSafetyMode_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/GetSafetyMode_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/IsProgramRunning.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsProgramRunning_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsProgramRunning_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/IsInRemoteControl.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsInRemoteControl_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsInRemoteControl_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/IsProgramSaved.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsProgramSaved_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/IsProgramSaved_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/Load.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/Load_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/Load_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/Popup.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/Popup_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/Popup_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/srv/RawRequest.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/RawRequest_Request.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/srv" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/srv/RawRequest_Response.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/action" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/action/SetMode.action")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/aubo_dashboard_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/aubo_dashboard_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/environment" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_index/share/ament_index/resource_index/packages/aubo_dashboard_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_generator_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_generator_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_generator_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cppExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_generator_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_generator_cppExport.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_introspection_cppExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cppExport.cmake"
         "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cppExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cppExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake/aubo_dashboard_msgs__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cppExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/CMakeFiles/Export/f5fce6fc2f3ec012cdaa13ab82f36124/aubo_dashboard_msgs__rosidl_typesupport_cppExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs/cmake" TYPE FILE FILES
    "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_core/aubo_dashboard_msgsConfig.cmake"
    "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/ament_cmake_core/aubo_dashboard_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_dashboard_msgs" TYPE FILE FILES "/home/mu/IVG/aubo_ros2_ws/src/aubo_ros2_driver/aubo_dashboard_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/aubo_dashboard_msgs__py/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/mu/IVG/aubo_ros2_ws/build/aubo_dashboard_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
