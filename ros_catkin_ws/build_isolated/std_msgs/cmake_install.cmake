# Install script for directory: /home/mu/IVG/ros_catkin_ws/src/std_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mu/IVG/ros_catkin_ws/install_isolated")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash;/home/mu/IVG/ros_catkin_ws/install_isolated/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/setup.bash"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/setup.sh;/home/mu/IVG/ros_catkin_ws/install_isolated/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/setup.sh"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/setup.zsh;/home/mu/IVG/ros_catkin_ws/install_isolated/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/setup.zsh"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/setup.fish;/home/mu/IVG/ros_catkin_ws/install_isolated/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/setup.fish"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mu/IVG/ros_catkin_ws/install_isolated/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/std_msgs/msg" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Bool.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Byte.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/ByteMultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Char.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/ColorRGBA.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Duration.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Empty.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Float32.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Float32MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Float64.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Float64MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Header.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int16.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int16MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int32.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int32MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int64.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int64MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int8.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Int8MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/MultiArrayDimension.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/MultiArrayLayout.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/String.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/Time.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt16.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt16MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt32.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt32MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt64.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt64MultiArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt8.msg"
    "/home/mu/IVG/ros_catkin_ws/src/std_msgs/msg/UInt8MultiArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/std_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/std_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/include/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/share/roseus/ros/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/share/common-lisp/ros/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/share/gennodejs/ros/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/lib/python3/dist-packages/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/std_msgs/lib/python3/dist-packages/std_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/std_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/std_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/std_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/std_msgs/cmake" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/std_msgsConfig.cmake"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/catkin_generated/installspace/std_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/std_msgs" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/src/std_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/std_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/src/std_msgs/include/std_msgs/" REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/mu/IVG/ros_catkin_ws/build_isolated/std_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
