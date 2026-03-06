# Install script for directory: /home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs

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
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/_setup_util.py")
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
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/env.sh")
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
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/setup.bash"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/local_setup.bash"
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
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/setup.sh"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/local_setup.sh"
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
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/setup.zsh"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/local_setup.zsh"
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
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/setup.fish"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/local_setup.fish"
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
  file(INSTALL DESTINATION "/home/mu/IVG/ros_catkin_ws/install_isolated" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs/msg" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/BatteryState.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/CameraInfo.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/ChannelFloat32.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/CompressedImage.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/FluidPressure.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Illuminance.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Image.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Imu.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/JointState.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Joy.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/JoyFeedback.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/JoyFeedbackArray.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/LaserEcho.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/LaserScan.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/MagneticField.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/MultiDOFJointState.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/MultiEchoLaserScan.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/NavSatFix.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/NavSatStatus.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/PointCloud.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/PointCloud2.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/PointField.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Range.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/RegionOfInterest.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/RelativeHumidity.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/Temperature.msg"
    "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/msg/TimeReference.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs/srv" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/srv/SetCameraInfo.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/sensor_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/include/sensor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/share/roseus/ros/sensor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/share/common-lisp/ros/sensor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/share/gennodejs/ros/sensor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/lib/python3/dist-packages/sensor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/lib/python3/dist-packages/sensor_msgs" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/lib/python3/dist-packages/sensor_msgs" FILES_MATCHING REGEX "/home/mu/IVG/ros_catkin_ws/devel_isolated/sensor_msgs/lib/python3/dist-packages/sensor_msgs/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/sensor_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs/cmake" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/sensor_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs/cmake" TYPE FILE FILES
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/sensor_msgsConfig.cmake"
    "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/installspace/sensor_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs" TYPE FILE FILES "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sensor_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/include/sensor_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_msgs" TYPE DIRECTORY FILES "/home/mu/IVG/ros_catkin_ws/src/common_msgs/sensor_msgs/migration_rules")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/gtest/cmake_install.cmake")
  include("/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/test/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
