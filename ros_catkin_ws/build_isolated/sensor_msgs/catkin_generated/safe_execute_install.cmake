execute_process(COMMAND "/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mu/IVG/ros_catkin_ws/build_isolated/sensor_msgs/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
