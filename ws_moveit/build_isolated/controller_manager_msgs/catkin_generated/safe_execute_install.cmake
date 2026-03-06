execute_process(COMMAND "/home/mu/IVG/ws_moveit/build_isolated/controller_manager_msgs/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mu/IVG/ws_moveit/build_isolated/controller_manager_msgs/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
