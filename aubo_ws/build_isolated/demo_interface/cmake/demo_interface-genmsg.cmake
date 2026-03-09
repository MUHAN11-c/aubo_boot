# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "demo_interface: 3 messages, 8 services")

set(MSG_I_FLAGS "-Idemo_interface:/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg;-Istd_msgs:/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg;-Igeometry_msgs:/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg;-Itrajectory_msgs:/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(demo_interface_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" "geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" "std_msgs/Header:demo_interface/ToolIOStatus"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" ""
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header:trajectory_msgs/JointTrajectory:trajectory_msgs/JointTrajectoryPoint:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" "std_msgs/Header:trajectory_msgs/JointTrajectory:trajectory_msgs/JointTrajectoryPoint"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" "geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" ""
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" ""
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" ""
)

get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_custom_target(_demo_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "demo_interface" "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_msg_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_msg_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)

### Generating Services
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)
_generate_srv_cpp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
)

### Generating Module File
_generate_module_cpp(demo_interface
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(demo_interface_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(demo_interface_generate_messages demo_interface_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_cpp _demo_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(demo_interface_gencpp)
add_dependencies(demo_interface_gencpp demo_interface_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS demo_interface_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_msg_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_msg_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)

### Generating Services
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)
_generate_srv_eus(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
)

### Generating Module File
_generate_module_eus(demo_interface
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(demo_interface_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(demo_interface_generate_messages demo_interface_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_eus _demo_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(demo_interface_geneus)
add_dependencies(demo_interface_geneus demo_interface_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS demo_interface_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_msg_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_msg_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)

### Generating Services
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)
_generate_srv_lisp(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
)

### Generating Module File
_generate_module_lisp(demo_interface
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(demo_interface_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(demo_interface_generate_messages demo_interface_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_lisp _demo_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(demo_interface_genlisp)
add_dependencies(demo_interface_genlisp demo_interface_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS demo_interface_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_msg_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_msg_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)

### Generating Services
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)
_generate_srv_nodejs(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
)

### Generating Module File
_generate_module_nodejs(demo_interface
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(demo_interface_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(demo_interface_generate_messages demo_interface_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_nodejs _demo_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(demo_interface_gennodejs)
add_dependencies(demo_interface_gennodejs demo_interface_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS demo_interface_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_msg_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_msg_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)

### Generating Services
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Point.msg;/home/mu/IVG/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)
_generate_srv_py(demo_interface
  "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
)

### Generating Module File
_generate_module_py(demo_interface
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(demo_interface_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(demo_interface_generate_messages demo_interface_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/RobotIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/msg/ToolIOStatus.msg" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/MoveToPose.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ExecuteTrajectory.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/GetCurrentState.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetSpeedFactor.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotEnable.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/SetRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mu/IVG/aubo_ws/src/aubo_robot/aubo_robot/demo_interface/srv/ReadRobotIO.srv" NAME_WE)
add_dependencies(demo_interface_generate_messages_py _demo_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(demo_interface_genpy)
add_dependencies(demo_interface_genpy demo_interface_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS demo_interface_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/demo_interface
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(demo_interface_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(demo_interface_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(demo_interface_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/demo_interface
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(demo_interface_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(demo_interface_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(demo_interface_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/demo_interface
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(demo_interface_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(demo_interface_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(demo_interface_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/demo_interface
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(demo_interface_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(demo_interface_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(demo_interface_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/demo_interface
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(demo_interface_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(demo_interface_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(demo_interface_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
