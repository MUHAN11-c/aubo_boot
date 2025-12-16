# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;actionlib_msgs;control_msgs;industrial_msgs;industrial_utils;roscpp;sensor_msgs;simple_message;std_msgs;trajectory_msgs;urdf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lindustrial_robot_client_dummy".split(';') if "-lindustrial_robot_client_dummy" != "" else []
PROJECT_NAME = "industrial_robot_client"
PROJECT_SPACE_DIR = "/home/mu/IVG/aubo_ws/install"
PROJECT_VERSION = "0.7.3"
