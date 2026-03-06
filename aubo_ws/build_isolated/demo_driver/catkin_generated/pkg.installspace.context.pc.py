# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;sensor_msgs;geometry_msgs;trajectory_msgs;industrial_msgs;demo_interface;aubo_msgs;moveit_core;moveit_ros_planning_interface".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldemo_driver".split(';') if "-ldemo_driver" != "" else []
PROJECT_NAME = "demo_driver"
PROJECT_SPACE_DIR = "/home/mu/IVG/aubo_ws/install_isolated"
PROJECT_VERSION = "1.0.0"
