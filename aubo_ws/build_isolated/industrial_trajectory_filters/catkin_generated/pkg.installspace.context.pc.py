# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3".split(';') if "${prefix}/include;/usr/local/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "class_loader;moveit_core;moveit_ros_planning;pluginlib;trajectory_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lindustrial_trajectory_filters;-lorocos-kdl".split(';') if "-lindustrial_trajectory_filters;-lorocos-kdl" != "" else []
PROJECT_NAME = "industrial_trajectory_filters"
PROJECT_SPACE_DIR = "/home/mu/IVG/aubo_ws/install_isolated"
PROJECT_VERSION = "0.7.3"
