# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3;/usr/include".split(';') if "${prefix}/include;/usr/local/include/eigen3;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "kdl_parser;roscpp;rosconsole;rostime;sensor_msgs;tf2_ros;tf2_kdl".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrobot_state_publisher_solver;-ljoint_state_listener;-lorocos-kdl".split(';') if "-lrobot_state_publisher_solver;-ljoint_state_listener;-lorocos-kdl" != "" else []
PROJECT_NAME = "robot_state_publisher"
PROJECT_SPACE_DIR = "/home/mu/IVG/ros_catkin_ws/install_isolated"
PROJECT_VERSION = "1.15.3"
