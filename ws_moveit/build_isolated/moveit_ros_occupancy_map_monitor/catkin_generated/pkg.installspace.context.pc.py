# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3;/usr/include".split(';') if "${prefix}/include;/usr/local/include/eigen3;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "moveit_core;moveit_msgs;geometric_shapes;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmoveit_ros_occupancy_map_monitor;/usr/lib/x86_64-linux-gnu/liboctomap.so;/usr/lib/x86_64-linux-gnu/liboctomath.so".split(';') if "-lmoveit_ros_occupancy_map_monitor;/usr/lib/x86_64-linux-gnu/liboctomap.so;/usr/lib/x86_64-linux-gnu/liboctomath.so" != "" else []
PROJECT_NAME = "moveit_ros_occupancy_map_monitor"
PROJECT_SPACE_DIR = "/home/mu/IVG/ws_moveit/install_isolated"
PROJECT_VERSION = "1.1.16"
