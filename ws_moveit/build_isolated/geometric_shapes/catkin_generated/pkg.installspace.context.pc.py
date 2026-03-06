# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include/eigen3;/usr/include".split(';') if "${prefix}/include;/usr/local/include/eigen3;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "eigen_stl_containers;random_numbers;shape_msgs;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgeometric_shapes;/usr/lib/x86_64-linux-gnu/liboctomap.so;/usr/lib/x86_64-linux-gnu/liboctomath.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0".split(';') if "-lgeometric_shapes;/usr/lib/x86_64-linux-gnu/liboctomap.so;/usr/lib/x86_64-linux-gnu/liboctomath.so;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0" != "" else []
PROJECT_NAME = "geometric_shapes"
PROJECT_SPACE_DIR = "/home/mu/IVG/ws_moveit/install_isolated"
PROJECT_VERSION = "0.7.7"
