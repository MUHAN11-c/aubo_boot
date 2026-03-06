#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mu/IVG/ros_catkin_ws/src/executive_smach/smach_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mu/IVG/ros_catkin_ws/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mu/IVG/ros_catkin_ws/install_isolated/lib/python3/dist-packages:/home/mu/IVG/ros_catkin_ws/build_isolated/smach_ros/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mu/IVG/ros_catkin_ws/build_isolated/smach_ros" \
    "/usr/bin/python3" \
    "/home/mu/IVG/ros_catkin_ws/src/executive_smach/smach_ros/setup.py" \
    egg_info --egg-base /home/mu/IVG/ros_catkin_ws/build_isolated/smach_ros \
    build --build-base "/home/mu/IVG/ros_catkin_ws/build_isolated/smach_ros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mu/IVG/ros_catkin_ws/install_isolated" --install-scripts="/home/mu/IVG/ros_catkin_ws/install_isolated/bin"
