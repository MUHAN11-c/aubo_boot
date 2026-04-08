Offline copies of the same libraries as upstream ros2-web-bridge demo (static.robotwebtools.org/.../current/...).
Pinned GitHub sources (curl-friendly; align with upstream demo + package.json):

  EventEmitter2 v6.4.9  (matches roslibjs dependency ^6.4.0)
    https://raw.githubusercontent.com/EventEmitter2/EventEmitter2/v6.4.9/lib/eventemitter2.js

  roslibjs 1.4.1 build/roslib.min.js
    https://raw.githubusercontent.com/RobotWebTools/roslibjs/1.4.1/build/roslib.min.js

  EaselJS v1.0.2  (matches ros2djs dependency ^1.0.2)
    https://raw.githubusercontent.com/CreateJS/EaselJS/v1.0.2/lib/easeljs.min.js

  ros2djs tag 0.10.0 build/ros2d.min.js
    https://raw.githubusercontent.com/RobotWebTools/ros2djs/0.10.0/build/ros2d.min.js

  three.js r128 (3D 视图)
    https://raw.githubusercontent.com/mrdoob/three.js/r128/build/three.min.js
    https://raw.githubusercontent.com/mrdoob/three.js/r128/examples/js/controls/OrbitControls.js

Re-fetch: run fetch_vendor.sh in this directory. ROS 2 side stays on system packages (rosbridge_suite, etc.).
