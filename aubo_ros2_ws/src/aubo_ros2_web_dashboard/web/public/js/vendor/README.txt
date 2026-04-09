Offline copies of RobotWebTools / aligned JS deps for ROS 2 + rosbridge_suite (browser clients).
Do not reimplement standard ROS message handling in app code when roslibjs / ros2djs / ros3djs already provide it; see package README.md「开发约定：官方库优先」.

Pinned GitHub sources (curl-friendly; re-run fetch_vendor.sh after changing URLs):

  EventEmitter2 v6.4.9  (matches roslibjs dependency ^6.4.0)
    https://raw.githubusercontent.com/EventEmitter2/EventEmitter2/v6.4.9/lib/eventemitter2.js

  roslibjs 1.4.1 build/roslib.min.js
    https://raw.githubusercontent.com/RobotWebTools/roslibjs/1.4.1/build/roslib.min.js

  EaselJS v1.0.2  (matches ros2djs dependency ^1.0.2)
    https://raw.githubusercontent.com/CreateJS/EaselJS/v1.0.2/lib/easeljs.min.js

  ros2djs tag 0.10.0 build/ros2d.min.js
    https://raw.githubusercontent.com/RobotWebTools/ros2djs/0.10.0/build/ros2d.min.js

  three.js r89 + ros3djs develop build（与 ros3djs 官方示例一致；勿与 r128 混用）
    https://raw.githubusercontent.com/mrdoob/three.js/r89/build/three.min.js
    https://raw.githubusercontent.com/RobotWebTools/ros3djs/develop/build/ros3d.min.js

Re-fetch: run fetch_vendor.sh in this directory. ROS 2 side stays on system packages (rosbridge_suite, etc.).
