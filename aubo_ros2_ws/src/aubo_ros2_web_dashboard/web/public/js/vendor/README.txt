Offline copies of RobotWebTools / aligned JS deps for ROS 2 + rosbridge_suite (browser clients).
Do not reimplement standard ROS message handling in app code when roslibjs / ros2djs / ros3djs already provide it; see package README.md「开发约定：官方库优先」.

Single source of truth:

  Run ../../../../scripts/bundle_roslib2_browser.sh

The script installs pinned npm packages, rebuilds roslib@2 as IIFE, copies the browser assets into this directory, and applies the ROS 2 / Humble compatibility patches documented in ../../docs/vendor_audit.md.

Upstream reference URLs (audit only, no longer a second generation path):

  EventEmitter2 v6.4.9  (matches roslibjs dependency ^6.4.0)
    https://raw.githubusercontent.com/EventEmitter2/EventEmitter2/v6.4.9/lib/eventemitter2.js

  roslibjs 2.x npm package
    https://github.com/RobotWebTools/roslibjs

  EaselJS v1.0.2  (matches ros2djs dependency ^1.0.2)
    https://raw.githubusercontent.com/CreateJS/EaselJS/v1.0.2/lib/easeljs.min.js

  ros2djs tag 0.10.0 build/ros2d.min.js
    https://raw.githubusercontent.com/RobotWebTools/ros2djs/0.10.0/build/ros2d.min.js

  three.js r89 + ros3djs develop build（与 ros3djs 官方示例一致；勿与 r128 混用）
    https://raw.githubusercontent.com/mrdoob/three.js/r89/build/three.min.js
    https://raw.githubusercontent.com/RobotWebTools/ros3djs/develop/build/ros3d.min.js

Rebuild vendor: run ../../../../scripts/bundle_roslib2_browser.sh. ROS 2 side stays on system packages (rosbridge_suite, etc.).
