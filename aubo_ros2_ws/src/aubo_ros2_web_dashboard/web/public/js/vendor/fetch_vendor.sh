#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
curl -fsSL "https://raw.githubusercontent.com/EventEmitter2/EventEmitter2/v6.4.9/lib/eventemitter2.js" -o eventemitter2.js
curl -fsSL "https://raw.githubusercontent.com/RobotWebTools/roslibjs/1.4.1/build/roslib.min.js" -o roslib.min.js
curl -fsSL "https://raw.githubusercontent.com/CreateJS/EaselJS/v1.0.2/lib/easeljs.min.js" -o easeljs.min.js
curl -fsSL "https://raw.githubusercontent.com/RobotWebTools/ros2djs/0.10.0/build/ros2d.min.js" -o ros2d.min.js
# ros3djs 与 Three 版本绑定；与官方 examples 一致使用 r89
curl -fsSL "https://raw.githubusercontent.com/mrdoob/three.js/r89/build/three.min.js" -o three.min.js
curl -fsSL "https://raw.githubusercontent.com/RobotWebTools/ros3djs/develop/build/ros3d.min.js" -o ros3d.min.js
echo "ok"
