#!/usr/bin/env bash
# RobotWebTools 官方 npm 包 → web/public/js/vendor（ROS 2 + rosbridge 浏览器栈）
# - roslib@2.x：esbuild 打成 IIFE，暴露 globalThis.ROSLIB（含 ROS2TFClient）
# - ros2d / ros3d / three / easeljs：与 npm 发布构建一致（钉版本，便于复现）
#
# 环境变量可覆盖版本（默认均为当前 npm 上适用于 ROS2 的稳定组合）：
#   ROSVER ROS2D_VER ROS3D_VER THREE_VER EASELJS_VER
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENDOR="$ROOT/web/public/js/vendor"
OUT="$VENDOR/roslib-2.iife.js"

ROSVER="${ROSVER:-2.1.0}"
ROS2D_VER="${ROS2D_VER:-0.10.0}"
ROS3D_VER="${ROS3D_VER:-1.1.0}"
THREE_VER="${THREE_VER:-0.89.0}"
EASELJS_VER="${EASELJS_VER:-1.0.2}"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
cd "$WORKDIR"
npm init -y >/dev/null 2>&1
npm install \
	"roslib@${ROSVER}" \
	esbuild@0.25.12 \
	"ros2d@${ROS2D_VER}" \
	"ros3d@${ROS3D_VER}" \
	"three@${THREE_VER}" \
	"easeljs@${EASELJS_VER}" \
	>/dev/null 2>&1

cat > entry.js <<'EOF'
import * as ROSLIB from "roslib";
globalThis.ROSLIB = ROSLIB;
EOF
npx esbuild entry.js --bundle --format=iife --outfile="$OUT" --platform=browser

cp -f "node_modules/ros2d/build/ros2d.min.js" "$VENDOR/ros2d.min.js"
perl -i -pe 's#nav_msgs/OccupancyGrid#nav_msgs/msg/OccupancyGrid#g' "$VENDOR/ros2d.min.js"
cp -f "node_modules/ros3d/build/ros3d.min.js" "$VENDOR/ros3d.min.js"
# PointCloud2 上游：this.throttle_rate=e.throttle_rate||null 会把合法的 0 变成 null，Humble rosbridge 报 Invalid value: None
perl -i -pe 's/this\.throttle_rate=e\.throttle_rate\|\|null/this.throttle_rate=null==e.throttle_rate?null:e.throttle_rate/g' "$VENDOR/ros3d.min.js"
# ROS2 rosbridge：npm ros3d 仍为 ROS1 短类型名（如 sensor_msgs/PointCloud2），需改为 package/msg/Name（RViz 走 DDS 不受此影响）
perl -i -pe '
  s#visualization_msgs/InteractiveMarkerFeedback#visualization_msgs/msg/InteractiveMarkerFeedback#g;
  s#visualization_msgs/InteractiveMarkerUpdate#visualization_msgs/msg/InteractiveMarkerUpdate#g;
  s#visualization_msgs/MarkerArray#visualization_msgs/msg/MarkerArray#g;
  s#visualization_msgs/Marker#visualization_msgs/msg/Marker#g;
  s#geometry_msgs/PoseWithCovarianceStamped#geometry_msgs/msg/PoseWithCovarianceStamped#g;
  s#geometry_msgs/PolygonStamped#geometry_msgs/msg/PolygonStamped#g;
  s#geometry_msgs/PointStamped#geometry_msgs/msg/PointStamped#g;
  s#geometry_msgs/PoseArray#geometry_msgs/msg/PoseArray#g;
  s#geometry_msgs/PoseStamped#geometry_msgs/msg/PoseStamped#g;
  s#nav_msgs/OccupancyGrid#nav_msgs/msg/OccupancyGrid#g;
  s#nav_msgs/Odometry#nav_msgs/msg/Odometry#g;
  s#nav_msgs/Path#nav_msgs/msg/Path#g;
  s#sensor_msgs/NavSatFix#sensor_msgs/msg/NavSatFix#g;
  s#sensor_msgs/LaserScan#sensor_msgs/msg/LaserScan#g;
  s#sensor_msgs/PointCloud2#sensor_msgs/msg/PointCloud2#g;
' "$VENDOR/ros3d.min.js"
cp -f "node_modules/three/build/three.min.js" "$VENDOR/three.min.js"
cp -f "node_modules/easeljs/lib/easeljs.min.js" "$VENDOR/easeljs.min.js"

echo "Wrote $OUT ($(wc -c < "$OUT") bytes) roslib@${ROSVER}"
echo "Synced ros2d@${ROS2D_VER} ros3d@${ROS3D_VER} three@${THREE_VER} easeljs@${EASELJS_VER} → $VENDOR"
