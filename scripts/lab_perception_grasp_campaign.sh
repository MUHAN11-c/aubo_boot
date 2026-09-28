#!/bin/bash
# 实验室感知+抓取战役助手：自检 / 网格命令 / IMU 核。
# 不发 hardware_mode:=real，不 SetIO。物理臂只用示教器停 global_photo_pose。
# 用法:
#   scripts/lab_perception_grasp_campaign.sh preflight
#   scripts/lab_perception_grasp_campaign.sh imu-tf
#   scripts/lab_perception_grasp_campaign.sh grid-cmd [tool_profile]
#   scripts/lab_perception_grasp_campaign.sh live-cmd [stereo|percipio] [tool_profile]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CMD=${1:-help}

echo "MUST: 物理臂只用示教器停在 global_photo_pose；ROS 不得 hardware_mode:=real。"
echo "测完 pgrep 清残留。bag 预算 100GB；失败短抓 record.level:=all；分析后 purge_analyzed_bags。"

ros_ok() {
  command -v ros2 >/dev/null 2>&1
}

preflight() {
  echo "--- 残留 ---"
  pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe' || true
  echo "--- 开批前自检（栈已起且已 source）---"
  if ! ros_ok; then
    echo "ros2 不在 PATH（先 source /opt/ros/jazzy 与 install/setup.bash）"
    return 0
  fi
  timeout 5 ros2 param get /peach_supervisor skip_reconstruction || true
  timeout 5 ros2 param get /peach_arm quality.allow_unrefined_geometry || true
  timeout 5 ros2 param get /peach_scene_perception_node publish_debug_image || true
  echo "手眼 / IMU TF（5s）："
  timeout 5 ros2 run tf2_ros tf2_echo wrist3_Link camera_link || true
  timeout 5 ros2 run tf2_ros tf2_echo tcp imu_link || true
  echo "深度探针：testing.md 30 帧口径；stereo 先于 Percipio。"
}

imu_tf() {
  echo "核 IMU TF（harvest_system imu_enabled:=true；自适应档案已随栈起 imu_follow）："
  if ros_ok; then
    timeout 5 ros2 run tf2_ros tf2_echo tcp imu_link || true
    timeout 5 ros2 topic echo --once /imu/data || true
  else
    echo "  timeout 5 ros2 run tf2_ros tf2_echo tcp imu_link"
    echo "  timeout 5 ros2 topic echo --once /imu/data"
  fi
  echo "FULL 接触窗由 peach 调 /imu_follow/enable…insert_retract…disable；"
  echo "  非 IMU 末端不起 imu_follow。peach 不自动 motion.enabled。"
  echo "  mock 跟随：ros2 param set /imu_follow motion.enabled true  # 真机须另授权"
}

grid_cmd() {
  local profile=${2:-shear_v1}
  local imu=false
  if [ "${profile}" = "adaptive_shear_v1" ]; then
    imu=true
  fi
  echo "mock 无相机网格 FULL（先基础剪切手再 adaptive；须与 launch tool_profile 一致）："
  cat <<EOF
ros2 launch peach_bringup harvest_system.launch.py \\
  hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true \\
  tool_profile:=${profile} autostart:=false imu_enabled:=${imu}
python3 $ROOT/scripts/sim_field_targets.py --grid --mode full --velocity 1.0 \\
  --tool-profile ${profile}
EOF
  if [ "${imu}" = "true" ]; then
    echo "# adaptive 无 USB 时另发假 IMU，否则 enable 失败："
    echo "# ros2 topic pub -r 20 /imu/data sensor_msgs/msg/Imu '{header: {frame_id: imu_link}, orientation: {w: 1.0}}'"
  fi
}

live_cmd() {
  local frontend=${2:-stereo}
  local profile=${3:-shear_v1}
  local imu=false
  if [ "${profile}" = "adaptive_shear_v1" ]; then
    imu=true
  fi
  echo "真相机 skip-recon（示教器已停拍照位；mock 先 Survey 同位；非 IMU 档不开 IMU）："
  cat <<EOF
ros2 launch peach_bringup harvest_system.launch.py \\
  hardware_mode:=mock camera_enabled:=true camera_frontend:=${frontend} \\
  skip_reconstruction:=true tool_profile:=${profile} \\
  autostart:=false imu_enabled:=${imu}
# 自检 skip_reconstruction / allow_unrefined / debug_image / TF / 深度 FPS≥2
# SURVEY_ONLY → 锁集稳定 → SetEnables(execution+grasp, tool=false)
# PICK_ALL PREGRASP → ACK → 仅 cartesian 放行时 execute_pregrasp_only false 再 FULL
# Percipio 量程 0.4–0.8 m。不放宽 min_views。
# RViz: scripts/record_rviz_harvest.sh <request_id>
EOF
}

case "$CMD" in
  preflight) preflight ;;
  imu-tf) imu_tf ;;
  grid-cmd) grid_cmd "$@" ;;
  live-cmd) live_cmd "$@" ;;
  *)
    echo "commands: preflight | imu-tf | grid-cmd [tool_profile] | live-cmd [stereo|percipio] [tool_profile]"
    preflight
    ;;
esac
