#!/usr/bin/env bash
# Open the bagged-peach meshes in Gazebo Harmonic only. Does not start harvest or bringup.
set -euo pipefail
source /opt/ros/jazzy/setup.bash
WORLD="${1:-/home/mu/Desktop/aubo_e5_jazzy_ws/runs/scene_models/peach_dataset_gz/preview.sdf}"
export DISPLAY="${DISPLAY:-:0}"
exec gz sim -v 3 "${WORLD}"
