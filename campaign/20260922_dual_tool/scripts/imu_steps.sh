#!/usr/bin/env bash
# S7 P2-IMU adaptive 专项 I1–I7 runbook（automatable 步自动核，人工步给判据）。
# 用法: imu_steps.sh [i1|i2|i3|i4|i5|i6|i7|all]
# 栈前提: tool_profile:=adaptive_cylinder_v1（imu_follow 随栈起）。
# 红线: motion.enabled 仅 I4/I6 且无 MTC 执行时经 imu_follow 自己的门开；
#       tool.enabled 恒 false；不 SetIO。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
OUT="campaign/20260922_dual_tool/analysis"
mkdir -p "$OUT"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
STEP="${1:-all}"
STAMP="$(date +%Y%m%d_%H%M%S)"

i1() {  # TF 核对: tcp→imu_link 存在且与档案 align 一致
  echo "== I1 TF 核对 =="
  timeout 15 ros2 run tf2_ros tf2_echo tcp imu_link --timeout 8.0 2>&1 | tee "$OUT/i1_tf_${STAMP}.log"
}
i2() {  # 静置零漂: status 流 60s 内 quaternion 增量阈值判（抽 5 快照人工判读）
  echo "== I2 静置零漂（抽 5 快照@10s 间隔，人工判读漂移）=="
  for k in 1 2 3 4 5; do
    timeout 6 ros2 topic echo /imu/data --once 2>/dev/null \
      | grep -A4 orientation | head -6 | tee -a "$OUT/i2_drift_${STAMP}.log"
    sleep 10
  done
}
i3() { echo "== I3 手转跟随（人工）：手转工具 ~30°，观察 /imu_follow/status 跟随方向与量程 =="
  timeout 20 ros2 topic echo /imu_follow/status 2>/dev/null | head -20 || echo '（status 话题不可达：查 imu_follow 是否随栈起）'; }
i4() { echo "== I4 开 motion 大幅转动（人工+锥钳）：imu_follow enable 后大幅转动，验证锥钳限幅 =="
  echo '  运行期开 motion（仅此步，无 MTC 执行）：ros2 param set /imu_follow motion.enabled true'
  echo '  判据：servo status=0、转角被锥钳限幅、结束必须 disable'; }
i5() {  # 拔串口自动 disable
  echo "== I5 拔串口自动 disable（人工拔线后观察 0.5s）=="
  timeout 15 ros2 topic echo /imu_follow/status 2>/dev/null | head -10 || echo '拔线后应见 disable/超时态'
  echo '  判据：串口断 → imu_follow 自动 disable，不残留使能'; }
i6() { echo "== I6 insert 推进 0.01m/s（人工）：插入模式推进可停 =="
  echo '  判据：0.01 m/s 匀速、随时可停、不越 l_insert'; }
i7() {  # 重对齐方向
  echo "== I7 重对齐方向（人工）：按档案 align 重对齐，方向须正确 =="
  scripts/lab_perception_grasp_campaign.sh imu-tf || true; }
case "$STEP" in
  i1) i1;; i2) i2;; i3) i3;; i4) i4;; i5) i5;; i6) i6;; i7) i7;;
  all) i1; i2; i3; i4; i5; i6; i7;;
  *) echo "用法: imu_steps.sh [i1..i7|all]" >&2; exit 2;;
esac
