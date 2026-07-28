#!/usr/bin/env bash
set -euo pipefail
NIC=${AUBO_NIC:-enp130s0}
fail=0
rtprio=$(ulimit -r)
memlock=$(ulimit -l)
if [[ "$rtprio" != "unlimited" ]] && (( rtprio < 50 )); then
  echo "FAIL: rtprio=$rtprio, required >=50"
  fail=1
else
  echo "PASS: rtprio=$rtprio"
fi
if [[ "$memlock" != "unlimited" ]] && (( memlock < 1048576 )); then
  echo "FAIL: memlock=${memlock}KiB, required unlimited or >=1GiB"
  fail=1
else
  echo "PASS: memlock=$memlock"
fi
for governor_file in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [[ -e "$governor_file" ]] || continue
  governor=$(<"$governor_file")
  if [[ "$governor" != "performance" ]]; then
    echo "FAIL: $governor_file=$governor"
    fail=1
  fi
done
if ! ip link show dev "$NIC" >/dev/null 2>&1; then
  echo "FAIL: AUBO NIC $NIC not found"
  fail=1
else
  echo "PASS: AUBO NIC $NIC exists"
fi
# NIC offload 检查（2026-07-24 网卡事件，docs/nic_driver_incident.md）：
# r8126 驱动 + gro/gso/tso 开启曾是 SDK 通道秒级停滞的诱因之一，
# configure_realtime.sh 会关闭，但该设置重启后不持久，这里做 WARN 提醒。
if command -v ethtool >/dev/null && ip link show dev "$NIC" >/dev/null 2>&1; then
  offload_on=$(ethtool -k "$NIC" 2>/dev/null | \
    grep -E '^(generic-receive-offload|generic-segmentation-offload|tcp-segmentation-offload):' | \
    grep -c ': on' || true)
  if (( offload_on > 0 )); then
    echo "WARN: $NIC has $offload_on offload(s) ON (gro/gso/tso) — run configure_realtime.sh (channel degradation risk, see docs/nic_driver_incident.md)"
  else
    echo "PASS: $NIC offloads gro/gso/tso all off"
  fi
fi
if [[ "$(uname -a)" != *PREEMPT_RT* ]]; then
  echo "WARN: kernel is not PREEMPT_RT; 200Hz is soft real-time only"
fi
exit "$fail"
