#!/usr/bin/env bash
set -euo pipefail
TARGET_USER=${SUDO_USER:-${USER}}
NIC=${AUBO_NIC:-enp130s0}
if (( EUID != 0 )); then
  exec sudo --preserve-env=AUBO_NIC "$0" "$@"
fi
getent group realtime >/dev/null || groupadd realtime
usermod -a -G realtime "$TARGET_USER"
install -d -m 0755 /etc/security/limits.d
printf "@realtime soft rtprio 99\n@realtime hard rtprio 99\n@realtime soft memlock unlimited\n@realtime hard memlock unlimited\n" > /etc/security/limits.d/99-aubo-realtime.conf
for governor_file in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  [[ -e "$governor_file" ]] || continue
  printf "performance" > "$governor_file"
done
if command -v ethtool >/dev/null && ip link show dev "$NIC" >/dev/null 2>&1; then
  ethtool -K "$NIC" gro off gso off tso off || true
fi
echo "Realtime limits installed for group realtime and CPU governor set to performance."
echo "Log out and log back in, then run realtime_preflight.sh."
echo "For strict hard real-time, install a PREEMPT_RT kernel and reboot."
