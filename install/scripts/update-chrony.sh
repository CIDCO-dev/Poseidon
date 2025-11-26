#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (sudo $0)" >&2
  exit 1
fi

cat >/etc/chrony/chrony.conf <<'EOF'
# PPS: /dev/pps0: Kernel-mode PPS ref-clock for the precise seconds
refclock  PPS /dev/pps0  refid PPS  precision 1e-9  poll 3  trust
# SHM(0), gpsd: NMEA data from shared memory provided by gpsd
refclock  SHM 0  refid NMEA  precision 1e-3  offset 0.0  delay 0.2  poll 3  trust  prefer
# Internet time source
pool pool.ntp.org iburst
# Allow LAN clients and local stratum when unsynced
allow
local
keyfile /etc/chrony/chrony.keys
driftfile /var/lib/chrony/chrony.drift
logdir /var/log/chrony
maxupdateskew 100.0
hwclockfile /etc/adjtime
rtcsync
makestep 1 3
EOF

systemctl restart chrony || true
echo "chrony configuration updated with PPS, NMEA (gpsd), and pool.ntp.org"
