#!/bin/bash
# Install the Jetson Orin NX system files (udev / netplan / chrony / systemd) for the
# inspection drone. Run with sudo on the drone's companion computer. Review each file first —
# netplan IPs and udev device ids are placeholders that MUST match your hardware.
set -e
HERE="$(cd "$(dirname "$0")/../system" && pwd)"

if [ "$(id -u)" != "0" ]; then echo "run with sudo"; exit 1; fi

echo ">> udev rules"
cp "$HERE/udev/99-inspection-sensors.rules" /etc/udev/rules.d/
udevadm control --reload && udevadm trigger

echo ">> chrony (time island)"
cp "$HERE/chrony/chrony-jetson.conf" /etc/chrony/chrony.conf
systemctl restart chrony || true

echo ">> netplan (REVIEW eth0 IP first — must match MID360_config.json)"
cp "$HERE/netplan/01-inspection-net.yaml" /etc/netplan/
echo "   run 'sudo netplan apply' after confirming the addresses"

echo ">> systemd unit (auto-start on boot — enable manually once validated)"
cp "$HERE/systemd/inspection.service" /etc/systemd/system/
systemctl daemon-reload
echo "   enable with: sudo systemctl enable inspection.service"

echo ">> done. Reboot or replug sensors to apply udev; run scripts/preflight_check.sh before flight."
