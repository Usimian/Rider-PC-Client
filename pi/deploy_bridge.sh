#!/usr/bin/env bash
# Deploy the Rider Pi bridge + DS4 controller + camera from the workstation to the robot's Pi.
# Idempotent + recreate-from-scratch: installs deps, the bridge/controller/camera programs,
# their systemd autostart units, retires the old rider-controller autostart, and starts them.
#
# Usage (from repo root):  pi/deploy_bridge.sh [host]      e.g.  pi/deploy_bridge.sh rider
#   default host: pi@10.0.0.95 (prefer the 'rider' ssh-config alias)
#
# Lives in pi/ alongside the programs + units it deploys (uses $HERE, so location-independent).
# Prerequisites (provided by the stock XGO Rider Pi image): the xgovenv venv,
# the xgoscreen LCD lib, the font /home/pi/model/msyh.ttc, /dev/ttyAMA0 free
# (serial console disabled), the pi user in the gpio+dialout groups, and a
# running mosquitto broker. Firmware is flashed separately (see ../docs/BRIDGE_SETUP.md).
set -euo pipefail
PI="${1:-pi@10.0.0.95}"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "==> copying bridge + controller + camera + ToF + units to $PI"
scp "$HERE/rider_status_screen.py" "$PI:/home/pi/rider_status_screen.py"
scp "$HERE/rider_controller.py"    "$PI:/home/pi/rider_controller.py"
scp "$HERE/rider_camera.py"        "$PI:/home/pi/rider_camera.py"
scp "$HERE/rider_tof.py"           "$PI:/home/pi/rider_tof.py"
scp "$HERE/rider_tof_safety.py"    "$PI:/home/pi/rider_tof_safety.py"
scp "$HERE/rider-bridge.service"   "$PI:/tmp/rider-bridge.service"
scp "$HERE/rider-joystick.service" "$PI:/tmp/rider-joystick.service"
scp "$HERE/rider-camera.service"   "$PI:/tmp/rider-camera.service"
scp "$HERE/rider-tof.service"      "$PI:/tmp/rider-tof.service"
scp "$HERE/rider-tof-safety.service" "$PI:/tmp/rider-tof-safety.service"

echo "==> installing on $PI"
ssh "$PI" 'bash -s' <<'REMOTE'
set -euo pipefail
# 1. runtime deps in the XGO venv (no-op if already satisfied). The ToF safety governor
#    also runs on xgovenv (needs only paho + stdlib), so it's covered here.
/home/pi/xgovenv/bin/pip install -q paho-mqtt lgpio pyserial psutil pillow
# 1b. rider_tof.py needs the tofvenv (vl53l5cx-ctypes + smbus2 + paho -- set up once by hand;
#     the driver uploads ~84KB sensor firmware). Ensure paho is present if the venv exists;
#     warn (don't abort) if it's missing so the rest of the deploy still lands.
if [ -x /home/pi/tofvenv/bin/pip ]; then
  /home/pi/tofvenv/bin/pip install -q paho-mqtt smbus2 || true
else
  echo "  WARN: /home/pi/tofvenv missing -- rider-tof.service (sensor publisher) will not start."
  echo "        Create it with vl53l5cx-ctypes + smbus2 + paho-mqtt, then re-run this deploy."
fi
# 2. ensure the broker is up
sudo systemctl enable --now mosquitto >/dev/null 2>&1 || true
# 2b. boot-speed: mask NetworkManager-wait-online. It burns ~6s waiting for full WiFi
#     connectivity, which gated the whole boot (mosquitto Wants=network-online.target, and
#     rider-bridge/rider-joystick order after mosquitto). The broker + our services only
#     use the LOCAL connection, so network-online.target resolving immediately is fine ->
#     services come up ~6s sooner. (Removing it from mosquitto via drop-in didn't reset
#     cleanly on this systemd; masking the wait service is the reliable fix.)
sudo systemctl mask NetworkManager-wait-online.service
# 3. install the systemd units
sudo install -m 644 /tmp/rider-bridge.service      /etc/systemd/system/rider-bridge.service
sudo install -m 644 /tmp/rider-joystick.service    /etc/systemd/system/rider-joystick.service
sudo install -m 644 /tmp/rider-camera.service      /etc/systemd/system/rider-camera.service
sudo install -m 644 /tmp/rider-tof.service         /etc/systemd/system/rider-tof.service
sudo install -m 644 /tmp/rider-tof-safety.service  /etc/systemd/system/rider-tof-safety.service
rm -f /tmp/rider-bridge.service /tmp/rider-joystick.service /tmp/rider-camera.service \
      /tmp/rider-tof.service /tmp/rider-tof-safety.service
sudo systemctl daemon-reload
# 4. retire the old (xgolib) controller autostart if present
if systemctl list-unit-files 2>/dev/null | grep -q '^rider-controller.service'; then
  sudo systemctl disable --now rider-controller.service || true
fi
# 5. enable on boot + (re)start now
sudo systemctl enable rider-bridge.service rider-joystick.service rider-camera.service \
                      rider-tof.service rider-tof-safety.service >/dev/null
sudo systemctl restart rider-bridge.service rider-joystick.service rider-camera.service \
                       rider-tof.service rider-tof-safety.service
sleep 2
echo "  bridge  : $(systemctl is-active rider-bridge.service)/$(systemctl is-enabled rider-bridge.service)"
echo "  joystick: $(systemctl is-active rider-joystick.service)/$(systemctl is-enabled rider-joystick.service)"
echo "  camera  : $(systemctl is-active rider-camera.service)/$(systemctl is-enabled rider-camera.service)"
echo "  tof     : $(systemctl is-active rider-tof.service)/$(systemctl is-enabled rider-tof.service)"
echo "  tof-safe: $(systemctl is-active rider-tof-safety.service)/$(systemctl is-enabled rider-tof-safety.service)"
echo "  old     : rider-controller = $(systemctl is-enabled rider-controller.service 2>&1)"
REMOTE
echo "==> done"
