# Rider — recreate the system from scratch

Everything needed to rebuild the running robot from the repo. Two halves: the
**ESP32 firmware** (balance + telemetry) and the **Pi bridge** (LCD + MQTT +
button). The workstation GUI needs no setup beyond `rider_config.ini`.

## Architecture

```
Workstation GUI / rider_cmd.py ──MQTT──► Pi (mosquitto + bridge) ──UART line proto──► ESP32
                                          │  rider_status_screen.py owns /dev/ttyAMA0
                                          └─ drives the XGO 2-inch LCD + C-button toggle
```

- **MQTT** (workstation ↔ Pi): telemetry on `rider/status`, `rider/status/imu`,
  `rider/status/battery`; commands on `rider/control/line` (`{"line":"en 1"}`)
  and `rider/control/system` (emergency-stop → `en 0`).
- **Line protocol** (Pi ↔ ESP32, 115200 on `/dev/ttyAMA0`): `th=`/`roll=`/`yaw=`/
  `wx=`/`vbat=`/`batt=`/`en=`… telemetry; `polrun`/`en`/`ptgt`/… commands.

## 1. ESP32 firmware

From the workstation, ESP32 USB-C connected (`/dev/ttyUSB0`):

```bash
cd esp32_rider_fw
/home/marc/.xgo-cal/bin/pio run -t upload
```

Provides balance policy + telemetry: `roll` (accel), `yaw` (gyro-Z integrated),
`vbat`/`batt` (GPIO33 divider, ratio 3.0 from schematic R8 20K / R7 10K, 2S 8.4 V).
Note: every flash/power-cycle resets `polrun`→0 (PID mode); the C button and
`rider_cmd.py start` both re-arm `polrun 1` so this is transparent.

## 2. Pi bridge (one command)

From the workstation:

```bash
./deploy_bridge.sh            # or: ./deploy_bridge.sh pi@<ip>
```

This copies `rider_status_screen.py` → `/home/pi/`, installs venv deps
(paho-mqtt, lgpio, pyserial, psutil, pillow), installs `rider-bridge.service`
→ `/etc/systemd/system/`, disables the old `rider-controller.service`, and
enables+starts the bridge. It autostarts on every boot.

### Stock-image prerequisites (already true on the XGO Rider Pi)

- `xgovenv` venv at `/home/pi/xgovenv` with `xgoscreen` (LCD) installed
- font `/home/pi/model/msyh.ttc`
- `/dev/ttyAMA0` free (serial console / `serial-getty@ttyAMA0` disabled)
- `pi` in groups `gpio` + `dialout`; `mosquitto` installed
- C button = GPIO17 (upper-left, next to RIDER); A=24, B=23, D=22

## 3. Verify

```bash
ssh pi@10.0.0.95 'systemctl is-active rider-bridge.service; \
  timeout 2 mosquitto_sub -h localhost -t rider/status/imu | head -1'
python3 rider_cmd.py polrun 1        # arm policy (no motion; en still 0)
```

Then press the **upper-left button** to start/stop balancing, or
`python3 rider_cmd.py start` / `stop`.
