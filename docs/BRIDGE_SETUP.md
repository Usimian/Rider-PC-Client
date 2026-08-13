# Rider — recreate the system from scratch

Everything needed to rebuild the running robot from the repo. Two halves: the
**ESP32 firmware** (balance + telemetry) and the **Pi stack** (six services —
LCD/serial bridge, joystick, camera, ToF publisher, ToF safety governor, recorder).
The workstation GUI needs no setup beyond `gui/rider_config.ini`.

> This assumes the Pi's **host state** is already in place (kernel, serial, WiFi, kernel
> modules, venvs, DS4 pairing). To build that from a bare image first, see
> [`PI_HOST_SETUP.md`](PI_HOST_SETUP.md).

## Architecture

```
Workstation GUI / tools/diag/rider_cmd.py ──MQTT──► Pi (mosquitto) ──UART line proto──► ESP32
                                                     │
   rider-bridge      rider_status_screen.py — owns /dev/ttyAMA0, drives the 2-inch LCD + buttons
   rider-joystick    rider_controller.py    — DS4 → drive/turn commands
   rider-camera      rider_camera.py        — CSI camera, on-demand image responder
   rider-tof         rider_tof.py           — VL53L5CX 8×8 frames  (venv: ~/tofvenv)
   rider-tof-safety  rider_tof_safety.py    — floor-referenced obstacle/cliff → forward cap
   rider-recorder    mosquitto_sub          — always-on telem log → /home/pi/riderlog.txt
```

Only `rider-bridge` touches the serial link; everything else talks MQTT.

- **MQTT** (workstation ↔ Pi): telemetry on `rider/status`, `rider/status/imu`,
  `rider/status/battery`, `rider/status/odom`, `rider/debug/telem`; ToF frames on
  `rider/tof` and the forward cap on `rider/safety/fwd_limit`; commands on
  `rider/control/line` (`{"line":"en 1"}`), `rider/control/movement`,
  `rider/control/drivemode`, `rider/control/settings`, `rider/control/image_capture`,
  and `rider/control/system` (emergency-stop → `en 0`).
- **Line protocol** (Pi ↔ ESP32, 115200 on `/dev/ttyAMA0`): `th=`/`roll=`/`yaw=`/
  `wx=`/`vbat=`/`batt=`/`en=`… telemetry; `en`/`ptgt`/`level`/… commands.

## 1. ESP32 firmware

From the workstation, ESP32 USB-C connected (`/dev/ttyUSB0`):

```bash
cd firmware/esp32_rider_fw
/home/marc/.xgo-cal/bin/pio run -t upload          # default env: esp32_lqr
```

Two build envs from one source, selected by a compile-time flag:

| env | flag | controller | telemetry `ctrl=` |
|---|---|---|---|
| `esp32_lqr` (**default**) | `-DCONTROLLER_LQR` | LQR full-state, torque-direct | `LQR` |
| `esp32` (`pio run -e esp32 -t upload`) | — | RL policy | `POL` |

Provides balance + telemetry: `roll` (accel), `yaw` (gyro-Z integrated),
`vbat`/`batt` (GPIO33 divider, ratio 3.0 from schematic R8 20K / R7 10K, 2S 8.4 V).
Balancing is armed with `en 1` and stopped with `en 0` — via the upper-left LCD
button, the DS4, or `rider_cmd.py start` / `stop`.

## 2. Pi stack (one command)

From the workstation, run from the repo root:

```bash
pi/deploy_bridge.sh rider              # or: pi/deploy_bridge.sh pi@<host>
```

This copies the five Pi programs → `/home/pi/`, installs venv deps
(paho-mqtt, lgpio, pyserial, psutil, pillow), installs all six `.service` units
→ `/etc/systemd/system/`, masks `NetworkManager-wait-online` (saves ~6 s of boot),
disables the old `rider-controller.service`, and enables + starts everything.
All six autostart on every boot.

### Stock-image prerequisites (already true on the XGO Rider Pi)

- `xgovenv` venv at `/home/pi/xgovenv` with `xgoscreen` (LCD) installed
- `tofvenv` venv at `/home/pi/tofvenv` with `vl53l5cx-ctypes` + `smbus2` + `paho-mqtt`
  — **built once by hand**; the deploy warns and skips `rider-tof` if it's absent
- font `/home/pi/model/msyh.ttc`
- `/dev/ttyAMA0` free (serial console / `serial-getty@ttyAMA0` disabled)
- `pi` in groups `gpio` + `dialout`; `mosquitto` **and `mosquitto-clients`** installed
  (the recorder unit is a bare `mosquitto_sub`; the deploy warns if it's missing)
- C button = GPIO17 (upper-left, next to RIDER); A=24, B=23, D=22

## 3. Verify

Use the `rider` ssh-config alias — never a hardcoded IP.

```bash
ssh rider 'for s in bridge joystick camera tof tof-safety recorder; do \
             printf "%-10s %s\n" $s "$(systemctl is-active rider-$s.service)"; done'
ssh rider 'timeout 2 mosquitto_sub -h localhost -t rider/status/imu | head -1'
ssh rider 'timeout 2 mosquitto_sub -h localhost -t rider/safety/fwd_limit | head -1'
```

Then press the **upper-left button** to start/stop balancing, or
`python3 tools/diag/rider_cmd.py start` / `stop`.

> ⚠️ Balancing drives the wheels. Never arm it with the robot on a stand —
> the wheels run away unloaded. Floor only.
