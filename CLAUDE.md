# Rider-PC-Client — Claude notes

## Hardware / firmware state lives outside the repo

The XGO Rider's controller is an **ESP32** flashed via USB-C (CH340). Two firmwares matter:

- **Stock xgolib firmware** (e.g. `R-1.1.3`) — what ships on the device. Speaks the xgolib protocol over `/dev/ttyUSB0`. On `R-1.1.3`:
  - `read_battery()`, `read_firmware()`, `read_roll/pitch/yaw()` all work.
  - `read_motor()` returns `[]` — motor angle read is **not** exposed.
  - `unload_allmotor()` / `unload_motor()` are **silently ignored** — torque stays on.
- **Passthrough firmware** (`esp32_passthrough/`, PlatformIO project) — bridges USB-C straight to the SCS servo bus at 1 Mbps. Required for any of the `servo_*.py` / `watch_*.py` scripts that import `scservo_sdk`. With passthrough flashed:
  - Direct register R/W on servo IDs (leg servos are **12** and **22**).
  - Torque enable at reg `0x28`, present position at reg `0x24`, encoder offset at reg `0x1F` (writes don't persist — see global memory `project_xgo_rider_cal_storage.md`; persistence goes through SPIFFS).
  - To flash: `cd esp32_passthrough && pio run -t upload` (esptool over CH340).

**Check what's currently flashed before assuming anything works:**

```bash
/home/marc/.xgo-cal/bin/python -c "from xgolib import XGO; d=XGO(port='/dev/ttyUSB0', version='xgorider'); print('fw:', d.read_firmware(), 'motor:', d.read_motor())"
```

If `motor:` is `[]` → stock firmware. If xgolib import fails or the call hangs → likely passthrough is flashed.

## Python environment

Use the **`/home/marc/.xgo-cal/`** venv — it has both `xgolib` and `scservo_sdk` installed. System pip is PEP 668 managed and does not have these modules.

```bash
/home/marc/.xgo-cal/bin/python <script>.py
```

## Raspberry Pi (the robot's brain) — reach it via `ssh rider`

The Pi is a **Compute Module 5** (the stock CM4 was swapped out; the carrier is still
silk-screened `XGO-CM4-V1.1`). Always reach it with **`ssh rider`** — an alias in
`~/.ssh/config` (`User pi`). **Do not hardcode the IP**; if it ever changes, update the one
`rider` `HostName` line (it currently resolves `raspberrypi.local` too, via mDNS).

It runs these systemd services (deployed by `./deploy_bridge.sh`):
- **mosquitto** — MQTT broker (also reachable from the workstation at `<pi-ip>:1883`).
- **rider-bridge** (`rider_status_screen.py`) — UART `/dev/ttyAMA0` ↔ ESP32 bridge: LCD status,
  telemetry → MQTT republish, command relay. It is the **single serial owner** of the Pi↔ESP32 link.
- **rider-joystick** (`rider_controller.py`) — DS4 → MQTT drive/turn commands.

Read-only `ssh rider …` commands are auto-allowed (`.claude/settings.json`); changes
(`sudo`, `systemctl restart`, `scp`, `rm`, …) prompt. LCD button + LED pin map: `xgo-cm4-pinout.md`.

## Script map

| Script | Needs passthrough? | What it does |
|---|---|---|
| `watch_right_leg.py` | yes | Disables torque on ID 22, prints encoder reads |
| `watch_servos_live.py` | yes (scservo_sdk path) | Disables torque on 12 & 22, prints position + load |
| `servo_watch.py` | yes | Full 0x00–0x7F register dump, IDs 12 & 22 |
| `servo_status.py` | yes | One-shot pos/goal/torque/lock/offset for 12 & 22 |
| `watch_legs_live.py` | no (uses xgolib) | Reads motor angles via xgolib — **does not work on R-1.1.3** (returns empty) |
| `calibration_tester.py` | no | High-level movement calibration over MQTT, separate from servo work |
| `cal_save.py`, `cal_spam.py`, `offset_test_v*.py` | yes | SPIFFS cal-offset patching tooling |

## Known firmware bug we worked around

Negative encoder readings on a gear-rotated leg servo caused runaway extension to the mechanical limit (commit `35319b5`). Fix: patch SPIFFS-stored cal offsets directly so commanded encoder positions stay strictly positive across the leg's range. Servo register `0x1F` writes do not persist; SPIFFS at `0x290000+0x080000` (two int16 LE encoder offsets) is authoritative.

## When in doubt

Ask before flashing — overwrites whatever is currently on the ESP32.
