# Rider-PC-Client — Claude notes

> **Hardware inventory: [`docs/HARDWARE.md`](docs/HARDWARE.md)** — servo IDs↔roles↔models,
> firmware states, Pi/camera/controller. Read it first; **update it on any hardware change.**

## Working conventions (any Claude session in this repo)

- **Git cadence.** Commit finished, *verified* work at stable checkpoints — **not** after
  every edit, and not mid-exploration (premature commits get invalidated when the approach
  changes). Cadence is at your discretion; regular commits are wanted so the repo stays
  reconstructible. Commit straight to **`main`**, never a feature branch. Verify
  `git status`/`log` before claiming anything about repo state. Force-push / history
  rewrites need explicit OK.
- **Reconstruction from scratch is the bar.** New hardware + this repo must rebuild the whole
  system. Keep the code, firmware, the six-service deploy (`pi/deploy_bridge.sh`), and the Pi
  host-state runbook (`docs/PI_HOST_SETUP.md`) current when you change how it's built or run.
  If a thing can be regenerated on a fresh system, don't commit a binary snapshot of it; if it
  can't (e.g. the last copy of a stock firmware image), preserve it in `firmware/prebuilt/`.
- **Secrets.** This repo is **PUBLIC — never commit a secret value.** The one reconstruction
  secret (WiFi PSK) lives in a gitignored `pi/secrets.env`, from the committed
  `pi/secrets.env.example` template. (Secrets may be folded in only if a repo is private *and*
  discloses it — not this one.)
- **Hosts / IPs.** Reach the Pi with **`ssh rider`**, never a hardcoded IP. Scripts read the
  broker from `gui/rider_config.ini` (see `tools/diag/mqtt_monitor.py`) or take it as an arg —
  don't embed `10.0.0.x`.
- **Code hygiene.** No dead/orphan code — when a firmware command or feature is removed, purge
  every caller (the `polrun` removal is the reference). Test before claiming it works: real
  output/behavior, not "it built." Propose substantive changes (new mechanisms, behavior
  swaps) before making them; a question ("can I…") wants an answer, not an implementation.
- **Robot safety.** Never arm/drive the wheels on the stand. Power-cycle the ESP32 after every
  flash. **Ask before flashing** — it overwrites whatever is on the ESP32. State the required
  physical state (USB-C, power, on-stand vs floor) at each step.

## Hardware / firmware state lives outside the repo

The XGO Rider's controller is an **ESP32** flashed via USB-C (CH340). Two firmwares matter:

- **Stock xgolib firmware** (e.g. `R-1.1.3`) — what ships on the device. Speaks the xgolib protocol over `/dev/ttyUSB0`. On `R-1.1.3`:
  - `read_battery()`, `read_firmware()`, `read_roll/pitch/yaw()` all work.
  - `read_motor()` returns `[]` — motor angle read is **not** exposed.
  - `unload_allmotor()` / `unload_motor()` are **silently ignored** — torque stays on.
- **Passthrough firmware** (`firmware/esp32_passthrough/`, PlatformIO project) — bridges USB-C straight to the SCS servo bus at 1 Mbps. Required for the `tools/servo/*.py` / cal scripts that import `scservo_sdk`. With passthrough flashed:
  - Direct register R/W on servo IDs (leg servos are **12** and **22**).
  - Torque enable at reg `0x28`, present position at reg `0x24`, encoder offset at reg `0x1F` (writes don't persist — see global memory `project_xgo_rider_cal_storage.md`; persistence goes through SPIFFS).
  - To flash: `cd firmware/esp32_passthrough && pio run -t upload` (esptool over CH340).

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

It runs these systemd services (deployed by `pi/deploy_bridge.sh`):
- **mosquitto** — MQTT broker (also reachable from the workstation at `<pi-ip>:1883`).
- **rider-bridge** (`pi/rider_status_screen.py`) — UART `/dev/ttyAMA0` ↔ ESP32 bridge: LCD status,
  telemetry → MQTT republish, command relay. It is the **single serial owner** of the Pi↔ESP32 link.
- **rider-joystick** (`pi/rider_controller.py`) — DS4 → MQTT drive/turn commands.
- **rider-camera** (`pi/rider_camera.py`) — CSI camera (OV5647/picamera2) → MQTT image responder, on-demand.
- **rider-tof** (`pi/rider_tof.py`) — VL53L5CX 8×8 ToF frames → `rider/tof`. Runs on the **`tofvenv`**
  venv (not xgovenv): it needs `vl53l5cx-ctypes` + `smbus2`.
- **rider-tof-safety** (`pi/rider_tof_safety.py`) — floor-referenced obstacle/cliff governor →
  `rider/safety/fwd_limit`; the joystick service caps **forward** drive by it (reverse never limited).
- **rider-recorder** — bare `mosquitto_sub` unit logging `rider/debug/telem` + `rider/debug/dcap`
  to `/home/pi/riderlog.txt`, always on so no fall/swing is ever missed. Needs `mosquitto-clients`.

Read-only `ssh rider …` commands are auto-allowed (`.claude/settings.json`); changes
(`sudo`, `systemctl restart`, `scp`, `rm`, …) prompt. LCD button + LED pin map: `docs/xgo-cm4-pinout.md`.

## Project layout (reorganized 2026-06-15) — three codebases, each in its own home

```
gui/       workstation GUI: pc_client_standalone.py + core/ ui/ communication/, rider_config.ini, requirements.txt
pi/        RPi CM5 code: rider_status_screen.py, rider_controller.py, rider_camera.py, *.service, deploy_bridge.sh
firmware/  ESP32 code: esp32_rider_fw/ (balance fw) + esp32_passthrough/ (servo-bus passthrough fw)
tools/     servo/ wheel/ balance/ capture/ diag/ tof/   bench + diagnostic scripts (need passthrough, or run via MQTT)
docs/      HARDWARE, BRIDGE_SETUP, servo_registers, xgo-cm4-pinout, factory_roll_balance, Rider-Pi_SCH.pdf
sim/       MuJoCo + SB3 RL sim (separate project)
README.md, CLAUDE.md, start_gui.sh    at root
```

Run the GUI: `./start_gui.sh` (cd's into gui/ so package imports + config resolve).
Flash firmware: `cd firmware/esp32_rider_fw && /home/marc/.xgo-cal/bin/pio run -t upload`.
Deploy the Pi stack: `pi/deploy_bridge.sh rider`.

The root holds only README/CLAUDE/start_gui.sh + the component dirs. Bench scripts live in
`tools/<group>/`; dead legacy-movement and failed-approach scripts were trashed 2026-06-15.
`tools/servo/` now holds only the read-only readers — `leg_read.py` (encoder monitor) and
`servo_status.py` (verified-register snapshot). The cal/SPIFFS-offset/set-zero tooling was trashed
2026-06-24 after the leg-servo replacement: the new healthy encoder + persistent hard angle limits
(`0x06`/`0x08`) replaced the runaway-extension workaround. See `docs/servo_registers.md`.

## Known firmware bug we worked around

Negative encoder readings on a gear-rotated leg servo caused runaway extension to the mechanical limit (commit `35319b5`). Fix: patch SPIFFS-stored cal offsets directly so commanded encoder positions stay strictly positive across the leg's range. Servo register `0x1F` writes do not persist; SPIFFS at `0x290000+0x080000` (two int16 LE encoder offsets) is authoritative.
