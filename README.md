# Rider-PC-Client

Control, firmware, and tooling for a modified **XGO Rider** — a two-wheel self-balancing
robot. The repo holds four codebases that run on three machines:

| Dir | Runs on | What it is |
|---|---|---|
| `gui/` | workstation | Tkinter monitoring + control client, talks MQTT |
| `pi/` | robot's Pi (CM5) | six systemd services: LCD/serial bridge, DS4 joystick, camera, ToF publisher, ToF safety governor, telemetry recorder |
| `firmware/` | ESP32 | `esp32_rider_fw/` self-balance firmware + `esp32_passthrough/` servo-bus passthrough |
| `tools/` | workstation | bench + diagnostic scripts (`balance/ capture/ diag/ servo/ tof/ wheel/`) |

`sim/` is a separate MuJoCo + SB3 RL project. `docs/` holds the hardware inventory and
setup guides — **start with [`docs/HARDWARE.md`](docs/HARDWARE.md)**.

## Architecture

```
workstation ──MQTT──► Pi (mosquitto) ──UART 115200 line proto──► ESP32 ──SCS bus 1Mbps──► servos
   gui/                rider-bridge owns /dev/ttyAMA0                    wheels 11/21, legs 12/22
   tools/              + 5 other services, all MQTT
```

The ESP32 runs the balance loop; the Pi is a bridge and sensor host; the workstation
only ever sends MQTT.

## Quick start

**Run the GUI** (from the repo root — the script `cd`s into `gui/` so package imports
and config resolve):

```bash
./start_gui.sh
```

Dependencies are in `gui/requirements.txt` (paho-mqtt, configparser, pillow; tkinter ships
with Python). Install them into a venv — **not** with bare `pip`, which fails on
PEP 668-managed systems like this workstation:

```bash
uv pip install -r gui/requirements.txt     # or: <your-venv>/bin/pip install -r gui/requirements.txt
```

Broker host is set in [`gui/rider_config.ini`](gui/rider_config.ini).

**Deploy the Pi stack:**

```bash
pi/deploy_bridge.sh rider
```

**Flash the balance firmware** (ESP32 on USB-C, `/dev/ttyUSB0`):

```bash
cd firmware/esp32_rider_fw && /home/marc/.xgo-cal/bin/pio run -t upload
```

One binary carries both balance controllers: it boots with the RL policy, and the firmware
command `ctrl 1` switches to the LQR loop (`ctrl 0` back), accepted only while balancing is
disabled. See `docs/BRIDGE_SETUP.md`.
Flashing overwrites whatever firmware is on the ESP32 — see `docs/HARDWARE.md`.

> ⚠️ **Never arm balancing with the robot on a stand.** The wheels run away unloaded.
> Power-cycle the ESP32 after every flash.

## Docs

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — servo IDs ↔ roles ↔ parts, firmware states, Pi, camera, ToF wiring. **Read first; update on any hardware change.**
- [`docs/PI_HOST_SETUP.md`](docs/PI_HOST_SETUP.md) — rebuild the Pi's host state from bare metal (kernel, WiFi, modules, venvs, DS4 pairing). The burn-down runbook.
- [`docs/BRIDGE_SETUP.md`](docs/BRIDGE_SETUP.md) — deploy the services + firmware onto a host that's already set up.
- [`docs/servo_registers.md`](docs/servo_registers.md) — SCS servo-bus register map, confirmed vs unverified.
- [`docs/xgo-cm4-pinout.md`](docs/xgo-cm4-pinout.md) — Pi GPIO, LCD buttons, ESP32 pins.
- [`docs/factory_roll_balance.md`](docs/factory_roll_balance.md) — decompiled stock roll-leveling, and how ours maps to it.
- [`firmware/prebuilt/README.md`](firmware/prebuilt/README.md) — known-good flashable images.
- [`docs/loop_delay_budget.md`](docs/loop_delay_budget.md) — the balance-loop delay budget, the sim calibration, and the 2026-09 policy floor tests.
- [`docs/WEB_UI_DESIGN.md`](docs/WEB_UI_DESIGN.md) — design (not built yet) for a browser control page served by the Rider, replacing the Tk GUI; optional no-WiFi hotspot mode.

## External assets (not tracked — re-fetch on a fresh clone)

`vendor/` is gitignored: ~390 MB of third-party reference code and CAD the docs cite, re-fetchable from
upstream. Put each at the path shown so the doc references resolve.

| path | what | source | version |
|---|---|---|---|
| `vendor/Luwu-OS-2.0.1/` | Stock Luwu OS for the Rider's Pi: `libs/xgolib/` (the xgolib serial protocol layer), `HARDWARE_PLAN.md` (CM5 build) | github.com/LuwuDynamics — Luwu-OS, release `2.0.1` (Apache-2.0) | 2.0.1 |
| `vendor/RaspberryPi-CM4-main/` | Stock XGO CM4 stack: `key.py` (LCD buttons), WS2812 LED driver, demos | github.com/Xgorobot/RaspberryPi-CM4, branch `main`, zip download (no license file; **do not commit**) | main as of 2026-10 |
| `vendor/xgo-cad/` | STEP models: `XGO-Rider.STEP` (full mechanical assembly, 847 solids, AP203), `XGO-Rider-V1.3.step` (ESP32 mainboard PCB), `XGO-CarrierBoard-V1.3.step` (Pi carrier PCB). 115 MB. | vendor CAD supplied by Marc 2026-10-18; upstream download location not recorded (no license; **do not commit**) | Rider assembly "_0624"; boards V1.3 |

Neither is needed to build or run anything here; they are the reference the register map and
pinout docs were derived from.

## Environment

Use the `/home/marc/.xgo-cal/` venv for anything importing `xgolib` or `scservo_sdk`
(system pip is PEP 668 managed). Reach the Pi with `ssh rider` — the alias lives in
`~/.ssh/config`; never hardcode the IP.
