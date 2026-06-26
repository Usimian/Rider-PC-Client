# XGO Rider — hardware inventory

Persistent description of the Rider's physical hardware so no session has to re-derive it.
**Update this file whenever hardware changes** (servo swap, firmware change, Pi swap, etc.).
Companion docs: `servo_registers.md` (SCS bus register map), `xgo-cm4-pinout.md` (Pi GPIO/LCD),
`CALIBRATION_GUIDE.md`, `BRIDGE_SETUP.md`.

> Confidence: **✅ verified on our hardware** vs **❔ not yet read/recorded**. Don't state a
> ❔ value as fact — read it and promote it to ✅.

## Controller (ESP32)
- **ESP32** on the XGO controller board, USB-C via **CH340** → host `/dev/ttyUSB0`.
- Three firmwares ever flashed (only one at a time — flashing overwrites):
  - **Stock xgolib** (`R-1.1.3`) — vendor protocol; `read_motor()`/`unload_*` are no-ops.
  - **Balance/LQR** (`firmware/esp32_rider_fw/`) — owns the serial link, our balancer.
  - **Passthrough** (`firmware/esp32_passthrough/`) — bridges USB-C ↔ SCS bus @ 1 Mbps for
    `scservo_sdk`/`tools/servo/*`. **Required for direct servo register R/W.**
- `fw: Null` from an xgolib probe means xgolib couldn't handshake — true for **both** balance
  **and** passthrough fw, so it does NOT identify which is flashed. Disambiguate by scanning the
  servo bus (`scservo_sdk` ping): a silent bus = not passthrough.

## Servo bus
- Servos are **NBD-branded, customized from K-power parts** (per Luwu source 2026-06-24). They
  speak the **FeeTech SCS protocol** (`0xFF 0xFF` header, `PacketHandler(0)`), half-duplex **1 Mbps**,
  but use **K-power's register map** (see `servo_registers.md` for the verified leg map).
- The ping "model number" is **bogus** — `scservo_sdk` reads it from addr `0x03` (the ID register),
  so it returns `ID | 0x0100` (ID12→268, ID22→278). NOT a real model id; do not treat as one.
- Read live via passthrough fw (`tools/servo/servo_status.py`) **or**, under balance fw,
  via `cfgdump <id>` over MQTT.

## Servo inventory

| ID | Role | Part | Status | Notes |
|----|------|------|--------|-------|
| 11 | Left wheel  | K-power **4012** (FOC BLDC) — likely (motor opened 2026-06-24), measure to confirm | original | velocity-open-loop, reg `0x11`=0; drive `0x1E`, limp `0x18` |
| 21 | Right wheel | K-power **4012** (FOC BLDC) — likely (motor opened 2026-06-24), measure to confirm | original | velocity-open-loop, reg `0x11`=0; drive `0x1E`, limp `0x18` |
| 12 | Left leg    | **K-power RC08P** (geared) | **original — kept** | torque off, mode `0x21`=0. **Travel `0x24`: 863→977, span 114** — re-confirmed 2026-06-24 (unchanged; never disconnected). **Hard limits `0x06`/`0x08` = 865/975** (set 2026-06-24; was factory 973/1023; **persists across power-cycle, verified**). |
| 22 | Right leg   | **K-power RC08P** (geared) | **NEW, installed** (2026-06-24) | pre-addressed as 22. **Travel `0x24` (reattached, re-measured 2026-06-24): 422→538, span 116** (short≈422, extended≈538). Pre-reattach disconnected-horn reading was 467→578 — shifted ~45 down by horn rotation. **Hard limits `0x06`/`0x08` = 425/535** (set 2026-06-24, just inside stops; **persists across power-cycle, verified**). |

**Wheel motor — CUSTOM K-power part, 4012-FAMILY (not the catalog 4012).** Per Luwu source
(2026-06-24): the wheel motor is a **custom product, not sold on K-power's site** — likely a 4012
with changes. Marc opened the motor; form factor matches the 4012 (`kpower.com/ec4012_bldc`).
Board marking: **`791401A_Y516_240716`** (custom controller PCB, dated 2024-07-16).
- **Catalog 4012 specs are a FAMILY reference only — NOT authoritative for this unit.** Treat
  voltage/KV/torque/RPM as approximate; measure anything that matters. (Catalog 4012: external-rotor
  BLDC, integrated FOC, 14-bit magnetic encoder, UART-TTL, 12V/9–16V, 0.13/0.2 N·m, 400 rpm — the
  12V rating likely differs on the custom rewind for the Rider's pack.)
- **What IS authoritative = our bench findings** (custom firmware, no datasheet exists): drive via
  sync-write `0x1E`, mode `0x11`=0, limp `0x18`; encoder read via RAW protocol (see
  `servo_registers.md` — scservo reads return garbage `516`), **bus position 10-bit/1024-wrap**
  (physical encoder 14-bit but not exposed), odometry verified 2026-06-24.
**Leg servo — K-power RC08P:** geared, 10-bit (0–1023), 200° travel, full register map verified
(see `servo_registers.md`).

**Only the right leg (ID22) was replaced** (2026-06-24); the left (ID12) original stays. Both legs
are the same part (**K-power RC08P**); the new right unit shipped **pre-addressed as 22** — no ID
reassignment needed. (The "268/278" seen earlier were the ID-register ping artifact, not models.)

**Original right leg (ID22)** had a dead/negative encoder → runaway extension (memory
`project_xgo_rider_wheel_*`, commit `35319b5`). Note: a removed original reads a clean *positive*
position at rest — the fault only showed mid-range under motion, so a static read doesn't clear it.

## Leg calibration storage
- Encoder cal = **SPIFFS** at `0x290000 + 0x080000`, two int16 LE encoder offsets.
- Servo register `0x1F` (encoder offset) writes **do NOT persist** — SPIFFS is authoritative
  (memory `project_xgo_rider_cal_storage`). Fresh servos read offset≈0/1; cal still TBD.

## Raspberry Pi (robot brain)
- **Compute Module 5** on an `XGO-CM4-V1.1` carrier (CM4 was swapped out). Reach via `ssh rider`
  (alias in `~/.ssh/config`, user `pi`; resolves `raspberrypi.local` via mDNS). Do not hardcode IP.
- Owns UART `/dev/ttyAMA0` ↔ ESP32 (single serial owner). Services: `mosquitto`, `rider-bridge`,
  `rider-joystick`, `rider-camera` (see project CLAUDE.md).

## Camera / input
- **CSI camera** OV5647 via picamera2 (`pi/rider_camera.py`).
- **DS4** PS4 controller over BT on the Pi (`/dev/input/js0`, pygame; ERTM left at default,
  kernel pinned 6.12.47 — memory `project_xgo_rider_controller`).

## I²C sensors (Pi — bus 1)
- Pi I²C **bus 1** = GPIO2 (SDA) / GPIO3 (SCL). The carrier's **WM8960 audio codec** lives here at
  **0x1A** (shows `UU` = kernel-driver-bound) — a handy "bus-alive" reference.
- **Scan:** `ssh rider 'export PATH=$PATH:/usr/sbin; i2cdetect -y -r 1'`. Two gotchas: the `i2cdetect`
  binary is in `/usr/sbin` (NOT on the non-interactive `ssh` PATH), and a bare *read* probe misses the
  WM8960 because Wolfson codecs are **write-only** over I²C — use `i2cdetect`, not a read-scan.
- **VL53L5CX ToF** (8×8 multizone ranging) — added 2026-06-26, ✅ **ranging confirmed** (62/64 zones
  valid @ ~1.8 m). Survived an accidental 5 V on VDDA (below) with no damage.
  - **Wiring:** 4 wires — VCC, GND, SDA, SCL. SDA/SCL tapped onto the **WM8960's SDIN (=SDA) /
    SCLK (=SCL)** pins: the CM5 exposes **no GPIO pads** (all I/O via the board-to-board connectors) and
    the carrier header is too fine-pitch, so flexible small-gauge wire soldered **to the traces** +
    **superglue strain relief**. Same bus as GPIO2/3.
  - **Power: 3.3 V (VDDA + VDDIO).** ⚠️ VDDA is a SEPARATE analog rail. Feeding it **5 V** (the carrier
    4-pin connector's power pin) lets the sensor enumerate, load firmware, and report `is_alive` — but it
    returns **0 ranges / status 255**: over-spec analog can't fire the VCSEL. **Diagnostic rule: talks +
    loads firmware + alive but no ranging ⇒ analog supply (VDDA) wrong.** Fix = VDDA on 3.3 V.
  - **3.3 V source:** the **WM8960** is a 3.3 V part — its 3.3 V pins are **DBVDD (pin 10)** and **AVDD
    (pin 32)**; AVOID **SPKVDD1 (26)/SPKVDD2 (21)** (can be 5 V) and **DCVDD (8)** (1.8 V). Tap the
    decoupling cap beside the pin (bigger pad); meter 3.3 V first.
  - **LPn** high enables I²C (pulled to VCC on the breakout → the single VCC feed handles it).
  - **Driver/test:** Pimoroni `vl53l5cx-ctypes` in venv **`~/tofvenv`** on the Pi (builds ST's ULD +
    bundles the ~84 KB firmware). Ranging test: **`~/tofvenv/bin/python ~/tof_test.py`**.
  - **Gotcha:** cyanoacrylate outgassing **fogs the ToF cover glass** — keep fresh CA off the sensor window.
