# XGO Rider servo-bus register reference (wheels 11/21, legs 12/22)

Low-level **FeeTech SCS servo-bus registers** for the Rider's servos, as used by the
ESP32 firmware over the 1 Mbps half-duplex bus. This is the layer `cfgdump <id>` reads.

> **⚠️ Two different "register" layers — do not conflate:**
> - **This file = servo-bus registers** (FeeTech SCS, e.g. `0x11` wheel mode). What the ESP32
>   writes directly to a servo ID over the SCS bus; what `cfgdump` dumps.
> - **xgolib `XGOorder`** (`BATTERY 0x01`, `VX 0x30`, `SET_ORIGIN 0x06` …, in
>   `Luwu-OS-2.0.1/libs/xgolib/xgolib_rider.py`) = the **Pi↔ESP32 command protocol**, a
>   totally separate address space. A number like `0x06` means different things in each layer.

**Confidence legend** — read this before acting on any value:
- ✅ **CONFIRMED** — from vendor source (`~/RIG-Omni/main/boards/hover/xgo.cc`) and/or
  verified on our hardware. Safe to rely on.
- ⚠️ **UNSURE** — meaning inferred/guessed, **not verified**. **Do NOT write these or make
  decisions from them** without first checking the vendor source or running a controlled
  hardware test. (This is the trap that cost us an afternoon on 2026-06-18 — see below.)

---

## ✅ CONFIRMED registers

| reg | name / meaning | size | source | notes |
|-----|----------------|------|--------|-------|
| `0x11` | **operating mode** — `0` = velocity-open-loop (wheel) mode | 1 | RIG `SetVelOpenloop` writes `0x11=0`; **hardware-verified** | **Wheels MUST be 0.** `0x11=1` → wheels don't track the velocity command → policy saturates, robot falls. Persistent EEPROM. Firmware re-asserts 0 at boot. See `wmode`. |
| `0x18` | **torque enable (broadcast limp)** — `0` = limp/free | 1 | our fw `wheelTorqueEnAll`; hardware-verified | `0x18=0` LIMPS the wheel (only effective when `0x1E` is not being driven). |
| `0x1E` | **drive command** (velocity/torque field) | 2 | RIG `WritePos_Sync_kp`; hardware-verified | Driven via SYNC_WRITE (`0x83`), 4 bytes/servo `[pos=0, pos=0, val_lo, val_hi]`. `0x1E=0` HOLDS zero = torque-locked (resists back-drive). |
| `0x24` | **present position** (read); present velocity adjacent | 2 | RIG `ReadWheelState`; hardware-verified | Only updates when the motor is **energized** — a freewheeling/torque-off wheel reads frozen. |
| `0x28` | **torque enable** (per-servo) | 1 | tools/servo, CLAUDE.md | Distinct path from `0x18`; relationship between the two not fully pinned (see UNSURE). |
| `0x1F` | **encoder offset** | 2 | tools/servo cal, CLAUDE.md | **Writes here do NOT persist** — persistence is via SPIFFS (`0x290000+0x080000`), see `project_xgo_rider_cal_storage`. |
| `0x37` | **lock** (EEPROM write-lock flag) | 1 | tools/servo `servo_status.py` | |
| `0x2A` | **goal position** | 2 | tools/servo `servo_status.py` | |

### Legs (ID 12 / 22) — non-standard, our-hardware-verified
| reg | name / meaning | source | notes |
|-----|----------------|--------|-------|
| `0x06` | **leg goal position** (2 bytes) — NON-STANDARD | our hardware (empirical) | `0x35` and `0x2A` did nothing; only `0x06` moved the leg. **Note `0x06` is ALSO `SET_ORIGIN` in xgolib — different layer.** |
| `0x18` | leg torque enable (write 1 on / 0 off; factory default 1) | our hardware | |
| `0x24` | leg present position (also mirrored at `0x1E`) | our hardware | leg-limit captures: ID12 ~863–974, ID22 ~30–135 (mirrored) |

### Vendor functions touching other regs (for reference, not our wheels/legs)
- `0x35` — `WriteByte_P_V` (pos+vel), used by RIG for the **head servo ID3**.
- `0x48` — `ReadMotorState` block read (RIG).

---

## ⚠️ UNSURE registers — meaning NOT verified, do not act on these

From the `cfgdump` config region (`0x00–0x37`), these bytes are present and (where noted)
**differ leg-vs-wheel or changed over time**, but their meaning is **inferred only**. The
`0x0B` "max voltage" guess on 2026-06-18 looked compelling and was **wrong** (the wheel's
`0x46` is its normal value) — treat everything here the same way until confirmed.

| reg | observed (wheel 11/21 / leg 12) | my GUESS (unverified) | status |
|-----|--------------------------------|------------------------|--------|
| `0x00–0x02` | `0C 00 1C` (all) | model / version | unconfirmed |
| `0x04` | `01` (all) | baud index (1 Mbps) | plausible, unconfirmed |
| `0x05` | `FA` (all) | return delay (250) | unconfirmed |
| `0x06–07` | wheel `00 00` / leg `96 03` | min-angle limit? (but `0x06`=leg goal too — conflict) | **unclear — flagged** |
| `0x08–09` | `FF 03` (=1023, all) | max-angle limit | plausible, unconfirmed |
| `0x0B` | wheel `46` / leg `55` | ~~max voltage~~ — **GUESS WAS WRONG** | unknown; wheel `46` is NORMAL |
| `0x0D` | wheel `8C` / leg `BE` | ? | unknown, differs by servo type |
| `0x10` | `02` (all) | ? | unconfirmed |
| `0x16–17` | wheel `00 00` / leg `5F 03` | ? | unknown, differs by servo type |
| `0x30` | wheel `20 00 43 01` / leg `00 00 00 1B` | ? | unknown |

**0x07 caution:** our firmware reads `0x07` at boot as a "return-delay" probe (`tRetDL`/`rd07`).
A `-1` there means the *read failed* (bus timing), NOT a register value — don't interpret `rd07=-1`
as config.

---

## How to read registers live (no passthrough fw, no USB needed)

Balance firmware exposes a config dump over the bridge:
```
mosquitto_pub -h <pi> -t rider/control/line -m '{"line":"cfgdump 11"}'   # 21=R wheel, 12/22=legs
# dump arrives on MQTT topic rider/debug/cfg as: "# cfg id=11 r0x00: 0C 00 ..." (regs 0x00-0x7F)
```
Diff two servos to spot a mis-set (how the `0x11` bug was found). Live telemetry regions
(`0x24-0x2B` position/speed/load, `0x3C-0x43` current/volt/temp) differ legitimately — ignore.

## Observed config maps (2026-06-18, after the 0x11 fix)
```
id 11 (L wheel)  r0x00: 0C 00 1C 0B 01 FA 00 00   r0x08: FF 03 00 46 3C 8C FF 03
                 r0x10: 02 00 24 00 00 00 00 00   r0x18: 00 00 01 01 20 20 00 00
id 21 (R wheel)  identical to id11 except 0x03=ID and live-telemetry bytes
id 12 (L leg)    r0x00: 0C 00 1C 0C 01 FA 96 03   r0x08: FF 03 00 55 3C BE FF 03
                 r0x10: 02 24 24 00 00 00 5F 03   r0x18: 00 00 01 01 20 20 5F 03
```

## Sources (authoritative → least)
1. `~/RIG-Omni/main/boards/hover/xgo.cc` + `xgo.h` — ESP32-S3 successor; drives the SAME servos
   directly. **The register Rosetta stone.** (`SetVelOpenloop`, `WritePos_Sync_kp`, `ReadWheelState`.)
2. `~/Downloads/Luwu-OS-2.0.1/libs/xgolib/` — Pi-side xgolib (**protocol layer**, not servo regs).
3. Our hardware-verified findings (memory: `project_xgo_rider_wheel_mode`, `_wheel_stop`,
   `_wheel_odometry`, `_cal_storage`).
4. FeeTech SCS datasheet — **not yet obtained**; would resolve the UNSURE rows. Get this to close the gap.
