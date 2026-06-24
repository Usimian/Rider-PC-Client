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
| `0x24` | **present position** (read); present velocity adjacent | 2 | RIG `ReadWheelState`; hardware-verified | **Reads correctly even FREEWHEELING (torque off)** — verified 2026-06-24 by hand-spinning ID21: tracked the full 0–1023 range + multi-rev odometry. (Earlier "only updates when energized" was wrong.) **Wheels: must use the RAW read protocol — see below.** |
| `0x28` | **torque enable** (per-servo) | 1 | tools/servo, CLAUDE.md | Distinct path from `0x18`; relationship between the two not fully pinned (see UNSURE). |
| `0x1F` | **encoder offset** | 2 | tools/servo cal, CLAUDE.md | **Writes here do NOT persist** — persistence is via SPIFFS (`0x290000+0x080000`), see `project_xgo_rider_cal_storage`. |
| `0x37` | **lock** (EEPROM write-lock flag) | 1 | tools/servo `servo_status.py` | |
| `0x2A` | **goal position** | 2 | tools/servo `servo_status.py` | |

### ⚠️ Reading wheel encoders (IDs 11/21) — RAW protocol only, NOT scservo

**`scservo_sdk` register reads (`read1ByteTxRx`/`read2ByteTxRx`/`readTxRx`) DO NOT WORK on the
wheels — they return a fixed `516` (`0x0204`) to *every* address.** The wheel FOC controller
(K-power 4012) replies with a **non-standard 11-byte frame** that scservo can't parse; the `516`
is just the request header (`0x04 0x02`) echoing back. (Cost ~3 failed attempts 2026-06-24 before
realizing it was the harness, not the wheel.)

**Correct method — raw serial** (see `tools/wheel/wheel_encoder_monitor.py`):
```
Request : FF FF ID 04 02 24 06 CK      (read 6 from reg 0x24)
Response: FF FF ID 0B ERR ...payload... CK   (LEN=0x0B = 11 bytes)
  pos = resp[LEN-3] | resp[LEN-2]<<8     (10-bit, 0..1023, wraps)
  vel = resp[LEN-1] | resp[LEN]<<8       (signed; sign = direction)
```
Odometry accumulates pos deltas with a ±800 wrap threshold over span 1024. **Verified 2026-06-24**:
hand-spun ID21 ±3 revs each way, full 0–1023 coverage, clean multi-wrap accumulation. The physical
encoder is 14-bit but the **bus reports 10-bit** — odometry is 10-bit/1024-wrap. Legs use scservo
fine; only the wheels need the raw path.

**CLOSED QUESTION — no 14-bit position is reachable over the bus (don't re-investigate).** The
motor encoder is 14-bit but only 10-bit position is exposed. Confirmed THREE ways 2026-06-24:
(1) live probe — reading 12 bytes from `0x24` returns extra fields, but the only >1023 candidate
(`f4`) did NOT track rotation when hand-spun (it just dithers its high byte); (2) vendor RIG-Omni +
our balance fw both read only `0x24`/6 bytes; (3) **factory firmware reverse-engineered** — the
R-1.1.3 binary (byte-verified in both backups, file offset `0x4498a`) issues exactly ONE SCS read:
`0x24`, length 6 — no read >6 bytes, no use of `0x48`, no 14-bit immediate anywhere in the code.
So 10-bit position is the hard ceiling; the firmware's g-h velocity observer is the correct fix for
the quantization noise, not a workaround. (Open lesser thread: is the reported `vel` cleaner than
differenced position? — run `tools/wheel/wheel_resolution_probe.py`.)

### Legs (ID 12 / 22) — **K-power RC08P** servo map, FULLY VERIFIED 2026-06-24

The leg servos are **NBD-branded, customized from the K-power RC08P** (per Luwu source). They
speak the FeeTech SCS *protocol* (header `0xFF 0xFF`, `PacketHandler(0)`) but use **K-power's
register map shifted +6 in the RAM block** vs K-power's generic table. Resolution is **0–1023**
(10-bit), 200° travel. The +6 shift is anchored by present-position landing on `0x24`. Map
confirmed by driving a disconnected ID22 into each register (K-power protocol guide:
`kpower.com/insight_gearbox/7335.html`).

| reg | name / meaning | bytes | notes (all hardware-verified on disconnected ID22) |
|-----|----------------|-------|------|
| `0x06` | **MIN angle limit** | 2 | EEPROM hard clamp. Raising it **drags the shaft UP** to the new min (this is the "snap"); lowering it doesn't command motion. Set to leg-min. |
| `0x08` | **MAX angle limit** | 2 | EEPROM hard clamp. Lowering it **drives the shaft DOWN** to the new max. Set to leg-max. min≤max enforced (min>max rejected). |
| `0x18` | **torque enable** (1=on, 0=limp) | 1 | On enable, servo drives to the goal in `0x1E`. **SAFE ENERGIZE: write `0x1E`=present-pos (`0x24`) FIRST, then `0x18=1`** → holds, no snap. |
| `0x1E` | **GOAL position** | 2 | The real position command — smooth, speed-controlled. Writing it (torque on) glides the shaft to target. Wheels sync-write here too (vendor `WritePos_Sync_kp`). |
| `0x20` | **MOVING speed** | 2 | Speed for the `0x1E` move (≈0–1023). Set before/with the goal for smooth motion. |
| `0x24` | **present position** | 2 | Live feedback (read). Hand-movement reads here ignore limits; under torque the shaft obeys `0x06`/`0x08`. |

**Leg travel ranges (hand-swept both stops 2026-06-24, NEW right + original left):**
**ID12 (L) 863–977, ID22 (R) 467–578** (mirrored; short = ID12≈975 / ID22≈467). NOTE: the
right horn was rotated while disconnected — **re-measure ID22's range after the leg is
reattached** before trusting these for the right leg.

**Dead ends (do NOT retry — verified ignored by the leg servos):** `0x2A` (reads garbage ~8530),
`0x35` pos+vel (that's RIG's **head** servo ID3 command — legs ignore it).

**Persistence:** angle-limit writes (`0x06`/`0x08`) **DO persist across power-cycle** — verified
2026-06-24 (both legs' limits survived a reboot). Unlike the `0x1F` encoder offset, which does NOT
persist (SPIFFS-only). Legs also boot **torque-OFF** (`0x18`=0) — no power-up snap.

### Vendor functions touching other regs (for reference)
- `0x35` — `WriteByte_P_V` (pos+vel), used by RIG for the **head servo ID3** (not the legs).
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
