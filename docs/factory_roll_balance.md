# Factory roll-balance — how the stock XGO Rider levels its body (decompiled)

Reverse-engineered from the stock app image `xgorider_app_v1.1.6.bin` (matches the `R-1.1.x`
line) on 2026-06-24, to fix our own leg-leveling loop that oscillated. This is the reference
our firmware's leveling (`level`/`levkp`/`levslew`/`levmax`/`levset`/`levsign`) is modeled on.

## TL;DR

The factory keeps the body level with a **gentle, slew-limited, closed-loop controller on legs**,
not a high-gain proportional loop. Our first attempt rang because it chased the error at high gain
directly in encoder counts. Theirs can't ring because the leg command **slews** toward a small,
clamped target and the loop is closed in **body-angle space** (then converted to leg position by IK).

## Where it lives

- `rider_balance_roll(mode)` in `xgolib_rider.py` just sets register **`0x61` (IMU)** = 0/1 and sends it.
  The Pi only toggles a flag — **the whole control loop runs in the ESP32 firmware.**
- The successor source (`~/RIG-Omni/main/boards/hover/xgo.cc`) does NOT do leg roll-leveling at all
  (roll is fall-detect only; body kept upright purely by the wheels). The working leg-roll algorithm
  exists **only** in the stock Rider binary — hence the decompile.

## The algorithm (function `FUN_400d4514`, runs every cycle in `taskControl`)

When the roll-balance flag is set:

1. **Closed-loop PID on body roll**, setpoint **0°** (level), feedback = measured IMU roll.
   Reuses the same generic PID routine as pitch (`FUN_400d3850`).
2. That PID has **anti-windup**: the integrator resets to 0 whenever the error crosses zero
   (sign change), the integral is clamped, and the P/I/D terms are each individually clamped.
3. PID output is **accumulated** by the caller (`accum += output` each cycle) and the whole
   accumulator is **clamped to ±20°**.
4. The ±20° body-roll command is added to a **45° neutral leg angle** and run through
   **inverse kinematics** (π + leg-geometry constants `4.5 / 3.125 / 3.086 / 1.75` + trig) into
   **left/right leg target positions**.
5. Applied **differentially** to the two leg servos (`PTR_DAT_400d00b0` left / `…00b4` right,
   position command field `+2`). Wheels are separate (`0x0158/0x015c`, torque/vel field `+6`;
   mirror-mounted, so opposite-sign = fore/aft balance, same-sign = yaw).

## Exact roll-PID gains (static `.data`, verified not overridden at runtime)

Controllers are a 0x48-byte struct array in DRAM; roll struct @ `0x3ffbdfd4`. Field map from the
PID routine: `p[0]=setpoint p[1]=feedback p[2]=Kp p[3]=Ki p[4]=Kd`, output summed into `p[8]`.

| gain | value |
|------|-------|
| Kp | **0** |
| Ki | **0** |
| Kd | **0.02** |
| setpoint | 0° |
| per-cycle D clamp | ±0.8 |
| total output clamp | ±20° |

Verified: the `0.02` constant appears only in DRAM data, never as a code literal — no runtime init
writes the gains. (Sibling controllers, for context: yaw Kd=3, pitch Kd=70, all also Kp=0 — this
firmware is D-dominant + accumulator throughout.)

**Why Kd-only works:** with Kp=Ki=0, `pid_out = Kd·(err − prevErr) = −Kd·Δroll`; accumulating that
telescopes back to `≈ −Kd·roll`. So functionally it's a **gentle proportional** roll→leg response
(effective gain 0.02), but implemented so the per-cycle D-clamp (±0.8) acts as a **slew-rate limit**
and the accumulator is hard-capped (±20°). Gentle gain + slew limit + angle-space IK = stable.

## How our firmware maps to it (`firmware/esp32_rider_fw/src/main.cpp`)

We don't have the factory's full IK, so we work directly in leg encoder counts, but keep the three
properties that matter:

- `target = levsign · levkp · (roll − levset)`, clamped to ±`levmax`  (gentle proportional)
- the leg command **slews** toward `target`, max `levslew` counts/cycle  (the anti-ring slew limit)
- the same-sign differential is applied to both legs with **spill-to-the-other-leg** when one hits
  its servo limit (our edge-case requirement).

Defaults: `levkp 3`, `levslew 0.5`, `levmax 40`, `levsign -1`. All live-tunable over MQTT.

## Reproducing the decompile

- Build an Xtensa ELF from the app image with `bin2elf.py` — **key gotcha:** flash-mmap'd segments
  (DROM/IROM) load **8 bytes below** esptool's reported addr (DROM `0x3f400018`, IROM `0x400d0018`),
  or string/literal xrefs won't resolve. IROM addr→file: `file = cpu − 0x400a0000`.
- Ghidra 12.1.2 has a built-in Xtensa LE module; scripts must be **Java** (PyGhidra not enabled).
- Anchor via FreeRTOS task-name strings (`taskControl` etc.) → `xTaskCreate` site → control task
  `FUN_400d582c`. Then trace the measured-IMU struct (`0x400d01bc`, roll at `+0x60`) to find
  `FUN_400d4514`. Gains read straight out of the DRAM segment image.
