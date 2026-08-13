# Factory roll-balance — how the stock XGO Rider levels its body (decompiled)

Reverse-engineered from the stock app image `xgorider_app_v1.1.6.bin` (matches the `R-1.1.x`
line) on 2026-06-24, to fix our own leg-leveling loop that oscillated. This is the reference
our firmware's leveling (`level`/`levki`/`levslew`/`levmax`/`levset`/`levsign`) is modeled on.

That app image is preserved in-repo as **`firmware/prebuilt/rider_stock_R-1.1.6_app.bin`**
(folded in 2026-07-23 from a scratch Downloads folder that has since been deleted) — it is the
only copy, so the decompile below stays reproducible.

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

We don't have the factory's full IK, so we work directly in leg encoder counts, but we now keep the
factory's **accumulator** structure (see `main.cpp` ~line 1151, the `levRun` branch):

- **integration step** `inc = levsign · levki · (roll − levset)`, in counts/cycle
- that step is **clamped to ±`levslew`** — the per-cycle rate limit, which is what keeps the leg
  loop below the body's ~2 Hz roll resonance (this is the anti-ring property)
- the step accumulates into the leg differential, **hard-clamped to ±`levmax`**
- the differential is split at **half-count resolution** (`t = round(2·acc)`, `cR = t/2`,
  `cL = t − cR`) so an odd total moves a single leg — roll can correct in 1-count steps instead of 2
- **spill-to-the-other-leg** when one leg hits its servo limit (our edge-case requirement)

Defaults: `levki 0.14`, `levslew 0.30`, `levmax 50`, `levsign -1`, `levset 0`. All live-tunable
over MQTT.

> **This replaced an earlier proportional version** (`target = levsign · levkp · (roll − levset)`,
> gain `levkp`). Proportional **stalls at a non-zero roll offset** — the leg differential needed to
> hold the body level is exactly where the error term stops growing. Integrating drives roll to
> zero instead, and as roll → 0 the step shrinks so it still settles. **`levkp` no longer exists;**
> the gain is `levki` and it has different units (counts accumulated per degree *per cycle*, not
> counts per degree), so old `levkp` values are not meaningful here.

## Reproducing the decompile

- Build an Xtensa ELF from the app image (`firmware/prebuilt/rider_stock_R-1.1.6_app.bin`) with a
  `bin2elf.py`-style wrapper. **That script was not kept** — it was a throwaway; rewrite it from the
  segment addresses below rather than hunting for it. **Key gotcha:** flash-mmap'd segments
  (DROM/IROM) load **8 bytes below** esptool's reported addr (DROM `0x3f400018`, IROM `0x400d0018`),
  or string/literal xrefs won't resolve. IROM addr→file: `file = cpu − 0x400a0000`.
- Ghidra 12.1.2 has a built-in Xtensa LE module; scripts must be **Java** (PyGhidra not enabled).
- Anchor via FreeRTOS task-name strings (`taskControl` etc.) → `xTaskCreate` site → control task
  `FUN_400d582c`. Then trace the measured-IMU struct (`0x400d01bc`, roll at `+0x60`) to find
  `FUN_400d4514`. Gains read straight out of the DRAM segment image.
