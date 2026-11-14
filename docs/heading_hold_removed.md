# Heading hold (removed 2026-11-14) -- notes in case it comes back

Removed from the firmware and the two-wheel RL sim on Marc's call ("the heading hold is kind of optional for
us"). The last firmware with it: commit `09ebf72`, `firmware/esp32_rider_fw/src/main.cpp` -- the globals block
"encoder heading-lock" (~l.191-198), `gHdgTarget` (~l.958), the turn controller in `balanceTask` (~l.1210-1233),
`poszero` (~l.1419), the parser commands (~l.2127-2130) and telemetry `htg=`. `git show 09ebf72:<path>` brings
it back.

## What it was

The wheel-differential command `gTurn` (added to both wheels: L = u + tn, R = -u + tn, clamped to +/-`gTurnMax`
250, low-passed by `gTurnLP` 0.5) came from one of two laws, every control cycle while armed:
- **Turning** (`|turnrate| > 0.001`) **or the hold off**: yaw-RATE control on the gyro -- `gYawFF * cmd +
  gYawKp * (cmd - gYawFb * yaw_rate)` (FF 75, Kp 40, sign -1). This is what remains, now always.
- **Not turning, hold on (the default)**: a PD on the two wheels' odometers --
  `gHdgSign * (gHdgKp * (gHdgTarget - (wheel1_x - wheel2_x)) - gHdgKd * (wheel1_vel - wheel2_vel))`,
  Kp 1.0 per count, Kd 2.0 per speed unit, sign +1. Pure encoder, no gyro yaw-bias dependence.
  `gHdgTarget` followed the robot while turning or disarmed (so a released stick latched the new heading) and
  was zeroed on arming and on `poszero`. Commands `hdghold`, `hdgkp`, `hdgkd`, `hdgsign`. Verified 2026-06-15
  (holds heading; a bump that rotates the robot is corrected).

## Why it went

The first get-up after every ESP32 boot failed (11-14: 11:21, 11:46, 12:57, 13:15, 13:20, 13:28 -- each the
first stand-up after a power cycle or a flash; the next ones worked). In each failed kick the turn command ran
to its +/-250 limit (`gturn` -140..-250; at 12:57 already +125 before the kick, from a phantom left-wheel speed
of 129 on its first reading after boot), so one wheel got far less than the other: left at ~45-60 % of the
right's speed. Good kicks: `gturn` within +/-25 and both wheels even. Not the wheels: on the stand both stepped
normally on their first drive after power-up (single-wheel steps and the kick's own two-wheel command path,
`wt 200`), torque read back on, all 128 registers the same before and after a drive. With `hdghold 0` the first
get-up after boot stood (13:55: wheels -190.7 / -184.4, `gturn` within +/-4).

Not found: why the hold misbehaved only on the first drive after boot. Suspects: the wheels' first readings
after the ESP32 starts (the phantom 129 at 12:57; `vrejL` counts), or the hold's D term on the wheel-speed
difference amplifying a small first-kick mismatch. A high-rate drive capture of a first get-up with the hold
on would show which comes first (`drivecap` records cmdL/cmdR and both speeds from each arm; dump before
re-arming).

## If it comes back

- Keep it out of the get-up and sit (and any open-loop wheel phase): no heading to hold there.
- Test the first stand-up after a power cycle AND after a flash (an ESP32-only reset reproduced it, 13:28).
- Look at the D term first (`gHdgKd` on a raw speed difference) and at the first wheel readings after boot.
- The sim mirror was `sim/rider_env_twowheel.py` (HDG_KP / HDG_KD, `hdg_target`).
