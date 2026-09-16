# Balance-loop delay budget (2026-09-16)

Why this exists: the forward shimmy was root-caused in August to a ~34 ms command→wheel-velocity
lag. The stock firmware balances the same hardware without a shimmy, so we went looking for how
much of that lag is ours. Provenance tags: **MEASURED** (telemetry / drivecap on the robot),
**DERIVED** (from the code + a measured rate), **ASSUMED**.

## The one real bug — fixed in commit `44dd034`

`wheelRead()` exited early only once it had **20** bytes. The wheel controller's echo+reply never
reaches 20, so **both wheel reads per cycle sat out the full 2 ms timeout**. Nothing failed
(the parser copes with the partial count), so nothing warned.

| quantity                     | before  | after   | tag      |
|------------------------------|---------|---------|----------|
| wheel-read block per cycle   | 4.36 ms | 2.86 ms | MEASURED |
| work per cycle               | 5.49 ms | 4.03 ms | MEASURED |
| loop rate, idle              | 165 Hz  | 234 Hz  | MEASURED |
| loop rate, driving           | ~163 Hz | ~199 Hz | MEASURED |
| read failures                | 0       | 0       | MEASURED |

Fix: exit as soon as a complete reply frame (`FF FF id LEN … chk`, `LEN != 0x04` echo) is in the
buffer — the same completeness test the parser uses. The remaining ~1.4 ms per read is the wheel
controller's own reply latency (MEASURED as the residual; not yet decomposed).

## Where the command→velocity lag actually lives

drivecap after the fix (policy build, floor, one reverse + two forward passes):

| quantity                           | Aug (before fix) | now     | tag      |
|------------------------------------|------------------|---------|----------|
| cmd→wheel-velocity lag (cross-corr) | 34 ms            | 30–35 ms| MEASURED |
| shimmy frequency, forward           | 8.3 Hz           | 8.6 Hz  | MEASURED |
| loop period while driving           | 6.1 ms           | 5.0 ms  | MEASURED |

Cutting ~1.5 ms of read time and ~1 ms of period moved the lag by only a few ms. **The bulk of the
~30 ms is inside the servo** (its velocity loop + its reported-velocity estimate), not in our loop.
The bench figure (3 ms onset + 13 ms tau, unloaded) understates it under load.

## Filters we add that the sim never modeled (DERIVED at 234 Hz)

| signal                          | coefficient | time constant | sim |
|---------------------------------|-------------|---------------|-----|
| velocity fed to the policy      | 0.7         | ~12 ms        | none|
| pitch rate (`dqf`)              | 0.6         | ~8 ms         | none|
| wheel velocity for the LQR build| 0.85        | ~26 ms        | none|
| policy output                   | 0 (off)     | —             | —   |

(These were 17 / 12 / 38 ms at the old 165 Hz.)

## Why forward shimmies and reverse doesn't — the policy, not the delay

The lag is the same in both directions. `sim/policy_symmetry.py` on the flashed `ppo_mc_l4a`:

| metric                          | ppo_mc_l4a (flashed) | ppo_delay34_l4 (fell at full stick) |
|---------------------------------|----------------------|-------------------------------------|
| action at the operating point   | −0.90                | −0.06                               |
| fraction of pitch sweep clipped | 68 %                 | 0 %                                 |
| local gain, one side / other    | 0 / +15.5            | +4.8 / +4.5                         |

A gain of zero on one side and 15.5 on the other is a one-directional limit-cycle generator:
on the flat side (reverse) it can't oscillate; on the steep side (forward) 15.5 of gain through
30 ms of dead time rings at ~8.6 Hz. Marc's observation matches: reverse "smooth and perfect",
forward shimmies hardest on the initial lean and decays as the drive settles.

## Factory comparison (from the Ghidra notes in `main.cpp`)

The factory R-1.1.3 loop runs ~167 Hz, low-passes pitch and rate at 0.95 (≈120 ms), slew-limits
the smoothed pitch to 5°/cycle, and drives the wheels by accumulating a position goal into the
servo's own inner loop. It does **not** have less delay than we do; it is a deliberately
low-bandwidth controller that never asks for the speed that exposes the servo's lag. Our policy
does the opposite: one-sided gain of 15 into the same lag.

## Sim calibration (same day, `sim/`)

Making the sim carry the filters above exposed two more problems in the sim itself:

1. **The pure delay was double-counted.** `rider_params.latency_s = 0.021` was "robot 34 ms minus
   the 13 ms tau", but the sim already carries ~20 ms of its own (zero-order hold + one-step
   observation delay + the tau). Measuring the sim's command→velocity lag by the *same*
   cross-correlation as the robot's drivecap gave **64 ms** vs the robot's 30–35. At that lag the
   flashed policy thrashes at 4 Hz with 41 % saturation in *both* directions, which the robot never
   does.
2. **The delay buffer was off by one step** (`ActuatorModel`: a deque of maxlen n delays n−1). Every
   run since June got one step less pure delay than its `latency_s` said.

Both fixed. `latency_s` is now **one control step (0.004 s, DERIVED)**, chosen so the sim's
cross-corr lag lands in the robot's 30–35 ms. With the firmware sensing path on:

| policy            | robot (floor)                        | sim after calibration                |
|-------------------|--------------------------------------|--------------------------------------|
| ppo_mc_l4a fwd    | 8.6 Hz ring, rate std 24–37 °/s, up  | 8.8 Hz ring, rate rms 100 °/s, up    |
| ppo_mc_l4a rev    | smooth, rate std 8 °/s               | quiet, no saturation                 |
| ppo_delay34_l4 fwd| falls at full stick                  | falls at 1.7 s                       |
| ppo_drive34_l4 fwd| ~2 Hz build-recover hunt, falls      | perfectly smooth (**not reproduced**)|

Character, frequency and asymmetry match; the sim's ring amplitude is ~3× the robot's, and the
drive-aware policy's 2 Hz hunt is an open gap (accelerometer coupling adds only a weak 3 Hz
component). Sweeps showed the pure delay is the dominant knob; stick-slip, servo tau and deadband
barely move the ring.

## Floor tests of the first faithful-sim seeds (same afternoon) — two more plant lessons

Ten seeds of the drive-aware recipe were trained in the faithful sim (`sim/seed_sweep.py`, ~3 min
each) and screened offline; five passed. Two were flashed:

| policy   | offline screen (at the time) | floor                                                        |
|----------|------------------------------|--------------------------------------------------------------|
| ppo_fw_s8| best conditioned (a_op +0.03, gain ratio 1.2) | **no shimmy**; leans too far and **falls at full-stick forward**; reverse good |
| ppo_fw_s5| smoothest drive (0.9 °/s)    | balances, then a **~2 Hz see-saw** (pitch ±2°, cmd ±50) grows and it falls    |

Both failures were then reproduced in sim by fixing the plant, not the policy:

1. **The loaded wheel is weaker than the bench.** Steady wheel velocity on the floor (7 episodes,
   two policies): **v ≈ 0.9·(cmd − 35)** raw; the unloaded June bench gave 1.0·(cmd − 21). With
   deadband 0.15 / 27 rad/s as the sim's nominal, s8 falls forward at 2.4 s (floor: same) and
   mc_l4a survives (floor: same). The two seeds nearest zero at the operating point (s7, s8) are
   the ones that fall; survivors carry a small positive offset that pre-loads the dead zone.
2. **The 2 Hz hold hunt is loaded stick-slip + the firmware hold loop.** On the hysteretic
   stick-slip actuator (June bench: static ~0.25, kinetic ~0.12) with the hold loop running, s5
   see-saws at 2.0 Hz in sim, drive34 at 3 Hz, s8 only mildly — the floor's ordering. The old
   clean-deadband sim rated s5 the smoothest of all.

The sweep screen now runs every seed on **both** loaded plants (four full-stick drives) plus a
stick-slip **hold** test that flags the hunt. It fails s8 (falls) and s5 (hold-hunt) — two
floor-validated negatives — and passes s2 and s9. The sim's ordering is sensitive to the actuator
model, so it is a screen, not an oracle: a PASS earns a floor test, nothing more.

## What this says about the next policy

- The delay is the plant's; we cannot train it away or tune it away in our loop.
- The shimmy is the policy's conditioning. `ppo_delay34_l4` is perfectly conditioned but too
  gentle to carry a full-stick forward lean; `ppo_mc_l4a` has the authority and the pathology.
- The sim should carry the filters above and the ~30 ms loaded lag before the next training run,
  so that what trains is what flies.
