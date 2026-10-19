# Handoff: the RL balance policy (live investigation)

Read at session start. Facts live in the repo (commit messages of 2026-10-19 carry the numbers,
`sim/rider_params.py` the provenance); this is only the running story, the open list, and what not to trust.

## Where it stands (2026-10-19 evening)

- **Flashed:** `ppo_ld8_s6` + the LQR-like policy stick drive (`polleash` 0.25 m, bias clamp 8°) + the
  frequency-sweep firmware (`chirp`). Floor: drives 0.30 m/s both ways, holds speed through turns, reverses
  cleanly. Marc: "that works great". LQR mode unchanged and works well.
- **The one open defect: a 4 Hz shimmy while SPINNING in place, policy only.** Measured (loop-rate captures,
  `data/captures/2026-10-19_*`, gitignored): pitch-rate RMS 16-22 °/s, ~95 % at 3-6 Hz, balance output ±20-25;
  LQR spins at 2.6-3.0 °/s with output ±6-14. Slow-mo video (`data/video/spin_policy_2026-10-19.mov`):
  the whole body rocks in pitch (front/back white arms see-saw, wheels shuffle) -- not visible leg flex.
  Marc's goal: fix it in the POLICY (no LQR hand-over).
- **Ruled out (sim or floor):** the turn output (steady ±2-3), gyro / fused-pitch offsets, "weaker/later
  spinning wheel" (a misreading -- the sweeps show the same wheel spinning or straight), output smoothing
  (`polulp 0.5` made it slightly worse: 17.1 -> 18.6 °/s, 4.0 -> 3.4 Hz -- delay feeds it), leg give or free
  play in the 2-D sim (they shake the sim LQR as much or more, unlike the floor).
- **Sim now matches the floor** on: wheel response holding (sweep) and moving (lag 36-56 ms), s9's falls,
  ld8_s6 / LQR drives, LQR calm (servo-style speed). It still does NOT produce the spin shimmy.

## Open list, in order

1. **Marc: tighten the legs/linkages/wheel hubs**, then spin in policy mode with the capture running
   (`tools/capture/drivecap_capture.py out.csv --spin`) -- before/after numbers. If it drops, looseness was it.
2. If not: the spin shimmy is a 3-D effect (sideways tyre scrub; a stalled wheel turning the spin into a
   pivot). Bring `sim/rider_env_3d.py` (two wheels) up to today's wheel/sensing/drive model, require: policy
   shimmies when spinning, LQR calm, all the floor facts above; only then retrain with spins.
3. Under-voltage on boot (5 V rail ~4.94 V idle) -- see HARDWARE.md Power; `/home/pi/riderlog.txt` is never
   rotated (archive + truncate when large, as on 2026-10-19).
4. ToF obstacle/edge stop runs but is OFF by default (web page switch). It false-flagged a flat floor before;
   the page's ToF card shows zone classes -- check them while driving before leaving it on.

## Do not trust

- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off by default, none
  matched the floor.
- Closed-loop estimates of the wheel's lag without an injected signal (they read 35-40 ms holding; the
  sweep shows 7-18) -- use `sim/wheel_sweep.py` + the firmware `chirp`.
- The DualShock button indices in `pi/rider_controller.py` for any other gamepad (Xbox pad stays with the
  Hover -- one controller per robot, Marc 2026-10-18).
