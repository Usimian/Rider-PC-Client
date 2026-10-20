# Handoff: the RL balance policy (live investigation)

Read at session start. Facts live in the repo (commit messages of 2026-10-19/20 carry the numbers,
`sim/rider_params.py` the provenance); this is only the running story, the open list, and what not to trust.

## Where it stands (2026-10-20 afternoon)

- **Flashed:** `ppo_ld8_s6` (committed `rider_policy.h`) + the drive watchdog. Drives 0.30 m/s both ways, turns,
  reverses; full-forward-into-full-reverse is a smooth ~2 s lean-back (LQR does it in 0.25 s with a 20° lurch).
- **The one open defect: a 4 Hz shimmy while SPINNING in place, policy only** (floor 16-22 °/s pitch-rate RMS,
  ~95 % at 3-6 Hz; LQR 3-4). Goal stays: fix it in the POLICY.
- **2026-10-20, what did not fix it (all floor-tested):**
  - `ppo_tw2_s6` (trained in the two-wheel sim): spin 8.5 °/s, but it drives with a 5° lean and rocked itself
    over at full speed (several falls). Sim kept it calm -> the screen now has a LEAN check (`LEAN_MAX` 2.5°).
  - `ppo_tt25_s4` (ld8_s6 fine-tuned with an anchor + a new |turn command| input): spin 20.7/20.9 °/s, unchanged.
    The replay shows the robot computed it exactly and the turn input moved its spin output by only ~3 raw.
- **Conclusion so far:** the sim calms the spin when the policy changes, the floor does not -> the floor's
  spin shimmy has a cause the sim does not contain. Checked and matching floor<->sim: policy computation
  (replay), body pitch dynamics (sweep), wheel response standing and at full speed (sweeps; the sim now has the
  measured +15 ms fast-wheel delay), drive lean/speed/output, turn drift (<0.7°, settles -- smaller than the
  sim's 2° test). No training until the missing mechanism is found.

## Open list, in order

1. **Find the spin-shimmy mechanism the sim lacks.** Ideas not yet tried: a frequency sweep WHILE SPINNING
   (`chirp` with a held turnrate -- does the wheel/body response change when the wheels counter-rotate?);
   compare the policy's captured spin inputs (`drivecap` now has accel pitch, yaw rate, turn command) with the
   two-wheel sim's for the same spin; tyre/caster scrub dynamics in the spin.
2. ToF obstacle/edge stop runs but is OFF by default (web switch); check its zone classes while driving before
   leaving it on.
3. Under-voltage on boot (5 V rail ~4.94 V idle); `/home/pi/riderlog.txt` is never rotated (195 MB on 10-20).
4. The controller low-battery cutoff (<= 15 %, unplugged) is untested -- it will show the first time a DS4 runs low.

## Do not trust

- Sim spin numbers as a floor prediction: ld8_s6 6 vs floor 18; tt25_s4 3.6 vs floor 21.
- Closed-loop fits of plant dynamics from drive recordings without an injected signal (they gave nonsense
  pendulum constants) -- use the `chirp` sweeps.
- 10 Hz telemetry for anything at 1-5 Hz (it aliases the rocking) -- use `drivecap` / `chirp` captures.
- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off, never matched.
