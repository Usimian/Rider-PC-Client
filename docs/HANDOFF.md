# Handoff: the RL balance policy (live investigation)

Read at session start. Facts live in the repo (`docs/loop_delay_budget.md`, commits
2026-09-16); this is only the running story, the open list, and what not to trust.

## Where it stands (2026-10-18)

- **Flashed:** policy build, `ppo_fw_s9` (header committed) + the integrator-freeze / `polff`
  firmware (commit after this file). The `polff` stiction knob defaults to 0 = off.
- **Floor result of s9 (2026-09-16, before that last flash):** balances, no shimmy, drives both
  ways, but a **~1 Hz hold see-saw** (pitch 2.5–6.5°, wheel pulses forward each cycle) =
  loaded stick-slip limit cycle.
- **Not yet observed on the floor:** the integrator-freeze build, and `polff` at any value
  (plan was 20 then 35 raw, set live over `rider/control/line` while balancing).

## Open list, in order

1. Floor test the flashed build: hold see-saw unchanged with `polff 0`? forward vs reverse
   drive authority now symmetric (the integrator change)? Then `polff 20`, `polff 35`.
2. **DIRECTION, Marc 2026-10-18: keep the RL policy AND have an algorithmic controller "as close
   to the factory code, which was pretty good", switchable at runtime.** So: one binary carrying
   both (today separate compile-time builds, deliberately, so the wrong one can't be armed by
   accident -> the switch must be disabled-only), and the LQR path brought toward the factory
   loop from the Ghidra notes in `main.cpp`: 167 Hz pacing (`loopus 6000` exists), the 0.95
   input LP + 5°/cycle slew (noted "we lacked this" -- NOT implemented), position-goal
   accumulate (`posmode 1` exists, floor status unknown), stiction boost (`ff`/`ffband`), position
   Ki (`lqrxi`). Proposal pending his call; see the session of 2026-10-18.
3. A retrain of the policy in the corrected sim (all ten seeds predate the drive-model fix)
   stays on the list for the policy side.
4. Vendor code: `vendor/` is gitignored, sources in README "External assets" (done 2026-10-18).

## Do not trust

- Any sim verdict as more than a screen: it reproduced s8's fall and s5's hunt only after
  plant fixes, and still under-states hunt amplitude ~3x and misses drive34's 2 Hz hunt.
- The DualShock button indices in `pi/rider_controller.py` for any other gamepad (Xbox pad
  stays with the Hover — one controller per robot, Marc 2026-10-18).
