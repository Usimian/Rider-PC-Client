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

## Done 2026-10-18: single binary, runtime controller switch (floor-verified)

`ctrl 0` policy / `ctrl 1` LQR, refused while `en 1` (verified: sent mid-balance, stayed LQR,
no disturbance); boots as the policy. Floor, same session, at hold: policy pitch std 1.8–2.0°,
1.2 Hz see-saw (= s9 in September); **LQR pitch std 0.82°, rate std 5.7 °/s, pos std 28 mm,
0.5 Hz** — calmer than the policy but looser than its June record (0.3°, 14 mm). Likely cause
(UNTESTED): the September read fix raised the loop 163 → 234 Hz and the LQR's per-cycle
filters/gains were tuned at 163 — try `loopus 6000` first. Also: LQR drive lags its target by
15–20 cm (pos err −222 mm fwd / +148 mm rev).

## 2026-10-18 evening: factory input filter sweep, turn fix, leg lock-up fix

- **Factory input filter on the LQR** (`lqrinlp`, `lqrinslew`; default 0 = off): floor sweep at the
  234 Hz loop — 0.3 feels unchanged, **0.5 falls at full-stick speed**. It cannot go on top of the
  current LQR drive. Reason found: the LQR drives by letting its position target run ~20 cm ahead
  (pos err −222 mm at full stick), so full speed = a large position-term lean; filter lag on top
  overshoots it. **Next: rework LQR driving** — speed-proportional lean feedforward (`driveff`,
  exists, 0) and a target that doesn't run far ahead — then redo the sweep (+ `loopus 6000`).
- **Turn fix (verified):** the turn-start re-anchor of the target now only happens from a HOLD.
  Mid-drive it wiped the target lead → LQR stopped, spun, re-accelerated. After: forward command
  and speed carry through curves.
- **Leg servos (RC08P, IDs 12/22) going deaf (verified fixed, one heavy run):** after heavy height
  use a leg stopped answering the bus while holding its last goal, so STOP couldn't limp it; only
  a power cycle freed it. Cause: per-cycle ADDRESSED goal writes drew replies that collided on the
  shared bus (wheel rfail up to 4 % during leg moves). Fix: all leg goal updates go through
  `legGoalWriter()` = one broadcast SYNC_WRITE for both legs (no reply), ≤ 25 Hz. After: 2.5 min of
  end-to-end height while balancing, both legs answer, STOP limps both, rfail 0 %. **Watch for a
  recurrence** — a single run is not proof. Free stuck legs with a power cycle.

## Pi boot / SD card baseline (2026-10-19, MEASURED)

- Boot 2026-10-19 was slow: systemd total 19.0 s (kernel 4.3 + userspace 14.7) vs 7.7-8.1 s on the
  five boots before; slowdown spread across everything (storage-like), no SD errors logged.
- SD card read test right after: **21.9 MB/s sequential (4 MB direct), 2.3 MB/s = ~560 IOPS at 4 KB**.
  No earlier baseline, so "degrading" is unproven. Re-measure on the next slow boot.
- **Under-voltage on every boot** (5-34 events/boot, first ~6 s in) — long-standing, not new; 5 V
  rail reads 4.94 V at idle. Worth fixing (SD writes during brown-outs are a corruption risk).
- `/home/pi/riderlog.txt` (always-on recorder) is 919 MB and never rotated.
- **Fixed same morning, no card swap:** `fstrim` (18.5 GiB trimmed — never ran) + log archived to
  `data/riderlog_2026-10-19.txt.gz` (gitignored, sha-verified) and truncated. After: 40.1 MB/s,
  ~1830 reads/s, boot 8.1 s (software reboot) and **8.2 s from a cold power-on**. Upkeep notes are in
  `PI_HOST_SETUP.md` → "Card upkeep"; that guide was corrected from an inventory of the card.

## Robot state NOT in the repo (2026-10-18)

- **ToF obstacle/edge stop** (2026-10-19): the governor runs again but limits only when switched ON
  from the web page (default off). It was false-flagging cliffs/obstacles on a flat floor (cutting
  forward speed 78-86 %); the page's ToF card now shows each zone's class (obstacle / floor / dropped)
  live, to find those before leaving it on (suspects: body height / lean changing the floor reference).

Marc's recollection of the factory firmware (for step 3 below): full-stick forward had a
**slow start, then reached a decent speed** — consistent with its heavy input filter and
rate-limited accumulating wheel command. That ramp is the target feel.

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
