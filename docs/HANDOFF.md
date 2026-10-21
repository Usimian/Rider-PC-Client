# Handoff: the RL balance policy + robot reliability (live investigation)

Read at session start. Facts live in the repo (commit messages carry the numbers, `sim/rider_params.py` the
provenance, `docs/HARDWARE.md` -> Power the supply measurements); this is only the running story, the open
list, and what not to trust.

## Where it stands (2026-10-21 morning)

- **Flashed: `ppo_lb25_s4`** (+ verified leg power-up, read-fail count). The committed `rider_policy.h` is still
  `ppo_ld8_s6`; lb25_s4 lives in `sim/ppo_lb25_s4_policy.h` until its confirmation run.
- **Spin shimmy (the policy defect): first real improvement.** The sim's own spin number never predicted the
  floor; the **floor-replay phase lead at 4 Hz** does (`sim/seed_sweep.py`, needs >= +25°). Training lever:
  `--lead_lambda` (pull actions on recorded floor spin states toward slightly later targets) + a firm drive
  anchor (`--anchor_lambda 25`, stick-pushed states) so starts/reversals stay ld8_s6-like.
  - Floor spin pitch-rate RMS (°/s): ld8_s6 13-23, sa5_s1 16-20, da5_s0 10.7 (fell on starts/reversals),
    tw2_s6 7.0 (5° lean, fell), **lb25_s4 3.9** -- ONE spin, one direction, from the Pi's always-on log
    (no drivecap). No falls in 2.3 min incl. three hard full-forward/full-reverse swings (~15°, recovered).
  - Backup: `ppo_lb25_s1` (lead +27°, drives closest to ld8_s6).
- **Pi power: a bad contact in the 5 V lead** (mainboard <-> carrier, same 1.5 mm connector both ends).
  Under-voltage alarms came back 10-20 09:43 with no software change; reseating both ends 10-21 cleared them
  (full-load min 4.60 -> 4.72 V, under-voltage half the time -> never). Vibration will loosen it again; Marc is
  weighing soldering one end. All three WiFi failures on record fell in alarm periods.
- **New on the Pi:** `rider-wifi-watchdog` (dropout -> snapshot -> driver restart -> NetworkManager connect;
  log `/home/pi/wifi_events.log`); the web fault item shows today's WiFi dropouts and Pi under-voltage alarms.

## Open list, in order

1. **Confirm lb25_s4 on the floor with `drivecap` running:** spins both ways (5-6 s), a couple of full
   reversals. If it holds, commit it as `rider_policy.h`, trash the other lead/anchor sweep policies
   (`ppo_lf*`, `la25*`, `lb25*` except the chosen + backup), and record the result.
2. **Leg power-up on the floor:** the first balance-on after a power-up is the real test (a leg used to stay
   limp). Telemetry `legrtL/R` (retries) / `legfL/R` (give-ups) say whether the fix had to catch it.
3. **`legstep` from limp is dropped** (both legs stay limp, no energize starts) -- found 10-21, not the leg fix
   (absolute `legL/legR` energize fine). Instrument `legService`'s step path before guessing.
4. **Wheel read failures at power-up** (469 / 496, then 0 on later boots): intermittent; telemetry now has
   `rfL/rfR`, `rft0/rft1` (ms since boot of first/last) and `ms` -- read them after the next boot that has them.
   Possibly the same 5 V supply.
5. Watch the Pi under-voltage count after floor driving; if it climbs, the connector has moved again.
6. ToF obstacle/edge stop runs but is OFF by default; check its zone classes while driving before leaving it on.
7. `/home/pi/riderlog.txt` is never rotated (321 MB on 10-21). Filter on the Pi; never copy it whole over WiFi
   (a ~60 MB pull was running when the WiFi died on 10-21).
8. The controller low-battery cutoff (<= 15 %, unplugged) is untested.

## Do not trust

- Sim spin numbers as a floor prediction (ld8_s6 6 vs floor 18; tt25_s4 3.6 vs 21). Use the floor-replay lead.
- The Pi's always-on telemetry log for phase or frequency at 1-5 Hz (it is ~11 rows/s). Its spin pitch-rate
  RMS did match `drivecap`'s within ~1 °/s on 10-20's spins -- magnitude only.
- Closed-loop fits of plant dynamics from drive recordings without an injected signal -- use `chirp` sweeps.
- The 2 GHz cap as the under-voltage fix: the quiet spell after it most likely came from that day's meter
  probing reseating the connector. The cap stays (it trims current peaks), but the contact is the cause.
- The WiFi watchdog against a real silent chip failure: only tested with a NetworkManager disconnect.
- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off, never matched.
