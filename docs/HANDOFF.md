# Handoff: the RL balance policy + robot reliability (live investigation)

Read at session start. Facts live in the repo (commit messages carry the numbers, `sim/rider_params.py` the
provenance, `docs/HARDWARE.md` -> Power / Audio the hardware facts); this is only the running story, the open
list, and what not to trust.

## Where it stands (2026-10-22)

- **Flashed + committed: `ppo_lb25_s4`** -- the spin shimmy fixed on the floor (spin pitch-rate RMS 3.1 / 5.3 /
  3.4 °/s by drivecap, ld8_s6 13-23). Backup `ppo_lb25_s1`. Training lever: `--lead_lambda` + a firm
  stick-pushed drive anchor; judge candidates by the floor-replay phase lead, not the sim's spin number.
- **Fall at full leg height** on full-forward-into-full-reverse (10-21). Cause found: a stick flip passes the
  dead zone, and the release used to be a full emergency stop (pitch +12.8° at ride height). Height barely
  moves the CoM (Marc: 80 mm above the floor at max = 50 mm above the axle vs 45 at ride). Fixed in firmware
  (`releaseStop`: keep speed, brake on the ramp) and in `drive_sim` (`release_s`). **Not floor-tested yet.**
- **First command after a real power-up was lost**: ModemManager probed the ESP32 UART ~3 s after the bridge
  started. Masked (deploy does it); the firmware drops a partial line on line noise.
- **Pi power: a bad contact in the 5 V lead**, reseated 10-21 (full-load min 4.60 -> 4.72 V, no alarms); it
  will loosen again. The web fault item shows Pi under-voltage alarms and WiFi dropouts.
- **WiFi:** watchdog (`rider-wifi-watchdog`); field network = Marc's iPhone hotspot (home has priority).
- **Cleanup 10-22:** the old `Rider-Robot-Controller` project is gone (folder trashed, GitHub repo deleted);
  the abandoned sim experiments and their firmware paths (balance+turn, position-aware, diff-drive) are removed.

## Open list, in order

1. **Floor test of the release / height changes** (firmware `releaseStop`, keep height after a pick-up):
   full-speed release (smooth stop ~11 cm on), flips at ride height, raise to max + pick up + set down (height
   kept), then flips at max height, gentle first.
2. **First balance-on after a power-up** (the left leg used to lag): `legstkL/R` and the `# leg` lines in
   riderlog say whether a leg arrived; `rider/debug/unknown` shows any stray line after boot (should be none
   now that ModemManager is masked -- not yet confirmed by a power-up).
3. Wheel read failures at power-up (469 / 496, then 0): read `rfL/rfR`, `rft0/rft1` after the next boot that has
   them. Possibly ModemManager too.
4. Voice: rebuild fresh in `pi/` (facts in HARDWARE.md -> Audio).
5. Watch the Pi under-voltage count after floor driving.
6. ToF obstacle/edge stop: check its zone classes while driving before leaving it on.
7. `/home/pi/riderlog.txt` is never rotated (321 MB on 10-21). Filter on the Pi; never copy it whole over WiFi.
8. The controller low-battery cutoff (<= 15 %, unplugged) is untested.
9. `.claude/settings.json` in the working tree has its read-only `ssh rider` allow list emptied (not mine, not
   committed) -- ask Marc.

## Do not trust

- Commit `dabc54f`'s line "after masking, a power-up took 'leghome' as its first command": that test was at
  14:24, BEFORE ModemManager was masked. No power-up has been checked since masking.
- Sim spin numbers as a floor prediction (ld8_s6 6 vs floor 18; tt25_s4 3.6 vs 21). Use the floor-replay lead.
- The sim's reversal before 10-21: it ramped smoothly through zero and never modelled the stick release.
- The Pi's always-on telemetry log for phase or frequency at 1-5 Hz (~11 rows/s); magnitude only.
- Closed-loop fits of plant dynamics from drive recordings without an injected signal -- use `chirp` sweeps.
- The 2 GHz cap as the under-voltage fix: the contact was the cause. The cap stays.
- The WiFi watchdog against a real silent chip failure: only tested with a NetworkManager disconnect.
- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off, never matched.
