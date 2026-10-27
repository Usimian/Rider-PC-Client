# Handoff: the RL balance policy + robot reliability (live investigation)

Read at session start. Facts live in the repo (commit messages carry the numbers, `sim/rider_params.py` the
provenance, `docs/HARDWARE.md` -> Power / Audio the hardware facts); this is only the running story, the open
list, and what not to trust.

## Where it stands (2026-10-26)

- **Flashed + committed: `ppo_lb25_s4`** -- the spin shimmy fixed on the floor (spin pitch-rate RMS 3.1 / 5.3 /
  3.4 °/s by drivecap, ld8_s6 13-23). Backup `ppo_lb25_s1`. Training lever: `--lead_lambda` + a firm
  stick-pushed drive anchor; judge candidates by the floor-replay phase lead, not the sim's spin number.
- **Fall at full leg height** on full-forward-into-full-reverse (10-21). Cause found: a stick flip passes the
  dead zone, and the release used to be a full emergency stop (pitch +12.8° at ride height). Height barely
  moves the CoM (Marc: 80 mm above the floor at max = 50 mm above the axle vs 45 at ride). Fixed in firmware
  (`releaseStop`: keep speed, brake on the ramp) and in `drive_sim` (`release_s`). **Floor 10-22, lb25_s4:**
  full flips swing 2.6-3.4 deg at ride height (was ~13), 3.8-7.0 at full height (fell before), 5.0 at the
  lowest; releases 1.5-6.8 deg, no roll-back; LQR at full height flip 5.0. No falls. Capture
  `data/captures/2026-10-22_release_floor.csv`. **Height kept after a pick-up** (10-26): raised to full height,
  tipped past 25° back and forward -- balancing cut out and came back, the leg target stayed 535 both times.
- **First command after a real power-up was lost**: ModemManager probed the ESP32 UART ~3 s after the bridge
  started. Masked (deploy does it); the firmware drops a partial line on line noise.
- **Pi power: a bad contact in the 5 V lead**, reseated 10-21 (full-load min 4.60 -> 4.72 V, no alarms); it
  will loosen again. The web fault item shows Pi under-voltage alarms and WiFi dropouts.
- **WiFi:** watchdog (`rider-wifi-watchdog`); field network = Marc's iPhone hotspot (home has priority).
- **Braking rate** (`f9fe690`, 10-26): the drive ramp brakes at `gPosDmax` 0.8 m/s² (live `posdmax`), speeds up
  at 0.4. Floor, LQR: from 0.30 m/s at rest in 0.3-0.5 s, peak 11-12 cm, 7.5-9.6° lean, no falls. The policy is
  unchanged (17-27 cm over 1.8-3 s, only 2.4-3.9° lean: it follows the commanded speed loosely). Options for
  the policy put to Marc, not chosen: make the sim reproduce the 2 Hz wobble then retrain firmer; LQR does the
  braking; leave it.
- **Cleanup 10-22:** the old `Rider-Robot-Controller` project is gone (folder trashed, GitHub repo deleted);
  the abandoned sim experiments and their firmware paths (balance+turn, position-aware, diff-drive) are removed.

## Open list, in order

1. **First balance-on after a power-up** (the left leg used to lag): two clean power-ups since ModemManager was
   masked (no stray lines, both legs "ok" on the arrival check). Keep watching `legstkL/R`.
2. Wheel read failures at power-up (469 / 496, then 0): read `rfL/rfR`, `rft0/rft1` after the next boot that has
   them. Possibly ModemManager too.
3. Voice (parked 10-26): rebuild fresh in `pi/` (facts in HARDWARE.md -> Audio). Options weighed: A spoken commands
   on the Pi (offline), B conversation via an LLM, C both. The Pi can't run a useful LLM; **Groq is the chosen
   cloud option** (Marc, 10-26): free tier, Whisper speech-to-text + open LLMs, sub-second, already the
   news-reader fallback, and it works in the field over the phone hotspot. It sends voice/text off-site --
   Marc's explicit OK under the local-only rule. The workstation's local model is the at-home alternative.
4. Watch the Pi under-voltage count after floor driving.
5. **ToF: wall stop only; the edge stop is REMOVED (Marc's call, 10-27).** Walls now slow from 0.40 m and stop at
   0.10 m (Marc asked, half the old 0.80 / 0.20) -- deployed 10-27. Floor (`tools/tof/wall_stop_report.py` on
   `data/captures/2026-10-27_wall_stop.csv`): LQR stops at 9.2 cm; the policy does not -- its wheels are still at
   ~0.25-0.29 m/s 12 cm out while the commanded speed is ~0.1 (11.2, 9.6, 0 cm = hit). **OPEN DEFECT: blind reads
   as clear.** At ~10 cm from a wall the sensor returns nothing at all; the governor reports "none" and the cap
   goes back to full (seen: commanded speed crept forward to 0.19 m/s against the wall). Fix designed, not built:
   while the corridor returns almost nothing and the last obstacle was inside the slowdown zone, hold the last
   cap, and say "blind" on the LCD/page. Policy wall stops: not decided. Why no edge stop: at ride height the sensor is
   ~6 cm up, so row 6 (the row that sees an edge in time, ~30 cm out) meets the surface at ~11 deg and goes blank
   as the body rocks back past ~4 deg (empty corridor zones: 0.2 at 3 deg, 1.3 at 4, 1.9 at 5, 3 at 6+). The
   continuity rule (`tools/tof/tof_edge_replay.py`) called the three real edges 1.2-1.4 s early but also stopped
   while standing on solid table; gating by tilt did not cure it; row 7 alone was too late. Pitch, roll and leg
   height ARE already in every depth point -- a missing return has no point to correct. Way forward if edges
   matter: a small ToF looking straight down at the front (as robot vacuums do).
6. `/home/pi/riderlog.txt` is never rotated (321 MB on 10-21). Filter on the Pi; never copy it whole over WiFi.
7. The controller low-battery cutoff (<= 15 %, unplugged) is untested.

## Do not trust

- `drive_gain_sweep.py` / `deep_check --k` for the policy's position loop: the sim liked K 40 / KD 20 (stop 28 ->
  19 cm, 0/90 falls); on the floor (10-22 15:24) it drove with a ~2 Hz pitch oscillation (+-7 deg, std 3-5 deg vs
  1.2) and the releases did not get shorter (18.5 / 23.2 cm). Put back to 20 / 10 / 10 live; the sim is missing
  whatever sets that 2 Hz mode.
- Commit `dabc54f`'s line "after masking, a power-up took 'leghome' as its first command": that test was at
  14:24, BEFORE ModemManager was masked. No power-up has been checked since masking.
- Sim spin numbers as a floor prediction (ld8_s6 6 vs floor 18; tt25_s4 3.6 vs 21). Use the floor-replay lead.
- The sim's reversal before 10-21: it ramped smoothly through zero and never modelled the stick release.
- The Pi's always-on telemetry log for phase or frequency at 1-5 Hz (~11 rows/s); magnitude only.
- Closed-loop fits of plant dynamics from drive recordings without an injected signal -- use `chirp` sweeps.
- The 2 GHz cap as the under-voltage fix: the contact was the cause. The cap stays.
- The WiFi watchdog against a real silent chip failure: only tested with a NetworkManager disconnect.
- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off, never matched.
