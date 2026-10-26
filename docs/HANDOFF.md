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
5. **ToF edge stop is NOT safe at full speed** (10-26, table edge, stop switched OFF / report-only, Marc caught the
   robot): far zones read 'd' ~1.8 s before the edge, the cliff rule asserted only 0.78 s (~23 cm) before, flickered
   off for a frame, and the policy needs ~0.75 s / 18-22 cm to stop on the gentle release ramp. Keep it OFF and the
   robot off tables until reworked: act on the far rows earlier, a hard stop for edges (not the stick-release ramp),
   likely a lower top speed while the stop is on. Wall/obstacle check (phase 2) not done yet. Recorder:
   `tools/tof/tof_safety_capture.py`; capture `data/captures/2026-10-26_tof_off_drive.csv`.
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
