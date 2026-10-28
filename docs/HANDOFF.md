# Handoff: the RL balance policy + robot reliability (live investigation)

Read at session start. Facts live in the repo (commit messages carry the numbers, `sim/rider_params.py` the
provenance, `docs/HARDWARE.md` -> Power / Audio the hardware facts); this is only the running story, the open
list, and what not to trust.

## Where it stands (2026-10-28 evening): LQR2 = the Hover's LQR exactly, physical numbers apart

- **Flashed + committed (`e387674`), Marc: "it drove great".** One IMU loop (the Hover's imu_task, core 0, the only
  IMU reader), the Hover's IMU step, law order and drive plan (1 m/s^2); gains -1916 / -7.46 / 59.4 / 1.39 (Tightest
  1936 / 8.26 / 53.8 / 1.25); top speed 0.45. Rule (Marc): copy the Hover exactly, change only what is physically
  different, name every difference -- so a remaining problem is the robot's, not the implementation's.
- **Later that evening:** the June LQR removed, LQR2 renamed LQR (`954109a`); the home held where the stick is released
  and Circle restarting the plan in place (`1c8761e`, the one deliberate change from the Hover, Marc's choice; the
  twitching of a first version is gone).
- **The start twitch (Marc: lean forward, lean back a little, then go; full stick from standing)** is the Hover's
  plan doing what it is built to do: full stick asks for the 1 m/s^2 cap (8/s x 0.3 m/s = 2.4, capped), built over
  0.12 s, so a 10.7 deg lean target appears in an eighth of a second; the LQR throws the body ~14 deg at 137-142 deg/s
  and catches it ~3.5 deg back. The model predicts it (135 deg/s) and the Hover's own drives do it harder (145-209
  deg/s, 14 deg/m/s^2). **Live `lqramax 0.6` (not flashed, RAM only -- a power cycle restores 1.0):** 82-90 deg/s,
  ~8.5 deg, a ~1 deg catch, gyro above 8 Hz halved; Marc: "it felt pretty good". Recordings `*_lqr_twitch_03`
  (1.0) and `*_lqr_amax06_05` (0.6); `*_lqr_amax06_04` is ALSO 1.0 (the command was sent as bare text and dropped
  -- the bridge takes only `{"line": ...}`). **Flashed (Marc's go): 0.6 is the default** -- the first change
  from the Hover's plan (the other option, a 0.25 s build-up keeping 1 m/s^2 for stops, untried) -- with the
  drive capture's `stick_mms plan_acc_x1000 plan_lean_x100` (the capture readers expect them; `drive_starts.py`
  finds starts from the stick) and the telemetry `amax=`. Floor check `*_lqr_amax06_flashed_06`: amax=0.60 on
  boot, full stick in ~25 ms, planned lean to 6.4 deg, body 86-92 deg/s, ~1.3 deg past the target.
- **Next, in order:** (1) the RL policy: first a floor baseline of `ppo_lb25_s4` on this firmware (the single IMU loop
  changed its timing: ~240 Hz, samples 0-2.4 ms old), then retraining with the measured body (CoM 40.5 mm, inertia
  17.5 kg cm^2, IMU 98 mm) by the same recipe -- Marc wants it revisited. **The RL sim vs the measured model**
  (`sim/rider_params.py`, `rider_model.py` vs `sim/lqr/model.py`, read 10-28): the motor is a stiff velocity
  servo (kv 5, 1 N m cap; lags fitted on top) where the measured one is a DC motor whose speed sags with load
  (b 1.63e-3 per motor, ~0.076 N m stall at the policy's full command) -- the big one; CoM 45 vs 40.5 mm; body
  inertia ~14.7 (box) vs 17.5 kg cm^2; wheel armature 3e-3 vs 3e-5 kg m^2; wheels 80 vs 110 g; 230 vs ~240 Hz.
  Same already: the encoder reads the wheel against the body; friction ~35 vs 33 units. **Built (`96ee0b5`):
  `RiderParams.plant = "measured"`** (`sim/measured_plant.py`, straight-line only). `ppo_lb25_s4` in it: standing
  rate rms 1.8-2.1 deg/s (floor 1.6-2.8, servo 3.2-3.4); release from 0.30 rolls 22.4 cm, the servo 22.1, the floor
  ~20 (`*_policy_baseline_07`: 197 mm, 1.46 s) -- the roll-on is the firmware's policy position loop, not the
  motor; retraining alone will not stop it. Floor baseline 10-28: standing 0.14-0.29 deg sd (LQR 0.03-0.08),
  206-225 Hz, start/release swings 3.6-6.3 deg, no reversal caught; (2) the ~10 Hz ringing after a stop or
  drive, three times now (`*_hover_exact_watch` w18, `*_renamed_check` w15, `*_smooth_home` w12); (3) the balance
  point moved: 3.9-4.3 deg on the IMU in the last run against the 2.8 reference (2.9-3.0 earlier that evening, legs
  at ride height both times, cause unknown) -- it parks 1-3 cm off home until the slow integral catches up;
  (4) redesign at the measured ~240 Hz (the gains assume 200); (5) the gyro bias (the Hover measures it each boot);
  (6) the +/-400 command limit (the Hover's +/-1000; 1000 browned out ID21 once).

## Earlier on 2026-10-28

- **Flashed + committed (`510957a`): LQR2 (`ctrl 2`)**, beside LQR and the policy: the Hover's method and its
  choices on the Rider's measured model (`sim/lqr/`, system Python): encoder corrected (it reads the wheel
  against the body), speed smoothed 0.1 s, gains -1694 / -6.78 / 53.3 / 1.32, friction compensation 32.5.
  Floor: standing much quieter than LQR, no fast mode; pushes and drives to 0.28 m/s clean. The robot boots on
  LQR2 (saved choice). Top speed as the Hover: full stick 0.30, web slider to 0.60, firmware caps 0.60.
- **Measured today** (`docs/HARDWARE.md`): IMU 96-100 mm above the axle (firmware lever correction uses 60);
  whole-robot CoM 40.5 mm; body inertia 17.5 kg cm^2 (drops); motor = the Hover's, friction 33 units (1.4x).
- **The model is blind above ~10 Hz:** a 1-30 Hz sweep shows a resonance at ~20-25 Hz it lacks (robot up to 4x).
  That is why the first LQR2 (8 ms smoothing, model-rated best) shook at 14 Hz. Flexible-joint model parked.
- Recording: `tools/capture/balance_watch.py` (full-rate windows, a line per window as it lands). Always run it
  before a floor test.

## Where it stood (2026-10-27)

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
- **Pi power: a bad contact in the 5 V lead**, reseated 10-21 (full-load min 4.60 -> 4.72 V, no alarms). **It is
  loosening again** (kernel count per boot): 0 through 10-22 morning; 1/3/0/1 later 10-22; 12 in 1 h 45 on 10-26
  afternoon; 10 in 20 min on 10-27, now at ~3 W peaks (was clean at 6 W). At rest the rail reads 4.94-5.06 V.
  The web fault item shows the alarms and WiFi dropouts.
- **WiFi:** watchdog (`rider-wifi-watchdog`); field network = Marc's iPhone hotspot (home has priority).
- **Braking rate** (`f9fe690`, 10-26): the drive ramp brakes at `gPosDmax` 0.8 m/s² (live `posdmax`), speeds up
  at 0.4. Floor, LQR: from 0.30 m/s at rest in 0.3-0.5 s, peak 11-12 cm, 7.5-9.6° lean, no falls. The policy is
  unchanged (17-27 cm over 1.8-3 s, only 2.4-3.9° lean: it follows the commanded speed loosely). Options for
  the policy put to Marc, not chosen: make the sim reproduce the 2 Hz wobble then retrain firmer; LQR does the
  braking; leave it.
- **ToF governor = a wall stop only** (`68bdace`, 10-27): slows from 0.40 m, stops at 0.10 m (Marc: half the old
  0.80 / 0.20). The **table-edge stop is removed** (Marc's call): the sensor sits ~6 cm up, so the row that would
  see an edge in time (~30 cm out) meets the surface at ~11° and goes blank as the body rocks back past ~4°;
  every rule tried either drove off (three table runs) or stopped on solid table (`tools/tof/tof_edge_replay.py`).
  Pitch, roll and leg height are already in every point -- a missing return has no point to correct. Edges
  would need a small ToF looking straight down at the front. Wall floor test: LQR stopped at 9.2 cm.
- **ToF at 10 Hz** (`ba0b388`): it ran at 5 Hz -- the Pi I2C bus was at its 100 kHz default; now
  `i2c_arm_baudrate=400000` (runbook). Status 9 (valid range, large pulse) is now accepted. The governor
  publishes each zone's floor-referenced point; the web obstacle pane switches its 8x8 grid for a 3D view of them.
- **ToF tools:** `tof_raw_capture.py` (raw distances + status codes + body state), `tof_raw_vs_verdict.py`
  (raw vs the governor, frame by frame), `wall_stop_report.py`, `tof_safety_capture.py` (verdicts).
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
4. **Pi 5 V contact** (see above): reseat it (robot off), then read the alarm count after a boot and a drive.
   For good: solder one side of the connector (Marc's idea -- still separable).
5. **ToF: striped blank verdicts while driving (OPEN, cause unknown).** On 2 of 3 stick-driven policy wall
   approaches (10-27 morning, local `data/captures/2026-10-27_wall_stop.csv`) whole columns of zones came back
   invalid, in stripes, even 1.4 m out, and near the wall the governor saw nothing and lifted the cap (commanded
   speed crept to 0.19 m/s against the wall). Ruled out: a range limit (stepping to the wall read all 64 zones
   to 1.9 cm), wall contact and bad sensor data (Marc watched), the governor's code (raw vs verdict match frame
   for frame). Not reproduced by a ptgt-driven 0.30 m/s drive. That run was at 5 Hz / 100 kHz I2C -- possibly
   related, unproven. Next: a stick-driven policy approach with `tof_raw_capture.py` + `tof_safety_capture.py`.
6. **ToF follow-ups:** the policy does not stop for walls (wheels still ~0.25-0.29 m/s 12 cm out with the
   commanded speed ~0.1; 11.2 / 9.6 / 0 cm = hit) -- not decided. Blind-hold (designed, not built): while the
   corridor returns almost nothing and the last obstacle was inside the slowdown zone, hold the last cap and
   show "blind".
7. `/home/pi/riderlog.txt` is never rotated (321 MB on 10-21). Filter on the Pi; never copy it whole over WiFi.
8. The controller low-battery cutoff (<= 15 %, unplugged) is untested.

## Do not trust

- The web page / LCD status **while a recorder runs**: each capture dump (~600 lines, ~57 kB at the 115200-baud
  Pi link) silences telemetry 3.7-6.2 s, so an arm shows up late (the robot balances at once). Gone without a
  recorder -- only the capture tools send `logdump`.
- `balance_watch.py` files saved before 2026-10-28 ~11:10 (`*_watch.csv`, `tap_test.csv`): t_us rounded to 1-10 ms;
  use `sample_times()`. The LQR2 8 ms gains (-1844 / -6.89 / 46.0 / 3.67): shook on the floor.
- The sim/lqr model above ~10 Hz (see above), and anything it rates for light smoothing or high lean-rate gain.
- Commit `59cad1e`'s IMU entry ("assumes 120 mm", "ride height"): corrected in `3e07460` (60 mm; legs were limp).
  Telemetry `legR/legL` show ride-height defaults for a limp leg: read `legst`.

- `drive_gain_sweep.py` / `deep_check --k` for the policy's position loop: the sim liked K 40 / KD 20 (stop 28 ->
  19 cm, 0/90 falls); on the floor (10-22 15:24) it drove with a ~2 Hz pitch oscillation (+-7 deg, std 3-5 deg vs
  1.2) and the releases did not get shorter (18.5 / 23.2 cm). Put back to 20 / 10 / 10 live; the sim is missing
  whatever sets that 2 Hz mode.
- Commit `dabc54f`'s line "after masking, a power-up took 'leghome' as its first command": that test was at
  14:24, BEFORE ModemManager was masked. No power-up has been checked since masking.
- Commit `68bdace`'s "within ~10 cm the sensor returns nothing": wrong, it reads to contact. Commit `b9a73ff`'s
  "the blank frames came after the policy hit the wall": my speculation, wrong (it did not hit).
- ToF timing reasoning before `ba0b388` (10-27) assumed 10 Hz; the sensor ran at 5 Hz. Capture timestamps are real.
- Sim spin numbers as a floor prediction (ld8_s6 6 vs floor 18; tt25_s4 3.6 vs 21). Use the floor-replay lead.
- The sim's reversal before 10-21: it ramped smoothly through zero and never modelled the stick release.
- The Pi's always-on telemetry log for phase or frequency at 1-5 Hz (~11 rows/s); magnitude only.
- Closed-loop fits of plant dynamics from drive recordings without an injected signal -- use `chirp` sweeps.
- The 2 GHz cap as the under-voltage fix: the contact was the cause. The cap stays.
- The WiFi watchdog against a real silent chip failure: only tested with a NetworkManager disconnect.
- The leg-give / free-play sim options (`leg_flex_*`, `leg_play_deg`): experiments, off, never matched.
