"""Physical + actuator parameters for the XGO Rider 2-wheel balancer sim.

Single source of truth for the model. Every field is tagged with provenance so
we always know how much to trust it:

  KNOWN     - from firmware constants (esp32_rider_fw/src/main.cpp) or hard spec.
  ESTIMATED - a reasonable guess; refine with a kitchen scale + ruler (NO robot
              power needed -- just weigh the robot and eyeball the CoM height).
  BENCH     - the make-or-break sim-to-real params. Measure on the bench with the
              passthrough firmware + wheel telemetry (needs the robot powered).
              Until these are measured, transfer to hardware is not trustworthy.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class RiderParams:
    # ---------- the plant ----------
    plant: str = "servo"                   # "servo": this file's body + rider_env.ActuatorModel (a stiff velocity
                                           # servo with fitted lags); "measured" (2026-10-28): measured_plant.py -- the
                                           # body and DC motor of sim/lqr/model.py, the LQR's checked model. With
                                           # "measured" the body, wheel and actuator fields below are not used.
    # ---------- geometry (KNOWN: firmware) ----------
    wheel_radius_m: float = 0.03           # KNOWN   6 cm dia wheels (WHEEL_CIRC_M = pi*0.06)
    encoder_counts_per_rev: int = 1024     # KNOWN   odometry scaling
    gyro_lsb_per_dps: float = 16.4         # KNOWN   ICM-42670 gyro scale
    control_hz: float = 230.0              # MEASURED 2026-09-16: 234 Hz idle / ~199 Hz driving after the wheel-read
                                           # timeout fix (commit 44dd034); was 165 Hz before it. Training randomizes
                                           # the rate (rider_env.CONTROL_RATE_HZ); this nominal is what drive_sim uses.

    # ---------- firmware sensing path (KNOWN: main.cpp, 2026-09-16 delay budget) ----------
    # The firmware does NOT feed the policy clean state. It differences the ENCODER position (10-bit,
    # wheel relative to body) at loop rate and low-passes that for the velocity obs, and low-passes
    # the gyro for the pitch-rate obs. Per-cycle coefficients, so the time constants scale with the
    # loop rate (~12 ms / ~8 ms at 234 Hz). See docs/loop_delay_budget.md.
    fw_pol_vel_lp: float = 0.7             # KNOWN  gPolVelLP: xvelP = lp*xvelP + (1-lp)*d(wheel_x)/dt
    fw_rate_lp: float = 0.6                # KNOWN  dqf = 0.6*dqf + 0.4*gyro_rate
                                           # (gPosVelLP and the rest of the position loop: fw_drive.Gains)

    # ---------- mass / inertia (MEASURED 2026-06-13, except wheel split) ----------
    track_width_m: float = 0.097           # MEASURED  wheel center-to-center (97 mm)
    body_mass_kg: float = 0.48             # MEASURED  560 g total - ~80 g wheels
    wheel_mass_kg: float = 0.04            # ESTIMATED  per wheel (total weighed, split is a guess)
    com_height_m: float = 0.045            # MEASURED  CoM 75 mm above floor - 30 mm axle = 45 mm above pivot
    body_half_extent_m: tuple = (0.025, 0.04, 0.05)  # ESTIMATED  torso box half-sizes (x,y,z)

    # ---------- actuator model (MEASURED 2026-06-13, bench step-response) ----------
    cmd_mode: str = "velocity"             # MEASURED  command is a velocity setpoint (settles, linear in cmd)
    vel_max_rad_s: float = 17.5            # 2026-10-19 (late): refit with deadband 0.08 to the same cruise data (2.45e-3
                                           # m/s per cmd beyond the dead zone -> 2.45e-3*(1-0.08)*232/0.03 = 17.5).
                                           # The 18.1 below was the fit with the 0.15 dead zone:
                                           # MEASURED 2026-10-19 on the floor: GROUND speed from the encoder distance
                                           # (wx, metres) over 140 one-second cruise windows, both controllers, battery
                                           # 7.3-7.7 V: |v| = 2.75e-3 m/s per command unit x (|cmd| - 35), resid 1.6 cm/s
                                           # -> vel_max = 2.75e-3*(1-0.15)*232/0.03 = 18.1 rad/s AT FULL command (the
                                           # ActuatorModel maps action a -> (a-db)/(1-db)*vel_max). Scatter 0.68-0.91 of
                                           # the old value -> DR vel_gain covers it. Earlier values were 27 then 22.9,
                                           # both from the servo's REPORTED speed x a bench rad/s-per-unit scale; that
                                           # chain was 26 % strong, so in sim ppo_fw_s9 never neared its output limit
                                           # while on the floor it pinned it and fell. With 18.1 + accel_couple the sim
                                           # reproduces those falls (fwd 0.30 ~2.5 s, rev 0.30 pinned; fwd 0.25 holds).
    cmd_limit: float = 1.0                 # KNOWN  largest |command| the controller may send, in units of the policy's
                                           # full scale (action 1 = 30 rad/s x 7.764 = 233 raw wheel-cmd, main.cpp
                                           # POL_VELMAX_RADS). The firmware clamps every controller at gUMax 400 raw:
                                           # the LQR uses that whole range (400/233 = 1.72), the policy only 1.0.
    actuator_tau_moving_s: Optional[float] = 0.045  # MEASURED 2026-10-19 (None = off): the lag once the wheel turns
                                           # steadily. Floor frequency sweeps (injected signal, sim/wheel_sweep.py):
                                           # a wheel reversing around standstill lags 7-18 ms, a MOVING one 36-56 ms at
                                           # 2-5 Hz; tau rising 13 -> 45 ms with wheel speed reproduces both (sim:
                                           # 21-26 ms holding, 29-44 ms driving at 0.30).
    tau_speed_ref_rad_s: float = 2.0       # wheel speed (rad/s) at which the moving lag is fully in
    latency_fast_extra_s: float = 0.015    # MEASURED 2026-10-20: extra pure delay at FULL speed. Floor sweeps at 0.32 m/s
                                           # (two forward runs, ld8_s6, +/-30 cmd 1-6 Hz): the wheel keeps its gain but
                                           # lags 12-25 deg more than the sim at 1-3 Hz (-43..-58 vs -23..-41); +15 ms
                                           # delay fits, a slower first-order lag does not (it loses the gain). Ramps in
                                           # linearly between fast_lat_from / fast_lat_full (the 0.11 m/s sweeps of
                                           # 2026-10-19 need none).
    fast_lat_from_rad_s: float = 3.7       # 0.11 m/s
    fast_lat_full_rad_s: float = 10.7      # 0.32 m/s
    leg_flex_stiffness_Nm_rad: Optional[float] = None  # EXPERIMENT 2026-10-19 (off = None): give in the legs, a
                                           # torsional spring between the legs (wheel servo) and the torso (IMU).
                                           # Marc, by hand: the legs give noticeably when torqued; the robot sits
                                           # 2-3 mm lower loaded than with the wheels just touching.
    leg_flex_damping: float = 0.005        # Nm*s/rad, with the above
    leg_play_deg: float = 0.0              # EXPERIMENT, with leg_flex: free play (backlash) +/- this many degrees, in
                                           # which only the weak leg_flex spring acts; beyond it leg_play_stiffness
    leg_play_stiffness_Nm_rad: float = 100.0
    leg_mass_kg: float = 0.08             # ESTIMATED, with the above: the legs' share of body_mass_kg (wheel servos ~2x20 g + links)
    actuator_tau_s: float = 0.013        # MEASURED  first-order lag, amplitude-independent
    deadband_frac: float = 0.08            # MEASURED 2026-10-19 (19 raw cmd): the floor HOLD sweep (the wheel follows the
                                           # command ~0.9-0.97 mm/s per cmd flat over 1-7 Hz) needs it -- at 0.15 the sim
                                           # wheel gave 0.7 -> 0.3 -- and the cruise fits with a free breakaway gave 20-28
                                           # raw. With it (+ vel_max 17.5, the moving lag) the sim still reproduces
                                           # ppo_fw_s9's floor falls and ld8_s6 / LQR's 0.30 m/s drives. Was 0.15:
                                           # MEASURED 2026-09-16 LOADED, on the floor (drivecap, 7 drive episodes, both
                                           # policies): steady v ~= 0.9*(cmd-35) raw -> breakaway 35/232 = 0.15. With
                                           # this + vel_max 27 the sim REPRODUCES ppo_fw_s8's full-stick forward fall
                                           # (2.4 s) and ppo_mc_l4a's survival; at the unloaded 0.09 it did not.
                                           # UNLOADED bench value was 0.09 -- MEASURED 2026-06-19 (LQR fw, command-ladder stepcap; sweep_actuator.py
                                           # -> sim/actuator_bench_lqr.csv, validated by replay_actuator.py): the
                                           # wheel breaks away at firmware-cmd ~21 (action ~0.09) then is LINEAR to
                                           # full -- a simple deadband, NO discrete creep-jump. Loaded sweep (held,
                                           # weight on wheel) agrees the breakaway stays ~this, just intermittent
                                           # (-> randomized 0.06-0.20 in DR_RANGES). Earlier 0.18 + creep-jump was a
                                           # coarse-measurement artifact #1 caught against the bench.
    latency_s: float = 0.004               # DERIVED 2026-09-16 = ONE control step. Calibrated so the SIM's
                                           # cmd->wheel-velocity cross-correlation lag (drive_sim, same method as the
                                           # robot's drivecap) equals the robot's MEASURED 30-35 ms: the sim already
                                           # carries ~20 ms of its own (ZOH + one-step obs delay + the 13 ms tau), so
                                           # only ~4 ms of extra pure delay is needed. With this the sim reproduces
                                           # ppo_mc_l4a's forward ~8.8 Hz ring + quiet reverse, and ppo_delay34_l4's
                                           # full-stick fall. The 2026-08-13 value 0.021 (robot 34 ms minus tau)
                                           # double-counted -> 64 ms sim lag and a 4 Hz thrash the robot never shows
                                           # (and the delay buffer was off by one step, fixed the same day). The
                                           # 2026-06 value 0.003 was the unloaded USB-bench onset.
    vel_kv: float = 5.0                    # sim velocity-servo gain (higher overshoots on the tiny wheel inertia)
    vel_forcerange_Nm: float = 1.0         # sim wheel torque cap (the real 13ms tracking lag lives in ActuatorModel)
    wheel_armature: float = 0.003          # reflected motor-rotor inertia on wheel DOF (rotor x gear^2); makes wheel
                                           # velocity smooth like the real geared servo -- was the key fidelity fix
    torque_max_Nm: float = 0.5             # (unused in velocity mode; kept for reference)

    # ---------- loaded stick-slip (BENCH 2026-06-25, sim/actuator_bench_lqr_loaded.csv) ----------
    # Under the robot's WEIGHT the wheel is not a clean deadband: it sticks until a HIGH static
    # breakaway (loaded sweep: dead through firmware-cmd ~60, clean slip at ~70 / full-scale 233),
    # slips with a discontinuous jump, then re-sticks below a LOWER kinetic floor (~cmd 30) =
    # hysteresis. That near-zero-speed stick-slip is the residual hold-hunt limit cycle. The clean
    # deadband above is the UNLOADED fit; this is the loaded reality. Off by default (v2 trained on
    # the clean model); the v5 experiment turns it on to see if the policy learns to break stiction.
    stick_slip: bool = False               # use the hysteretic stick-slip actuator instead of the clean deadband
    stiction_static_frac: float = 0.25     # BENCH  loaded breakaway from rest (norm cmd; ~60-70/233)
    stiction_kinetic_frac: float = 0.12    # BENCH  kinetic floor / re-stick threshold once moving (~30/233)

    # ---------- sensing noise (ESTIMATED: for domain randomization) ----------
    gyro_noise_dps: float = 1.0            # ESTIMATED  gyro white-noise std
    gyro_bias_drift_dps: float = 0.3       # ESTIMATED  residual bias after gcal
    accel_pitch_noise_deg: float = 0.5     # ESTIMATED  accel-derived pitch noise
    accel_couple: float = -0.85            # MEASURED 2026-10-19 (recorder, policy drives): the firmware's accelerometer
                                           # pitch (lever-arm compensated, the one it fuses) shifts by +0.66..+0.97 x
                                           # atan(a/g) deg in the robot frame during chassis acceleration a -- a
                                           # forward-accelerating robot "reads" a backward lean. Sim frame = minus that.
                                           # The complementary filter (FirmwareSensing) lets it leak into the policy's
                                           # pitch; without it the sim never showed the floor's ~1-1.5 Hz drive hunt.

    # ---------- sim integration ----------
    physics_timestep_s: float = 0.001      # 1 kHz physics

    def control_decimation(self) -> int:
        """Physics steps per control step (control runs at control_hz)."""
        return max(1, round((1.0 / self.control_hz) / self.physics_timestep_s))


DEFAULT = RiderParams()
