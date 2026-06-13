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


@dataclass
class RiderParams:
    # ---------- geometry (KNOWN: firmware) ----------
    wheel_radius_m: float = 0.03           # KNOWN   6 cm dia wheels (WHEEL_CIRC_M = pi*0.06)
    encoder_counts_per_rev: int = 1024     # KNOWN   odometry scaling
    gyro_lsb_per_dps: float = 16.4         # KNOWN   ICM-42670 gyro scale
    control_hz: float = 250.0              # KNOWN   measured loop rate (lhz~251, deterministic)

    # ---------- mass / inertia (MEASURED 2026-06-13, except wheel split) ----------
    track_width_m: float = 0.097           # MEASURED  wheel center-to-center (97 mm)
    body_mass_kg: float = 0.48             # MEASURED  560 g total - ~80 g wheels
    wheel_mass_kg: float = 0.04            # ESTIMATED  per wheel (total weighed, split is a guess)
    com_height_m: float = 0.045            # MEASURED  CoM 75 mm above floor - 30 mm axle = 45 mm above pivot
    body_half_extent_m: tuple = (0.025, 0.04, 0.05)  # ESTIMATED  torso box half-sizes (x,y,z)

    # ---------- actuator model (MEASURED 2026-06-13, bench step-response) ----------
    cmd_mode: str = "velocity"             # MEASURED  command is a velocity setpoint (settles, linear in cmd)
    vel_max_rad_s: float = 30.0            # action=1 -> 30 rad/s (measured 0.129 rad/s per raw cmd unit; cmd 200 -> 22.8)
    actuator_tau_s: float = 0.013          # MEASURED  first-order lag, amplitude-independent
    deadband_frac: float = 0.0             # MEASURED  negligible (only a small low-end gain droop)
    latency_s: float = 0.003               # MEASURED  ~3 ms command -> response onset
    vel_kv: float = 5.0                    # sim velocity-servo gain (higher overshoots on the tiny wheel inertia)
    vel_forcerange_Nm: float = 1.0         # sim wheel torque cap (the real 13ms tracking lag lives in ActuatorModel)
    wheel_armature: float = 0.003          # reflected motor-rotor inertia on wheel DOF (rotor x gear^2); makes wheel
                                           # velocity smooth like the real geared servo -- was the key fidelity fix
    torque_max_Nm: float = 0.5             # (unused in velocity mode; kept for reference)

    # ---------- sensing noise (ESTIMATED: for domain randomization) ----------
    gyro_noise_dps: float = 1.0            # ESTIMATED  gyro white-noise std
    gyro_bias_drift_dps: float = 0.3       # ESTIMATED  residual bias after gcal
    accel_pitch_noise_deg: float = 0.5     # ESTIMATED  accel-derived pitch noise

    # ---------- sim integration ----------
    physics_timestep_s: float = 0.001      # 1 kHz physics

    def control_decimation(self) -> int:
        """Physics steps per control step (control runs at control_hz)."""
        return max(1, round((1.0 / self.control_hz) / self.physics_timestep_s))


DEFAULT = RiderParams()
