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

    # ---------- mass / inertia (ESTIMATED: weigh to refine) ----------
    track_width_m: float = 0.10            # ESTIMATED  lateral wheel separation
    body_mass_kg: float = 0.65             # ESTIMATED  torso + electronics + battery
    wheel_mass_kg: float = 0.05            # ESTIMATED  per wheel
    com_height_m: float = 0.09             # ESTIMATED  CoM height above the wheel axle
    body_half_extent_m: tuple = (0.03, 0.045, 0.06)  # ESTIMATED  torso box half-sizes (x,y,z)

    # ---------- actuator model (BENCH: characterize, make-or-break) ----------
    cmd_mode: str = "torque"               # BENCH   "torque"|"velocity" - which the servo tracks cleanly
    torque_max_Nm: float = 0.5             # BENCH   wheel torque saturation (placeholder)
    vel_max_rad_s: float = 30.0            # BENCH   wheel speed saturation (placeholder)
    actuator_tau_s: float = 0.02           # BENCH   first-order command-tracking lag
    deadband_frac: float = 0.0             # BENCH   no-response fraction of range (tight loop => ~0)
    latency_s: float = 0.010               # BENCH   command/comms round-trip delay

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
