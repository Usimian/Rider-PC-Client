"""The firmware's position loop around the policy, stick-driven (main.cpp, policy path), as one shared copy
(2026-10-28): the training env (rider_env) and drive_sim both run this, so the policy trains on the loop the
robot runs. Moved from drive_sim.simulate's stick branch unchanged (that copy matched the floor: releaseStop,
the hold trapezoid to the home, the 'lead' target on its leash).

Each control step: the stick's speed command (m/s, 0 = released) in, the setpoint tilt the policy's pitch is
biased by (deg, real frame) out:
  stick held   'lead' -- the target advances at the ramped speed, held within `leash` of the robot, posErr acts;
               'latch' -- the target IS the robot every cycle (posErr 0), only the speed term acts
  released     releaseStop: the home is where the braking ramp would stop the target; the hold trapezoid then
               brakes the target to it from its current speed, the hold integral running once it is there
  bias = clamp(K posErr + KI int + KD (speed - ramped speed), +/-POSMAX)
"""
from dataclasses import dataclass

import numpy as np


def drive_ramp(v, tgt, dt, amax, dmax):
    """One step of the firmware's drive-speed ramp (main.cpp driveRamp): dmax while the speed falls (braking), amax
    while it rises; a braking step stops at 0 instead of carrying through it."""
    braking = (v > 0.0 and tgt < v) or (v < 0.0 and tgt > v)
    d = (dmax if braking else amax) * dt
    n = v + d if tgt > v + d else (v - d if tgt < v - d else tgt)
    if braking and ((v > 0.0 and n < 0.0) or (v < 0.0 and n > 0.0)):
        n = 0.0
    return n


@dataclass
class Gains:
    """main.cpp's settings for the loop (the live command in brackets)."""
    K: float = 40.0          # gPosHoldK (poshold), deg/m -- 20 -> 40 floor-liked 2026-10-28 (with DMAX 1.2)
    KI: float = 1.0          # gPosHoldKi (poshi), deg/(m s)
    KD: float = 10.0         # gDriveKd (drivekd), deg/(m/s)
    POSMAX: float = 8.0      # gPosHoldMax, deg
    AMAX: float = 0.4        # gPosAmax (posamax), m/s^2, speeding up
    DMAX: float = 1.2        # gPosDmax (posdmax), m/s^2, braking -- 0.8 -> 1.2 floor-liked 2026-10-28
    VMAX: float = 0.30       # gPosVmax: the hold trapezoid's speed cap (m/s)
    VELLP: float = 0.9       # gPosVelLP: low-pass on the speed the KD term sees
    LEASH: float = 0.25      # 'lead': the target stays within this of the robot (m)
    DEADBAND: float = 0.012  # gPosDeadband (m)
    INTBAND: float = 0.15    # gPosIntBand (m)


class PolicyStickDrive:
    def __init__(self, x0: float, gains: Gains = None, lead: bool = True, drive_int: bool = False):
        self.g = gains or Gains()
        self.lead, self.drive_int = lead, drive_int
        self.tgt_eff = self.hold_tgt = x0
        self.vdes_s = 0.0
        self.xvelF = 0.0
        self.pint_pos = 0.0
        self.was_driving = False

    def step(self, stick: float, x: float, xvel: float, dt: float) -> float:
        g = self.g
        self.xvelF = g.VELLP * self.xvelF + (1 - g.VELLP) * xvel
        kiLim = g.POSMAX / g.KI if g.KI > 0.01 else 1e6
        driving = stick != 0.0
        if self.was_driving and not driving:             # release edge (releaseStop): keep the speed, home = where
            self.hold_tgt = self.tgt_eff + np.sign(self.vdes_s) * self.vdes_s ** 2 / (2 * g.DMAX)   # the ramp stops it
        self.was_driving = driving
        if driving:
            vcmd = stick
        else:
            dist = self.hold_tgt - self.tgt_eff
            vcap = min(g.VMAX, np.sqrt(2 * g.DMAX * abs(dist)))
            vcmd = vcap if dist >= 0 else -vcap
        self.vdes_s = drive_ramp(self.vdes_s, vcmd, dt, g.AMAX, g.DMAX)
        vdes = self.vdes_s
        if driving and not self.lead:
            self.tgt_eff = x                                 # 'latch': the target is the robot, posErr 0
            pos_err = 0.0
        elif driving:
            self.tgt_eff += self.vdes_s * dt                 # 'lead': advance, never further than the leash
            self.tgt_eff = float(np.clip(self.tgt_eff, x - g.LEASH, x + g.LEASH))
            pos_err = x - self.tgt_eff
            if self.drive_int:
                self.pint_pos = float(np.clip(self.pint_pos + pos_err * dt, -kiLim, kiLim))
        else:
            self.tgt_eff += self.vdes_s * dt
            if (dist >= 0 and self.tgt_eff > self.hold_tgt) or (dist < 0 and self.tgt_eff < self.hold_tgt):
                self.tgt_eff = self.hold_tgt; self.vdes_s = 0.0      # never coast past the home
            vdes = self.vdes_s
            pos_err = x - self.tgt_eff
            inband = abs(pos_err) < g.DEADBAND
            if self.tgt_eff == self.hold_tgt and abs(pos_err) < g.INTBAND and not inband:
                self.pint_pos = float(np.clip(self.pint_pos + pos_err * dt, -kiLim, kiLim))
            if inband:
                pos_err = 0.0
        return float(np.clip(g.K * pos_err + g.KI * self.pint_pos + g.KD * (self.xvelF - vdes), -g.POSMAX, g.POSMAX))
