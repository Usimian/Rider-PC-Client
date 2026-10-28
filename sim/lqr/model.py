"""A model of the Rider as a wheeled inverted pendulum, for designing its LQR (2026-10-28).

Ported from the Hover's sim/hover_model.py (~/hover, commit e2fa373): the two robots have the same
hub-motor wheels and the same ESP32 drive, so the motor model and its fitted numbers start out
shared, and the Rider's own recordings check them (fit_sweep.py). Run with the system Python
(numpy + scipy), not sim/.venv (that is the MuJoCo sim the RL policy trains in).

Planar: the body pitches about the wheel axle while the wheels roll without slipping. States:
x (axle position, m, + forward), v (dx/dt), th (body pitch from vertical, rad, + forward lean),
w (dth/dt). The legs are held rigid at ride height (480/921); the body is everything above the axle.

    (M + Iw/r^2) x'' + mb L cos(th) th'' - mb L sin(th) th'^2 = T / r
    mb L cos(th) x''  + (Ib + mb L^2) th''  - mb g L sin(th)   = -T

M = mb + mw. T is the motors' total torque on the wheels (the body takes -T). Each motor, in the
open-loop drive mode both robots use (wheel register 0x11 = 0), behaves like a voltage-driven DC
motor with friction:

    T_one = ku * dz(u) - b * rel - fric * tanh(rel / fric_w) + b * cogging_rate(phi)
    rel = v/r - w (wheel rate against the body), dz(u) = sign(u) * max(|u| - u0, 0)

u is in the firmware's wheel-command units, + driving forward (on the Rider the firmware's
balance output drives backward when positive: model u = -u_fw, closed_loop.py).

The encoders are in the wheel servos on the legs: they read the wheel's turn against the body
(encoder()), MEASURED on the Rider 2026-10-28 (tools/balance/imu_position_fit.py, -0.92 of r x lean
rate with the axle held by hand).

Parameters carry their provenance. Shared with the Hover = the same part, its value from the
Hover's measurements and fits (docs/hover_params.md there), to be checked on the Rider.
"""
from dataclasses import dataclass

import numpy as np

G = 9.81


@dataclass
class Params:
    m_total: float = 0.560     # kg, MEASURED 2026-06-13 (560 g total)
    com_all: float = 0.0405    # m, whole robot's centre of mass above the axle, MEASURED 2026-10-28, legs
                               # powered at ride height: on its back balanced across a round pencil parallel to
                               # the axle, the pencil 40 mm level from the wheel centre (Marc, +/-1-2 mm), the IMU
                               # at 84.3 deg there (data/captures/2026-10-28_pencil_balance_and_flat.csv) and
                               # 3.55 deg balancing upright: 40 / sin(84.3 - 3.55) = 40.5 mm. The 2026-06-13
                               # 45 mm (RL sim) is superseded
    r: float = 0.030           # m, wheel radius, KNOWN (firmware WHEEL_CIRC_M)
    m_wheels: float = 0.110    # kg, both wheel assemblies at the axle, shared with the Hover (ASSUMED 55 g
                               # each there; the RL sim assumed 40 g -- one of the two is wrong)
    iw_one: float = 3.0e-5     # kg m^2, one wheel's rotating part about the axle, shared (ASSUMED)
    ib_axle: float = 15.9e-4   # kg m^2, body about the axle, ASSUMED: the RL sim's 50 x 100 mm box
                               # (4.7e-4 about its centre) + mb L^2; to be fitted (drop tests, not yet run)
    ku_over_b: float = 0.100   # rad/s per unit, shared (Hover MEASURED in the air: 3.0 mm/s per unit);
                               # Rider bench 2026-06-19: 0.129 from a speed chain later found 26 % strong
                               # (rider_params.vel_max_rad_s) -> 0.102
    b: float = 1.53e-3         # N m s/rad, one motor's damping, shared (Hover DERIVED, its fit_sweep.py)
    u0: float = 0.0            # dead zone in the drive, units, shared (Hover DERIVED: none)
    fric: float = 3.55e-3      # N m, Coulomb friction per wheel, shared (Hover DERIVED; ~23 units)
    fric_w: float = 0.094      # rad/s, the friction rounded off (tanh) below this, shared (Hover DERIVED)
    cog20_rate: float = 0.30   # cogging per wheel at 20 and 24 per turn of the wheel against the body,
    cog24_rate: float = 0.23   # rad/s of speed ripple (torque = that x b), shared (Hover MEASURED in the air)
    cog_phase2: float = 1.3    # rad, wheel 2's offset against wheel 1, shared (ASSUMED)

    @property
    def mb(self):  # body mass
        return self.m_total - self.m_wheels

    @property
    def L(self):  # body centre of mass above the axle (the wheels' mass sits at the axle)
        return self.com_all * self.m_total / self.mb

    @property
    def ib_com(self):  # body inertia about its own centre of mass
        return self.ib_axle - self.mb * self.L ** 2

    @property
    def ku(self):  # torque per unit of command above the dead zone, one motor, N m
        return self.ku_over_b * self.b


def encoder(p: Params, s):
    """What the firmware's wheel_x and wheel speed read for state s: x - r th and v - r w (the
    wheel's turn against the body, times r), not the axle's travel. Works on (4, N) states too."""
    return s[0] - p.r * s[2], s[1] - p.r * s[3]


def dead_zone(u, u0):
    return np.sign(u) * np.maximum(np.abs(u) - u0, 0.0)


def accel(p: Params, s, u):
    """Accelerations (x'', th'') for state s = (x, v, th, w) and command u (both wheels the same,
    + drives forward). s may be (4, N) and u (N,): N cases at once."""
    x, v, th, w = s
    rel = v / p.r - w                    # wheel rate against the body, rad/s
    phi = x / p.r - th                   # wheel angle against the body, rad
    drive = dead_zone(u, p.u0)
    cog = 0.0
    for off in (0.0, p.cog_phase2):
        cog += p.b * (p.cog20_rate * np.sin(20 * phi + off) + p.cog24_rate * np.sin(24 * phi + 2 * off))
    T = 2.0 * (p.ku * drive - p.b * rel - p.fric * np.tanh(rel / p.fric_w)) + cog
    M = p.mb + p.m_wheels
    a11 = M + 2.0 * p.iw_one / p.r ** 2
    a12 = p.mb * p.L * np.cos(th)
    a22 = p.ib_com + p.mb * p.L ** 2
    rhs1 = T / p.r + p.mb * p.L * np.sin(th) * w ** 2
    rhs2 = p.mb * G * p.L * np.sin(th) - T
    det = a11 * a22 - a12 * a12
    xdd = (rhs1 * a22 - a12 * rhs2) / det
    thdd = (a11 * rhs2 - a12 * rhs1) / det
    return xdd, thdd


def step(p: Params, s, u, dt):
    """One RK4 step of length dt with the command held."""
    def f(s):
        xdd, thdd = accel(p, s, u)
        return np.array([s[1], xdd, s[3], thdd])
    k1 = f(s)
    k2 = f(s + 0.5 * dt * k1)
    k3 = f(s + 0.5 * dt * k2)
    k4 = f(s + dt * k3)
    return s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


if __name__ == "__main__":
    p = Params()
    lam = np.sqrt(p.mb * G * p.L / p.ib_axle)   # fixed-pivot tipping rate, motors off
    print(f"body {p.mb * 1000:.0f} g, its centre of mass {p.L * 1000:.1f} mm above the axle, inertia "
          f"{p.ib_axle * 1e4:.1f} kg cm^2 about the axle ({p.ib_com * 1e4:.2f} about its centre); "
          f"tipping rate {lam:.1f} /s (the lean doubles every {np.log(2) / lam * 1000:.0f} ms)")
    print(f"motor: b {p.b:.2e} N m s, ku {p.ku:.2e} N m per unit, friction {p.fric / p.ku:.0f} units, "
          f"in the air {p.ku_over_b * p.r * 1000:.1f} mm/s per unit")
